
#include "hardware/flash.h"
#include "hardware/gpio.h"
#include "hardware/sync.h"
#include "pico/stdlib.h"
#include "tusb.h"

#include "anim.h"
#include "ws2812.h"
#include <hardware/pio.h>
#include "ws2812.pio.h"
#include <cstdlib>

// ---------- CDC write helpers ----------
static inline void cdc_write(const char *s) {
  if (!tud_cdc_connected()) return;
  tud_cdc_write(s, (uint32_t)strlen(s));
  tud_cdc_write_flush();
}

static void cdc_printf(const char *fmt, ...) {
  if (!tud_cdc_connected()) return;
  char buf[256];
  va_list ap;
  va_start(ap, fmt);
  int n = vsnprintf(buf, sizeof buf, fmt, ap);
  va_end(ap);
  if (n > 0) {
    tud_cdc_write(buf, (uint32_t)n);
    tud_cdc_write_flush();
  }
}

static inline void cdc_prompt(void) { cdc_write("> "); }

// ===============================
//      ENCODER CONFIG / PINS
// ===============================
#define ENC0_A_GPIO 7
#define ENC0_B_GPIO 6
#define ENC1_A_GPIO 1
#define ENC1_B_GPIO 4

#define STEPS_PER_DETENT 4

// ===============================
//       MATRIX CONFIG / PINS
// ===============================
static const uint8_t COL_PINS[3] = {10, 9, 8};
static const uint8_t ROW_PINS[3] = {11, 12, 13};

#define SCAN_SLEEP_MS 20
#define REPEAT_DELAY_MS 55
#define SETTLE_US 80
#define CONFIRM_US 10
#define DISCHARGE_US 5

// ===============================
//    RUNTIME-CUSTOMIZABLE MAPS
// ===============================
// HID modifier bits (Left): bit0=Ctrl, bit1=Shift, bit2=Alt, bit3=GUI(Win)
typedef struct {
  uint8_t mods;
  uint8_t key;
} action_t;

typedef struct {
  action_t cw;
  action_t ccw;
} encoder_map_t;

static action_t g_keymap[3][3] = {
    {{0, 0x04}, {0, 0x05}, {0, 0x06}},  // A B C
    {{0, 0x07}, {0, 0x08}, {0, 0x09}},  // D E F
    {{0, 0x0A}, {0, 0x0B}, {0, 0x0C}},  // G H I
};

static encoder_map_t g_enc[2] = {
    {.cw = {0, 0x4F}, .ccw = {0, 0x50}},  // Right / Left
    {.cw = {0, 0x52}, .ccw = {0, 0x51}},  // Up / Down
};

// ===============================
//       RGB LED CONFIG
// ===============================

static volatile uint8_t g_r = 255;
static volatile uint8_t g_g = 0;
static volatile uint8_t g_b = 0;
static volatile uint8_t g_brightness = 5;

// ===============================
//       ENCODER CORE
// ===============================
static volatile int32_t encoder0_count = 0;
static volatile int32_t encoder1_count = 0;
static volatile uint8_t ab0_state = 0;
static volatile uint8_t ab1_state = 0;

static const int8_t quad_table[16] = {0, +1, -1, 0, -1, 0, 0, +1, +1, 0, 0, -1, 0, -1, +1, 0};

static inline uint8_t read_ab(uint a_gpio, uint b_gpio) {
  uint8_t a = gpio_get(a_gpio) & 1u;
  uint8_t b = gpio_get(b_gpio) & 1u;
  return (b << 1) | a;
}

void gpio_isr(uint gpio, uint32_t events) {
  (void)events;
  if (gpio == ENC0_A_GPIO || gpio == ENC0_B_GPIO) {
    uint8_t curr = read_ab(ENC0_A_GPIO, ENC0_B_GPIO);
    uint8_t idx = (ab0_state << 2) | curr;
    encoder0_count += quad_table[idx];
    ab0_state = curr;
  }
  if (gpio == ENC1_A_GPIO || gpio == ENC1_B_GPIO) {
    uint8_t curr = read_ab(ENC1_A_GPIO, ENC1_B_GPIO);
    uint8_t idx = (ab1_state << 2) | curr;
    encoder1_count += quad_table[idx];
    ab1_state = curr;
  }
}

static void init_encoder_gpio(uint a, uint b) {
  gpio_init(a);
  gpio_set_dir(a, GPIO_IN);
  gpio_pull_up(a);
  gpio_init(b);
  gpio_set_dir(b, GPIO_IN);
  gpio_pull_up(b);
}

// ===============================
//       MATRIX / HID HELPERS
// ===============================
static inline void cols_all_hiz_pd(void) {
  for (int c = 0; c < 3; ++c) {
    gpio_set_dir(COL_PINS[c], GPIO_IN);
    gpio_pull_down(COL_PINS[c]);
  }
}

static inline void col_drive_high(int c) {
  gpio_set_dir(COL_PINS[c], GPIO_OUT);
  gpio_put(COL_PINS[c], 1);
}

static inline void discharge_rows(void) {
  for (int r = 0; r < 3; ++r) {
    gpio_set_dir(ROW_PINS[r], GPIO_OUT);
    gpio_put(ROW_PINS[r], 0);
  }
  sleep_us(DISCHARGE_US);
  for (int r = 0; r < 3; ++r) {
    gpio_set_dir(ROW_PINS[r], GPIO_IN);
    gpio_pull_down(ROW_PINS[r]);
  }
}

static inline bool read_row_clean(int r) {
  (void)gpio_get(ROW_PINS[r]);
  sleep_us(SETTLE_US);
  bool a = gpio_get(ROW_PINS[r]);
  sleep_us(CONFIRM_US);
  bool b = gpio_get(ROW_PINS[r]);
  return a && b;
}

static inline void usb_sleep_ms(uint32_t ms) {
  for (uint32_t i = 0; i < ms; ++i) {
    tud_task();
    sleep_ms(1);
  }
}

// ===============================
//        ENCODER TAP QUEUE
// ===============================
#define TAP_QUEUE_SIZE 32
typedef struct {
  action_t buf[TAP_QUEUE_SIZE];
  uint8_t head, tail;
  bool active_this_frame;
  bool need_release;
} tap_queue_t;

static tap_queue_t tapq = {0};

static inline bool tapq_is_empty(void) { return tapq.head == tapq.tail; }
static inline bool tapq_is_full(void) { return (uint8_t)(tapq.head + 1) == tapq.tail; }

static inline void tapq_push(action_t a) {
  if (!tapq_is_full()) {
    tapq.buf[tapq.head] = a;
    tapq.head++;
  }
}
static inline bool tapq_pop(action_t *a) {
  if (tapq_is_empty()) return false;
  *a = tapq.buf[tapq.tail];
  tapq.tail++;
  return true;
}

static void process_encoder_to_taps(void) {
  static int32_t acc0 = 0, acc1 = 0;
  int32_t c0 = encoder0_count;
  encoder0_count = 0;
  int32_t c1 = encoder1_count;
  encoder1_count = 0;
  acc0 += c0;
  acc1 += c1;
  while (acc0 >= STEPS_PER_DETENT) {
    tapq_push(g_enc[0].cw);
    acc0 -= STEPS_PER_DETENT;
  }
  while (acc0 <= -STEPS_PER_DETENT) {
    tapq_push(g_enc[0].ccw);
    acc0 += STEPS_PER_DETENT;
  }
  while (acc1 >= STEPS_PER_DETENT) {
    tapq_push(g_enc[1].cw);
    acc1 -= STEPS_PER_DETENT;
  }
  while (acc1 <= -STEPS_PER_DETENT) {
    tapq_push(g_enc[1].ccw);
    acc1 += STEPS_PER_DETENT;
  }
}

// ===============================
//   FLASH-BACKED CONFIG (1 page)
// ===============================
#define FLASH_SECTOR_SIZE (1u << 12)
#define FLASH_PAGE_SIZE (1u << 8)
#define CONFIG_MAGIC 0x4B4D4150u  // 'KMAP'
#define CONFIG_VERSION 2          // v2

#ifndef PICO_FLASH_SIZE_BYTES
#warning "PICO_FLASH_SIZE_BYTES not defined by board; using default 2MB"
#define PICO_FLASH_SIZE_BYTES (2 * 1024 * 1024)
#endif
#define CONFIG_FLASH_OFFSET (PICO_FLASH_SIZE_BYTES - FLASH_SECTOR_SIZE)

// v1 layout (no modifiers)
typedef struct __attribute__((packed)) {
  uint32_t magic;
  uint8_t version;
  uint8_t keymap[3][3];
  struct {
    uint8_t cw, ccw;
  } enc[2];
  uint8_t _pad[FLASH_PAGE_SIZE - (sizeof(uint32_t) + 1 + 3 * 3 + sizeof(uint8_t) * 2 * 2)];
} v1_config_t;

// new v2 layout
typedef struct __attribute__((packed)) {
  uint32_t magic;
  uint8_t version;
  action_t keymap[3][3];
  encoder_map_t enc[2];
  uint8_t _pad[FLASH_PAGE_SIZE - (sizeof(uint32_t) + 1 + sizeof(action_t) * 3 * 3 + sizeof(encoder_map_t) * 2)];
} v2_config_t;

static void load_config_defaults(void) {
  action_t def[3][3] = {
      {{0, 0x04}, {0, 0x05}, {0, 0x06}},
      {{0, 0x07}, {0, 0x08}, {0, 0x09}},
      {{0, 0x0A}, {0, 0x0B}, {0, 0x0C}},
  };
  memcpy(g_keymap, def, sizeof(g_keymap));
  g_enc[0].cw = (action_t){0, 0x4F};
  g_enc[0].ccw = (action_t){0, 0x50};
  g_enc[1].cw = (action_t){0, 0x52};
  g_enc[1].ccw = (action_t){0, 0x51};
}

static bool load_config_from_flash(void) {
  const uint8_t *raw = (const uint8_t *)(XIP_BASE + CONFIG_FLASH_OFFSET);
  const uint32_t magic = *(const uint32_t *)raw;
  if (magic != CONFIG_MAGIC) return false;

  uint8_t version = *(const uint8_t *)(raw + sizeof(uint32_t));
  if (version == 1) {
    const v1_config_t *cfg = (const v1_config_t *)raw;
    for (int r = 0; r < 3; r++)
      for (int c = 0; c < 3; c++) g_keymap[r][c] = (action_t){0, cfg->keymap[r][c]};
    g_enc[0].cw = (action_t){0, cfg->enc[0].cw};
    g_enc[0].ccw = (action_t){0, cfg->enc[0].ccw};
    g_enc[1].cw = (action_t){0, cfg->enc[1].cw};
    g_enc[1].ccw = (action_t){0, cfg->enc[1].ccw};
    return true;
  } else if (version == CONFIG_VERSION) {
    const v2_config_t *cfg = (const v2_config_t *)raw;
    memcpy(g_keymap, cfg->keymap, sizeof(g_keymap));
    memcpy(g_enc, cfg->enc, sizeof(g_enc));
    return true;
  }
  return false;
}

static bool save_config_to_flash(void) {
  v2_config_t page;
  memset(&page, 0xFF, sizeof(page));
  page.magic = CONFIG_MAGIC;
  page.version = CONFIG_VERSION;
  memcpy(page.keymap, g_keymap, sizeof(g_keymap));
  memcpy(page.enc, g_enc, sizeof(g_enc));

  uint32_t ints = save_and_disable_interrupts();
  flash_range_erase(CONFIG_FLASH_OFFSET, FLASH_SECTOR_SIZE);
  flash_range_program(CONFIG_FLASH_OFFSET, (const uint8_t *)&page, FLASH_PAGE_SIZE);
  restore_interrupts(ints);

  const v2_config_t *cfg = (const v2_config_t *)(XIP_BASE + CONFIG_FLASH_OFFSET);
  return (cfg->magic == CONFIG_MAGIC && cfg->version == CONFIG_VERSION);
}

// ===============================
//  CLI helpers
// ===============================
static char *trim(char *s) {
  while (*s == ' ' || *s == '\t') s++;
  char *e = s + strlen(s);
  while (e > s && (e[-1] == ' ' || e[-1] == '\t')) --e;
  *e = 0;
  return s;
}
static int icmp(const char *a, const char *b) {
  while (*a && *b) {
    char ca = *a, cb = *b;
    if (ca >= 'A' && ca <= 'Z') ca += 'a' - 'A';
    if (cb >= 'A' && cb <= 'Z') cb += 'a' - 'A';
    if (ca != cb) return (uint8_t)ca - (uint8_t)cb;
    ++a;
    ++b;
  }
  return (uint8_t)*a - (uint8_t)*b;
}

// parse macro: "ctrl+shift+s" or "win+shift+s" or "0xMM:0xKK" or "0xKK"
static bool parse_action(const char *in, action_t *out) {
  char buf[64];
  strncpy(buf, in, sizeof(buf) - 1);
  buf[sizeof(buf) - 1] = 0;
  char *s = trim(buf);
  // hex forms first
  unsigned m, k;
  if (sscanf(s, "0x%x:0x%x", &m, &k) == 2 && m <= 0xFF && k <= 0xFF) {
    out->mods = m;
    out->key = k;
    return true;
  }
  if (sscanf(s, "0x%x", &k) == 1 && k <= 0xFF) {
    out->mods = 0;
    out->key = k;
    return true;
  }

  // normalize lowercase
  for (char *p = s; *p; ++p) {
    if (*p >= 'A' && *p <= 'Z') *p += 'a' - 'A';
  }

  uint8_t mods = 0;
  const char *last = NULL;
  // split by '+'
  char *save = 0;
  char *tok = strtok_r(s, "+", &save);
  while (tok) {
    if (!strcmp(tok, "ctrl") || !strcmp(tok, "control"))
      mods |= 0x01;
    else if (!strcmp(tok, "shift"))
      mods |= 0x02;
    else if (!strcmp(tok, "alt") || !strcmp(tok, "option"))
      mods |= 0x04;
    else if (!strcmp(tok, "gui") || !strcmp(tok, "win") || !strcmp(tok, "windows") || !strcmp(tok, "meta") || !strcmp(tok, "cmd") || !strcmp(tok, "command"))
      mods |= 0x08;
    else
      last = tok;
    tok = strtok_r(NULL, "+", &save);
  }
  if (!last) return false;

  // key mapping
  uint8_t key = 0;
  if (strlen(last) == 1) {
    char ch = last[0];
    if (ch >= 'a' && ch <= 'z') {
      key = 0x04 + (ch - 'a');
      goto ok;
    }
    if (ch >= '1' && ch <= '9') {
      key = 0x1E + (ch - '1');
      goto ok;
    }
    if (ch == '0') {
      key = 0x27;
      goto ok;
    }
  }
  if (!strcmp(last, "enter")) {
    key = 0x28;
    goto ok;
  }
  if (!strcmp(last, "esc") || !strcmp(last, "escape")) {
    key = 0x29;
    goto ok;
  }
  if (!strcmp(last, "tab")) {
    key = 0x2B;
    goto ok;
  }
  if (!strcmp(last, "space")) {
    key = 0x2C;
    goto ok;
  }
  if (!strcmp(last, "bksp") || !strcmp(last, "backspace")) {
    key = 0x2A;
    goto ok;
  }
  if (!strcmp(last, "left")) {
    key = 0x50;
    goto ok;
  }
  if (!strcmp(last, "right")) {
    key = 0x4F;
    goto ok;
  }
  if (!strcmp(last, "up")) {
    key = 0x52;
    goto ok;
  }
  if (!strcmp(last, "down")) {
    key = 0x51;
    goto ok;
  }
  if (last[0] == 'f') {
    int n = atoi(last + 1);
    if (1 <= n && n <= 12) {
      key = 0x3A + (n - 1);
      goto ok;
    }
  }
  return false;
ok:
  out->mods = mods;
  out->key = key;
  return true;
}

/*

aaabbbcccbbcccbbcccbbcccbbccc

*/

// ===============================
//  CDC text interface
// ===============================
static char cdc_buf[128];/*  */
static int cdc_len = 0;

static void cdc_print_keymap(void) {
  char out[256];
  int n = 0;
  //n += snprintf(out + n, sizeof(out) - n, "\nKeymap (mods:key)\r\n");
  for (int r = 0; r < 3; r++) {
    n += snprintf(out + n, sizeof(out) - n, "[r%d]", r);
    for (int c = 0; c < 3; c++) n += snprintf(out + n, sizeof(out) - n, " %02X:%02X", g_keymap[r][c].mods, g_keymap[r][c].key);
    n += snprintf(out + n, sizeof(out) - n, " ");
  }
  n += snprintf(out + n, sizeof(out) - n, "[enc0] %02X:%02X %02X:%02X ", g_enc[0].cw.mods, g_enc[0].cw.key, g_enc[0].ccw.mods, g_enc[0].ccw.key);
  n += snprintf(out + n, sizeof(out) - n, "[enc1] %02X:%02X %02X:%02X", g_enc[1].cw.mods, g_enc[1].cw.key, g_enc[1].ccw.mods, g_enc[1].ccw.key);
  tud_cdc_write(out, (uint32_t)n);
  tud_cdc_write_flush();
  for (int i = 0; i < 3; i++) tud_task();
}

static void cdc_show_help(void) {
  cdc_write("\nCommands:\r\n");
  cdc_write("  show\r\n");
  cdc_write("  set key <row 0..2> <col 0..2> <macro>\r\n");
  cdc_write("  set enc <0|1> <cw|ccw> <macro>\r\n");
  cdc_write("  save\r\n");
  cdc_write("  resetdefaults\r\n");
  cdc_write("Macro: 'ctrl+c', 'win+shift+s', 'alt+f4', or hex '0xMM:0xKK' / '0xKK'.\r\n");
}

static void cdc_handle_line(const char *line_in) {
  char line[128];
  strncpy(line, line_in, sizeof(line) - 1);
  line[sizeof(line) - 1] = 0;
  char *s = trim(line);
  if (*s == 0) return;
  if (icmp(s, "help") == 0) {
    cdc_show_help();
    return;
  }
  if (icmp(s, "show") == 0) {
    cdc_print_keymap();
    return;
  }

  int r, c, idx;
  char dir[8];
  char macro[64];
  if (sscanf(s, "set key %d %d %63[ -~]", &r, &c, macro) == 3) {
    if (r >= 0 && r < 3 && c >= 0 && c < 3) {
      action_t a;
      if (parse_action(trim(macro), &a)) {
        g_keymap[r][c] = a;
        cdc_printf("\nOK r%d c%d = %02X:%02X\r\n", r, c, a.mods, a.key);
      } else
        cdc_write("\nERR invalid macro\r\n");
    } else
      cdc_write("\nERR invalid coords\r\n");
    return;
  }
  if (sscanf(s, "set enc %d %7s %63[ -~]", &idx, dir, macro) == 3) {
    if (!(idx == 0 || idx == 1)) {
      cdc_write("\nERR invalid encoder index\r\n");
      return;
    }
    for (char *p = dir; *p; ++p)
      if (*p >= 'A' && *p <= 'Z') *p += 'a' - 'A';
    action_t a;
    if (!parse_action(trim(macro), &a)) {
      cdc_write("\nERR invalid macro\r\n");
      return;
    }
    if (!strcmp(dir, "cw"))
      g_enc[idx].cw = a;
    else if (!strcmp(dir, "ccw"))
      g_enc[idx].ccw = a;
    else {
      cdc_write("\nERR direction (cw|ccw)\r\n");
      return;
    }
    cdc_printf("\nOK enc%d %s = %02X:%02X\r\n", idx, dir, a.mods, a.key);
    return;
  }

  if (icmp(s, "save") == 0) {
    cdc_write(save_config_to_flash() ? "\nOK saved\r\n" : "\nERR save failed\r\n");
    return;
  }
  if (icmp(s, "resetdefaults") == 0) {
    load_config_defaults();
    cdc_write("\nOK defaults restored (use 'save' to persist)\r\n");
    return;
  }

  cdc_printf("\nERR unknown: \"%s\"\r\n", s);
}

static void cdc_task(void) {
  while (tud_cdc_available()) {
    char ch;
    tud_cdc_read(&ch, 1);
    if (ch == 0x08 || ch == 0x7F) {
      if (cdc_len > 0) {
        cdc_len--;
        tud_cdc_write("\b \b", 3);
        tud_cdc_write_flush();
      }
      continue;
    }
    if ((uint8_t)ch >= 0x20 && ch != 0x7F) {
      tud_cdc_write_char(ch);
      tud_cdc_write_flush();
    }
    if (ch == '\r' || ch == '\n') {
      while (cdc_len > 0 && (cdc_buf[cdc_len - 1] == ' ' || cdc_buf[cdc_len - 1] == '\t')) cdc_len--;
      if (cdc_len > 0) {
        cdc_buf[cdc_len] = 0;
        cdc_handle_line(cdc_buf);
        cdc_len = 0;
      }
      tud_cdc_write("\r\n> ", 4);
      tud_cdc_write_flush();
      continue;
    }
    if ((uint8_t)ch >= 0x20 && cdc_len < (int)sizeof(cdc_buf) - 1) cdc_buf[cdc_len++] = ch;
  }
}

void tud_cdc_line_state_cb(uint8_t itf, bool dtr, bool rts) {
  (void)itf;
  (void)rts;
  if (dtr) {
    put_rgb((uint8_t)g_r, (uint8_t)g_g, (uint8_t)g_b, (uint8_t)g_brightness);
    cdc_write("\r\nMLDH MacroPad ready. Type 'help'.\r\n");
    cdc_prompt();
  } else {
    put_rgb(0, 0, 0, 0);
  }
}

// ===============================
//             MAIN
// ===============================
int main(void) {
  stdio_init_all();
  sleep_ms(300);
  tusb_init();

  anim_start_core1();
  //ws2812_start_core1();
  PIO pio = pio0;
  int sm = 0;
  uint offset = pio_add_program(pio, &ws2812_program);

  ws2812_program_init(pio, sm, offset, 16, 800000, true);

  //put_rgb((uint8_t)g_r, (uint8_t)g_g, (uint8_t)g_b, (uint8_t)g_brightness);

  if (!load_config_from_flash()) {
    load_config_defaults();
    printf("Config: defaults\r\n");
  } else {
    printf("Config: loaded from flash\r\n");
  }

  for (int c = 0; c < 3; ++c) {
    gpio_init(COL_PINS[c]);
  }
  cols_all_hiz_pd();
  for (int r = 0; r < 3; ++r) {
    gpio_init(ROW_PINS[r]);
    gpio_set_dir(ROW_PINS[r], GPIO_IN);
    gpio_pull_down(ROW_PINS[r]);
  }

  init_encoder_gpio(ENC0_A_GPIO, ENC0_B_GPIO);
  init_encoder_gpio(ENC1_A_GPIO, ENC1_B_GPIO);
  ab0_state = read_ab(ENC0_A_GPIO, ENC0_B_GPIO);
  ab1_state = read_ab(ENC1_A_GPIO, ENC1_B_GPIO);
  gpio_set_irq_enabled_with_callback(ENC0_A_GPIO, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true, &gpio_isr);
  gpio_set_irq_enabled(ENC0_B_GPIO, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true);
  gpio_set_irq_enabled(ENC1_A_GPIO, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true);
  gpio_set_irq_enabled(ENC1_B_GPIO, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true);

  bool is_down[3][3] = {0};
  int pressed_count[3][3] = {0};
  uint8_t last_keys[6] = {0};
  bool last_nonempty = false;
  uint8_t last_mods = 0;

  while (true) {
    tud_task();
    cdc_task();

    // scan matrix
    bool raw[3][3] = {0};
    for (int c = 0; c < 3; ++c) {
      cols_all_hiz_pd();
      discharge_rows();
      col_drive_high(c);
      for (int r = 0; r < 3; ++r) raw[r][c] = read_row_clean(r);
    }

    bool any_pressed_now = false;
    bool row_still_held[3] = {0, 0, 0};
    for (int r = 0; r < 3; ++r)
      for (int c = 0; c < 3; ++c) {
        bool pressed_exclusive = false;
        if (raw[r][c]) {
          pressed_exclusive = true;
          for (int c2 = 0; c2 < 3; ++c2)
            if (c2 != c && raw[r][c2]) {
              pressed_exclusive = false;
              break;
            }
        }
        if (pressed_exclusive) {
          any_pressed_now = true;
          row_still_held[r] = true;
          if (!is_down[r][c]) {
            is_down[r][c] = true;
            pressed_count[r][c] = 1;
            gpio_set_input_enabled(ROW_PINS[r], false);
            sleep_us(5);
            gpio_set_input_enabled(ROW_PINS[r], true);
          } else
            pressed_count[r][c]++;
        } else {
          if (is_down[r][c]) {
            is_down[r][c] = false;
            pressed_count[r][c] = 0;
          }
        }
      }

    process_encoder_to_taps();

    // Build HID report: collect modifiers + up to 6 keys
    uint8_t report_keys[6] = {0};
    uint8_t n = 0;
    uint8_t mods = 0;

    for (int r = 0; r < 3 && n < 6; ++r)
      for (int c = 0; c < 3 && n < 6; ++c)
        if (is_down[r][c]) {
          action_t a = g_keymap[r][c];
          mods |= a.mods;
          if (a.key) report_keys[n++] = a.key;
        }

    tapq.active_this_frame = false;
    if (n < 6) {
      action_t a;
      if (tapq_pop(&a)) {
        mods |= a.mods;
        if (a.key) report_keys[n++] = a.key;
        tapq.active_this_frame = true;
        tapq.need_release = true;
      }
    }

    bool changed = (mods != last_mods) || ((n == 0) != (!last_nonempty));
    if (!changed && n > 0)
      for (int i = 0; i < 6; i++)
        if (report_keys[i] != last_keys[i]) {
          changed = true;
          break;
        }

    if (tud_mounted() && tud_hid_ready() && changed) {
      tud_hid_keyboard_report(0 /*report_id*/, mods, n ? report_keys : NULL);
      for (int i = 0; i < 6; i++) last_keys[i] = report_keys[i];
      last_nonempty = (n > 0);
      last_mods = mods;
    }

    if (tapq.need_release && tud_mounted() && tud_hid_ready()) {
      usb_sleep_ms(1);
      tud_hid_keyboard_report(0, 0, NULL);
      for (int i = 0; i < 6; i++) last_keys[i] = 0;
      last_nonempty = false;
      last_mods = 0;
      tapq.need_release = false;
    }

    if (tud_suspended()) {
      if (n > 0 || !tapq_is_empty()) tud_remote_wakeup();
    }

    if (any_pressed_now) {
      usb_sleep_ms(REPEAT_DELAY_MS);
      for (int r = 0; r < 3; ++r)
        if (row_still_held[r]) {
          gpio_set_input_enabled(ROW_PINS[r], false);
          sleep_us(10);
          gpio_set_input_enabled(ROW_PINS[r], true);
        }
    }

    usb_sleep_ms(SCAN_SLEEP_MS);
  }
  return 0;
}

/* ---------------- TinyUSB HID callbacks ---------------- */
uint16_t tud_hid_get_report_cb(uint8_t itf, uint8_t report_id, hid_report_type_t report_type, uint8_t *buffer, uint16_t reqlen) {
  (void)itf;
  (void)report_id;
  (void)report_type;
  (void)buffer;
  (void)reqlen;
  return 0;
}

void tud_hid_set_report_cb(uint8_t itf, uint8_t report_id, hid_report_type_t report_type, const uint8_t *buffer, uint16_t bufsize) {
  (void)itf;
  (void)report_id;
  (void)report_type;
  (void)buffer;
  (void)bufsize;
}
