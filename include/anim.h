
#pragma once

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
  ANIM_MODE_SOLAR = 0,
  ANIM_MODE_EQ    = 1,
  ANIM_MODE_MAZE  = 2
} anim_mode_t;

// Configure panel/pins/address/baud (call before anim_start_core1)
void anim_config(int width_px, int height_px,
                 void* i2c_inst ,
                 int sda_pin, int scl_pin,
                 int i2c_address /*0x3C*/, int i2c_baud /*400000*/);

void anim_start_core1(void);

void anim_next_mode(void);
void anim_set_mode(anim_mode_t m);
void anim_pause(int paused /*0 or 1*/);
void anim_set_speed(int frame_ms /*default 16*/);
void anim_set_brightness(uint8_t level);

#ifdef __cplusplus
} // extern "C"
#endif
