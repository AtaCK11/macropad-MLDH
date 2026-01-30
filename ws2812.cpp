#include <stdio.h>
#include <stdlib.h>

#include "pico/stdlib.h"
#include "hardware/pio.h"
#include "hardware/clocks.h"
#include "ws2812.pio.h"

#include "ws2812.h"
#include <pico/multicore.h>

static volatile uint8_t g_r = 255;
static volatile uint8_t g_g = 0;
static volatile uint8_t g_b = 0;
static volatile uint8_t g_brightness = 5;

void put_pixel(uint32_t pixel_grb)
{
    pio_sm_put_blocking(pio0, 0, pixel_grb << 8u);
}

static inline uint8_t scale8(uint8_t v, uint8_t scale) {
    return (uint16_t(v) * uint16_t(scale) + 127) >> 8;
}

void put_rgb(uint8_t red, uint8_t green, uint8_t blue, uint8_t brightness)
{
    red   = scale8(red,   brightness);
    green = scale8(green, brightness);
    blue  = scale8(blue,  brightness);

    uint32_t grb = ((uint32_t)green << 16) | ((uint32_t)red << 8) | (uint32_t)blue;
    put_pixel(grb);
}

extern "C" {
    
void ws_set_rgb(uint8_t r, uint8_t g, uint8_t b) {
    g_r = r; g_g = g; g_b = b;
}

void ws_set_brightness(uint8_t b) {
    g_brightness = b;
}

int core1_ws2812_loop(void)
{
    PIO pio = pio0;
    int sm = 0;
    uint offset = pio_add_program(pio, &ws2812_program);

    ws2812_program_init(pio, sm, offset, 16, 800000, true);

    while (1)
    {
        put_rgb((uint8_t)g_r, (uint8_t)g_g, (uint8_t)g_b, (uint8_t)g_brightness);
    }
    // never returns
    // return 0;
}

void ws2812_start_core1(void){
  static bool started = false;
  if(started) return;
  started = true;
  multicore_launch_core1(ws2812_loop_trampoline);
}

void ws2812_loop_trampoline(){
  core1_ws2812_loop();
}

}