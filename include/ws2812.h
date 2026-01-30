#pragma once
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

void put_pixel(uint32_t pixel_grb);
void put_rgb(uint8_t red, uint8_t green, uint8_t blue, uint8_t brightness);

int core1_ws2812_loop(void);
void ws2812_start_core1(void);
void ws2812_loop_trampoline(void);

void ws_set_rgb(uint8_t r, uint8_t g, uint8_t b);
void ws_set_brightness(uint8_t b);

#ifdef __cplusplus
} // extern "C"
#endif
