#ifndef __WS2812_H__
#define __WS2812_H__

#include "main.h"
#include "stdio.h"
#include "string.h"

#define CODE_ONE_DUTY 140
#define CODE_ZERO_DUTY 70
// #define CODE_ONE_DUTY	90
// #define CODE_ZERO_DUTY	32
#define RST_PERIOD_NUM 1000
#define WS2812_NUM 100

#define WS2812TIM htim3


extern uint32_t ws2812_color[WS2812_NUM];
extern TIM_HandleTypeDef WS2812TIM;



void ws2812_update(void);
void ws2812_gradient(uint8_t steps, uint16_t delay_ms);
void ws2812_set(uint8_t led_id, uint32_t color);
void ws2812_set_rgb(uint8_t led_id, uint8_t r, uint8_t g, uint8_t b);
void ws2812_set_all(uint32_t color);
void color_to_rgb(uint32_t color, uint8_t *r, uint8_t *g, uint8_t *b);
uint32_t rgb_to_color(uint8_t r, uint8_t g, uint8_t b);
uint32_t rainbow_color(float frequency, int phase, int center, int width);
void rainbow_effect(uint8_t steps, uint16_t delay_ms);

#endif