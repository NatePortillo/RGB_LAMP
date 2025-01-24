/*
 * sk6812.h
 *
 *  Created on: Dec 16, 2024
 *      Author: natha
 */

#ifndef INC_SK6812_H_
#define INC_SK6812_H_

#include <stdint.h>

typedef union
{
  struct
  {
    uint8_t b;
    uint8_t r;
    uint8_t g;
  } color;
  uint32_t data;
} PixelRGB_t;

void SEND_DATA(PixelRGB_t *colors,
               uint32_t *dma_buffer,    // Pointer to DMA buffer
               uint8_t num_of_leds,     // Number of LEDs
               uint32_t high_time,      // PWM value for bit 1
               uint32_t low_time);
void LEDS_OFF(PixelRGB_t *colors, uint8_t num_of_leds);
void RGB_COLORS(PixelRGB_t *colors, uint8_t num_of_leds, uint8_t red_hue, uint8_t blue_hue, uint8_t green_hue);

#endif /* INC_SK6812_H_ */
