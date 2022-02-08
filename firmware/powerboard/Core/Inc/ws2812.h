/*
 * ws2812.h
 *
 *  Created on: 28 juin 2021
 *      Author: tflayols
 */

#ifndef INC_WS2812_H_
#define INC_WS2812_H_

#include <stdint.h>

uint32_t rgb_to_uint32(uint8_t r, uint8_t g, uint8_t b);
void set_rgb_led_color(uint8_t r, uint8_t g, uint8_t b);

#endif /* INC_WS2812_H_ */
