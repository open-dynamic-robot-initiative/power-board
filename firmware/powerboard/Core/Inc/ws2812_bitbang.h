/*
 * ws2812_bitbang.h
 *
 *  Created on: 28 juin 2021
 *      Author: tflayols
 */

#ifndef INC_WS2812_BITBANG_H_
#define INC_WS2812_BITBANG_H_

#include <stdint.h>


void bitbangledRGB(uint8_t r, uint8_t g, uint8_t b);
void bitbangled(uint32_t color);


#endif /* INC_WS2812_BITBANG_H_ */
