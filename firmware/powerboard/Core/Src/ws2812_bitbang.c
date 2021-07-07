/*
 * ws2812_bitbang.c
 *
 *  Created on: 28 juin 2021
 *      Author: tflayols
 */
#include "stm32f1xx_hal.h"
#include "ws2812_bitbang.h"
//#define WS2812_PIN GPIOB, GPIO_PIN_3
/*
 HAL_GPIO_WritePin(WS2812_PIN, GPIO_PIN_SET)
#define RESETPIN HAL_GPIO_WritePin(WS2812_PIN, GPIO_PIN_RESET)
*/

#define SETPIN   GPIOB->BSRR = 1 << 3;
#define RESETPIN GPIOB->BSRR = 1 << (3+16);



void bitbangledRGB(uint8_t r, uint8_t g, uint8_t b)
{
  bitbangled( ((uint32_t)g<<16) |
              ((uint32_t)r<<8) |
              (uint32_t) b );
}

void bitbangled(uint32_t color)
{
  int i;
  uint32_t b;
  uint32_t colorbits;
    colorbits = color;
      colorbits = color;
      for(i=0;i<24;i++)
      {
    	SETPIN;
        b = (1<<23) & colorbits;
        colorbits = colorbits << 1;
        asm("nop");
        asm("nop");
        asm("nop");
        asm("nop");
        if(!b)
        {
        	RESETPIN;
        }
        asm("nop");
        asm("nop");
        asm("nop");
        asm("nop");
        asm("nop");
        asm("nop");
        asm("nop");
        asm("nop");
        asm("nop");
        asm("nop");
        asm("nop");
        asm("nop");
        asm("nop");
        asm("nop");
        asm("nop");
        asm("nop");
        asm("nop");
        asm("nop");
        asm("nop");
        asm("nop");
        asm("nop");
        asm("nop");
        asm("nop");
        asm("nop");
        asm("nop");
        asm("nop");
        asm("nop");
        asm("nop");
        asm("nop");
        asm("nop");
        asm("nop");
        asm("nop");
        asm("nop");
        RESETPIN;
        asm("nop");

     }
}
