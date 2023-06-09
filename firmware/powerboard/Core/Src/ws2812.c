/*
 * ws2812.c
 *
 *  Created on: 28 juin 2021
 *      Author: tflayols
 */
#include "stm32f1xx_hal.h"
#include "ws2812.h"

#define PWM_0_LOGIC 30 	// PWM Value corresponding to 0 logic for WS2812
#define PWM_1_LOGIC 60 	// PWM Value corresponding to 1 logic for WS2812
#define DATA_SIZE 74 	  // 24 (1 ws2812 led) + 50 (reset code)

uint32_t pwmData[DATA_SIZE];
extern TIM_HandleTypeDef htim2;
uint8_t dataSentFlag = 0;

uint32_t rgb_to_uint32(uint8_t r, uint8_t g, uint8_t b) {
	uint32_t color=0;
	color = ((uint32_t)g << 16) | ((uint32_t)r << 8) | ((uint32_t)b);
	return color;
}

void set_rgb_led_color(uint8_t r, uint8_t g, uint8_t b) {
	uint32_t color = rgb_to_uint32(r,g,b);
	for (int i=0; i<50; i++)  // send reset code
			pwmData[i] = 0;
	for (int i=23; i>=0; i--) // send G,R,B color data MSB First
		pwmData[50+i] = (color&(1<<i)) ? PWM_1_LOGIC : PWM_0_LOGIC;

	HAL_TIM_PWM_Start_DMA(&htim2, TIM_CHANNEL_2, (uint32_t *)pwmData, DATA_SIZE);
	while (!dataSentFlag);
	dataSentFlag = 0;
}

void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim)
{
	HAL_TIM_PWM_Stop_DMA(htim, TIM_CHANNEL_2);
	dataSentFlag=1;
}