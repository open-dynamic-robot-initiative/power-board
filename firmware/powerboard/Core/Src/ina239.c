/*
 * ina239.c
 *
 *  Created on: Jun 30, 2021
 *      Author: tflayols
 */

#include "ina239.h"


void write_register16_INA239(SPI_HandleTypeDef spi_Handle, uint8_t reg, uint16_t value)
{/*
  digitalWrite(INA_CS_PIN,LOW);
  MySPI1.transfer(reg<<2 | INA_WRITE);
  MySPI1.transfer16(value);
  digitalWrite(INA_CS_PIN,HIGH);*/
}

uint16_t read_register16_INA239(SPI_HandleTypeDef spi_Handle, uint8_t reg)
{
  uint8_t spi_tx = (reg<<2 | INA_READ);
  uint8_t spi_rx[2]= {0};

  //HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET); //CS for SPI1
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET); //CS for SPI2
  HAL_SPI_Transmit(&spi_Handle, (uint8_t *)&spi_tx, 1, 100); //send address of the register to be read
  HAL_SPI_Receive(&spi_Handle, spi_rx, 2, 100); //read a 16 bits register
  //HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET); //CS for SPI1
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET); //CS for SPI2

  return(spi_rx[0]<<8 | spi_rx[1]);
}

uint32_t read_register24_INA239(SPI_HandleTypeDef spi_Handle, uint8_t reg)
{
  uint8_t spi_tx = (reg<<2 | INA_READ);
  uint8_t spi_rx[3]= {0};

  //HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET); //CS for SPI1
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET); //CS for SPI2
  HAL_SPI_Transmit(&spi_Handle, (uint8_t *)&spi_tx, 1, 100); //send address of the register to be read
  HAL_SPI_Receive(&spi_Handle, spi_rx, 3, 100); //read a 16 bits register
  //HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET); //CS for SPI1
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET); //CS for SPI2
  return(spi_rx[0]<<16 | spi_rx[1]<<8 | spi_rx[2]);
}



void read_INA239(SPI_HandleTypeDef spi_Handle)
{
	/*
  vbus = read_register16(REG_INA_VBUS);
  vshunt = read_register16(REG_INA_VSHUNT);
  */
}
