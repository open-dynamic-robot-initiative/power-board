/*
 * ina239.h
 *
 *  Created on: Jun 30, 2021
 *      Author: tflayols
 */

#ifndef INA239_H_
#define INA239_H_

#include <stdint.h>
#include "stm32f1xx_hal.h"

#define REG_INA_CONFIG          0x00  //Configuration 16 bits
#define REG_INA_ADC_CONFIG      0x01  //ADC Configuration 16 bits
#define REG_INA_SHUNT_CAL       0x02  //Shunt Calibration 16  bits
#define REG_INA_VSHUNT          0x04  //Shunt Voltage Measurement 16  bits
#define REG_INA_VBUS            0x05  //Bus Voltage Measurement 16  bits
#define REG_INA_DIETEMP         0x06  //Temperature Measurement 16  bits
#define REG_INA_CURRENT         0x07  //Current Result 16  bits
#define REG_INA_POWER           0x08  //Power Result 24 bits
#define REG_INA_DIAG_ALRT       0x0B  //Diagnostic Flags and Alert 16 bits
#define REG_INA_SOVL            0x0C  //Shunt Overvoltage Threshold 16 bits
#define REG_INA_SUVL            0x0D  //Shunt Undervoltage Threshold 16 bits
#define REG_INA_BOVL            0x0E  //Bus Overvoltage Threshold 16 bits
#define REG_INA_BUVL            0x0F  //Bus Undervoltage Threshold 16 bits
#define REG_INA_TEMP_LIMIT      0x10  //Temperature Over-Limit Threshold 16 bits
#define REG_INA_PWR_LIMIT       0x11  //Power Over-Limit Threshold 16 bits
#define REG_INA_MANUFACTURER_ID 0x3E  //Manufacturer ID 16 bits
#define REG_INA_DEVICE_ID       0x3F  //Device ID 16 bits

#define INA_READ 1
#define INA_WRITE 0

void setup_INA239(SPI_HandleTypeDef spi_Handle);
void write_register16_INA239(SPI_HandleTypeDef spi_Handle, uint8_t reg, uint16_t value);
uint16_t read_register16_INA239(SPI_HandleTypeDef spi_Handle, uint8_t reg);
uint32_t read_register24_INA239(SPI_HandleTypeDef spi_Handle, uint8_t reg);
void read_INA239(SPI_HandleTypeDef spi_Handle);

#endif /* INA239_H_ */
