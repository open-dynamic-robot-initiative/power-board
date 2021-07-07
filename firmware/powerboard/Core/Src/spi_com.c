/*
 * spi_com.c
 *
 *  Created on: Jul 6, 2021
 *      Author: tflayols
 */

#include "spi_com.h"
uint16_t crc16_ccitt(uint8_t * data, int len)
{
    uint16_t crc = 0xFFFF;
    for (unsigned int i = 0; i < len; ++i) {
        uint16_t dbyte = data[i];
        crc ^= dbyte << 8;
        for (uint8_t j = 0; j < 8; ++j) {
            uint16_t mix = crc & 0x8000;
            crc = (crc << 1);
            if (mix)
                crc = crc ^ 0x1021;
        }
    }
    return crc;
}
