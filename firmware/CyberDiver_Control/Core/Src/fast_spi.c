/*
 * fast_spi.c
 *
 *  Created on: Aug 30, 2024
 *      Author: johnt
 */

#include "fast_spi.h"
extern inline void SPI_TxRx_Fast(uint8_t* tx, uint8_t* rx, uint8_t len, SPI_TypeDef* spi, GPIO_TypeDef* cs_port, uint16_t cs_pin);


