/*
 * fast_spi.h
 *
 *  Created on: Aug 30, 2024
 *      Author: johnt
 */

#ifndef INC_FAST_SPI_H_
#define INC_FAST_SPI_H_

#include "stm32h723xx.h"

/* this uses the SPI FIFO on the H7, so it only works for transactions 16 bytes or less */
/* (the FIFO size depends on the specific SPI channel too, so check the RM) */
/* On the H723, SPI1, 2, 3 have 16 byte FIFO and SPI4, 5, 6 have 8 bytes FIFO */
/* the peripheral is configured using Cube to have 8 bit data frames and 1 data frame FIFO threshold */
__attribute__((optimize("-Ofast"))) inline void SPI_TxRx_Fast(uint8_t* tx, uint8_t* rx, uint8_t len, SPI_TypeDef* spi, GPIO_TypeDef* cs_port, uint16_t cs_pin)
{
	cs_port->BSRR = (uint32_t)cs_pin << 16U;  /* CS low */


	/* If using both the HAL SPI functions and our own, we need the next 2 lines to put SPI in a known state */
	/* If only using our function, I think these could be omitted */
	spi->CR2 &= ~SPI_CR2_TSIZE;  /* set TSIZE 0 for endless transfer (we disable SPI after a known number of bytes rather than looking for EOT) */
	spi->CFG2 &= ~SPI_CFG2_COMM;  /* this puts the SPI in full-duplex mode (for some reason the HAL takes it out of this mode sometimes, even when it is configured in Cube as full duplex) */


	spi->CR1 |= SPI_CR1_SPE;  /* enable SPI */
	spi->CR1 |= SPI_CR1_CSTART;  /* start transmission */

	for (uint8_t i = 0; i < len; )  /* put all the TX data in the FIFO at once - this FIFO is 16 bytes max so this won't work for arbitrary amounts of data */
	{
		if(((spi->SR) & SPI_SR_TXP) == SPI_SR_TXP)  /* wait until TX FIFO has enough space for a data packet */
		   *(__IO uint8_t*)&(spi->TXDR) = tx[i++];  /* put data in TXDR */
	}

	for (uint8_t i = 0; i < len; )  /* read out everything from the RX buffer */
	{
		if(((spi->SR) & SPI_SR_RXP) == SPI_SR_RXP)  /* wait until RX FIFO contains a data packet */
			rx[i++] = *(__IO uint8_t*)&(spi->RXDR);  /* read the data from RXDR */
	}

	spi->CR1 &= ~SPI_CR1_SPE;  /* disable SPI */
	cs_port->BSRR = cs_pin;  /* CS high */
}

#endif /* INC_FAST_SPI_H_ */
