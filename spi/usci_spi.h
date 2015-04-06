/*
 * usci_spi.h
 *
 *  Created on: 21/03/2015
 *      Author: Victor
 */

#ifndef F5529___CC3000_WEBSERVER_SPI_USCI_SPI_H_
#define F5529___CC3000_WEBSERVER_SPI_USCI_SPI_H_

#include <msp430.h>
#include <inttypes.h>

#define LSBFIRST 0
#define MSBFIRST 1

#define SPI_CLOCK_DIV1   1
#define SPI_CLOCK_DIV2   2
#define SPI_CLOCK_DIV4   4
#define SPI_CLOCK_DIV8   8
#define SPI_CLOCK_DIV16  16
#define SPI_CLOCK_DIV32  32
#define SPI_CLOCK_DIV64  64
#define SPI_CLOCK_DIV128 128

/**
 * USCI flags for various the SPI MODEs
 *
 * Note: The msp430 UCCKPL tracks the CPOL value. However,
 * the UCCKPH flag is inverted when compared to the CPHA
 * value described in Motorola documentation.
 */

#define SPI_MODE0 0
#define SPI_MODE1 1
#define SPI_MODE2 2
#define SPI_MODE3 4

#define SPI_MODE_0 (UCCKPH)			    /* CPOL=0 CPHA=0 */
#define SPI_MODE_1 (0)                 	/* CPOL=0 CPHA=1 */
#define SPI_MODE_2 (UCCKPL | UCCKPH)    /* CPOL=1 CPHA=0 */
#define SPI_MODE_3 (UCCKPL)			    /* CPOL=1 CPHA=1 */

#define SPI_MODE_MASK (UCCKPL | UCCKPH)

void usci_spi_initialize(void);
uint8_t usci_spi_send(const uint8_t);
void usci_spi_set_bitorder(const uint8_t);
void usci_spi_set_datamode(const uint8_t);
void usci_spi_set_divisor(const uint16_t clkdivider);

#endif /* F5529___CC3000_WEBSERVER_SPI_USCI_SPI_H_ */
