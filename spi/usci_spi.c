/*
 * usci_spi.c
 *
 *  Created on: 21/03/2015
 *      Author: Victor
 */

#include "usci_spi.h"
#include "../setup.h"

/*
 * Configura USCI UCB0 para o modo SPI
 * P3.2 - SCLK
 * P3.0 - MISO
 * P3.1 - MOSI
 */
void usci_spi_initialize(void)
{
	UCB0CTL1 = UCSWRST | UCSSEL_2;      // Put USCI in reset mode, source USCI clock from SMCLK
	UCB0CTL0 = SPI_MODE_0 | UCMSB | UCSYNC | UCMST;  // Use SPI MODE 0 - CPOL=0 CPHA=0

	SPI_SEL |= (SPI_CLK + SPI_MISO + SPI_MOSI);
	/*SPI_DIR |= SPI_CLK + SPI_MOSI;
	SPI_REN |= SPI_MISO;
	SPI_OUT |= SPI_MISO;*/

	UCB0BR0 = SPI_CLOCK_DIV() & 0xFF;   // set initial speed to 4MHz
	UCB0BR1 = (SPI_CLOCK_DIV() >> 8 ) & 0xFF;

	UCB0CTL1 &= ~UCSWRST;			    // release USCI for operation
}

/**
 * usci_spi_send() - send a byte and recv response
 */
uint8_t usci_spi_send(const uint8_t _data)
{
	UCB0TXBUF = _data; // setting TXBUF clears the TXIFG flag
	while (UCB0STAT & UCBUSY); // wait for SPI TX/RX to finish

	return UCB0RXBUF; // reading clears RXIFG flag
}


/***SPI_MODE_0
 * usci_spi_set_divisor() - set new clock divider for USCI
 *
 * USCI speed is based on the SMCLK divided by BR0 and BR1
 *
 */
void usci_spi_set_divisor(const uint16_t clkdiv)
{
	UCB0CTL1 |= UCSWRST;		// go into reset state
	UCB0BR0 = clkdiv & 0xFF;
	UCB0BR1 = (clkdiv >> 8 ) & 0xFF;
	UCB0CTL1 &= ~UCSWRST;		// release for operation
}

/**
 * spi_set_bitorder(LSBFIRST=0 | MSBFIRST=1)
 */
void usci_spi_set_bitorder(const uint8_t order)
{
    UCB0CTL1 |= UCSWRST;        // go into reset state
    UCB0CTL0 = (UCB0CTL0 & ~UCMSB) | ((order == 1 /*MSBFIRST*/) ? UCMSB : 0); /* MSBFIRST = 1 */
    UCB0CTL1 &= ~UCSWRST;       // release for operation
}

/**
 * spi_set_datamode() - mode 0 - 3
 */
void usci_spi_set_datamode(const uint8_t mode)
{
    UCB0CTL1 |= UCSWRST;        // go into reset state
    switch(mode) {
    case 0: /* SPI_MODE0 */
        UCB0CTL0 = (UCB0CTL0 & ~SPI_MODE_MASK) | SPI_MODE_0;
        break;
    case 1: /* SPI_MODE1 */
        UCB0CTL0 = (UCB0CTL0 & ~SPI_MODE_MASK) | SPI_MODE_1;
        break;
    case 2: /* SPI_MODE2 */
        UCB0CTL0 = (UCB0CTL0 & ~SPI_MODE_MASK) | SPI_MODE_2;
        break;
    case 4: /* SPI_MODE3 */
        UCB0CTL0 = (UCB0CTL0 & ~SPI_MODE_MASK) | SPI_MODE_3;
        break;
    default:
        break;
    }
    UCB0CTL1 &= ~UCSWRST;       // release for operation
}
