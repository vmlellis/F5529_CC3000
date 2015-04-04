/*****************************************************************************
 * config.h
 *
 * Arquivo para a configuração
 *
 *  Criado em: 28/02/2015
 *      Autor: Victor
 *
 *  Construido com a IDE CCSv6 e compilador GNU v4.9.1 (Red Hat)
 *****************************************************************************/

#ifndef CONFIG_H_
#define CONFIG_H_

#include <msp430.h>
#include <inttypes.h>

#define F_CPU 25000000L   	// Frequencia da CPU: 25 MHz
#define BAUD_RATE 115200L	// Baud Rate do UART: 115200
#define I2C_FREQ 400000L  	// Frequencia do I2C: 400 kHz
#define SPI_CLOCK_SPEED 4000000L  // Frequencia do SPI: 4MHz

#define SPI_CLOCK_DIV() ((F_CPU / SPI_CLOCK_SPEED) + (F_CPU % SPI_CLOCK_SPEED == 0 ? 0:1))

//#define SPI_CLK 	BIT2 // P3.2
//#define SPI_DIN 	BIT0 // P3.0
//#define SPI_DOUT 	BIT1 // P3.1

#define WLAN_CS_PIN        BIT2  // P2.2
#define WLAN_EN_PIN        BIT5	 // P6.5
#define WLAN_IRQ_PIN       BIT0  // P2.0


// Ports
#define WLAN_CS_SEL       	P2SEL
#define WLAN_CS_OUT       	P2OUT
#define WLAN_CS_DIR       	P2DIR

#define WLAN_EN_DIR      	P6DIR
#define WLAN_EN_OUT       	P6OUT

#define WLAN_IRQ_DIR    	P2DIR
#define WLAN_IRQ_IN        	P2IN
#define WLAN_IRQ_IES       	P2IES
#define WLAN_IRQ_IE        	P2IE
#define WLAN_IFG_PORT      	P2IFG

#define BMP085_OSS 3	// Ultra High Resolution (Delay de ~ 26ms na leitura)

/*
 * Definições para controle do clock
 */
#define clockCyclesPerMicrosecond() ( F_CPU / 1000000L )
#define clockCyclesToMicroseconds(a) ( (a) / clockCyclesPerMicrosecond() )
#define microsecondsToClockCycles(a) ( (a) * clockCyclesPerMicrosecond() )

/*
 * Definições para o WatchDog
 */
#define TICKS_PER_WDT_OVERFLOW 8192

// the whole number of microseconds per WDT overflow
#define MICROSECONDS_PER_WDT_OVERFLOW (clockCyclesToMicroseconds(TICKS_PER_WDT_OVERFLOW))

// o número em milissegundos para o estouro do WDT
#define MILLIS_INC (MICROSECONDS_PER_WDT_OVERFLOW / 1000)

// numero fracional de milisegundos para o overflow de WDT
#define FRACT_INC (MICROSECONDS_PER_WDT_OVERFLOW % 1000)
#define FRACT_MAX 1000

void disableWatchDog(void);
void enableWatchDog(void);
void saveUsbPower(void);
void initClocks(void);
void setupUart(void);
void setupI2C(void);
void delay(uint32_t);
void delayMicroseconds(uint32_t);
uint8_t read_bits(uint8_t, uint8_t, uint8_t);
uint8_t write_bits(uint8_t, uint8_t, uint8_t);
unsigned long millis();

#endif /* CONFIG_H_ */
