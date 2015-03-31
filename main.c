/*
 * main.c
 *
 *  Construido com a IDE CCSv6 e compilador GNU v4.9.1 (Red Hat)
 */


#include <msp430f5529.h>
#include "setup.h"
#include "uart/uart_tx.h"

volatile int counter = 0;
volatile int counterRX = 0;
volatile int counterTX = 0;

void initPorts();

// Para sair do modo de baixo consumo no modo I2C
static volatile uint8_t FLAG_wakeUpI2C = 0;

/* Metodos externos para o I2C */
//extern void i2c_nack(void);
extern void i2c_rx(void);
extern void i2c_tx(void);
//extern void i2c_state_isr(void);
//extern void i2c_txrx_isr(void);

/* Metodo externo para o UART RX */
extern void uart_rx(void);

/* Variaveis WDT */
volatile unsigned long wdt_overflow_count = 0;
volatile unsigned long wdt_millis = 0;
volatile unsigned int wdt_fract = 0;

/* Para o controle do sleep */
volatile uint8_t sleeping = 0;

// Incrementa quando sleeping.
uint16_t SMILLIS_INC = 0;
uint16_t SFRACT_INC = 0;

// Informa se deve ser realizado a leitura dos sensores
volatile uint8_t readSensors = 0;

// Parametros do magnometro (HMC5884L)
uint8_t magEnabled = 0;			// Informa que o magnometro estah habilitado
int mx = 0, my = 0, mz = 0;		// Dados do magnometro

// Parametros do barometro (BMP085)
uint8_t barEnabled = 0;
long temp10 = 0, pressure = 0;
float temp = 0.0, altitude = 0.0;

// Parametros do accel/gyro (MPU6050)
uint8_t mpuEnabled = 0;
int16_t mpuTemp = 0;
float mpuTempDegrees = 0.0;
int16_t gx = 0, gy = 0, gz = 0;	// Dados do giroscopio
int16_t ax = 0, ay = 0, az = 0; // Dados do acelerometro


// LCD
uint8_t lcdEnabled = 0;

// WiFi
extern void IntSpiGPIOHandler(void);

/*
 * main.c
 */
int main(void) {
	//unsigned int i = 0;

	initPorts();
    disableWatchDog();
    initClocks();
    enableWatchDog();
    saveUsbPower();
    setupUart();
    //setupSPI();
    setupI2C();

    magEnabled = 0;
    barEnabled = 0;
    mpuEnabled = 0;
    lcdEnabled = 0;



    while (1) {

    	/*
    	uint8_t enable = lcd_blue_detect();

    	if (enable) {
    		uart_printf("LCD Habilitado\r\n");
    		if (lcdEnabled == 0) {
    			uart_printf("Configurando LCD...\r\n");

    			lcdEnabled = 1;
    		}

    	}
    	else {
    		uart_printf("LCD Desabilitado\r\n");
    		if (lcdEnabled)
    			lcdEnabled = 0;
    	}
    	*/


    	delay(500);

    	counter++;

    	__bis_SR_register(LPM0_bits + GIE);       // Entra no modo de baixo consumo com as interrupções habilitadas

    }


	return 0;
}

void initPorts() {
	P1DIR |= BIT0;	// P1.0 output
	P1OUT &= ~BIT0;	// Disable P1.0
}

/*
 * Configuração para o TIMER_0 (disparado a cada 20ms)
 */
/*void setupTimer0(void) {
	TA0CTL = TASSEL_2 +		// Fonte do clock: SMCLK (25MHz)
			 MC_1 +			// Modo de contagem: progressiva
			 ID_3 +			// Fator de divisão: 8 ( 3125kHz = 0,32ms )
			 TACLR;         // Limpa contador

	TA0CCTL0 = CCIE;        // Habilita interrupção do Timer A Bloco CCR0
	TA0CCR0 = 62500;		// Valor a ser comparado: 62500 --> 20ms
}*/

/*
 * Disparado por UART RX
 * Para sair do modo de baixo consumo em determinado caracter recebido
 */
void uart_rx_auxiliar(uint8_t c) {}

void wakeUpI2C() {
	FLAG_wakeUpI2C = 1;
}

/*
 * Interrupção UART (USCI_A1)
 */
__attribute__ ((interrupt(USCI_A1_VECTOR)))
void USCI_A1_ISR (void)
{
  switch(UCA1IV)
  {
  	  case USCI_UCRXIFG: uart_rx(); break;
  	  case USCI_UCTXIFG: break;
  }

  __bic_SR_register_on_exit(LPM4_bits);
}

/*
 * Interrupção I2C (USCI_B1)
 */
__attribute__ ((interrupt(USCI_B1_VECTOR)))
void USCI_B1_ISR (void)
{
	switch(__even_in_range(UCB1IV,12))
	{
		case  0: break;			// Vector  0: No interrupts
		case  2: break;	        // Vector  2: ALIFG
		case  4: break;         // Vector  4: NACKIFG
		case  6: break;         // Vector  6: STTIFG
		case  8: break;         // Vector  8: STPIFG
		case 10:                // Vector 10: RXIFG
			counterRX++;
			i2c_rx();
			break;
		case 12:                // Vector 12: TXIFG
			counterTX++;
			i2c_tx();
			break;
		default: break;
	}

	// Sair do modo de baixo consumo caso solicitado
	if (FLAG_wakeUpI2C) {
		FLAG_wakeUpI2C = 0;
		__bic_SR_register_on_exit(LPM0_bits); // Sair do modo LPM0
	}

}

/*
 * Interrupção do WatchDog
 */
__attribute__((interrupt(WDT_VECTOR)))
void WDT_ISR (void)
{
	// Copia para variaveis locais para que possam ser armazenadas em registros
	// (variaveis volateis devem ser lidas da memoria em cada acesso)
	unsigned long m = wdt_millis;
	unsigned int f = wdt_fract;

	m += sleeping ? SMILLIS_INC : MILLIS_INC;
	f += sleeping ? SFRACT_INC : FRACT_INC;
	if (f >= FRACT_MAX) {
		f -= FRACT_MAX;
		m += 1;
	}

	wdt_fract = f;
	wdt_millis = m;
	wdt_overflow_count++;

	/* Sair do modo de baixo consumo */
	__bic_SR_register_on_exit(LPM0_bits);
}


// Timer0 A0 interrupt service routine
/*__attribute__((interrupt(TIMER_A0_VECTOR)))
void TIMER0_A0_ISR (void)
{
  P1OUT ^= 0x01;                            // Toggle P1.0
}
*/


__attribute__((interrupt(PORT2_VECTOR)))
void PORT2_ISR (void)
{
	if (P2IFG & WLAN_IRQ_PIN) {
		IntSpiGPIOHandler();
		P2IFG &= ~WLAN_IRQ_PIN;
	}
}
