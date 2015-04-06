/*
 * main.c
 *
 *  Construido com a IDE CCSv6 e compilador GNU v4.9.1 (Red Hat)
 */

#define WLAN_SSID       "mySSID"        // cannot be longer than 32 characters!
#define WLAN_PASS       "supersecret"

#include <msp430f5529.h>
#include "setup.h"
#include "uart/uart_tx.h"
#include "i2c/lcd/lcd.h"
#include "i2c/lcd/lcd_blue.h"
#include "spi/cc3000/WiFi.h"
#include "spi/cc3000/WiFiServer.h"
#include "spi/cc3000/WiFiClient.h"

volatile int counter = 0;
volatile int counterRX = 0;
volatile int counterTX = 0;
volatile int counterSPI = 0;

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
uint8_t wifiInit = 0;
unsigned int timeoutWifi = 30000;   // Milliseconds
uint8_t wiFiServerInitialized = 0;

char ssid[] = WLAN_SSID;     //  your network SSID (name)
char pass[] = WLAN_PASS;     //  your network SSID (name)


// Prototipos
//char* append(const char *s, char c);
void connectWiFi();
uint8_t endsWith(char *str, char *suffix);
uint8_t startsWith(char *str, char *pre);
void printIndex();

char current_line[100] = "";

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
    setupI2C();

    uart_printf("Inicializando o WiFi...\r\n");
    _enable_interrupts();
    if (WiFi_init()) {
    	wifiInit = 1;
    }


    magEnabled = 0;
    barEnabled = 0;
    mpuEnabled = 0;
    lcdEnabled = 0;

    lcdEnabled = lcd_blue_detect();
    if (lcdEnabled) {
    	lcd_blue_config();
    	lcd_clear();
    	lcd_print("LCD HABILITADO! ");
    }

    connectWiFi();

    while (1) {
    	if (wiFiServerInitialized) {
    		if (WiFiServer_available()) {
    			WiFiClient_init();
    			long sock = WiFiServer_clientSocket();
    			WiFiClient_openSocket(sock);
    			uart_printf("Novo cliente\r\n");
    			strcpy(current_line, "");
    			//char *current_line = "";

    			while (WiFiClient_connected()) {
    				if (WiFiClient_available()) {
						char c = WiFiClient_read();

						//uart_putc(c);
						if (c == '\n') {
							uart_printf("%s\r\n", current_line);


							if (strlen(current_line) == 0) {
								break;
						    }
						    else {  // if you got a newline, then clear currentLine:
						    	strcpy(current_line, "");
						    	//currentLine = "";

						    	//realloc(current_line, 1);
						    	//current_line[0] = '\0';
						    	//free(current_line);
						    	//current_line = "";
						    	/*char *aux = malloc(1);
						    	aux[0] = '\0';
						    	free(current_line);
						    	current_line = aux;*/
						    }
						}
						else if (c != '\r') {
							// add character to current line
							strncat(current_line, &c, 1);
							//uart_printf("Add: %s\r\n", current_line);
							/*char *new_line = append(current_line, c);
							if (new_line != NULL) {
								//free(current_line);
								//current_line = new_line;
								//uart_printf("Add: %s\r\n", current_line);
							}*/
							/*if (c != (char)-1) {

								// add character to current line
								size_t len = strlen(current_line);
								realloc(current_line, len+1);
								current_line[len] = c;
								current_line[len+1] = '\0';
								/*size_t len = strlen(current_line);
								char *aux = malloc(len + 2);
								strcpy(aux, current_line);
								aux[len] = c;
								aux[len+1] = '\0';
								free(current_line);
								current_line = aux;
								//uart_printf("Add: %s\r\n", current_line);
							//}*/
						}

						if (endsWith(current_line, "GET / ") || endsWith(current_line, "GET /index.html ")) {
							printIndex();
						}


    				}
    			}
    			// Close the connection
    			WiFiClient_close();
    			uart_printf("Ciente desconectou\r\n");
    			//free(current_line);
    		}

    	}


    	//delay(500);

    	//counter++;

    	//__bis_SR_register(LPM0_bits + GIE);       // Entra no modo de baixo consumo com as interrupções habilitadas

    }


	return 0;
}

/*char* append(const char *s, char c) {
    int len = strlen(s);
    char *buf = malloc(len+2);
    strcpy(buf, s);
    buf[len] = c;
    buf[len + 1] = '\0';
    uart_printf("Str: %s\r\n", s);
    uart_printf("Add: %c\r\n", c);
    uart_printf("Buffer: %s\r\n", buf);
    return strdup(buf);
}*/

void connectWiFi() {
    if (wifiInit) {
		uart_printf("Conectando no WiFi...\r\n");
		if (lcdEnabled) {
			lcd_clear();
			lcd_print("Conectando...   ");
		}

		if (WiFi_connectClosedAP(ssid, WLAN_SEC_WPA2, pass, timeoutWifi)) {
			if (lcdEnabled) {
				lcd_clear();
				lcd_print("Conectado!      ");
			}

			uint8_t ip_address[4];
			if (WiFi_getLocalIP((uint32_t *)ip_address)) {
				uart_printf("IP: %i.%i.%i.%i\r\n", ip_address[3], ip_address[2], ip_address[1], ip_address[0]);
			}

			uint8_t mac[6];
			if (Wifi_getMacAddress(mac)) {
				uart_printf("MAC: %h:%h:%h:%h:%h:%h\r\n", mac[5], mac[4], mac[3], mac[2], mac[1], mac[0]);
			}

			uint8_t ver[2];
			if (WiFi_getFirmwareVersion(ver)) {
				uart_printf("FW Version: %i.%i\r\n", ver[0], ver[1]);
			}

			uint8_t subnet[4];
			if (WiFi_getSubnetMask((uint32_t *) subnet)) {
				uart_printf("Subnet: %i.%i.%i.%i\r\n", subnet[3], subnet[2], subnet[1], subnet[0]);
			}

			uint8_t gateway[4];
			if (WiFi_getGatewayIP((uint32_t *) gateway)) {
				uart_printf("Gateway: %i.%i.%i.%i\r\n", gateway[3], gateway[2], gateway[1], gateway[0]);
			}

			char wifi_ssid[32];
			if(WiFi_getSSID(wifi_ssid)) {
				uart_printf("SSID: %s\r\n", wifi_ssid);
			}

			uint8_t hostIp[4];
			if (WiFi_dnsLookup("google.com", (uint32_t *)hostIp)) {
				uart_printf("Google IP: %i.%i.%i.%i\r\n", hostIp[3], hostIp[2], hostIp[1], hostIp[0]);
			}

			if (WiFiServer_init(80)) {
				wiFiServerInitialized = 1;
				uart_printf("Servidor inicializado\r\n");
			}
			else {
				uart_printf("Erro na inicialização do servidor!\r\n");
			}


		}
		else {
			if (lcdEnabled) {
				lcd_clear();
				lcd_print("Erro na conexão!");
			}
		}

    }
    else {
    	uart_printf("Erro na Inicialização do WiFi!\r\n");
    	if (lcdEnabled) {
    		lcd_clear();
			lcd_print("Erro no Wifi!   ");
		}
    }
}

uint8_t startsWith(char *str, char *pre)
{
	if (str == NULL || pre == NULL || strlen(str) < strlen(pre)) return 0;
	return strncmp(pre, str, strlen(pre)) == 0;
}

uint8_t endsWith(char *str, char *suffix) {
	if (str == NULL || suffix == NULL || strlen(str) < strlen(suffix)) return 0;
	return strcmp(&str[strlen(str) - strlen(suffix)], suffix) == 0;
}

void client_println(const char *str) {
	WifiClient_write_buffer((const uint8_t*) str, strlen(str));
	/*int size = strlen(str);
	while (size > 0) {
		char c = str[size];
		WiFiClient_write(c);
		size--;
	}*/
	WiFiClient_write('\r');
	WiFiClient_write('\n');

	/*char c;
	while ((c = *str++)) {
		WiFiClient_write(c);
		//uart_putc(c);
	}
	WiFiClient_write('\n');
	//uart_putc('\n');
	 */

	//uart_printf("%s\r\n", str);
}

void printIndex()
{
	// HTTP headers always start with a response code (e.g. HTTP/1.1 200 OK)
	// and a content-type so the client knows what's coming, then a blank line:
	client_println("HTTP/1.1 200 OK");
	client_println("Content-type:text/html");
	WiFiClient_write('\r');
	WiFiClient_write('\n');
	client_println("<html><head><title>CC3000 Energia Webpage</title></head><body align=center>");
	client_println("<h1 align=center><font color=\"blue\">Welcome To CC3000 Web Server</font></h1>");
	client_println("</body></html>");

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

/*
 * Interrupção necessária para o funcionamento do CC3000
 */
__attribute__((interrupt(PORT2_VECTOR)))
void PORT2_ISR (void)
{
	if (P2IFG & WLAN_IRQ_PIN) {
		IntSpiGPIOHandler();
		P2IFG &= ~WLAN_IRQ_PIN;
	}
	counterSPI++;
}
