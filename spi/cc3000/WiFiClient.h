/*
 * WiFiClient.h
 *
 *  Created on: 03/04/2015
 *      Author: Victor
 */

#ifndef F5529___CC3000_WEBSERVER_SPI_CC3000_WIFICLIENT_H_
#define F5529___CC3000_WEBSERVER_SPI_CC3000_WIFICLIENT_H_

#include "WiFi.h"

void WiFiClient_init();
void WiFiClient_openSocket(long sock);
uint8_t WiFiClient_available();
int WiFiClient_read() ;
int WiFiClient_read_buffer(uint8_t *buf, size_t size);
uint8_t WiFiClient_write(uint8_t b);
size_t WifiClient_write_buffer(const uint8_t *buf, size_t size);
int WiFiClient_peek();
void WiFiClient_flush();
uint8_t WiFiClient_connected();
uint8_t WifiClient_connectHost(const char* hostname, uint16_t port);
uint8_t WifiClient_connect(uint32_t ip, uint16_t port);
uint8_t WiFiClient_close();

#endif /* F5529___CC3000_WEBSERVER_SPI_CC3000_WIFICLIENT_H_ */
