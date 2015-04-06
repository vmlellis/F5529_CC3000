/*
 * WiFiServer.h
 *
 *  Created on: 31/03/2015
 *      Author: Victor
 */

#ifndef F5529___CC3000_WEBSERVER_SPI_CC3000_WIFISERVER_H_
#define F5529___CC3000_WEBSERVER_SPI_CC3000_WIFISERVER_H_

#include "WiFi.h"

uint8_t WiFiServer_init(uint16_t port);
uint8_t WiFiServer_available();
long WiFiServer_clientSocket();
uint8_t WiFiServer_write(uint8_t b);

#endif /* F5529___CC3000_WEBSERVER_SPI_CC3000_WIFISERVER_H_ */
