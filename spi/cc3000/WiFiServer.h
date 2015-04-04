/*
 * WiFiServer.h
 *
 *  Created on: 31/03/2015
 *      Author: Victor
 */

#ifndef SPI_CC3000_WIFISERVER_H_
#define SPI_CC3000_WIFISERVER_H_

#include "WiFi.h"

uint8_t WiFiServer_init(uint16_t port);
uint8_t WiFiServer_available();
long WiFiServer_clientSocket();
uint8_t WiFiServer_write(uint8_t b);

#endif /* SPI_CC3000_WIFISERVER_H_ */
