/*
 * WiFiUDP.h
 *
 *  Created on: 03/04/2015
 *      Author: Victor
 */

#ifndef F5529___CC3000_WEBSERVER_SPI_CC3000_WIFIUDP_H_
#define F5529___CC3000_WEBSERVER_SPI_CC3000_WIFIUDP_H_

#include "WiFi.h"

#define NO_SOCKET_AVAIL 255
#define UDP_TX_PACKET_MAX_SIZE 255
#define UDP_RX_PACKET_MAX_SIZE 255
#define MAX_SENDTO_SIZE 95
#define MAX_RECVFROM_SIZE 95

void WiFiUDP_init();
uint8_t WiFiUDP_start(uint16_t port);
int WiFiUDP_available();
int WiFiUDP_read();
int WiFiUDP_read_buffer(unsigned char* buffer, size_t size);
int WiFiUDP_beginPacketHost(const char *hostname, uint16_t port);
int WiFiUDP_beginPacket(uint32_t ip, uint16_t port);
int WiFiUDP_peek();
void WiFiUDP_flush();
uint32_t WiFiUDP_remoteIP();
uint16_t  WiFiUDP_remotePort();
size_t WiFiUDP_write(uint8_t byte);
size_t WiFiUDP_write_buffer(const uint8_t *buffer, size_t size);
int WiFiUDP_endPacket();
void WiFiUDP_close();

#endif /* F5529___CC3000_WEBSERVER_SPI_CC3000_WIFIUDP_H_ */
