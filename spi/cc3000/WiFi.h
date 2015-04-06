/*
 * WiFi.h
 *
 *  Created on: 22/03/2015
 *      Author: Victor
 */

#ifndef F5529___CC3000_WEBSERVER_SPI_CC3000_WIFI_H_
#define F5529___CC3000_WEBSERVER_SPI_CC3000_WIFI_H_

#include "utility/SimplelinkWifi.h"

#define CC3000_SUCCESS  0

uint8_t WiFi_init();
uint8_t WiFi_getFirmwareVersion(unsigned char *fwVersion);
uint8_t Wifi_getMacAddress(unsigned char *mac_addr);
int8_t WiFi_scanNetworks();
char* Wifi_Networks_getSSID(uint8_t networkItem);
int32_t WiFi_Networks_RSSI(uint8_t networkItem);
uint8_t Wifi_Networks_getEncryptionType(uint8_t networkItem);
uint8_t WiFi_connectOpenAP(char* ssid, unsigned int timeout);
uint8_t WiFi_connectClosedAP(char* ssid, uint8_t security, const char* pass, unsigned int timeout);
uint8_t WiFi_startSmartConfig(unsigned int timeout);
uint8_t WiFi_disconnect();
uint8_t WiFi_getStatus();
uint8_t WiFi_config(uint32_t local_ip, uint32_t dns_server, uint32_t gateway, uint32_t subnet);
uint8_t WiFi_getLocalIP(uint32_t* localIP);
uint8_t WiFi_getGatewayIP(uint32_t* gatewayIP);
uint8_t WiFi_getSubnetMask(uint32_t* subnetMask);
uint8_t WiFi_getSSID(char* ssid);
uint8_t WiFi_fastConnect(unsigned int timeout);
uint8_t WiFi_dnsLookup(const char *hostname, uint32_t* ip_address);
//uint8_t WiFi_updateFirmware();
uint8_t WiFi_countSocket(uint8_t add_sock);
uint8_t WiFi_getInitStatus();
uint8_t WiFi_getDHCPStatus();

#endif /* F5529___CC3000_WEBSERVER_SPI_CC3000_WIFI_H_ */
