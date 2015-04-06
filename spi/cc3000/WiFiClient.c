/*
 * WiFiClient.c
 *
 *  Created on: 30/03/2015
 *      Author: Victor
 */

#include "WiFiClient.h"

extern fd_set gConnectedSockets;

long clientSocket = 255;

// Para o controle de buffer
static uint8_t rx_buf[16];
static int rx_buf_pos = 0;
static int rx_buf_fill = 0;
static int no_more_bytes = 0;

void WiFiClient_init() {
	rx_buf_pos=0;
	rx_buf_fill=0;
	no_more_bytes = 0;
	clientSocket = 255;
}

void WiFiClient_openSocket(long sock) {
	clientSocket = sock;
	FD_CLR(sock, &gConnectedSockets);
}

uint8_t WiFiClient_available() {
	if(clientSocket == 255) return 0;

	if(!rx_buf_fill){

		timeval timeout;
		memset(&timeout, 0, sizeof(timeval));

		/* Minimum timeout for select() is 5ms */
		timeout.tv_sec = 0;
		timeout.tv_usec = 500000;

		fd_set readsds, errorsds;
		FD_ZERO(&readsds);
		FD_ZERO(&errorsds);
		FD_SET(clientSocket, &readsds);
		FD_SET(clientSocket, &errorsds);

		/* Call select() to see if there is any data waiting */
		int ret = select(clientSocket + 1, &readsds, NULL, &errorsds, &timeout);

		/* If select() returned anything greater than 0, there's data for us */
		if (ret <= 0) {
			return 0;
		}

		if(!FD_ISSET(clientSocket, &readsds)) return 0;
		rx_buf_fill = recv(((long)clientSocket) & 0xFF, rx_buf, 16, 0);

		if(rx_buf_fill <= 0) {
			rx_buf_pos = 0;
			rx_buf_fill = 0;
		}
	}

	return rx_buf_fill - rx_buf_pos;
}

int WiFiClient_read() {
	uint8_t b = 0;

	if (!WiFiClient_available())
		return -1;

	b = rx_buf[rx_buf_pos];
	rx_buf_pos++;

	if(rx_buf_pos >= rx_buf_fill) {
		rx_buf_pos = 0;
		rx_buf_fill = 0;
	}

	return b;
}

int WiFiClient_read_buffer(uint8_t *buf, size_t size) {
	size_t i = 0;
	uint8_t *ptr = buf;
	int b;

	if(!size) return -1;

	while(i < size) {
		b = WiFiClient_read();

		if(b < 0)
			return i;

		*ptr = (uint8_t) b;
		ptr++;
		i++;
	}

	return i;
}

uint8_t WiFiClient_write(uint8_t b) {
	if (!WiFiClient_connected()) {
		return 0;
	}

	send(((long)clientSocket)&0xFF, &b, 1, 0);
	return 1;
}

size_t WifiClient_write_buffer(const uint8_t *buf, size_t size) {
	if (!WiFiClient_connected()) {
		return 0;
	}

	int strlen = size, i = 0;

	if(size == 0) return 0;

	do{
		if(strlen-i < TX_BUF_SIZE){
			i += send(((long)clientSocket)&0xFF, buf+i, strlen-i, 0);
		}
		else {
			i += send(((long)clientSocket)&0xFF, buf+i, TX_BUF_SIZE, 0);
		}
	}while( i < strlen);

	return strlen;
}

int WiFiClient_peek() {
	if(!WiFiClient_available())
		return 0;
	return rx_buf[rx_buf_pos];
}

void WiFiClient_flush() {
	while (WiFiClient_available()) {
		WiFiClient_read();
	}
}

uint8_t WiFiClient_connected() {
	if(clientSocket == 255) return 0;

	/* If the connection was open or there is data */
	return FD_ISSET(clientSocket, &gConnectedSockets) || WiFiClient_available();
	//return !(!FD_ISSET(clientSocket, &gConnectedSockets) && !WiFiClient_available());
}

/*
 * Conectar a um servidor remoto utilizando TCP
 */
uint8_t WifiClient_connectHost(const char* hostname, uint16_t port) {

	/* If CC3000 is not connected to a network, return false. */
	if (!WiFi_getInitStatus() || (WiFi_getStatus() != WL_CONNECTED) || !WiFi_getDHCPStatus()) {
		return 0;
	}

	uint32_t remote_ip;

	/* Perform a DNS lookup of the site */
	if (!WiFi_dnsLookup(hostname, &remote_ip)) {
		return 0;
	}

	/* Connect to remote host using IP address */
	return WifiClient_connect(remote_ip, port);
}

/*
 * Conectar a um server remoto utilizando TCP
 */
uint8_t WifiClient_connect(uint32_t ip, uint16_t port) {
	/* If CC3000 is not connected to a network, return false. */
	if (!WiFi_getInitStatus() || (WiFi_getStatus() != WL_CONNECTED) || !WiFi_getDHCPStatus()) {
		return 0;
	}

	/* Create a socket */
	clientSocket = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
	if (clientSocket == -1) {
		return 0;
	}

	sockaddr clientSocketAddr;

	/* Set address family to AF_INET (only one that works right now) */
	clientSocketAddr.sa_family = AF_INET;

	/* Fill out the destination port */
	clientSocketAddr.sa_data[0] = (port & 0xFF00) >> 8;
	clientSocketAddr.sa_data[1] = (port & 0x00FF);

	/* Fill out the destination IP address */
	memcpy(&clientSocketAddr.sa_data[2], &ip, sizeof(ip));

	/* Set the rest of the dest_addr struct to 0 */
	memset(&clientSocketAddr.sa_data[6], 0, 9);

	/* Attempt to make a connection with a remote socket */
	if (sl_connect(clientSocket, &clientSocketAddr, sizeof(clientSocketAddr)) !=
														CC3000_SUCCESS) {
		WiFiClient_close();
		return 0;
	}

	WiFi_countSocket(1);

	/* Indicate that the socket is connected in the fd_set */
	FD_SET(clientSocket, &gConnectedSockets);

	return 1;
}

uint8_t WiFiClient_close() {
	if (clientSocket == 255)
		return 0;

	closesocket(clientSocket);
	FD_CLR(clientSocket, &gConnectedSockets);
	WiFi_countSocket(0);
	clientSocket = 255;

	return 1;
}


