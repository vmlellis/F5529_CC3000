/*
 * WiFiUDP.c
 *
 *  Created on: 03/04/2015
 *      Author: Victor
 */


#include "WiFiUDP.h"

/* socket descriptor for client and server */
static long udpSocket = NO_SOCKET_AVAIL;
/* Port for when in server mode */
static uint16_t _serverPort;
/* IP of the remote host talking to us */
static uint8_t _remoteIp[4];
/* Port of the remote host talking to us */
static uint16_t _remotePort;

/* Position in the rx buffer */
static uint16_t rx_buf_pos = 0;
/* Counter to keep track of if we need to read fromt the socket to fill the rx buffer */
static uint16_t rx_buf_fill = 0;
/* rx buffer that is filled with a call to recvfrom */
static unsigned char rx_buf[UDP_RX_PACKET_MAX_SIZE];

/* Socket address when in client mode */
static sockaddr clientSocketAddr;
/* Counter to keep track of how many bytes are in the tx buffer */
static uint16_t tx_buf_fill = 0;
/* tx buffer */
static unsigned char tx_buf[UDP_TX_PACKET_MAX_SIZE];

/*
 * Inicicializa as variaveis
 */
void WiFiUDP_init() {
	udpSocket = NO_SOCKET_AVAIL;
	memset(rx_buf, 0, sizeof(rx_buf));
	memset(tx_buf, 0, sizeof(tx_buf));
	rx_buf_pos = 0;
	rx_buf_fill = 0;
	tx_buf_fill = 0;
}

/*
 * Start WiFiUDP socket, listening at local port PORT
 */
uint8_t WiFiUDP_start(uint16_t port) {

	if (!WiFi_getStatus() != WL_CONNECTED)
		return 0;

	/* Allocate a UDP socket */
	udpSocket = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);

	/* Allocate socket fail? */
	if (udpSocket == -1) {
		return 0;
	}

	/* Any sockets available? */
	if (udpSocket == NO_SOCKET_AVAIL) {
		return 0;
	}

	/* Socket address when in server mode */
	sockaddr serverSocketAddr;

	/* Set address family to AF_INET (only one that works right now) */
	serverSocketAddr.sa_family = AF_INET;

	/* Fill out the destination port */
	serverSocketAddr.sa_data[0] = (port & 0xFF00) >> 8;
	serverSocketAddr.sa_data[1] = (port & 0x00FF);
	memset(&serverSocketAddr.sa_data[2], 0, 4);

	/* Set the rest of the dest_addr struct to 0 */
	memset(&serverSocketAddr.sa_data[6], 0, 9);

	if (bind(udpSocket, &serverSocketAddr, sizeof(sockaddr)) == -1) {
		return 0;
	}

	/* Set internal port to port passed to us */
	_serverPort = port;

	/* up the socket count */
	WiFi_countSocket(1);

	return 1;
}

/*
 * Number of bytes remaining in the current packet
 */
int WiFiUDP_available() {
	if(udpSocket == NO_SOCKET_AVAIL) return 0;

	if (!rx_buf_fill) {

		timeval timeout;
		memset(&timeout, 0, sizeof(timeval));
		timeout.tv_sec = 0;
		timeout.tv_usec = 0;
		fd_set readsds, errorsds;
		FD_ZERO(&readsds);
		FD_ZERO(&errorsds);
		FD_SET(udpSocket, &readsds);
		FD_SET(udpSocket, &errorsds);

		select(udpSocket + 1, &readsds, NULL, &errorsds, &timeout);

		/* If the socket is reported to be closed then the only way to deal
		 * with this is to bluntly open it again using begin(...) */
		if(FD_ISSET(udpSocket, &errorsds)) {
			if(_serverPort != 0) {
				WiFiUDP_start(_serverPort);
			}
			return 0;
		}

		if(!FD_ISSET(udpSocket, &readsds))
			return 0;

		socklen_t fromlen;
		sockaddr from;
		rx_buf_fill = recvfrom(udpSocket, rx_buf, MAX_RECVFROM_SIZE, 0, &from, &fromlen);

		/* Fill the remote ip */
		int i;
		for (i = 0; i < 4; i++) {
			_remoteIp[i] = from.sa_data[i+2];
		}

		_remotePort = ((from.sa_data[0] << 8) & 0xFF00) | (from.sa_data[1] & 0x00FF);
	}

	return rx_buf_fill - rx_buf_pos;
}

/*
 * Read a single byte from the current packet
 */
int WiFiUDP_read() {
	uint8_t b = 0;

	if (WiFiUDP_available() == 0)
		return -1;

	b = rx_buf[rx_buf_pos];
	rx_buf_pos++;

	if(rx_buf_pos >= rx_buf_fill) {
		rx_buf_pos = 0;
		rx_buf_fill = 0;
	}

	return b;
}

/*
 * Read up to len bytes from the current packet and place them into buffer
 * Returns the number of bytes read, or 0 if none are available
 */
int WiFiUDP_read_buffer(unsigned char* buffer, size_t size) {
	size_t i = 0;
	uint8_t *ptr = buffer;
	int b;

	if(!size) return -1;

	while(i < size) {
		b = WiFiUDP_read();

		if(b < 0)
			return i;

		*ptr = (uint8_t)b;
		ptr++;
		i++;
	}

	return i;
}

/*
 * Start building up a packet to send to the remote host specific in host and port
 * Returns 1 if successful, 0 if there was a problem resolving the hostname or port
 */
int WiFiUDP_beginPacketHost(const char *hostname, uint16_t port) {

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
	return WiFiUDP_beginPacket(remote_ip, port);
}

/*
 * Start building up a packet to send to the remote host specific in ip and port
 * Returns 1 if successful, 0 if there was a problem with the supplied IP address or port
 */
int WiFiUDP_beginPacket(uint32_t ip, uint16_t port) {
	clientSocketAddr.sa_family = AF_INET;
	// Set the Port Number
	clientSocketAddr.sa_data[0] = (unsigned char)((port & 0xFF00)>> 8);
	clientSocketAddr.sa_data[1] = (unsigned char)(port & 0x00FF);

	memcpy(&clientSocketAddr.sa_data[2], &ip, sizeof(ip));

	/* Set the rest of the dest_addr struct to 0 */
	memset(&clientSocketAddr.sa_data[6], 0, 9);

	tx_buf_fill = 0;

	timeval timeout;
	memset(&timeout, 0, sizeof(timeval));
	timeout.tv_sec = 0;
	timeout.tv_usec = 0;
	fd_set errorsds;
	FD_ZERO(&errorsds);
	FD_SET(udpSocket, &errorsds);

	select(udpSocket + 1, NULL, NULL, &errorsds, &timeout);

	/* If we do not already have a socket then allocate one */
	if ((udpSocket != NO_SOCKET_AVAIL) && !FD_ISSET(udpSocket, &errorsds))
		return 1;

	/* Allocate a socket */
	udpSocket = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);

	/* No socket available or somthing bad happened */
	if(udpSocket < 0) {
		udpSocket = NO_SOCKET_AVAIL;
		return 0;
	}

	WiFi_countSocket(1);

	return 1;
}

/*
 * Return the next byte from the current packet without moving on to the next byte
 */
int WiFiUDP_peek() {
	if(!WiFiUDP_available())
		return -1;
	return rx_buf[rx_buf_pos];
}

/*
 * Finish reading the current packet
 */
void WiFiUDP_flush() {
	while (WiFiUDP_available()) {
		WiFiUDP_read();
	}
}

/*
 * Return the IP address of the host who sent the current incoming packet
 */
uint32_t WiFiUDP_remoteIP() {
	uint32_t ret;
	memcpy(&ret, _remoteIp, sizeof(_remoteIp));
	return ret;
}

/*
 * Return the port of the host who sent the current incoming packet
 */
uint16_t  WiFiUDP_remotePort()
{
	return _remotePort;
}

/*
 * Write a single byte into the packet
 */
size_t WiFiUDP_write(uint8_t byte) {
	return WiFiUDP_write_buffer(&byte, 1);
}

/*
 * Write size bytes from buffer into the packet
 */
size_t WiFiUDP_write_buffer(const uint8_t *buffer, size_t size)
{
	memcpy(&tx_buf[tx_buf_fill], buffer, size);
	tx_buf_fill += size;
	return sizeof(buffer);
}

int WiFiUDP_endPacket()
{
	int len = tx_buf_fill, i = 0;

	if(tx_buf_fill == 0) return 0;

	timeval timeout;
	memset(&timeout, 0, sizeof(timeval));
	timeout.tv_sec = 0;
	timeout.tv_usec = 5000;
	fd_set writesds, errorsds;
	FD_ZERO(&writesds);
	FD_ZERO(&errorsds);
	FD_SET(udpSocket, &writesds);
	FD_SET(udpSocket, &errorsds);

	select(udpSocket + 1, NULL, &writesds, &errorsds, &timeout);

	/* For whatever reason sento chokes when sending more than 96 bytes.
	 * Sending in chunks of 64 bytes works around this limitation. */
	do {
		if(len - i < MAX_SENDTO_SIZE) {
			i += sendto(udpSocket, tx_buf+i, len-i, 0, &clientSocketAddr, sizeof(sockaddr));
		} else {
			i += sendto(udpSocket, tx_buf+i, MAX_SENDTO_SIZE, 0, &clientSocketAddr, sizeof(sockaddr));
		}
	} while(i < len);

	return 0;
}

void WiFiUDP_close()
{
	_serverPort = 0;

	if (udpSocket == NO_SOCKET_AVAIL)
		return;

	closesocket(udpSocket);
	WiFi_countSocket(0);
	udpSocket = NO_SOCKET_AVAIL;
}
