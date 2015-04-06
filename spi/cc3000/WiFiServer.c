/*
 * WiFiServer.c
 *
 *  Created on: 30/03/2015
 *      Author: Victor
 */

#include "WiFiServer.h"

static long serverSocket = -1;
static long clientDescriptor = -1;

/*
 * Inicializa o servidor com a porta que serah utilizada
 */
uint8_t WiFiServer_init(uint16_t port) {
	serverSocket = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);

	if (serverSocket == -1)
		return 0;

	sockaddr serverSocketAddr;
	serverSocketAddr.sa_family = AF_INET;

	// Set the Port Number
	serverSocketAddr.sa_data[0] = (unsigned char)((port & 0xFF00)>> 8);
	serverSocketAddr.sa_data[1] = (unsigned char)(port & 0x00FF);
	memset(&serverSocketAddr.sa_data[2], 0, 4);

	if(serverSocket>=0 && serverSocket != 255 ){
		if (bind(serverSocket, &serverSocketAddr, sizeof(sockaddr)) != CC3000_SUCCESS) {
			return 0;
		}
	}

	if (listen(serverSocket, 1) != CC3000_SUCCESS)
	{
		return 0;
	}

	return 1;
}


uint8_t WiFiServer_available() {
	sockaddr clientaddr;
	socklen_t addrlen;

	if (!WiFi_countSocket(1)) {
		WiFi_countSocket(0);
		return 0;
	}

	addrlen = sizeof(clientaddr);

	clientDescriptor = accept(serverSocket, (sockaddr *) &clientaddr, &addrlen);
	if(clientDescriptor < 0) {
		WiFi_countSocket(0);
		return 0;
	}

	return 1;
}

long WiFiServer_clientSocket() {
	return clientDescriptor;
}

uint8_t WiFiServer_write(uint8_t b)
{
	if (send(((long)clientDescriptor)&0xFF, &b, 1, 0) == -1)
		return 0;
    return 1;
}
