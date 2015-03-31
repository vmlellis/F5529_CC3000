/*
 * WiFi.c
 *
 *  Created on: 22/03/2015
 *      Author: Victor
 *
 *  Referencias:
 *  - Energia (SimplelinkWifi)
 *  - https://learn.sparkfun.com/tutorials/cc3000-hookup-guide/smartconfig-and-fastconnect
 *  - https://github.com/sparkfun/SFE_CC3000_Library
 *  - http://www.embarcados.com.br/iniciando-com-cc3000-seu-microcontrolador-com-wi-fi-parte-2/
 *
 */

#include "WiFi.h"
#include "utility/SimplelinkWifi.h"

#define CC3000_SUCCESS  0

#define DISABLE	(0)
#define ENABLE	(1)

/* Sets the name of the device. Used by SmartConfig. */
#define DEVICE_NAME     "CC3000"

extern volatile unsigned long ulSmartConfigFinished, ulCC3000Connected, ulCC3000DHCP,
OkToDoShutDown, ulCC3000WasConnected;
extern volatile unsigned char ucStopSmartConfig;

// Simple Config Prefix
const char cc3000_prefix[] = {'T', 'T', 'T'};

//AES key "smartconfigAES16"
const unsigned char smartconfigkey[] = {0x73,0x6d,0x61,0x72,0x74,0x63,0x6f,0x6e,0x66,0x69,0x67,0x41,0x45,0x53,0x31,0x36};

//static tNetappIpconfigRetArgs ipConfig = {{0}};
//unsigned char fwVersion[] = {0, 0};

// Array of data to cache the information related to the networks discovered
static char _networkSsid[WL_NETWORKS_LIST_MAXNUM][WL_SSID_MAX_LENGTH];
static int32_t _networkRssi[WL_NETWORKS_LIST_MAXNUM];
static uint8_t _networkEncr[WL_NETWORKS_LIST_MAXNUM];

// Para checar se foi inicializado
uint8_t cc3000_is_initialized = 0;
uint8_t cc3000_numNetworsScanned = 0;

/*
 * Seta os valores de Timeout do CC3000;
 */
void WiFi_setTimeoutValues() {
	/* default */
	uint32_t aucDHCP = 14400;
	/* default */
	uint32_t aucARP = 3600;
	/* default */
	uint32_t aucKeepalive = 10;
	/* inifinity */
	uint32_t aucInactivity = 0;

	netapp_timeout_values(&aucDHCP, &aucARP, &aucKeepalive, &aucInactivity);
}

/*
 * Inicializa o CC3000
 */
uint8_t WiFi_init() {
	// Não inicializa se jah foi inicializado
	if (cc3000_is_initialized) {
		return 0;
	}

	pio_init();
	init_spi();

	wlan_init(CC3000_UsynchCallback, sendWLFWPatch, sendDriverPatch, sendBootLoaderPatch, ReadWlanInterruptPin, WlanInterruptEnable, WlanInterruptDisable, WriteWlanPin);
	wlan_start(0);
	wlan_set_event_mask(HCI_EVNT_WLAN_KEEPALIVE|HCI_EVNT_WLAN_UNSOL_INIT|HCI_EVNT_WLAN_ASYNC_PING_REPORT);
	WiFi_setTimeoutValues();

	cc3000_is_initialized = 1;
	return 1;
}

/*
 * Realiza a leitura da versão de firwmare do CC3000
 */
uint8_t WiFi_getFirmwareVersion(unsigned char *fwVersion)
{
	if (!cc3000_is_initialized) {
		return 0;
	}

	if (nvmem_read_sp_version(fwVersion) != CC3000_SUCCESS) {
		return 0;;
	}

	return 1;
}

/*
 * Realiza a leitura do endereço MAC do CC3000
 */
uint8_t Wifi_getMacAddress(unsigned char *mac_addr) {
	if (!cc3000_is_initialized) {
		return 0;
	}

	if (nvmem_get_mac_address(mac_addr) != CC3000_SUCCESS) {
		return 0;
	}

	return 1;
}

/*
 * Realiza o scan das redes WiFi
 */
int8_t WiFi_scanNetworks() {
	uint8_t numOfNetworks = 0;
	unsigned long aiIntervalList[NUM_CHANNELS];
	uint8_t rval;
	scanResults sr;

	if (!cc3000_is_initialized) {
		return 0;
	}

	int i = 0;
	for (i=0; i<NUM_CHANNELS; i++) {
		aiIntervalList[i] = 2000;
	}

	rval = wlan_ioctl_set_scan_params(
			1,					// enable start application scan
			100,				// minimum dwell time on each channel
			100,				// maximum dwell time on each channel
			5,					// number of probe requests
			0x7ff,				// channel mask
			-80,				// RSSI threshold
			0,					// SNR threshold
			205,				// probe TX power
			aiIntervalList		// table of scan intervals per channel
		);

	if(rval != CC3000_SUCCESS) return 0;

	// Give it 5 sec to complete the scan
	delay(5000);

	do {
		if(wlan_ioctl_get_scan_results(2000, (unsigned char *)&sr) != CC3000_SUCCESS) return 0;

		if(sr.isValid != 1 || !strlen((const char*)sr.ssid_name)) continue; //Result not valid
		/* only have space for WL_NETWORKS_LIST_MAXNUM
		 * Continue to clear the list */
		if(numOfNetworks > WL_NETWORKS_LIST_MAXNUM - 1) continue;

		memcpy(_networkSsid[numOfNetworks], sr.ssid_name, WL_SSID_MAX_LENGTH);
		_networkRssi[numOfNetworks] = -sr.rssi;
		_networkEncr[numOfNetworks] = sr.securityMode;
		numOfNetworks++;

	} while (sr.numNetworksFound > 0);

	cc3000_numNetworsScanned = numOfNetworks;

	return numOfNetworks;
}


/*
 * Retorna o SSID de uma rede escaneada
 */
char* Wifi_Networks_getSSID(uint8_t networkItem) {
	if (networkItem >= WL_NETWORKS_LIST_MAXNUM)
		return NULL;
	if (cc3000_numNetworsScanned == 0)
		return NULL;
	return _networkSsid[networkItem];
}

/*
 * Indicação da força do sinal (RSSI) de uma rede escaneada
 */
int32_t WiFi_Networks_RSSI(uint8_t networkItem)
{
	if (networkItem >= WL_NETWORKS_LIST_MAXNUM)
		return 0;
	if (cc3000_numNetworsScanned == 0)
		return 0;
	return _networkRssi[networkItem];
}

/*
 * Tipo de encriptaçao de uma rede escaneada
 * WLAN_SEC_UNSEC / WLAN_SEC_WEP / WLAN_SEC_WPA / WLAN_SEC_WPA2
 */
uint8_t Wifi_Networks_getEncryptionType(uint8_t networkItem)
{
	if (networkItem >= WL_NETWORKS_LIST_MAXNUM)
		return 0;
	if (cc3000_numNetworsScanned == 0)
		return 0;
	return _networkEncr[networkItem];
}

/*
 * Conecta em um AP aberto (sem senha)
 * Retorno: Sucesso na conexao (Não necessariamente estah conectado, pois o processo eh assincrono)
 */
uint8_t WiFi_connectOpenAP(char* ssid, unsigned int timeout) {

	if (!cc3000_is_initialized)
		return 0; // ERRO: Não inicializado

	if (ulCC3000Connected)
		return 0; // ERRO: Já conectado

	ulCC3000WasConnected = 0;

	/* This will ensure that CC3000 does not attempt to connect to a previously configuration from SmartConfig session */
	if (wlan_ioctl_set_connection_policy(DISABLE, DISABLE, DISABLE) != CC3000_SUCCESS)
		return 0; // ERRO: Erro na funcao wlan_ioctl_set_connection_policy

	if (wlan_connect(WLAN_SEC_UNSEC, ssid, strlen(ssid), NULL, NULL, 0) != CC3000_SUCCESS)
		return 0;// ERRO: Erro na funcao wlan_connect

	if (timeout > 0)
		delay(timeout); // timeout informado pelo usuario

	return 1;
}

/*
 * Conecta em um AP com senha (WEP / WPA / WPA2)
 * security: WLAN_SEC_WEP / WLAN_SEC_WPA / WLAN_SEC_WPA2
 * Retorno: Sucesso na conexao (Não necessariamente estah conectado, pois o processo eh assincrono)
 */
uint8_t WiFi_connectClosedAP(char* ssid, unsigned int security, const char* pass, unsigned int timeout) {

	if (!cc3000_is_initialized)
		return 0;

	if (ulCC3000Connected)
		return 0;

	if ( !(security == WLAN_SEC_WEP || security == WLAN_SEC_WPA || security == WLAN_SEC_WPA2) )
		return 0;


	ulCC3000WasConnected = 0;

	/* This will ensure that CC3000 does not attempt to connect to a previously configuration from SmartConfig session */
	if (wlan_ioctl_set_connection_policy(DISABLE, DISABLE, DISABLE) != CC3000_SUCCESS)
		return 0; // ERRO: Erro na funcao wlan_ioctl_set_connection_policy


	if (wlan_connect(security, ssid, strlen(ssid), NULL, (unsigned char *)pass, strlen((char *)(pass))) != CC3000_SUCCESS) {
		return 0;
	}

	if (timeout > 0)
		delay(timeout); // timeout informado pelo usuario

	return 1;
}

//// connect to a WPA AP
//void WiFi_connectWpaAP(char* ssid, const char* pass)
//{
//	ulCC3000WasConnected = 0;
//
//	/* This will ensure that CC3000 does not attempt to connect to a previously configuration from SmartConfig session */
//	wlan_ioctl_set_connection_policy(DISABLE, DISABLE, DISABLE);
//
//	wlan_connect(WLAN_SEC_WPA2, ssid, strlen(ssid), NULL, (unsigned char *)pass, strlen((char *)(pass)));
//}
//
//// connect to a WEP AP
//void WiFi_connectWepAP(char* ssid, uint8_t key_idx, unsigned char *key) {
//	ulCC3000WasConnected = 0;
//
//	/* This will ensure that CC3000 does not attempt to connect to a previously configuration from SmartConfig session */
//	wlan_ioctl_set_connection_policy(DISABLE, DISABLE, DISABLE);
//
//	wlan_connect(WLAN_SEC_WEP, ssid, strlen(ssid), NULL, key, key_idx);
//}

/*
 * Inicializa o SmartConfig
 */
uint8_t WiFi_startSmartConfig(unsigned int timeout) {
	ulCC3000WasConnected = 0;
	ucStopSmartConfig   = 0;

	ulSmartConfigFinished = 0;
	ulCC3000Connected = 0;
	ulCC3000DHCP = 0;
	OkToDoShutDown=0;

	if (!cc3000_is_initialized)
		return 0;

	// Reset all the previous configuration
	if (wlan_ioctl_set_connection_policy(DISABLE, DISABLE, DISABLE) != CC3000_SUCCESS) {
		return 0;
	}

	if (wlan_ioctl_del_profile(255) != CC3000_SUCCESS) {
		return 0;
	}

	//Wait until CC3000 is disconnected
	while (ulCC3000Connected == 1)
	{
		delay(500);
	}

	// Trigger the Smart Config process

	if (wlan_smart_config_set_prefix((char*)cc3000_prefix) != CC3000_SUCCESS) {
		return 0;
	}


	// Start the SmartConfig start process (with encryption)
	if (wlan_smart_config_start(1) != CC3000_SUCCESS) {
		return 0;
	}


	// Wait for Smartconfig process complete
	while (ulSmartConfigFinished == 0)
	{
		delay(500);
	}


#ifndef CC3000_UNENCRYPTED_SMART_CONFIG
	// create new entry for AES encryption key
	if (nvmem_create_entry(NVMEM_AES128_KEY_FILEID,16) != CC3000_SUCCESS) {
		return 0;;
	}

	// write AES key to NVMEM
	if (aes_write_key((unsigned char *)(&smartconfigkey[0])) != CC3000_SUCCESS) {
		return 0;
	}

	// Decrypt configuration information and add profile
	if (wlan_smart_config_process() != CC3000_SUCCESS) {
		return 0;
	}
#endif

	// Configure to connect automatically to the AP retrieved in the
	// Smart config process
	if (wlan_ioctl_set_connection_policy(DISABLE, DISABLE, ENABLE) != CC3000_SUCCESS) {
		return 0;
	}

	// reset the CC3000
	wlan_stop();

	delay(1000);

	wlan_start(0);

	if (timeout > 0)
		delay(timeout);


	/* If we make it this far, we need to tell the SmartConfig app to stop */
	mdnsAdvertiser(1, DEVICE_NAME, strlen(DEVICE_NAME));

	return 1;
}

// Disconnect
uint8_t WiFi_disconnect()
{
	if (!cc3000_is_initialized) {
		return 0;
	}

	if(!ulCC3000Connected)
		return 0;

	if (wlan_disconnect() != CC3000_SUCCESS) {
		return 0;
	}


	return 1;
}

/*
 * Retorna o status da conexão
 */
uint8_t WiFi_getStatus()
{
	if(ulCC3000Connected == 0)
		return ulCC3000WasConnected ? WL_CONNECTION_LOST : WL_DISCONNECTED;
	else
		return WL_CONNECTED;
}


/*
 * Configura o IP, DNS Server, gateway e subnet
 */
uint8_t WiFi_config(uint32_t local_ip, uint32_t dns_server, uint32_t gateway, uint32_t subnet) {
	if (!cc3000_is_initialized) {
		return 0;
	}

	if (netapp_dhcp(&local_ip, &dns_server, &gateway, &subnet) != CC3000_SUCCESS) {
		return 0;
	}
	wlan_stop();
	wlan_start(0);
	return 1;
}

/*void WiFiClass_getConfig(uint32_t* localIP, uint32_t* gatewayIP, uint32_t* subnetMask)
{
	tNetappIpconfigRetArgs config;
	netapp_ipconfig(&config);
	memcpy(localIP, &(config.aucIP), sizeof(config.aucIP));
	memcpy(gatewayIP, &(config.aucDefaultGateway), sizeof(config.aucDefaultGateway));
	memcpy(subnetMask, &(config.aucSubnetMask), sizeof(config.aucSubnetMask));
}*/


/*
 * IP do CC3000
 */
uint8_t WiFi_getLocalIP(uint32_t* localIP)
{
	if (!cc3000_is_initialized) {
		return 0;
	}

	tNetappIpconfigRetArgs config;
	netapp_ipconfig(&config);
	memcpy(localIP, &(config.aucIP), sizeof(config.aucIP));
	return 1;
}

/*
 * Gateway do CC3000
 */
uint32_t WiFi_getGatewayIP()
{
	uint32_t* gatewayIP = NULL;
	tNetappIpconfigRetArgs config;
	netapp_ipconfig(&config);
	memcpy(gatewayIP, &(config.aucDefaultGateway), sizeof(config.aucDefaultGateway));
	return *gatewayIP;
}

/*
 * Mascara de rede do CC3000
 */
uint32_t WiFi_getSubnetMask()
{
	uint32_t* subnetMask = NULL;
	tNetappIpconfigRetArgs config;
	netapp_ipconfig(&config);
	memcpy(subnetMask, &(config.aucSubnetMask), sizeof(config.aucSubnetMask));
	return *subnetMask;
}

/*
 * SSID do CC3000
 */
uint8_t WiFi_getSSID(char* ssid) {
	if (!cc3000_is_initialized) {
		return 0;
	}

	if (!ulCC3000Connected) {
		return 0;
	}

	tNetappIpconfigRetArgs ipConfig;
	netapp_ipconfig(&ipConfig);
	memcpy(ssid, &(ipConfig.uaSSID), sizeof(ipConfig.uaSSID));
	return 1;
}

/*
 * Conecta em uma rede armazenada na memoria do CC3000
 * PS.: Precisa ser realizado o SmartConfig primeiro
 */
uint8_t WiFi_fastConnect(unsigned int timeout) {
	// TODO
	return 1;
}

/*
 * Procura um endereço IP de um determinado nome de host
 */
uint8_t WiFi_dnsLookup(char *hostname, uint32_t* ip_address) {
	uint32_t ret_ip_addr;

	if (!cc3000_is_initialized) {
		return 0;
	}

	if (!ulCC3000Connected) {
		return 0;
	}

	if (!ulCC3000DHCP) {
		return 0;
	}

	if (gethostbyname((char *)hostname, strlen(hostname), &ret_ip_addr) <= 0) {
		return 0;
	}

	*ip_address = ntohl(ret_ip_addr);

	return 1;
}

/*
 * Realizar o update do Firmware para a versão 1.1.0
 */
uint8_t WiFi_updateFirmware() {
	// TODO
	return 1;
}







