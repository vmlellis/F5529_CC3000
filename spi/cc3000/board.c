/*****************************************************************************
*
*  board.c
*  Copyright (C) 2011 Texas Instruments Incorporated - http://www.ti.com/
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*    Redistributions of source code must retain the above copyright
*    notice, this list of conditions and the following disclaimer.
*
*    Redistributions in binary form must reproduce the above copyright
*    notice, this list of conditions and the following disclaimer in the
*    documentation and/or other materials provided with the   
*    distribution.
*
*    Neither the name of Texas Instruments Incorporated nor the names of
*    its contributors may be used to endorse or promote products derived
*    from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS 
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT 
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
*  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT 
*  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, 
*  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT 
*  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
*  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
*  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT 
*  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE 
*  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*
*****************************************************************************/
#include "utility/wlan.h"
#include "utility/evnt_handler.h"    // callback function declaration
#include "utility/nvmem.h"
#include "utility/socket.h"
#include "utility/netapp.h"
#include "board.h"
#include "utility/interface.h"
#include "../../setup.h"

#define FIRST_TIME_CONFIG_SET 0xAA

extern unsigned char * ptrFtcAtStartup;

extern volatile void (*FuncP2Bit0)(void);

//extern uint8_t CSpin;
//extern uint8_t ENpin;
//extern uint8_t IRQpin;
//*****************************************************************************
//
//! pio_init
//!
//! @param  none
//!
//! @return none
//!
//! @brief  Initialize the board's I/O
//
//*****************************************************************************    

void pio_init()
{
	//WLAN enable full DS
	//pinMode(ENpin, OUTPUT);
	//digitalWrite(ENpin, LOW);

	// WLAN enable
	WLAN_EN_DIR |= WLAN_EN_PIN;  // OUTPUT
	WLAN_EN_OUT &= ~WLAN_EN_PIN; // LOW

	//pinMode(IRQpin, INPUT_PULLUP);

	// SPI IRQ enable (INPUT)
	WLAN_IRQ_DIR &= ~WLAN_IRQ_PIN;

	// Configure the SPI CS
	//pinMode(CSpin, OUTPUT);
	//digitalWrite(CSpin, HIGH);

	// Configure the SPI CS
	WLAN_CS_SEL &= ~WLAN_CS_PIN;
	WLAN_CS_DIR |= WLAN_CS_PIN;   // OUTPUT
	WLAN_CS_OUT |= WLAN_CS_PIN;   // HIGH


	//delay(500);
}
//*****************************************************************************
//
//! ReadWlanInterruptPin
//!
//! @param  none
//!
//! @return none
//!
//! @brief  return wlan interrup pin
//
//*****************************************************************************

long ReadWlanInterruptPin(void)
{
	// Return the status of IRQ
	return (WLAN_IRQ_IN & WLAN_IRQ_PIN);
	//return digitalRead(IRQpin);
}

//*****************************************************************************
//
//! WlanInterruptEnable
//!
//! @param  none
//!
//! @return none
//!
//! @brief  Enable waln IrQ pin
//
//*****************************************************************************

void WlanInterruptEnable()
{
	WLAN_IRQ_IES |= WLAN_IRQ_PIN;
	WLAN_IRQ_IE |= WLAN_IRQ_PIN;
	WLAN_IFG_PORT &= ~WLAN_IRQ_PIN;

	if (!ReadWlanInterruptPin()) {
		IntSpiGPIOHandler();
	}



	/*attachInterrupt(IRQpin, IntSpiGPIOHandler, FALLING);
	if (!digitalRead(IRQpin)) {
		IntSpiGPIOHandler();
	}*/
}

//*****************************************************************************
//
//! WlanInterruptDisable
//!
//! @param  none
//!
//! @return none
//!
//! @brief  Disable waln IrQ pin
//
//*****************************************************************************

void WlanInterruptDisable()
{
	//detachInterrupt(IRQpin);
	WLAN_IRQ_IE &= ~WLAN_IRQ_PIN;
}

//*****************************************************************************
//
//! WriteWlanPin
//!
//! @param  val value to write to wlan pin
//!
//! @return none
//!
//! @brief  write value to wlan pin
//
//*****************************************************************************

void WriteWlanPin( unsigned char val )
{
	if (val) {
		//digitalWrite(ENpin, HIGH);
		WLAN_EN_OUT |= WLAN_EN_PIN;                 // WLAN_EN_PIN high
	} else {
		//digitalWrite(ENpin, LOW);
		WLAN_EN_OUT &= ~WLAN_EN_PIN;                // WLAN_EN_PIN low
	}
}
