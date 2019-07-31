/* --COPYRIGHT--,BSD
 * Copyright (c) 2017, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * --/COPYRIGHT--*/
/*
 * serial.c
 *
 *  Created on: Oct 23, 2018
 *      Author: Lin Yi-Lie
 */

#include "serial.h"

typedef enum SERIAL_RX_STATUS {
	RX_PREAMBLE, RX_RECEIVING, RX_EOL
} serialRxStatus_t;

int8_t *serialTxBuffer;
int8_t serialRxBuffer[SERIAL_RX_BUF_SIZE];
serialRxStatus_t serialRxStatus;
uint16_t serialTxCount, serialRxCount, serialTxIndex, serialRxIndex;
EUSCI_A_UART_initParam uartCfg;

void SERIAL_init() {
	serialTxCount = 0;
	serialRxStatus = RX_PREAMBLE;
	// Settings P1.2 and P1.3 as UART pins.
	GPIO_setAsPeripheralModuleFunctionInputPin(GPIO_PORT_P1,
	GPIO_PIN2 | GPIO_PIN3,
	GPIO_PRIMARY_MODULE_FUNCTION);
	//9600,8,n,1,SMCLK
	uartCfg.selectClockSource = EUSCI_A_UART_CLOCKSOURCE_SMCLK;
	uartCfg.clockPrescalar = 26; //UCBRW value
	uartCfg.uartMode = EUSCI_A_UART_MODE;
	uartCfg.numberofStopBits = EUSCI_A_UART_ONE_STOP_BIT;
	uartCfg.msborLsbFirst = EUSCI_A_UART_LSB_FIRST;
	uartCfg.parity = EUSCI_A_UART_NO_PARITY;
	uartCfg.overSampling = EUSCI_A_UART_OVERSAMPLING_BAUDRATE_GENERATION;
	uartCfg.firstModReg = 0x0a; //UCBRFx value
	uartCfg.secondModReg = 0xb5; //UCBRSx value
	EUSCI_A_UART_init(EUSCI_A0_BASE, &uartCfg);
	EUSCI_A_UART_enable(EUSCI_A0_BASE);
	//Start RX ISR
	SERIAL_startRx();
	//Set TX flag
	UCA0IFG |= UCTXIFG;
}

int16_t SERIAL_isBusy() {
	return serialTxCount;
}

void SERIAL_startRx() {
	serialRxStatus = RX_PREAMBLE;
	//Clear RX flag
	EUSCI_A_UART_clearInterrupt(EUSCI_A0_BASE,
	EUSCI_A_UART_RECEIVE_INTERRUPT_FLAG);
	EUSCI_A_UART_enableInterrupt(EUSCI_A0_BASE,
	EUSCI_A_UART_RECEIVE_INTERRUPT);
}

void SERIAL_send(int8_t* data, uint16_t length) {
	//serialTxCount==1 && length!=0
	if (!serialTxCount && length) {
		serialTxBuffer = data;
		serialTxCount = length;
		serialTxIndex = 0;
		//start sending
		EUSCI_A_UART_enableInterrupt(EUSCI_A0_BASE,
		EUSCI_A_UART_TRANSMIT_INTERRUPT);
	}
}

#pragma vector=USCI_A0_VECTOR
__interrupt void EUSCI_A0_ISR(void) {
	/*
	 *eUSCI_A interrupt vector value
	 * 00h = No interrupt pending
	 * 02h = Interrupt Source: Receive buffer full; Interrupt Flag: UCRXIFG; Interrupt
	 * Priority: Highest
	 * 04h = Interrupt Source: Transmit buffer empty; Interrupt Flag: UCTXIFG
	 * 06h = Interrupt Source: Start bit received; Interrupt Flag: UCSTTIFG
	 * 08h = Interrupt Source: Transmit complete; Interrupt Flag: UCTXCPTIFG;
	 * Interrupt Priority: Lowest
	 */
	switch (__even_in_range(UCA0IV, USCI_UART_UCTXCPTIFG)) {
	case USCI_NONE: {
	}
		break;
	case USCI_UART_UCRXIFG: {
		UCA0IFG &= ~UCRXIFG;
		uint8_t rxBuf = UCA0RXBUF;
		switch (serialRxStatus) {
		case RX_PREAMBLE:
			if (SERIAL_PREAMBLE == rxBuf) {
				serialRxStatus = RX_RECEIVING;
				serialRxIndex = 0;
				serialRxCount = sizeof(SERIAL_rxPackage_t);
			}
			break;
		case RX_RECEIVING:
			if (SERIAL_SYNC == rxBuf) {
				serialRxStatus = RX_PREAMBLE;
			} else {
				serialRxBuffer[serialRxIndex++] = rxBuf;
#if(SERIAL_RX_WITH_EOL)
				if (!--serialRxCount) {
					serialRxStatus = RX_EOL;
				}
#else
				if (!--serialRxCount) {
					EUSCI_A_UART_disableInterrupt(EUSCI_A0_BASE,
					EUSCI_A_UART_RECEIVE_INTERRUPT);
					backGroundStatus.serv_serial = 1;
				}
#endif
			}
			break;
		case RX_EOL: //for MSG with EOL
			if (SERIAL_RX_EOL == rxBuf) {
				EUSCI_A_UART_disableInterrupt(EUSCI_A0_BASE,
				EUSCI_A_UART_RECEIVE_INTERRUPT);
				backGroundStatus.serv_serial = 1;
			}
			break;
		}
		break; //RX ISR
	}
	case USCI_UART_UCTXIFG: {
		UCA0TXBUF = serialTxBuffer[serialTxIndex++];
		if (!--serialTxCount) {
			EUSCI_A_UART_disableInterrupt(EUSCI_A0_BASE,
			EUSCI_A_UART_TRANSMIT_INTERRUPT);
		}
		break;
	}
	case USCI_UART_UCSTTIFG: {
	}
		break;
	case USCI_UART_UCTXCPTIFG: {
	}
		break;
	} //switch
} //ISR
