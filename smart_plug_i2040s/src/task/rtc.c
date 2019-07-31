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
 * rtc.c
 *
 *  Created on: Mar 15, 2018
 *      Author: Lin Yi-Lie
 *
 */

#include "rtc.h"

RTC_STATUS rtcStatus;
int8_t *i2cRxBuffer, *i2cTxBuffer;
uint16_t i2cCount, i2cIndex;
EUSCI_B_I2C_initMasterParam i2cCfg;

void RTC_initMaster() {
	rtcStatus = RTC_IDLE;
	// Setting P1.6 and P1.7 as I2C pins
	GPIO_setAsPeripheralModuleFunctionInputPin(GPIO_PORT_P1,
	GPIO_PIN6 | GPIO_PIN7, GPIO_PRIMARY_MODULE_FUNCTION);
	i2cCfg.selectClockSource = EUSCI_B_I2C_CLOCKSOURCE_SMCLK; //SMCLK=4096kHz
	i2cCfg.i2cClk = 4096000; //source:SMCLK=4096kHz
	i2cCfg.dataRate = EUSCI_B_I2C_SET_DATA_RATE_400KBPS; // Desired I2C Clock of 400kHz
	i2cCfg.byteCounterThreshold = 0; // No byte counter threshold
	i2cCfg.autoSTOPGeneration = EUSCI_B_I2C_NO_AUTO_STOP;
	// Setting up I2C communication at 400kHz using SMCLK
	EUSCI_B_I2C_initMaster(EUSCI_B0_BASE, &i2cCfg);
	// Enable the module for operation
	EUSCI_B_I2C_enable(EUSCI_B0_BASE);
	// Settings slave address
	EUSCI_B_I2C_setSlaveAddress(EUSCI_B0_BASE, DS3231_I2C_ADDRESS);
	//Reset Flag,Clear RX NACK
	EUSCI_B_I2C_clearInterrupt(EUSCI_B0_BASE,
	EUSCI_B_I2C_RECEIVE_INTERRUPT0 | EUSCI_B_I2C_NAK_INTERRUPT);
	// Enable needed I2C interrupts
	EUSCI_B_I2C_enableInterrupt(EUSCI_B0_BASE, EUSCI_B_I2C_TRANSMIT_INTERRUPT0 |
	EUSCI_B_I2C_NAK_INTERRUPT);
}

void RTC_writeTimeToSlave(RTC_time_t* wTime) {
	rtcStatus = RTC_WRITE;
	i2cTxBuffer = (int8_t*) wTime;
	i2cIndex = 0;
	i2cCount = sizeof(RTC_time_t);
	EUSCI_B_I2C_masterSendMultiByteStart(EUSCI_B0_BASE, DS3231_SEC_ADDR);
}
void RTC_readTimeFromSlave(RTC_time_t* rTime) {
	rtcStatus = RTC_READ;
	i2cRxBuffer = (int8_t*) rTime;
	i2cIndex = 0;
	i2cCount = 0; //count=RTC size
	// Stop receiving data if last byte
	EUSCI_B_I2C_enableInterrupt(EUSCI_B0_BASE,
	EUSCI_B_I2C_RECEIVE_INTERRUPT0);
	EUSCI_B_I2C_masterSendMultiByteStart(EUSCI_B0_BASE, DS3231_SEC_ADDR);
}

RTC_STATUS RTC_isBusy() {
	return rtcStatus;
}

#pragma vector=USCI_B0_VECTOR
__interrupt void USCIB0_ISR(void) {
	switch (__even_in_range(UCB0IV, USCI_I2C_UCBIT9IFG)) {
	case USCI_NONE:
		break;
	case USCI_I2C_UCALIFG:
		break;
	case USCI_I2C_UCNACKIFG:
		EUSCI_B_I2C_masterSendStart(EUSCI_B0_BASE);
		break;
	case USCI_I2C_UCSTTIFG:
		break;
	case USCI_I2C_UCSTPIFG:
		break;
	case USCI_I2C_UCRXIFG3:
		break;
	case USCI_I2C_UCTXIFG3:
		break;
	case USCI_I2C_UCRXIFG2:
		break;
	case USCI_I2C_UCTXIFG2:
		break;
	case USCI_I2C_UCRXIFG1:
		break;
	case USCI_I2C_UCTXIFG1:
		break;
	case USCI_I2C_UCRXIFG0:
		// Send stop if second to last byte
		if (i2cIndex == i2cCount - 2) {
			EUSCI_B_I2C_masterReceiveMultiByteStop(EUSCI_B0_BASE);
		}
		i2cRxBuffer[i2cIndex] = EUSCI_B_I2C_masterReceiveMultiByteNext(
		EUSCI_B0_BASE);
		if (i2cIndex == i2cCount - 1) {
			// Stop receiving data if last byte
			EUSCI_B_I2C_disableInterrupt(EUSCI_B0_BASE,
			EUSCI_B_I2C_RECEIVE_INTERRUPT0);
			rtcStatus = RTC_IDLE;
		}
		i2cIndex++;
		break;
	case USCI_I2C_UCTXIFG0:
		if (i2cCount) {
			EUSCI_B_I2C_masterSendMultiByteNext(EUSCI_B0_BASE,
					i2cTxBuffer[i2cIndex++]);
			i2cCount--;
		} else {
			//RTC
			if (rtcStatus == RTC_READ) {
				i2cCount = sizeof(RTC_time_t);
				i2cIndex = 0;
				// Clear needed I2C interrupts
				EUSCI_B_I2C_clearInterrupt(EUSCI_B0_BASE,
				EUSCI_B_I2C_RECEIVE_INTERRUPT0);
				//Start receiving data
				EUSCI_B_I2C_masterReceiveStart(EUSCI_B0_BASE);
			} else {
				EUSCI_B_I2C_masterSendMultiByteStop(EUSCI_B0_BASE);
				rtcStatus = RTC_IDLE;
			}
		}
		break;
	case USCI_I2C_UCBCNTIFG:
		break;
	case USCI_I2C_UCCLTOIFG:
		break;
	case USCI_I2C_UCBIT9IFG:
		break;
	default:
		break;
	}
}

