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
 * app.c
 *  Created on: Mar 1, 2018
 *      Author: Lin Yi-Lie
 *
 */

#include "app.h"

#define logInitSeg(flashPtr) 			FlashCtl_eraseSegment((uint8_t*)flashPtr)
#define logCreate(headerPtr,flashPtr,size) 	FlashCtl_write8((uint8_t*)headerPtr,(uint8_t*)flashPtr,size)
#define logWrite(dataPtr,flashPtr,size)  FlashCtl_write8((uint8_t*)dataPtr, (uint8_t*)flashPtr,size)

#define uint2BCD(x)		(((x / 10) << 4) | (x % 10))
#define BCD2uint(x)		(((x & 0xF0) >> 4)*10 + (x & 0x0F))

void sysInit();
void softwareRTCPlus1sec(RTC_time_t* time);

/*
 * Global variable
 */
int8_t appVersionStr[] = _APP_VERSION_;
const uint16_t flashSegStart[] = { 0xFC00, 0xF800, 0xF400, 0xF000, 0xEC00 };
const uint16_t flashSegEnd[] = { 0xFFFF, 0xFBFF, 0xF7FF, 0xF3FF, 0xEFFF };
//BackGround Status
taskStatus_t backGroundStatus;
//System time structure
RTC_time_t sysTime = DEFAULT_RTC_TIME;
RTC_time_t sysTimeTmp = DEFAULT_RTC_TIME;  //for time cache
		RTC_time_t sysNextTime = DEFAULT_RTC_TIME;//Updating by RTC
//System EPower info
		EPOWER_powerInfo_t sysEPowerInfo,
sysEPowerInfoTmp;
uint32_t energyPerDay = 0;
uint32_t energyPerDayTmp = 0;
//System log info
logInfo_t sysLogInfo;
//System schedule table
schedule_t sysSchedule[APP_SCHEDULE_TABLE_LENGTH] = { 0 };
//password
uint8_t passwd[] = APP_DEFAULT_PASSWD;
//bluetooth timeout
uint8_t sysBLETimeout = 0;
//GPIO SW BT lock var
uint8_t btnSWLock = 0;
uint8_t btnBTLock = 0;
/*
 * Main function
 */
void main(void) {
	WDTCTL = WDTPW | WDTHOLD;	// Stop WDT timer
	sysInit(); //System initialization
	_enable_interrupt(); //Enable GIE , System run
#if  (_SUPPORT_RTC)
	RTC_writeTimeToSlave(&sysTime); //Writing default time to RTC module
#endif
	//start WDT timer
	WDT_resetTimer(WDT_BASE);
#if(APP_SOFT_RTC)
	//Software RTC , ACLK,32k mode
	WDT_initIntervalTimer(WDT_BASE, WDT_CLOCKSOURCE_ACLK,
	WDT_CLOCKDIVIDER_32K);
#else
	//Hardware RTC(external),ACLK,8192 mode
	WDT_initIntervalTimer(WDT_BASE, WDT_CLOCKSOURCE_ACLK,
			WDT_CLOCKDIVIDER_8192);
#endif
	SFR_enableInterrupt(SFR_WATCHDOG_INTERRUPT);
	WDT_start(WDT_BASE);
	//Super loop
	while (1) {
		_enable_interrupt();
		//if SERIAL/UART receive a package
		if (backGroundStatus.serv_serial) {
			serialService();
			backGroundStatus.serv_serial = 0;
		} //if backGroundStatus.serial
		  //RTC one second event
		if (backGroundStatus.serv_rtc) {
			rtcService();
			backGroundStatus.serv_rtc = 0;
		} // if backGroundStatus.rtc
		  //if sd24 convert finished
		if (backGroundStatus.serv_epower) {
			epowerService();
			backGroundStatus.serv_epower = 0;
		} // epower background

		//BLE STATUS CHECK
		if (GPIO_getInputPinValue(BLUETOOTH_PORT,
		BLUETOOTH_EN) == BLUETOOTH_EN_ON) {
			//if BT disconnect , clear flag
			if (GPIO_getInputPinValue(BLUETOOTH_PORT,
			BLUETOOTH_STAT) == BLUETOOTH_STAT_OFF) {
				if (backGroundStatus.passwdPass) {
					sysBLETimeout = 10;
					backGroundStatus.passwdPass = 0;
				}
			}
		}
	} //idle loop
} //main

void epowerService() {
#if  (_SUPPORT_EPOWER )
	EPOWER_calc();
	sysEPowerInfoTmp = EPOWER_getInfo();
	//power , MA2 filter , (old_p+current_p)/2
	sysEPowerInfo.currentRms += sysEPowerInfoTmp.currentRms;
	sysEPowerInfo.currentRms >>= 1; // /2
	sysEPowerInfo.voltRms += sysEPowerInfoTmp.voltRms;
	sysEPowerInfo.voltRms >>= 1; // /2
	sysEPowerInfo.powerActive += sysEPowerInfoTmp.powerActive;
	sysEPowerInfo.powerActive >>= 1; // /2
	sysEPowerInfo.powerApperent += sysEPowerInfoTmp.powerApperent;
	sysEPowerInfo.powerApperent >>= 1; // /2
	sysEPowerInfo.powerFactor += sysEPowerInfoTmp.powerFactor;
	sysEPowerInfo.powerFactor >>= 1; // /2
	sysEPowerInfo.powerFreq += sysEPowerInfoTmp.powerFreq;
	sysEPowerInfo.powerFreq >>= 1; // /2
	EPOWER_startConversion(); //Start next conversion
	//Current protect
//	if (sysEPowerInfo.currentRms > APP_CURRENT_LIMIT) {
//		GPIO_setOutputLowOnPin(RELAY_PORT, RELAY_PIN); //turn off
//	}
#endif //epower
}

void rtcService() {
	_NOP();
	WDT_hold(WDT_BASE);
	sysTimeTmp = sysTime; //clone time
	WDT_start(WDT_BASE);
	switch (sysTimeTmp.sec) {
	case 0x00:
		//schedule task
		for (int8_t i = APP_SCHEDULE_TABLE_LENGTH - 1; i >= 0; i--) {
			if (sysSchedule[i].day == sysTimeTmp.day) {
				if (sysSchedule[i].hour == sysTimeTmp.hour
						&& sysSchedule[i].min == sysTimeTmp.min) {
					if (sysSchedule[i].status == RELAY_PIN_ON)
						GPIO_setOutputHighOnPin(RELAY_PORT, RELAY_PIN); //turn on
					else
						GPIO_setOutputLowOnPin(RELAY_PORT, RELAY_PIN); // turn off
				}
				i = -1; //break for
			}
		}
		break;
	}
#if  (_SUPPORT_EPOWER )
	energyPerDayTmp = sysEPowerInfo.powerActive; //uint16 Q6 format to uint32 Q6
	energyPerDay += energyPerDayTmp; //add 1 second energy
#endif
	/* if one day passed */
	uint8_t today = ((sysTimeTmp.date & 0xF0) >> 4) * 10
			+ (sysTimeTmp.date & 0x0F);	//convert BCD to  DEC
	if (sysLogInfo.date != today || backGroundStatus.rtcSet) {
		_NOP();
		energyPerDay /= 3600000u; //convert w-sec to kw-hr
		//WRITE LOG to FLASH
		//write to flash , convert uint32_t Q6 to uint16_t Q6
#if(_SUPPORT_LOG )
		WDT_hold(WDT_BASE);
		uint16_t data = (uint16_t) energyPerDay;
		logWrite(&data, sysLogInfo.dataFlashPtr++, sizeof(uint16_t));
		WDT_start(WDT_BASE);
#endif
		sysLogInfo.date++;
		energyPerDay = 0;
	} // if one day passed

	/* if time modify */
	if (sysLogInfo.header.mon != sysTimeTmp.mon || backGroundStatus.rtcSet) {
		if (sysLogInfo.headerNum >= LOG_HEADER_MAX_NUM) {
			//new segment
			sysLogInfo.headerNum = 0;
			if (sysLogInfo.segNum >= LOG_SEG_MAX_NUM)
				sysLogInfo.segNum = 0;
			else
				sysLogInfo.segNum++;
			//INIT SEG
			sysLogInfo.headerFlashPtr =
					(uint8_t*) flashSegStart[sysLogInfo.segNum];
#if (_SUPPORT_LOG)
			WDT_hold(WDT_BASE);
			logInitSeg(sysLogInfo.headerFlashPtr);
			WDT_start(WDT_BASE);
#endif
		} else {
			sysLogInfo.headerNum++;
			sysLogInfo.headerFlashPtr += sizeof(logHeader_t)
					+ LOG_HEADER_MAX_SIZE;
		}
		sysLogInfo.header.mon = sysTimeTmp.mon;
		sysLogInfo.header.year = sysTimeTmp.year;
		sysLogInfo.date = ((sysTimeTmp.date & 0xF0) >> 4) * 10
				+ (sysTimeTmp.date & 0x0F);	//convert BCD to  DEC
		sysLogInfo.dataFlashPtr = (uint16_t *) (sysLogInfo.headerFlashPtr
				+ sizeof(logHeader_t));
		sysLogInfo.dataFlashPtr += sysLogInfo.date - 1;
		//new segment
#if (_SUPPORT_LOG)
		WDT_hold(WDT_BASE);
		logCreate(&sysLogInfo.header, sysLogInfo.headerFlashPtr,
				sizeof(logHeader_t));
		WDT_start(WDT_BASE);
#endif //ENDIF LOG
		backGroundStatus.rtcSet = 0; //reset flag
	} // if rtc_set

#if(APP_HMI_EN)
	/* BLE timeout , timeout>0 -> EN */
	if (sysBLETimeout != 0) {
		//timeout==1 -> time's up
		if (sysBLETimeout == 1) {
			//time's up , disable BT
			GPIO_setOutputLowOnPin(BLUETOOTH_PORT, BLUETOOTH_EN);
			sysBLETimeout = 0;
		} else {
			//timeout -= 1
			sysBLETimeout--;
		}
	}
#endif

	//reset btnXXlock
	static int btnSWLockCount = 0;
	if (btnSWLockCount) {
		btnSWLockCount--;
		if (!btnSWLockCount) {
			btnSWLock = 0;
		}
	} else {
		if (btnSWLock)
			btnSWLockCount = btnSWLock;
	}

	static int btnBTLockCount = 0;
	if (btnBTLockCount) {
		btnBTLockCount--;
		if (!btnBTLockCount) {
			btnBTLock = 0;
		}
	} else {
		if (btnBTLock)
			btnBTLockCount = btnBTLock;
	}

} // RTC task

void serialService() {
#if (_SUPPORT_SERIAL)
	SERIAL_rxPackage_t* rxPackage = (SERIAL_rxPackage_t*) serialRxBuffer;
	static int8_t reply[20];
	static uint8_t index;
	index = 0;
//password passed?
	if (!backGroundStatus.passwdPass) {
		//password check
		if (rxPackage->cmd == RX_CMD_PASSWD) {
			uint8_t* ptr = (uint8_t*) &rxPackage->arg0; //ptr <= &arg0
			index = sizeof(passwd) - 1; // -1 <= exclude '\0'
			while (--index) {
				if ((*(ptr + index)) != passwd[index]) {
					break; //not passed
				} //if
			} //while
			if (!index) //if index == 0 , password correct
			{
				//password correct
				backGroundStatus.passwdPass = 1;
				sysBLETimeout = 0; // disable BT close timer
			}
		} //if
	} else {
		switch (rxPackage->cmd) {
		case RX_CMD_GET_EINFO: {
			reply[index++] = sysEPowerInfo.voltRms; //L
			reply[index++] = sysEPowerInfo.voltRms >> 8; //H
			reply[index++] = sysEPowerInfo.currentRms; //L
			reply[index++] = sysEPowerInfo.currentRms >> 8; //H
			reply[index++] = sysEPowerInfo.powerActive; //L
			reply[index++] = sysEPowerInfo.powerActive >> 8; //H
			reply[index++] = sysEPowerInfo.powerFreq; //L
			reply[index++] = sysEPowerInfo.powerFreq >> 8; //H
			reply[index++] = sysEPowerInfo.powerFactor; //L
			reply[index++] = sysEPowerInfo.powerFactor >> 8; //H
			SERIAL_send(reply, index);
			break;
		} //RX_CMD_GET_EINFO
		case RX_CMD_GET_TIME: {
			reply[index++] = sysTime.sec;
			reply[index++] = sysTime.min;
			reply[index++] = sysTime.hour;
			reply[index++] = sysTime.day;
			reply[index++] = sysTime.date;
			reply[index++] = sysTime.mon;
			reply[index++] = sysTime.year;
			SERIAL_send(reply, index);
			break;
		} //RX_CMD_GET_TIME
		case RX_CMD_SET_TIME: {
			WDT_hold(WDT_BASE);
			sysNextTime.sec = rxPackage->arg0;
			sysNextTime.min = rxPackage->arg1;
			sysNextTime.hour = rxPackage->arg2;
			sysNextTime.day = rxPackage->arg3;
			sysNextTime.date = rxPackage->arg4;
			sysNextTime.mon = rxPackage->arg5;
			sysNextTime.year = rxPackage->arg6;
#if  (_SUPPORT_RTC)
			RTC_writeTimeToSlave(&sysNextTime); //Writing default time to RTC module
#endif
			WDT_start(WDT_BASE);
			backGroundStatus.rtcSet = 1;
			reply[index++] = 'O';
			reply[index++] = 'K';
			SERIAL_send(reply, index);
			break;
		} //RX_CMD_SET_TIME
		case RX_CMD_GET_LOG_NUM: {
			reply[index++] = LOG_SEG_NUM;
			SERIAL_send(reply, index);
			break;
		}
		case RX_CMD_GET_LOG: {
			if (rxPackage->arg0 >= LOG_SEG_START_NUM
					&& rxPackage->arg0 <= LOG_SEG_MAX_NUM) {
				SERIAL_send((int8_t*) flashSegStart[rxPackage->arg0], 1024); //1024 byte a segment
			} else {
				reply[index++] = 'N';
				reply[index++] = 'O';
				SERIAL_send(reply, index);
			}
			break;
		} //RX_CMD_GET_LOG
		case RX_CMD_GET_SCH_NUM: {
			reply[index++] = APP_SCHEDULE_TABLE_LENGTH;
			SERIAL_send(reply, index);
			break;
		} //RX_CMD_GET_SCH_NUM
		case RX_CMD_GET_SCH: {
			if (rxPackage->arg0 <= (APP_SCHEDULE_TABLE_LENGTH - 1)) {
				reply[index++] = sysSchedule[rxPackage->arg0].min;
				reply[index++] = sysSchedule[rxPackage->arg0].hour;
				reply[index++] = sysSchedule[rxPackage->arg0].day;
				reply[index++] = sysSchedule[rxPackage->arg0].status;
				SERIAL_send(reply, index);
			} else {
				reply[index++] = 'N';
				reply[index++] = 'O';
				SERIAL_send(reply, index);
			}
			break;
		} //RX_CMD_GET_SCH
		case RX_CMD_SET_SCH: {
			if (rxPackage->arg0 < ( APP_SCHEDULE_TABLE_LENGTH - 1)) {
				sysSchedule[rxPackage->arg0].min = rxPackage->arg1;
				sysSchedule[rxPackage->arg0].hour = rxPackage->arg2;
				sysSchedule[rxPackage->arg0].day = rxPackage->arg3;
				sysSchedule[rxPackage->arg0].status = rxPackage->arg4;
				reply[index++] = 'O';
				reply[index++] = 'K';
				SERIAL_send(reply, index);
			} else {
				reply[index++] = 'N';
				reply[index++] = 'O';
				SERIAL_send(reply, index);
			}
			break;
		} //RX_CMD_SET_SCH
		case RX_CMD_CLOSE_BT: {
			//BTN2=> Bluetooth EN
#if(APP_HMI_EN)
			GPIO_setOutputLowOnPin(BLUETOOTH_PORT, BLUETOOTH_EN);
#endif
			backGroundStatus.passwdPass = 0; //reset flag
			break;
		} //RX_CMD_CLOSE_BT
		case RX_CMD_GET_SW_STAT: {
			reply[index++] = GPIO_getInputPinValue(RELAY_PORT, RELAY_PIN);
			SERIAL_send(reply, index);
			break;
		} //RX_CMD_GET_SW_STAT
		case RX_CMD_SET_SW_STAT: {
			switch (rxPackage->arg0) {
			case RELAY_PIN_ON:
				GPIO_setOutputHighOnPin(RELAY_PORT, RELAY_PIN);
				reply[index++] = 'O';
				reply[index++] = 'K';
				SERIAL_send(reply, index);
				break;
			case RELAY_PIN_OFF:
				GPIO_setOutputLowOnPin(RELAY_PORT, RELAY_PIN);
				reply[index++] = 'O';
				reply[index++] = 'K';
				SERIAL_send(reply, index);
				break;
			default:
				reply[index++] = 'N';
				reply[index++] = 'O';
				SERIAL_send(reply, index);
				break;
			} //inner switch
			break;
		} //RX_CMD_SET_SW_STAT
		case RX_CMD_GET_ID: {
			reply[index++] = APP_DEVICE_ID;
			SERIAL_send(reply, index);
			break;
		}
		case RX_CMD_GET_VERSION: {
			SERIAL_send(appVersionStr, sizeof(appVersionStr) - 1);
			break;
		}
		default:
			reply[index++] = 'N';
			reply[index++] = 'O';
			SERIAL_send(reply, index);
			break;
		}

	} //password check
// finally,start next RX
	SERIAL_startRx();
#endif //serial
}

/*
 * System Initialization :
 */
void sysInit() {
//GPIO reset
	GPIO_clearInterrupt(GPIO_PORT_P1, GPIO_PIN_ALL8);
	GPIO_clearInterrupt(GPIO_PORT_P2, GPIO_PIN_ALL8);
	GPIO_setAsInputPin(GPIO_PORT_P1, GPIO_PIN_ALL8);
	GPIO_setAsInputPin(GPIO_PORT_P2, GPIO_PIN_ALL8);
	GPIO_setOutputLowOnPin(GPIO_PORT_P1, GPIO_PIN_ALL8);
	GPIO_setOutputLowOnPin(GPIO_PORT_P2, GPIO_PIN_ALL8);
	/* Clock system initializing
	 * DCO=16Mhz
	 * MCLK=16Mhz
	 * SMCLK=4Mhz
	 */
	CS_setupDCO(CS_INTERNAL_RESISTOR);
	CS_initClockSignal(CS_MCLK, CS_CLOCK_DIVIDER_1);
	CS_initClockSignal(CS_SMCLK, CS_CLOCK_DIVIDER_4);

//Serial
#if(_SUPPORT_SERIAL)
	SERIAL_init();	//9600,8,n,1,SMCLK
#endif //ENDIF SERIAL
//EPOWER
	energyPerDay = 0;
#if( _SUPPORT_EPOWER)
	EPOWER_init();
#endif //ENDIF EPOWER
//RTC
#if (_SUPPORT_RTC && !APP_SOFT_RTC)
	RTC_initMaster();
#endif //ENDIF RTC
//LOG
#if (_SUPPORT_LOG)
// MCLK for Flash Timing Generator
// Flash clock will run at ~390kHz. Datasheet recommends 257kHz - 476kHz
	FlashCtl_setupClock(390095, 16384000, FLASHCTL_MCLK);
#endif //ENDIF LOG
	sysTimeTmp = sysTime;
	sysLogInfo.headerNum = 0;
	sysLogInfo.segNum = LOG_SEG_START_NUM;
	sysLogInfo.header.mon = sysTimeTmp.mon;
	sysLogInfo.header.year = sysTimeTmp.year;
	sysLogInfo.headerFlashPtr = (uint8_t*) flashSegStart[sysLogInfo.segNum];
	sysLogInfo.dataFlashPtr = (uint16_t *) (sysLogInfo.headerFlashPtr
			+ sizeof(logHeader_t));
//new segment
#if (_SUPPORT_LOG)
	logInitSeg(sysLogInfo.headerFlashPtr);
	logCreate(&sysLogInfo.header, sysLogInfo.headerFlashPtr,
			sizeof(logHeader_t));
#endif //ENDIF LOG
	sysLogInfo.date = ((sysTimeTmp.date & 0xF0) >> 4) * 10
			+ (sysTimeTmp.date & 0x0F);	//convert BCD to  DEC
	sysLogInfo.dataFlashPtr += sysLogInfo.date - 1;

	/*GPIO init */
// Turn on power led
	GPIO_setAsOutputPin(LED_POWER_PORT, LED_POWER_PIN);
	GPIO_setOutputLowOnPin(LED_POWER_PORT, LED_POWER_PIN);
//HMI(button only)
#if(APP_HMI_EN)
	GPIO_setAsInputPin(BUTTON_PORT, BUTTON_1_PIN + BUTTON_2_PIN);
	GPIO_selectInterruptEdge(BUTTON_PORT, BUTTON_1_PIN + BUTTON_2_PIN,
			GPIO_HIGH_TO_LOW_TRANSITION);
	GPIO_setOutputLowOnPin(BUTTON_PORT, BUTTON_1_PIN + BUTTON_2_PIN);
	GPIO_clearInterrupt(BUTTON_PORT, BUTTON_1_PIN + BUTTON_2_PIN);
	GPIO_enableInterrupt(BUTTON_PORT, BUTTON_1_PIN + BUTTON_2_PIN);
	_delay_cycles(1600000); //delay 100ms for CAP charge
#endif
//Bluetooth
	GPIO_setAsOutputPin(BLUETOOTH_PORT, BLUETOOTH_EN);
	GPIO_setAsInputPin(BLUETOOTH_PORT, BLUETOOTH_STAT);
#if(APP_HMI_EN)
	GPIO_setOutputLowOnPin(BLUETOOTH_PORT, BLUETOOTH_EN);
#else
	GPIO_setOutputHighOnPin(BLUETOOTH_PORT, BLUETOOTH_EN);
#endif
//Relay
	GPIO_setOutputLowOnPin(RELAY_PORT, RELAY_PIN);	// turn off relay
	GPIO_setAsOutputPin(RELAY_PORT, RELAY_PIN);
//Reset power info
	sysEPowerInfo.currentRms = 0;
	sysEPowerInfo.voltRms = 0;
	sysEPowerInfo.powerActive = 0;
	sysEPowerInfo.powerApperent = 0;
	sysEPowerInfo.powerFactor = 0;
	sysEPowerInfo.powerFreq = 0;
	sysEPowerInfoTmp = sysEPowerInfo;
//Reset status flag
	*(uint16_t*) &backGroundStatus = 0;
//Reset btnXXlock
	btnSWLock = 0;
	btnBTLock = 0;
} // SYS_Init

#pragma vector=PORT1_VECTOR
__interrupt void PORT1_ISR(void)

{
	switch (__even_in_range(P1IV, 0x10)) {
	case 0x00:
		break;               // No interrupt
	case 0x02:               // P1.0 interrupt
		//BTN1 RELAY TOGGLE
		if (P1IFG & BIT0) {
			P1IFG &= ~BIT0;
			break;
		}
		if (!btnSWLock) {
			GPIO_toggleOutputOnPin(RELAY_PORT, RELAY_PIN);
			btnSWLock = 1;
			btnBTLock = 1;//lock BT
		}
		break;
	case 0x04: 				// P1.1 interrupt
		break;
	case 0x06:
		break;               // P1.2 interrupt
	case 0x08:
		break;               // P1.3 interrupt
	case 0x0A:               // P1.4 interrupt
		if (P1IFG & BIT4) {
			P1IFG &= ~BIT4;
			break;
		}
		//BTN2=>toggle bluetooth EN
		if (!btnBTLock) {
			if (GPIO_getInputPinValue(BLUETOOTH_PORT, BLUETOOTH_EN)) {
				GPIO_setOutputLowOnPin(BLUETOOTH_PORT, BLUETOOTH_EN);
				sysBLETimeout = 0;
			} else {
				GPIO_setOutputHighOnPin(BLUETOOTH_PORT, BLUETOOTH_EN);
				sysBLETimeout = 20;
			}
			btnBTLock = 1;
		}
		break;
	case 0x0C:
		break;               // P1.5 interrupt
	case 0x0E:
		break;               // P1.6 interrupt
	case 0x10:
		break;               // P1.7 interrupt
	}
}               //PORT1 ISR

#pragma vector=PORT2_VECTOR
__interrupt void PORT2_ISR(void)

{
	switch (__even_in_range(P2IV, 0x10)) {
	case 0x00:
		break;               // No interrupt
	case 0x02:                      // P2.0 interrupt
		break;
	case 0x04:
		break;               // P2.1 interrupt
	case 0x06:
		break;               // P2.2 interrupt
	case 0x08:
		break;               // P2.3 interrupt
	case 0x0A:
		break;               // P2.4 interrupt
	case 0x0C:
		break;               // P2.5 interrupt
	case 0x0E:
		break;               // P2.6 interrupt
	case 0x10:
		break;               // P2.7 interrupt
	}
}               //PORT2 ISR

/*
 * WDT ISR function
 */
#pragma vector=WDT_VECTOR
__interrupt void WDT_ISR(void) {
	_NOP();
//RTC Task
#if  (_SUPPORT_RTC)
#if(APP_SOFT_RTC)
//SW RTC
	softwareRTCPlus1sec(&sysTime);
	backGroundStatus.serv_rtc = 1;
#else
//HW RTC
	if (!RTC_isBusy()) {
		if (sysTime.sec != sysNextTime.sec) {
			sysTime = sysNextTime;
			backGroundStatus.serv_rtc = 1;
		}
		RTC_readTimeFromSlave(&sysNextTime);
	}
#endif
#endif
}

void softwareRTCPlus1sec(RTC_time_t* mTime) {
	static uint8_t yearDays[12] = { 31, 29, 31, 30, 31, 30, 31, 31, 30, 31, 30,
			31 };
	static uint8_t tmpHour = 0;
	static uint16_t y = 0;
	static uint8_t tmpDate = 0;
	static uint8_t tmpMon = 0;
	//sec
	mTime->sec++;
	if ((mTime->sec & 0x0F) >= 0x0A) {
		mTime->sec &= 0xF0;
		mTime->sec += 0x10;
		if ((mTime->sec & 0xF0) >= 0x60) {
			mTime->sec = 0x00;
			//min
			mTime->min++;
			if ((mTime->min & 0x0F) >= 0x0A) {
				mTime->min &= 0xF0;
				mTime->min += 0x10;
				if ((mTime->min & 0xF0) >= 0x60) {
					mTime->min = 0x00;
					//hour
					tmpHour = BCD2uint(mTime->hour);
					tmpHour++;
					if (tmpHour >= 24) {
						mTime->hour = 0x00;
						//day
						if (mTime->day >= 0x07) {
							mTime->day = 0x01;
						} else {
							mTime->day++;
						}
						//leap year
						y = RTC_DS3231_YEAR_OFFSET
								+ ((uint16_t) BCD2uint(mTime->year));
						if (y / 4 || (y / 100 || !(y / 400))) {
							//leap year
							yearDays[3] = 29;
						} else {
							//not leap year
							yearDays[3] = 28;
						}
						//date,month and year
						tmpDate = BCD2uint(mTime->date);
						tmpMon = BCD2uint(mTime->mon);
						tmpDate++;
						if (tmpDate > yearDays[tmpMon - 1]) {
							mTime->date = 0x01;
							tmpMon++;
							if (tmpMon > 12) {
								mTime->mon = 0x01;
								mTime->year = uint2BCD(BCD2uint(mTime->year)+1);
							} else {
								mTime->mon = uint2BCD(tmpMon);
							}
						} else {
							mTime->date = uint2BCD(tmpDate);
						}
					} else {
						mTime->hour = uint2BCD(tmpHour);
					}
				}
			}
		}
	}
}
