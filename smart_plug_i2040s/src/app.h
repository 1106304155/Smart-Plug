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
 * app.h
 *
 *  Created on: Oct 22, 2018
 *      Author: Lin Yi-Lie
 */

#ifndef APP_APP_H_
#define APP_APP_H_
#include "common.h"
#include "task/serial.h"
#include "task/rtc.h"
#include "task/epower.h"

#if  (!_SUPPORT_LOG)
#warning "LOG NOT SUPPORT"
#endif
#if  (!_SUPPORT_SERIAL)
#warning "SERIAL NOT SUPPORT"
#endif
#if  (!_SUPPORT_RTC)
#warning "RTC NOT SUPPORT"
#endif
#if  (!_SUPPORT_EPOWER )
#warning "EPOWER NOT SUPPORT"
#endif

#define _APP_VERSION_  ("19.6.21")
#define _COMPILE_VERSION_ ("18.1.5LTS")
#define _CCS_VERSION_ ("8.3.0")

//Default System Time
#define DEFAULT_RTC_SEC 	(0x55)
#define DEFAULT_RTC_MIN 	(0x59)
#define DEFAULT_RTC_HOUR	(0x23)
#define DEFAULT_RTC_DAY 	(0x06)
#define DEFAULT_RTC_DATE 	(0x01)
#define DEFAULT_RTC_MON		(0x06)
#define DEFAULT_RTC_YEAR 	(0x19)
#define DEFAULT_RTC_TIME { \
	.sec = DEFAULT_RTC_SEC, .min = DEFAULT_RTC_MIN, .hour = DEFAULT_RTC_HOUR, .day =DEFAULT_RTC_DAY, .date = DEFAULT_RTC_DATE, .mon = DEFAULT_RTC_MON, .year = DEFAULT_RTC_YEAR, \
	}

/*
 * Module define
 */
//POWER LED (on board)
#define LED_POWER_PORT 	GPIO_PORT_P2
#define LED_POWER_PIN 	GPIO_PIN0
//HMI (button only)
#define BUTTON_PORT		GPIO_PORT_P1
#define BUTTON_1_PIN 		GPIO_PIN0 //RELAY
#define BUTTON_2_PIN 		GPIO_PIN4 //BT_EN
//Bluetooth
#define BLUETOOTH_PORT  	GPIO_PORT_P2
#define BLUETOOTH_EN	GPIO_PIN2 //BT POWER
#define BLUETOOTH_STAT	GPIO_PIN3
#define BLUETOOTH_EN_ON  (1)
#define BLUETOOTH_EN_OFF  (0)
#define BLUETOOTH_STAT_ON  (1)
#define BLUETOOTH_STAT_OFF  (0)
//Relay
#define RELAY_PORT 	GPIO_PORT_P2
#define RELAY_PIN 	GPIO_PIN1
#define RELAY_PIN_ON	(1)
#define RELAY_PIN_OFF	(0)
//LOG
#define LOG_SEG_START_NUM  (1)
#define LOG_SEG_MAX_NUM   (4)
#define LOG_SEG_NUM   (LOG_SEG_MAX_NUM - LOG_SEG_START_NUM+1)
#define LOG_HEADER_MAX_NUM   (16) //1024/(2+31*2)=16 ,headers in a SEG
#define LOG_HEADER_MAX_SIZE    (31*sizeof(uint16_t)) //31 day(1 month) , in a header record

/*
 *  APP CONFIG
 */
//SCHEDULE
#define APP_SCHEDULE_TABLE_LENGTH   (8)
//password
#define APP_DEFAULT_PASSWD  "1234567"
//Device id
#define APP_DEVICE_ID		(0x01)
//power protect
#define APP_CURRENT_LIMIT   (10*64)  //Q6 format => current*2^6

#define FULL_FUN   _CONFIG_TRUE
#if(FULL_FUN)
// Software RTC (Reserved) _CONFIG_FALSE _CONFIG_TRUE
#define APP_SOFT_RTC   	_CONFIG_FALSE
//HMI ENABLE
#define APP_HMI_EN 		 _CONFIG_TRUE
#else
// Software RTC (Reserved) _CONFIG_FALSE _CONFIG_TRUE
#define APP_SOFT_RTC   	_CONFIG_TRUE
//HMI ENABLE
#define APP_HMI_EN 		 _CONFIG_FALSE
#endif

//Serial RX command
typedef enum SERIAL_rxCmdSet_t {
	RX_CMD_GET_EINFO,
	RX_CMD_GET_TIME,
	RX_CMD_SET_TIME,
	RX_CMD_GET_LOG,
	RX_CMD_GET_LOG_NUM,
	RX_CMD_GET_SCH,
	RX_CMD_GET_SCH_NUM,
	RX_CMD_SET_SCH,
	RX_CMD_GET_SW_STAT,
	RX_CMD_SET_SW_STAT,
	RX_CMD_PASSWD,
	RX_CMD_CLOSE_BT,
	RX_CMD_GET_ID,
	RX_CMD_GET_VERSION,
} SERIAL_rxCmdSet_t;

//LOG header structure
typedef struct logHeader_t {
	uint8_t mon;
	uint8_t year;
} logHeader_t;

/* log information */
typedef struct logInfo_t {
	uint8_t segNum;
	uint8_t headerNum;
	logHeader_t header;
	uint8_t* headerFlashPtr;
	uint16_t* dataFlashPtr;
	uint16_t date;
} logInfo_t;

/*
 *	schedule table
 */
typedef struct schedule_t {
	uint8_t day;
	uint8_t hour;
	uint8_t min;
	uint8_t status;
} schedule_t;


//Task function
void serialService();
void epowerService();
void rtcService();
#endif /* APP_APP_H_ */
