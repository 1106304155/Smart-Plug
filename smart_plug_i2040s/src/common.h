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
 * common.h
 *
 *  Created on: Feb 27, 2019
 *      Author: Lin Yi-Lie
 */

#ifndef SRC_TASK_COMMON_H_
#define SRC_TASK_COMMON_H_
#include <msp430.h>
#include <stdint.h>
#include <math.h>
#include "driver/driverlib.h"

/* Define TRUE logic for ALL system settings. */
#define _CONFIG_TRUE       1
/* Define FALSE logic for ALL system settings. */
#define _CONFIG_FALSE      0

/*
 * Clock system configuration
 * CPU CLOCK(MCLK): DCO(16.384Mhz,FIXED) / CFG_MCLK_DIV\n
 * MOUDLES CLOCK(SMCLK): DCO(16.384Mhz,FIXED) / CFG_SMCLK_DIV\n
 * 	DIVS_0 = DIVM_0 = Divide by 1\n
 * 	DIVS_1 = DIVM_1 = Divide by 2\n
 * 	DIVS_2 = DIVM_2 = Divide by 4\n
 * 	DIVS_3 = DIVM_3 = Divide by 8\n
 * 	DIVS_4 = DIVM_4 = Divide by 16\n
 */
#define _CFG_MCLK_DIV      DIVM_0 //16.384Mhz / DIV = 16.384M
#define _CFG_SMCLK_DIV     DIVS_2 //16.384Mhz / DIV = 4.096M

//_CONFIG_FALSE , _CONFIG_TRUE
#define _SUPPORT_SERIAL         _CONFIG_TRUE   // UART
#define _SUPPORT_RTC        	_CONFIG_TRUE   // Real time clock,dependent on I2C
#define _SUPPORT_EPOWER    		_CONFIG_TRUE  // Electrical power calculation
#define _SUPPORT_LOG         	_CONFIG_TRUE  //Dependent on RTC

/*
 * task flag struct
 */
typedef struct taskStatus_t {
	uint16_t serv_epower :1;
	uint16_t serv_serial :1;
	uint16_t serv_rtc :1;
	uint16_t rtcSet :1;
	uint16_t passwdPass:1;
} taskStatus_t;  //For bgTaskFlag


/*
 * RTC time record
 */
typedef struct RTC_time_t {
	uint8_t sec;
	uint8_t min;
	uint8_t hour;
	uint8_t day;
	uint8_t date;
	uint8_t mon;
	uint8_t year;
} RTC_time_t;



//BackGround Status Variable
extern taskStatus_t backGroundStatus;

#endif /* SRC_TASK_COMMON_H_ */
