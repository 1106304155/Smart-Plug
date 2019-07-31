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
 * rtc.h
 *
 *  Created on: Mar 15, 2018
 *      Author: Lin Yi-Lie
 *
 */

#ifndef TASKS_RTC_RTC_H_
#define TASKS_RTC_RTC_H_
#include "../common.h"

#define DS3231_I2C_ADDRESS			0x0068
#define RTC_DS3231_YEAR_OFFSET			2000    //year=20xx

/*
 * RTC_STATUS
 * RTC status definition
 * Used in TX/RX function and isBusy().
 */
typedef enum RTC_STATUS {
	RTC_IDLE=0, RTC_READ, RTC_WRITE,
} RTC_STATUS;

/* RTC_DS3231_REG , DS3231 register address */
typedef enum RTC_DS3231_REG_ADDR {
	DS3231_SEC_ADDR = 0x00, /*!< second register */
	DS3231_MIN_ADDR = 0x01, /*!< minute register*/
	DS3231_HOUR_ADDR = 0x02, /*!< hour register*/
	DS3231_DAY_ADDR = 0x03, /*!< day register*/
	DS3231_DATE_ADDR = 0x04, /*!< date register*/
	DS3231_MON_ADDR = 0x05, /*!< month register*/
	DS3231_YEAR_ADDR = 0x06, /*!< year register*/
	DS3231_CTRL_ADDR = 0x0e, /*!< control register*/
	DS3231_CTRL_STAT_ADDR = 0x0f /*!< control and status register*/
} RTC_DS3231_REG_ADDR;

//  RTC configuration
#define RTC_TX_BUF_SIZE  	10

void RTC_initMaster();
void RTC_writeTimeToSlave(RTC_time_t* wTime);
void RTC_readTimeFromSlave(RTC_time_t* rTime);
RTC_STATUS RTC_isBusy();
#endif /* TASKS_RTC_RTC_H_ */
