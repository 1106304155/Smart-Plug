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
 * serial.h
 *
 *  Created on: Mar 1, 2018
 *      Author: Lin Yi-Lie
 *
 */

#ifndef TASKS_SERIAL_SERIAL_H_
#define TASKS_SERIAL_SERIAL_H_
#include "../common.h"



//CONFIG
#define SERIAL_RX_WITH_EOL _CONFIG_FALSE    //with \r\n ,_CONFIG_TRUE _CONFIG_FALSE
#define SERIAL_PREAMBLE (0xFE)
#define SERIAL_SYNC (0xFE)
#define SERIAL_RX_EOL ('\n')

/* serial RX package format */
typedef struct SERIAL_rxPackage_t {
	uint8_t cmd;
	uint8_t arg0;
	uint8_t arg1;
	uint8_t arg2;
	uint8_t arg3;
	uint8_t arg4;
	uint8_t arg5;
	uint8_t arg6;
} SERIAL_rxPackage_t;


#define SERIAL_RX_BUF_SIZE  (10)
extern int8_t serialRxBuffer[];

void SERIAL_init();
int16_t SERIAL_isBusy();
void SERIAL_startRx();
void SERIAL_send(int8_t* data, uint16_t length);

#endif /* TASKS_SERIAL_SERIAL_H_ */

