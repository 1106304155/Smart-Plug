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
 * epower_calc.h
 *
 *  Created on: Sep 21, 2018
 *      Author: Lin Yi-Lie
 */
#ifndef TASKS_EPOWER_EPOWER_H_
#define TASKS_EPOWER_EPOWER_H_
#include "../common.h"

typedef struct EPOWER_powerInfo {
	uint16_t voltRms;  //Q6 format
	uint16_t currentRms; //Q6 format
	uint16_t powerApperent; //Q6 format
	uint16_t powerActive; //Q6 format
	uint16_t powerFreq; //Q6 format
	uint16_t powerFactor; //Q6 format
} EPOWER_powerInfo_t;

/*
 * Constant for converting (for SD24)
 */
//Sample window size
#define SD24_SAMPLE_SIZE			(150)

//Voltage
//Note: input range = +-0.928V / GAIN=1.0
#define SD24_V_INPUT_RES 		(2.0*0.928/1.0/65535.0)
#define SD24_V_CAL		(1.288)
#define SD24_V_OFFSET		(-54) //Trim sd24 offset SP01: -54
#define SD24_V_SCALE			(991.5/1.5) // see Schematic design (991.5/1.5)
#define SD24_V_GAIN		(SD24_V_CAL*SD24_V_SCALE * SD24_V_INPUT_RES)
#define SD24_V_MIN		(30.0) //Unit: V
//I
//Note: input range = +-0.928V / GAIN=16.0
#define SD24_I_INPUT_RES 		(2.0*0.928/16.0/65535.0)
#define SD24_I_CAL			(1.18) //SP01: 1.18 SP02:1.0
#define SD24_I_OFFSET		(240) //Trim sd24 offset SP01: 240 SP02: -350
#define SD24_I_SCALE			(2000.0) // see Schematic design (2000.0)
#define SD24_I_GAIN		(SD24_I_CAL*SD24_I_INPUT_RES*SD24_I_SCALE)
#define SD24_I_MIN		(0.1) //Unit: A
//POWER
#define SD24_P_GAIN			(1.0*SD24_V_GAIN*SD24_I_GAIN)

//Frequency
#define SD24_SEC_PER_SAMPLE 	(256.0/1024000.0) //sd24 sample frequency=1.024Mhz(fixed)/OSR,OSR=256

#if(SD24_SAMPLE_SIZE>=200)
#error "SD24_SAMPLE_SIZE must <=200 (i2041 memory limited)"
#endif

void EPOWER_init();
void EPOWER_startConversion();
void EPOWER_calc();
EPOWER_powerInfo_t EPOWER_getInfo();
#endif /* TASKS_EPOWER_EPOWER_H_ */

