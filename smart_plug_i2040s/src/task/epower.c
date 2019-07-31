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
 * epower_calc.c
 *
 *  Created on: Sep 21, 2018
 *      Author: Lin Yi-Lie
 */

#include "epower.h"

volatile int16_t vSample[SD24_SAMPLE_SIZE];
volatile int16_t iSample[SD24_SAMPLE_SIZE];
uint16_t sd24SampleIndex;
uint16_t vZeroPoint1, vZeroPoint2;
uint16_t iZeroPoint1, iZeroPoint2;
SD24_initConverterAdvancedParam sd24Cfg = { 0 };

volatile float voltRms;
volatile float currentRms;
volatile float powerApperent;
volatile float powerActive;
volatile float powerFreq;
volatile float powerFactor;
EPOWER_powerInfo_t EPOWER_tmp;

void EPOWER_init() {
	// Internal ref
	SD24_init(SD24_BASE, SD24_REF_INTERNAL);
	//Group with Channel 0 -> V
	sd24Cfg.converter = SD24_CONVERTER_0;
	sd24Cfg.conversionMode = SD24_CONTINUOUS_MODE;
	sd24Cfg.groupEnable = SD24_GROUPED;
	sd24Cfg.inputChannel = SD24_INPUT_CH_ANALOG;
	sd24Cfg.dataFormat = SD24_DATA_FORMAT_2COMPLEMENT;
	sd24Cfg.interruptDelay = SD24_FOURTH_SAMPLE_INTERRUPT;
	sd24Cfg.oversampleRatio = SD24_OVERSAMPLE_256;
	sd24Cfg.gain = SD24_GAIN_1;
	SD24_initConverterAdvanced(SD24_BASE, &sd24Cfg);
	//Group with Channel 1 -> I
	sd24Cfg.converter = SD24_CONVERTER_1;
	sd24Cfg.conversionMode = SD24_CONTINUOUS_MODE;
	sd24Cfg.groupEnable = SD24_NOT_GROUPED;
	sd24Cfg.inputChannel = SD24_INPUT_CH_ANALOG;
	sd24Cfg.dataFormat = SD24_DATA_FORMAT_2COMPLEMENT;
	sd24Cfg.interruptDelay = SD24_FOURTH_SAMPLE_INTERRUPT;
	sd24Cfg.oversampleRatio = SD24_OVERSAMPLE_256;
	sd24Cfg.gain = SD24_GAIN_16;
	SD24_initConverterAdvanced(SD24_BASE, &sd24Cfg);
	// Delay ~200us for 1.2V ref to settle
	__delay_cycles(3200);
	// Enable interrupt
	SD24_enableInterrupt(SD24_BASE, SD24_CONVERTER_1, SD24_CONVERTER_INTERRUPT);
	// Start conversion
	EPOWER_startConversion();
}

void EPOWER_startConversion() {
	vZeroPoint2 = vZeroPoint1 = 0;
	iZeroPoint2 = iZeroPoint1 = 0;
	sd24SampleIndex = SD24_SAMPLE_SIZE - 1;
	// Start conversion
	SD24_startConverterConversion(SD24_BASE, SD24_CONVERTER_1);
}

EPOWER_powerInfo_t EPOWER_getInfo() {
	return EPOWER_tmp;
}

void EPOWER_calc() {
	volatile static uint16_t scan;
	volatile static float period;
	volatile static int32_t tmpI, tmpI2;
	volatile static uint32_t sum;
	volatile static int32_t sum2;
	//Current
	sum = 0;
	period = iZeroPoint2 - iZeroPoint1 + 1;
	for (scan = iZeroPoint1; scan <= iZeroPoint2; scan++) {
		tmpI = iSample[scan]; //tmpI=vSample
		tmpI *= tmpI;
		sum += tmpI;
	}
	currentRms = (float) sum;
	currentRms /= period;
	currentRms = sqrtf(currentRms) * SD24_I_GAIN;
	//Voltage , Active Power
	sum = 0; //8 cycles
	sum2 = 0;
	period = vZeroPoint2 - vZeroPoint1 + 1; //5 cycles
	for (scan = vZeroPoint1; scan <= vZeroPoint2; scan++) { //108 cycles per loop
		tmpI = vSample[scan]; //tmpI=vSample
		tmpI *= tmpI;
		sum += tmpI;
		//Active Power
		tmpI2 = vSample[scan];
		tmpI2 *= iSample[scan];
		sum2 += tmpI2;
	}
	voltRms = (float) sum; //120 cycles
	voltRms /= period; //413 cycles
	voltRms = sqrtf(voltRms) * SD24_V_GAIN; //2742 cycles
	if (voltRms < SD24_V_MIN) {
		EPOWER_tmp.currentRms = 0;
		EPOWER_tmp.powerActive = 0;
		EPOWER_tmp.powerApperent = 0;
		EPOWER_tmp.powerFactor = 0;
		EPOWER_tmp.powerFreq = 0;
	} else {
		powerActive = (float) sum2;
		powerActive /= period;
		powerActive *= SD24_P_GAIN;
		_NOP();
		EPOWER_tmp.voltRms = (uint16_t) voltRms; //INT
		EPOWER_tmp.voltRms <<= 6; // *64  Q6 format
		EPOWER_tmp.voltRms += ((uint16_t) (voltRms * 64)) - EPOWER_tmp.voltRms;

		//Frequency ,455 cycles
		powerFreq = 1.0 / SD24_SEC_PER_SAMPLE / period;
		_NOP();
		EPOWER_tmp.powerFreq = (uint16_t) powerFreq; //INT
		EPOWER_tmp.powerFreq <<= 6; // *64  Q6 format
		EPOWER_tmp.powerFreq += ((uint16_t) (powerFreq * 64))
				- EPOWER_tmp.powerFreq;
		//Apparent power , PF
		if (currentRms < SD24_I_MIN) {
			currentRms = 0.0;
			EPOWER_tmp.currentRms = 0;
			EPOWER_tmp.powerActive = 0;
			powerFactor = 0.0;
		} else {
			EPOWER_tmp.currentRms = (uint16_t) currentRms; //INT
			EPOWER_tmp.currentRms <<= 6; // *64  Q6 format
			EPOWER_tmp.currentRms += ((uint16_t) (currentRms * 64))
					- EPOWER_tmp.currentRms;
			EPOWER_tmp.powerActive = (uint16_t) powerActive; //INT
			EPOWER_tmp.powerActive <<= 6; // *64  Q6 format
			EPOWER_tmp.powerActive += ((uint16_t) (powerActive * 64))
					- EPOWER_tmp.powerActive;
			powerApperent = currentRms * voltRms;
			EPOWER_tmp.powerApperent = (uint16_t) powerApperent; //INT
			EPOWER_tmp.powerApperent <<= 6; // *64  Q6 format
			EPOWER_tmp.powerApperent += ((uint16_t) (powerApperent * 64))
					- EPOWER_tmp.powerApperent;
			powerFactor = powerActive / powerApperent;
			EPOWER_tmp.powerFactor = (uint16_t) powerFactor; //INT
			EPOWER_tmp.powerFactor <<= 6; // *64  Q6 format
			EPOWER_tmp.powerFactor += ((uint16_t) (powerFactor * 64))
					- EPOWER_tmp.powerFactor;
		}
	}
}

#if defined(__TI_COMPILER_VERSION__) || defined(__IAR_SYSTEMS_ICC__)
#pragma vector=SD24_VECTOR
__interrupt void SD24_ISR(void)
#elif defined(__GNUC__)
void __attribute__ ((interrupt(SD24_VECTOR))) SD24_ISR (void)
#else
#error Compiler not supported!
#endif
{
	switch (__even_in_range(SD24IV, SD24IV_SD24MEM3)) {
	case SD24IV_NONE:
		break;
	case SD24IV_SD24OVIFG:
		break;
	case SD24IV_SD24MEM0:
		break;
	case SD24IV_SD24MEM1:
// Save CH0 results (clears IFG)
		vSample[sd24SampleIndex] = SD24MEM0;
		vSample[sd24SampleIndex] += SD24_V_OFFSET;
// Save CH1 results (clears IFG)
		iSample[sd24SampleIndex] = SD24MEM1;
		iSample[sd24SampleIndex] += SD24_I_OFFSET;
		if (sd24SampleIndex < SD24_SAMPLE_SIZE - 4) {
			//FIR filter
			vSample[sd24SampleIndex] = (vSample[sd24SampleIndex] >> 1)
					+ (vSample[sd24SampleIndex + 1] >> 2)
					+ (vSample[sd24SampleIndex + 2] >> 3)
					+ (vSample[sd24SampleIndex + 3] >> 3);
			iSample[sd24SampleIndex] = (iSample[sd24SampleIndex] >> 1)
					+ (iSample[sd24SampleIndex + 1] >> 2)
					+ (iSample[sd24SampleIndex + 2] >> 3)
					+ (iSample[sd24SampleIndex + 3] >> 3);
			//find 1st zero point
			//V
			if (vSample[sd24SampleIndex] > 0
					&& vSample[sd24SampleIndex + 1] <= 0) {
				if (!vZeroPoint2)
					vZeroPoint2 = sd24SampleIndex;
				else if (!vZeroPoint1)
					vZeroPoint1 = sd24SampleIndex + 1;
			}
			//I
			if (iSample[sd24SampleIndex] > 0
					&& iSample[sd24SampleIndex + 1] <= 0) {
				if (!iZeroPoint2)
					iZeroPoint2 = sd24SampleIndex;
				else if (!iZeroPoint1)
					iZeroPoint1 = sd24SampleIndex + 1;
			}
		}
		//enable background task?
		if (sd24SampleIndex) {
			sd24SampleIndex--;
		} else {
			SD24_stopConverterConversion(SD24_BASE, SD24_CONVERTER_1);
			backGroundStatus.serv_epower = 1; //Enable EPOWER background task
		}
		break;
	case SD24IV_SD24MEM2:
		break;

	case SD24IV_SD24MEM3:
		break;
	} //	switch (__even_in_range(SD24IV, SD24IV_SD24MEM3))
} //SD24 ISR
