/*
 * DG412.c
 *
 *  Created on: Nov 8, 2026
 *      Author: grzegorz
 */

#include "dg412.h"

void dg412_init(void)
{
	LL_GPIO_ResetOutputPin(MCU_ADC_SW1_GPIO_Port, MCU_ADC_SW1_Pin);
	LL_GPIO_ResetOutputPin(MCU_ADC_SW2_GPIO_Port, MCU_ADC_SW2_Pin);
	LL_GPIO_ResetOutputPin(MCU_ADC_SW3_GPIO_Port, MCU_ADC_SW3_Pin);
	HAL_Delay(10);
	LL_GPIO_SetOutputPin(MCU_ADC_SW1_GPIO_Port, MCU_ADC_SW1_Pin);
	HAL_Delay(10);
}

void dg412_meas(void)
{
	LL_GPIO_ResetOutputPin(MCU_ADC_SW1_GPIO_Port, MCU_ADC_SW1_Pin);
	LL_GPIO_ResetOutputPin(MCU_ADC_SW2_GPIO_Port, MCU_ADC_SW2_Pin);
	LL_GPIO_ResetOutputPin(MCU_ADC_SW3_GPIO_Port, MCU_ADC_SW3_Pin);
	HAL_Delay(10);
	LL_GPIO_SetOutputPin(MCU_ADC_SW1_GPIO_Port, MCU_ADC_SW1_Pin);
	HAL_Delay(10);
}

void dg412_vref(void)
{
	LL_GPIO_ResetOutputPin(MCU_ADC_SW1_GPIO_Port, MCU_ADC_SW1_Pin);
	LL_GPIO_ResetOutputPin(MCU_ADC_SW2_GPIO_Port, MCU_ADC_SW2_Pin);
	LL_GPIO_ResetOutputPin(MCU_ADC_SW3_GPIO_Port, MCU_ADC_SW3_Pin);
	HAL_Delay(10);
	LL_GPIO_SetOutputPin(MCU_ADC_SW2_GPIO_Port, MCU_ADC_SW2_Pin);
	HAL_Delay(10);
}

void dg412_gnd(void)
{
	LL_GPIO_ResetOutputPin(MCU_ADC_SW1_GPIO_Port, MCU_ADC_SW1_Pin);
	LL_GPIO_ResetOutputPin(MCU_ADC_SW2_GPIO_Port, MCU_ADC_SW2_Pin);
	LL_GPIO_ResetOutputPin(MCU_ADC_SW3_GPIO_Port, MCU_ADC_SW3_Pin);
	HAL_Delay(10);
	LL_GPIO_SetOutputPin(MCU_ADC_SW3_GPIO_Port, MCU_ADC_SW3_Pin);
	HAL_Delay(10);
}
