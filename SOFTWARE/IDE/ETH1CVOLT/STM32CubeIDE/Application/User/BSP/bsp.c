/*
 * bsp.c
 *
 *  Created on: Nov 8, 2026
 *      Author: grzegorz
 */


#include "bsp.h"

bsp_t bsp;


void bsp_init(void)
{
	bsp.range = 10;

	HAL_Delay(20);
	LL_GPIO_SetOutputPin(MCU_REL_RESET_GPIO_Port, MCU_REL_RESET_Pin);
	HAL_Delay(20);
	LL_GPIO_ResetOutputPin(MCU_REL_RESET_GPIO_Port, MCU_REL_RESET_Pin);
}
