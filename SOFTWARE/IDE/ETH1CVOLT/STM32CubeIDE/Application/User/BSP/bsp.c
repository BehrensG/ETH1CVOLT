/*
 * bsp.c
 *
 *  Created on: Nov 8, 2026
 *      Author: grzegorz
 */


#include "bsp.h"
#include "dg412.h"
#include "ltc2400.h"

bsp_t bsp;

#define INA159A_MID (double)2.5
#define INA159A_GAIN (double)5.0

void bsp_init(void)
{
	bsp.range = 10;

	HAL_Delay(20);
	LL_GPIO_SetOutputPin(MCU_REL_RESET_GPIO_Port, MCU_REL_RESET_Pin);
	HAL_Delay(20);
	LL_GPIO_ResetOutputPin(MCU_REL_RESET_GPIO_Port, MCU_REL_RESET_Pin);
	LL_GPIO_SetOutputPin(MCU_ADC_NSS_GPIO_Port, MCU_ADC_NSS_Pin);

	dg412_init();
}

HAL_StatusTypeDef bsp_read_voltage(SPI_HandleTypeDef *hspi, double *voltage)
{
	HAL_StatusTypeDef hal_status = HAL_OK;
	double v_read = 0.0;
	hal_status = ltc2400_read_voltage(hspi, &v_read);
	if(HAL_OK == hal_status)
	{
		v_read = (v_read - INA159A_MID)*INA159A_GAIN;
		*voltage = v_read;
		return hal_status;
	}
	else
		return hal_status;
}
