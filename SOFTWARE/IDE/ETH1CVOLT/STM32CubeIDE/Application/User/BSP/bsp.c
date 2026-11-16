/*
 * bsp.c
 *
 *  Created on: Nov 8, 2026
 *      Author: grzegorz
 */

#include "bsp.h"
#include "dg412.h"
#include "ltc2400.h"
#include "max5719.h"

bsp_t bsp;

#ifdef BIPOLAR
#define INA159A_MID (double)2.5
#define INA159A_GAIN (double)5.0
#endif

void bsp_init(void) {
	bsp.range = 10;

	HAL_Delay(20);
	LL_GPIO_SetOutputPin(MCU_REL_RESET_GPIO_Port, MCU_REL_RESET_Pin);
	HAL_Delay(20);
	LL_GPIO_ResetOutputPin(MCU_REL_RESET_GPIO_Port, MCU_REL_RESET_Pin);
	LL_GPIO_SetOutputPin(MCU_ADC_NSS_GPIO_Port, MCU_ADC_NSS_Pin);

	dg412_init();
	max5719_init();
}

HAL_StatusTypeDef bsp_read_voltage(SPI_HandleTypeDef *hspi, double *voltage) {
	HAL_StatusTypeDef hal_status = HAL_OK;
	double v_read = 0.0;
	hal_status = ltc2400_read_voltage(hspi, &v_read);
	if (HAL_OK == hal_status) {

#ifdef BIPOLAR
		v_read = (v_read - INA159A_MID ) * INA159A_GAIN;
#endif

		*voltage = v_read * (double) 2.0;
		return hal_status;
	} else
		return hal_status;
}

HAL_StatusTypeDef bsp_set_voltage(SPI_HandleTypeDef *hspi, double voltage) {
	HAL_StatusTypeDef hal_status = HAL_OK;
	double v_set = 0.0;

	if (20 == bsp.range) {
		v_set = voltage / (double) 4.0;
	} else {
		v_set = voltage / (double) 2.0;
	}

	hal_status = max5719_set_voltage(hspi, v_set);

	return hal_status;
}

