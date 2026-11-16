/*
 * MAX5719.c
 *
 *  Created on: Nov 8, 2026
 *      Author: grzegorz
 */

#include "math.h"

#include "max5719.h"

#define MAX5719_V_REF (double)5.0
#define MAX5719_20BIT (double)1048576

void max5719_init(void) {
	LL_GPIO_SetOutputPin(MCU_DAC_nLDAC_GPIO_Port, MCU_DAC_nLDAC_Pin);
	LL_GPIO_SetOutputPin(MCU_DAC_NSS_GPIO_Port, MCU_DAC_NSS_Pin);

}

static uint32_t max5719_volt_to_uint32(double voltage) {

	uint32_t code_20bit = (uint32_t)((voltage / MAX5719_V_REF ) * MAX5719_20BIT - 1);

	return code_20bit;

}

HAL_StatusTypeDef max5719_set_voltage(SPI_HandleTypeDef *hspi, double voltage) {
	HAL_StatusTypeDef hal_status = HAL_OK;
	uint8_t tx_buf[3] = { 0, 0, 0 };
	uint32_t spi_word = 0;
	uint32_t dac_value = 0;

	dac_value = max5719_volt_to_uint32(voltage);

	tx_buf[0] = (uint8_t) ((dac_value >> 16) & 0xFF);
	tx_buf[1] = (uint8_t) ((dac_value >> 8) & 0xFF);
	tx_buf[2] = (uint8_t) (dac_value & 0xFF);

	LL_GPIO_SetOutputPin(MCU_DAC_nLDAC_GPIO_Port, MCU_DAC_nLDAC_Pin);
	LL_GPIO_SetOutputPin(MCU_DAC_NSS_GPIO_Port, MCU_DAC_NSS_Pin);

	LL_GPIO_ResetOutputPin(MCU_DAC_NSS_GPIO_Port, MCU_DAC_NSS_Pin);
	hal_status = HAL_SPI_Transmit(hspi, tx_buf, 3, 2000);
	LL_GPIO_SetOutputPin(MCU_DAC_NSS_GPIO_Port, MCU_DAC_NSS_Pin);
	if (HAL_OK == hal_status) {
		HAL_Delay(1);
		LL_GPIO_ResetOutputPin(MCU_DAC_nLDAC_GPIO_Port, MCU_DAC_nLDAC_Pin);
		HAL_Delay(1);
		LL_GPIO_SetOutputPin(MCU_DAC_nLDAC_GPIO_Port, MCU_DAC_nLDAC_Pin);
	}
	return hal_status;
}
