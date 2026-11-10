/*
 * LTC2400.c
 *
 *  Created on: Nov 10, 2026
 *      Author: grzegorz
 */

#include "ltc2400.h"

#define MAX_TIMEOUT 250
#define V_REF 5.0f

HAL_StatusTypeDef ltc2400_read_voltage(SPI_HandleTypeDef *hspi, double *voltage) {
	double adc_voltage;
	HAL_StatusTypeDef hal_status;
	uint32_t timeout = 0;
	uint8_t rx_buf[4] = { 0, 0, 0, 0 };
	uint32_t raw_adc = 0;

	LL_GPIO_ResetOutputPin(MCU_ADC_NSS_GPIO_Port, MCU_ADC_NSS_Pin);

	while (1 == LL_GPIO_IsInputPinSet(MCU_ADC_SDO_GPIO_Port, MCU_ADC_SDO_Pin)) {
		HAL_Delay(1);
		timeout++;
		if (timeout > MAX_TIMEOUT) {
			LL_GPIO_SetOutputPin(MCU_ADC_NSS_GPIO_Port, MCU_ADC_NSS_Pin);
			return HAL_TIMEOUT;
		}
	}

	//HAL_Delay(5);
	LL_GPIO_SetOutputPin(MCU_ADC_NSS_GPIO_Port, MCU_ADC_NSS_Pin);
	//HAL_Delay(200);
	LL_GPIO_ResetOutputPin(MCU_ADC_NSS_GPIO_Port, MCU_ADC_NSS_Pin);
	hal_status = HAL_SPI_Receive(hspi, rx_buf, 4, 2000);
	LL_GPIO_SetOutputPin(MCU_ADC_NSS_GPIO_Port, MCU_ADC_NSS_Pin);

	raw_adc = ((uint32_t) rx_buf[3] << 24) | ((uint32_t) rx_buf[2] << 16)
			| ((uint32_t) rx_buf[1] << 8) | ((uint32_t) rx_buf[0]);

	raw_adc = (raw_adc >> 4) & 0x00FFFFFF;

	adc_voltage = ((double) raw_adc * (double) V_REF) / (double) 16777216;
	*voltage = adc_voltage;

	return hal_status;
}
