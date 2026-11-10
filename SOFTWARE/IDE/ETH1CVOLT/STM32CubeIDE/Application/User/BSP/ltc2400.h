/*
 * LTC2400.c
 *
 *  Created on: Nov 8, 2026
 *      Author: grzegorz
 */

#ifndef APPLICATION_USER_BSP_LTC2400_C_
#define APPLICATION_USER_BSP_LTC2400_C_

#include "main.h"

HAL_StatusTypeDef ltc2400_read_voltage(SPI_HandleTypeDef* hspi, double* voltage);

#endif /* APPLICATION_USER_BSP_LTC2400_C_ */
