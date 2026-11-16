/*
 * max5719.h
 *
 *  Created on: Nov 10, 2026
 *      Author: grzegorz
 */

#ifndef APPLICATION_USER_BSP_MAX5719_H_
#define APPLICATION_USER_BSP_MAX5719_H_

#include "main.h"

void max5719_init();
HAL_StatusTypeDef max5719_set_voltage(SPI_HandleTypeDef *hspi, double voltage);

#endif /* APPLICATION_USER_BSP_MAX5719_H_ */
