/*
 * bsp.h
 *
 *  Created on: Nov 8, 2026
 *      Author: grzegorz
 */

#ifndef APPLICATION_USER_BSP_BSP_H_
#define APPLICATION_USER_BSP_BSP_H_

#include "stdbool.h"
#include "main.h"

// --------------------------------------------------------------------------------------------------------------------

#define IP_ADDRESS_0 192
#define IP_ADDRESS_1 168
#define IP_ADDRESS_2 1
#define IP_ADDRESS_3 12

#define NETMASK_ADDRESS_0 255
#define NETMASK_ADDRESS_1 255
#define NETMASK_ADDRESS_2 255
#define NETMASK_ADDRESS_3 0

#define GATEWAY_ADDRESS_0 192
#define GATEWAY_ADDRESS_1 168
#define GATEWAY_ADDRESS_2 1
#define GATEWAY_ADDRESS_3 1

#define MAC_0 0x00
#define MAC_1 0x80
#define MAC_2 0xE1
#define MAC_3 0x00
#define MAC_4 0x00
#define MAC_5 0x00

#define STRING_LENGTH 		16

// --------------------------------------------------------------------------------------------------------------------

#pragma pack(push, 1)

typedef struct {

	char serial_number[STRING_LENGTH];

} bsp_scpi_info_t;

typedef struct {
	uint8_t ip[4];
	uint8_t netmask[4];
	uint8_t gateway[4];
	uint8_t MAC[6];

} bsp_ip4_lan_t;

typedef struct {

	float gain;
	float offset;
	uint32_t count;

} bsp_calibration_t;

typedef struct {
	uint8_t nominal_current;
	float multiplier;

} bsp_setup_t;

typedef struct {
	uint8_t mdns;

} bsp_services_t;

#define EEPROM_CFG_SIZE 	sizeof(bsp_ip4_lan_t) + sizeof(bsp_scpi_info_t) + 2*sizeof(bsp_calibration_t) + sizeof(bsp_setup_t) + sizeof(bsp_services_t)

typedef union {
	struct data {

		bsp_ip4_lan_t ip4_static;
		bsp_scpi_info_t info;
		bsp_calibration_t calib_dac;
		bsp_calibration_t calib_dac;
		bsp_setup_t setup;
		bsp_services_t services;

	} structure;
	uint8_t bytes[EEPROM_CFG_SIZE];

} bsp_eeprom_t;

#pragma pack(pop)

// --------------------------------------------------------------------------------------------------------------------

typedef struct {
	bool status;

} bsp_security_t;

// --------------------------------------------------------------------------------------------------------------------

typedef struct {
	bool enable;
	uint16_t ratio;

} bsp_oversampling_t;

// --------------------------------------------------------------------------------------------------------------------

typedef struct {
	bool enable;
	float zero[3];

} bsp_offset_t;

// --------------------------------------------------------------------------------------------------------------------

typedef struct {
	uint8_t value;
	uint8_t index;

} bsp_gain_t;

// --------------------------------------------------------------------------------------------------------------------

typedef struct {
	bsp_eeprom_t eeprom;
	bsp_security_t security;
	bsp_ip4_lan_t ip4_current;
	bool default_cfg;
	float voltage;
	uint32_t range;
} bsp_t;

// --------------------------------------------------------------------------------------------------------------------


void bsp_init();

#endif /* APPLICATION_USER_BSP_BSP_H_ */
