/*
 * HiSLIP.h
 *
 *  Created on: Jun 3, 2024
 *      Author: grzegorz
 */

#ifndef HISLIP_INC_HISLIP_H_
#define HISLIP_INC_HISLIP_H_

#include <scpi_def.h>
#include "string.h"
#include "stdbool.h"
#include "types.h"
#include "app_azure_rtos.h"
#include "app_netxduo.h"
#include "device_info.h"
#include "scpi_def.h"

#define HISLIP_LINE_ENDING									"\n"

#define HISLIP_PROLOGUE									0x4853

// TBD
#define HISLIP_VENDOR_ID								0x1111

#define HISLIP_MAX_DATA_SIZE							1024

#define HISLIP_TCP_BUFFER          1536


#define HISLIP_PORT 4880
#define  err_t int32_t

#define HISLIP_INITIALIZE									(uint8_t)0
#define HISLIP_INITIALIZE_RESPONSE							(uint8_t)1
#define	HISLIP_FATAL_ERROR									(uint8_t)2
#define	HISLIP_ERROR										(uint8_t)3
#define HISLIP_ASYN_CLOCK									(uint8_t)4
#define HISLIP_ASYN_CLOCK_RESPONSE							(uint8_t)5
#define HISLIP_DATA											(uint8_t)6
#define HISLIP_DATAEND										(uint8_t)7
#define HISLIP_DEVICE_CLEAR_COMPLETE						(uint8_t)8
#define HISLIP_DEVICE_CLEAR_ACKNOWLEDGE						(uint8_t)9
#define HISLIP_ASYNC_REMOTE_LOCAL_CONTROL					(uint8_t)10
#define HISLIP_ASYNC_REMOTE_LOCAL_RESPONSE					(uint8_t)11
#define HISLIP_TRIGGER										(uint8_t)12
#define HISLIP_INTERRUPTED									(uint8_t)13
#define HISLIP_ASYNC_INTERRUPTED							(uint8_t)14
#define HISLIP_ASYNC_MAXIMUM_MESSAGE_SIZE					(uint8_t)15
#define HISLIP_ASYNC_MAXIMUM_MESSAGE_SIZE_RESPONSE			(uint8_t)16
#define HISLIP_ASYNC_INITIALIZE								(uint8_t)17
#define HISLIP_ASYNC_INITIALIZE_RESPONSE					(uint8_t)18
#define HISLIP_ASYNC_DEVICE_CLEAR							(uint8_t)19
#define HISLIP_ASYNC_SERVICE_REQUEST						(uint8_t)20
#define HISLIP_ASYNC_STATUS_QUERY							(uint8_t)21
#define HISLIP_ASYNC_STATUS_RESPONSE						(uint8_t)22
#define HISLIP_ASYNC_DEVICE_CLEAR_ACKNOWLEDGE				(uint8_t)23
#define HISLIP_ASYNC_LOCK_INFO								(uint8_t)24
#define HISLIP_ASYNC_LOCK_INFO_RESPONSE						(uint8_t)25
#define HISLIP_GET_DESCRIPTORS								(uint8_t)26
#define HISLIP_GET_DESCRIPTORS_RESPONSE						(uint8_t)27
#define HISLIP_START_TLS									(uint8_t)28
#define HISLIP_ASYNC_START_TLS								(uint8_t)29
#define HISLIP_ASYNC_START_TLS_RESPONSE						(uint8_t)30
#define HISLIP_END_TLS										(uint8_t)31
#define HISLIP_ASYNC_END_TLS								(uint8_t)32
#define HISLIP_ASYNC_END_TLS_RESPONSE						(uint8_t)33
#define HISLIP_GET_SASL_MECHANISM_LIST						(uint8_t)34
#define HISLIP_GET_SASL_MECHANISM_LIST_RESPONSE				(uint8_t)35
#define HISLIP_AUTHENTICATION_START							(uint8_t)36
#define HISLIP_AUTHENTICATION_EXCHANGE						(uint8_t)37
#define HISLIP_AUTHENTICATION_RESULT						(uint8_t)38

typedef enum
{
	Initialize = HISLIP_INITIALIZE,
	InitializeResponse = HISLIP_INITIALIZE_RESPONSE,
	FatalError = HISLIP_FATAL_ERROR,
	Error = HISLIP_ERROR,
	AsyncLock = HISLIP_ASYN_CLOCK,
	AsyncLockResponse = HISLIP_ASYN_CLOCK_RESPONSE,
	Data = HISLIP_DATA,
	DataEnd = HISLIP_DATAEND,
	DeviceClearComplete = HISLIP_DEVICE_CLEAR_COMPLETE,
	DeviceClearAcknowledge = HISLIP_DEVICE_CLEAR_ACKNOWLEDGE,
	AsyncRemoteLocalControl = HISLIP_ASYNC_REMOTE_LOCAL_CONTROL,
	AsyncRemoteLocalResponse = HISLIP_ASYNC_REMOTE_LOCAL_RESPONSE,
	Trigger = HISLIP_TRIGGER,
	Interrupted = HISLIP_INTERRUPTED,
	AsyncInterrupted = HISLIP_ASYNC_INTERRUPTED,
	AsyncMaximumMessageSize = HISLIP_ASYNC_MAXIMUM_MESSAGE_SIZE,
	AsyncMaximumMessageSizeResponse = HISLIP_ASYNC_MAXIMUM_MESSAGE_SIZE_RESPONSE,
	AsyncInitialize = HISLIP_ASYNC_INITIALIZE,
	AsyncInitializeResponse = HISLIP_ASYNC_INITIALIZE_RESPONSE,
	AsyncDeviceClear = HISLIP_ASYNC_DEVICE_CLEAR,
	AsyncServiceRequest = HISLIP_ASYNC_SERVICE_REQUEST,
	AsyncStatusQuery = HISLIP_ASYNC_STATUS_QUERY,
	AsyncStatusResponse = HISLIP_ASYNC_STATUS_RESPONSE,
	AsyncDeviceClearAcknowledge = HISLIP_ASYNC_DEVICE_CLEAR_ACKNOWLEDGE,
	AsyncLockInfo = HISLIP_ASYNC_LOCK_INFO,
	AsyncLockInfoResponse = HISLIP_ASYNC_LOCK_INFO_RESPONSE,
	GetDescriptors = HISLIP_GET_DESCRIPTORS,
	GetDescriptorsResponse = HISLIP_GET_DESCRIPTORS_RESPONSE,
	StartTLS = HISLIP_START_TLS,
	AsyncStartTLS = HISLIP_ASYNC_START_TLS,
	AsyncStartTLSResponse = HISLIP_ASYNC_START_TLS_RESPONSE,
	EndTLS = HISLIP_END_TLS,
	AsyncEndTLS = HISLIP_ASYNC_END_TLS,
	AsyncEndTLSResponse = HISLIP_ASYNC_END_TLS_RESPONSE,
	GetSaslMechanismList = HISLIP_GET_SASL_MECHANISM_LIST,
	GetSaslMechanismListResponse = HISLIP_GET_SASL_MECHANISM_LIST_RESPONSE,
	AuthenticationStart = HISLIP_AUTHENTICATION_START,
	AuthenticationExchange = HISLIP_AUTHENTICATION_EXCHANGE,
	AuthenticationResult = HISLIP_AUTHENTICATION_RESULT,
	HISLIP_NOT_CONNECTED = 39,
	HISLIP_NO_PACKET = 40,

} hislip_msg_type_t;

#pragma pack(push, 1)

typedef struct
{
	uint32_t hi;
	uint32_t lo;
} payload_len_t;

typedef struct
{
	uint16_t prologue;
	uint8_t msg_type;
	uint8_t control_code;
	uint32_t msg_param;
	payload_len_t payload_len;
} hislip_msg_t;

#pragma pack(pop)

typedef struct
{
	int8_t data[HISLIP_MAX_DATA_SIZE + sizeof(hislip_msg_t)];
	int32_t len;
} hislip_netbuf_t;

typedef struct
{
	hislip_msg_t msg;
	hislip_netbuf_t netbuf;
	int32_t socket;
	uint16_t session_id;
	int8_t end[2];
} hislip_instr_t;

err_t hislip_InitializeResponse(hislip_instr_t *hislip_instr);
err_t hislip_DataEnd(hislip_instr_t *hislip_instr);
err_t hislip_Data(hislip_instr_t *hislip_instr);
err_t hislip_Trigger(hislip_instr_t *hislip_instr);
err_t hislip_GetDescriptorsResponse(hislip_instr_t *hislip_instr);
err_t hislip_DeviceClearAcknowledge(hislip_instr_t *hislip_instr);

err_t hislip_AsyncInitialize(hislip_instr_t *hislip_instr);
err_t hislip_AsyncMaximumMessageSize(hislip_instr_t *hislip_instr);
//err_t hislip_AsyncStatusQuery(hislip_instr_t *hislip_instr);
err_t hislip_AsyncDeviceClearAcknowledge(hislip_instr_t *hislip_instr);
err_t hislip_AsyncLockResponse(hislip_instr_t *hislip_instr);
err_t hislip_AsyncLockInfoResponse(hislip_instr_t *hislip_instr);
err_t hislip_AsyncRemoteLocalResponse(hislip_instr_t *hislip_instr);
err_t hislip_AsyncStatusQuery(hislip_instr_t* hislip_instr);

hislip_msg_t hislip_MsgParser(hislip_instr_t *hislip_instr);
hislip_msg_type_t hislip_Recv(hislip_instr_t *hislip_instr);
err_t hislip_MsgSend(hislip_instr_t *hislip_instr, hislip_msg_t *msg_tx, size_t msg_size);
void hislip_htonl(hislip_msg_t *hislip_msg);

void hislip_CreateTask(void);

#endif /* HISLIP_INC_HISLIP_H_ */
