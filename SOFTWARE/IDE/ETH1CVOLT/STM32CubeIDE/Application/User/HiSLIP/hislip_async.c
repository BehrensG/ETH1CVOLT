/*
 * hislip_async.c
 *
 *  Created on: Nov 4, 2026
 *      Author: grzegorz
 */


#include "hislip.h"

#define ERR_OK 0

// --------------------------------------------------------------------------------------------------------------------

err_t hislip_AsyncInitialize(hislip_instr_t *hislip_instr)
{
	err_t err = 0;
	hislip_msg_t msg_rx, msg_tx;

	msg_rx = hislip_MsgParser(hislip_instr);

	hislip_instr->session_id = msg_rx.msg_param;

	msg_tx.prologue = HISLIP_PROLOGUE;
	msg_tx.msg_type = HISLIP_ASYNC_INITIALIZE_RESPONSE;
	msg_tx.control_code = 0x00;
	msg_tx.msg_param = HISLIP_VENDOR_ID;
	msg_tx.payload_len.hi = 0;
	msg_tx.payload_len.lo = 0;

	hislip_htonl(&msg_tx);

	err = hislip_MsgSend(hislip_instr, &msg_tx,sizeof(msg_tx));

	return err;
}

// --------------------------------------------------------------------------------------------------------------------

err_t hislip_AsyncMaximumMessageSize(hislip_instr_t *hislip_instr)
{
	err_t err = 0;
	hislip_msg_t msg_tx;
	payload_len_t max_msg_size;

	void *sources[] =
	{ &msg_tx, &max_msg_size };
	size_t sizes[] =
	{ sizeof(hislip_msg_t), sizeof(payload_len_t) };

	size_t sum = hislip_SumSize(sizes, 2);

	char payload[sum];

	msg_tx.prologue = HISLIP_PROLOGUE;
	msg_tx.msg_type = HISLIP_ASYNC_MAXIMUM_MESSAGE_SIZE_RESPONSE;
	msg_tx.control_code = 0x00;
	msg_tx.msg_param = 0x00000000;
	msg_tx.payload_len.hi = 0;
	msg_tx.payload_len.lo = 8;

	max_msg_size.hi = 0;
	max_msg_size.lo = 1024;

	hislip_htonl(&msg_tx);

	hislip_CopyMemory(payload, sources, sizes, 2);

	err = hislip_MsgSend(hislip_instr, &payload, sizeof(payload));

	return err;
}

// --------------------------------------------------------------------------------------------------------------------

err_t hislip_AsyncMaximumMessageSizeResponse(hislip_instr_t* hislip_instr)
{
	err_t err = 0;
	hislip_msg_t msg_tx;
	payload_len_t max_msg_size;


	void* sources[] = {&msg_tx, &max_msg_size};
	size_t sizes[] = {sizeof(hislip_msg_t), sizeof(payload_len_t)};

	size_t sum = hislip_SumSize(sizes, 2);

	char payload[sum];

	msg_tx.prologue = HISLIP_PROLOGUE;
	msg_tx.msg_type = HISLIP_ASYNC_MAXIMUM_MESSAGE_SIZE_RESPONSE;
	msg_tx.control_code = 0x00;
	msg_tx.msg_param = 0x00000000;
	msg_tx.payload_len.hi = 0;
	msg_tx.payload_len.lo = 8;

	max_msg_size.hi = 0;
	max_msg_size.lo = 1024;

	hislip_htonl(&msg_tx);

	hislip_CopyMemory(payload, sources, sizes, 2);

	err = hislip_MsgSend(hislip_instr, &payload, sizeof(payload));

	return err;
}


// --------------------------------------------------------------------------------------------------------------------

err_t hislip_AsyncInitializeResponse(hislip_instr_t* hislip_instr)
{
	err_t err = 0;
	hislip_msg_t msg_rx, msg_tx;

	msg_rx = hislip_MsgParser(hislip_instr);

	hislip_instr->session_id = msg_rx.msg_param;

	msg_tx.prologue = HISLIP_PROLOGUE;
	msg_tx.msg_type = HISLIP_ASYNC_INITIALIZE_RESPONSE;
	msg_tx.control_code = 0x00;
	msg_tx.msg_param = HISLIP_VENDOR_ID;
	msg_tx.payload_len.hi = 0;
	msg_tx.payload_len.lo = 0;

	hislip_htonl(&msg_tx);

	err = hislip_MsgSend(hislip_instr, &msg_tx,sizeof(msg_tx));

	return err;
}

// --------------------------------------------------------------------------------------------------------------------

err_t hislip_AsyncDeviceClearAcknowledge(hislip_instr_t* hislip_instr)
{
	err_t err = 0;
	//hislip_msg_t msg_rx;
	hislip_msg_t msg_tx;

	//msg_rx = hislip_MsgParser(hislip_instr);

	//hislip_instr->session_id = msg_rx.msg_param;

	msg_tx.prologue = HISLIP_PROLOGUE;
	msg_tx.msg_type = HISLIP_ASYNC_DEVICE_CLEAR_ACKNOWLEDGE;
	msg_tx.control_code = 0x00;
	msg_tx.msg_param = 0x00000000;
	msg_tx.payload_len.hi = 0;
	msg_tx.payload_len.lo = 0;

	hislip_htonl(&msg_tx);

	err = hislip_MsgSend(hislip_instr, &msg_tx,sizeof(msg_tx));

	return err;
}

// --------------------------------------------------------------------------------------------------------------------

err_t hislip_AsyncLockResponse(hislip_instr_t* hislip_instr)
{
	err_t err = ERR_OK;
	hislip_msg_t msg_rx, msg_tx;

	msg_rx = hislip_MsgParser(hislip_instr);

	hislip_instr->session_id = msg_rx.msg_param;

	msg_tx.prologue = HISLIP_PROLOGUE;
	msg_tx.msg_type = HISLIP_ASYNC_INITIALIZE_RESPONSE;
	msg_tx.control_code = 0x00;
	msg_tx.msg_param = 0x00000000;
	msg_tx.payload_len.hi = 0;
	msg_tx.payload_len.lo = 0;

	hislip_htonl(&msg_tx);

	err = hislip_MsgSend(hislip_instr, &msg_tx,sizeof(msg_tx));

	return err;
}


// --------------------------------------------------------------------------------------------------------------------

err_t hislip_AsyncLockInfoResponse(hislip_instr_t* hislip_instr)
{
	err_t err = ERR_OK;
	hislip_msg_t msg_tx;

	msg_tx.prologue = HISLIP_PROLOGUE;
	msg_tx.msg_type = HISLIP_ASYNC_LOCK_INFO_RESPONSE;
	msg_tx.control_code = 0x00;
	msg_tx.msg_param = 0x00000000;
	msg_tx.payload_len.hi = 0;
	msg_tx.payload_len.lo = 0;

	hislip_htonl(&msg_tx);

	err = hislip_MsgSend(hislip_instr, &msg_tx,sizeof(msg_tx));

	return err;
}


// --------------------------------------------------------------------------------------------------------------------

err_t hislip_AsyncRemoteLocalResponse(hislip_instr_t* hislip_instr)
{
	err_t err = ERR_OK;
	hislip_msg_t msg_tx;

	msg_tx.prologue = HISLIP_PROLOGUE;
	msg_tx.msg_type = HISLIP_ASYNC_REMOTE_LOCAL_RESPONSE;
	msg_tx.control_code = 0x00;
	msg_tx.msg_param = 0x00000000;
	msg_tx.payload_len.hi = 0;
	msg_tx.payload_len.lo = 0;

	hislip_htonl(&msg_tx);

	err = hislip_MsgSend(hislip_instr, &msg_tx,sizeof(msg_tx));

	return err;
}

// --------------------------------------------------------------------------------------------------------------------

err_t hislip_AsyncInterrupted(hislip_instr_t* hislip_instr)
{
	err_t err = ERR_OK;
	//TBD
	return err;
}


// --------------------------------------------------------------------------------------------------------------------

err_t hislip_AsyncServiceRequest(hislip_instr_t* hislip_instr)
{
	err_t err = ERR_OK;
	//TBD
	return err;
}


// --------------------------------------------------------------------------------------------------------------------

err_t hislip_AsyncStatusResponse(hislip_instr_t* hislip_instr)
{
	err_t err = ERR_OK;
	//TBD
	return err;
}

// --------------------------------------------------------------------------------------------------------------------

err_t hislip_AsyncStatusQuery(hislip_instr_t* hislip_instr)
{
	err_t err = ERR_OK;

	hislip_msg_t  msg_tx;

	//msg_rx = hislip_MsgParser(hislip_instr);


	msg_tx.prologue = HISLIP_PROLOGUE;
	msg_tx.msg_type = HISLIP_ASYNC_STATUS_RESPONSE;
	msg_tx.control_code = 0x00;
	msg_tx.msg_param = 0x00000000;
	msg_tx.payload_len.hi = 0;
	msg_tx.payload_len.lo = 0;

	hislip_htonl(&msg_tx);

	err = hislip_MsgSend(hislip_instr, &msg_tx,sizeof(msg_tx));


	return err;
}

