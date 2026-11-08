/*
 * scpi_server.c
 *
 *  Created on: Feb 2, 2020
 *      Author: grzegorz
 */

/*-
 * Copyright (c) 2012-2013 Jan Breuer,
 *
 * All Rights Reserved
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are
 * met:
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHORS ``AS IS'' AND ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE AUTHORS OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR
 * BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
 * OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN
 * IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/**
 * @file   scpi_server.c
 * @date   Thu Nov 15 10:58:45 UTC 2012
 *
 * @brief  TCP/IP SCPI Server
 *
 *
 */

// --------------------------------------------------------------------------------------------------------------------
#include <scpi_def.h>
#include <scpi_server.h>

#include "hislip.h"
#include "printf.h"
#include "scpi/scpi.h"
#include "stdlib.h"
#include "string.h"

// --------------------------------------------------------------------------------------------------------------------

// --------------------------------------------------------------------------------------------------------------------

#define SCPI_MSG_TIMEOUT 0
#define SCPI_MSG_TEST 1
#define SCPI_MSG_IO_LISTEN 2
#define SCPI_MSG_CONTROL_IO_LISTEN 3
#define SCPI_MSG_IO 4
#define SCPI_MSG_CONTROL_IO 5
#define SCPI_MSG_SET_ESE_REQ 6
#define SCPI_MSG_SET_ERROR 7

// --------------------------------------------------------------------------------------------------------------------

// -----------------------------------------------------------------------------------------------------------

#define SCPI_OUT_DATA_SIZE 64000
__attribute__((aligned(4))) static char scpi_out[SCPI_OUT_DATA_SIZE];

static size_t scpi_out_sum = 0;

// --------------------------------------------------------------------------------------------------------------------

size_t SCPI_WriteHiSLIP(scpi_t *context, const char *data, size_t len) {
	hislip_instr_t *hislip_instr = (hislip_instr_t*) context->user_context;

	if (NULL != hislip_instr) {
		uint16_t hs = 0;
		size_t test = 0;

		test = strlen(&context->end[0]);

		memcpy(&hs, data, sizeof(hs));
		hs = htons(hs);

		if ((len < SCPI_OUT_DATA_SIZE) && (HISLIP_PROLOGUE != hs)) {
			hislip_msg_t msg_tx;

			bool end = ((len == strlen(&context->end[0]))
					&& (!strcmp(data, &context->end[0])));

			if (!end) {
				memcpy(scpi_out + sizeof(hislip_msg_t) + scpi_out_sum, data,
						len);
				scpi_out_sum += len;
			}

			if (end) {
				hislip_DataHeader(hislip_instr, &msg_tx, HISLIP_DATAEND,
						scpi_out_sum + strlen(HISLIP_LINE_ENDING));

				memcpy(scpi_out, &msg_tx, sizeof(hislip_msg_t));

				memcpy(scpi_out + sizeof(hislip_msg_t) + scpi_out_sum,
				HISLIP_LINE_ENDING, strlen(HISLIP_LINE_ENDING));

				if (NX_SOC_ERROR
						!= send(hislip_instr->socket, scpi_out,
								sizeof(hislip_msg_t) + scpi_out_sum
										+ strlen(HISLIP_LINE_ENDING), 0)) {
					len = 0;
				}

				scpi_out_sum = 0;
				memset(scpi_out, 0, SCPI_OUT_DATA_SIZE);

			} else if (SCPI_OUT_DATA_SIZE <= scpi_out_sum)  // hislip_out full
					{
				// Should not enter this
				Error_Handler();
			}
		} else {
			uint32_t loops = len / SCPI_OUT_DATA_SIZE;
			uint32_t rest = len % SCPI_OUT_DATA_SIZE;
			uint32_t index = 0;

			for (size_t i = 0; i < loops; i++) {

				if (NX_SOC_ERROR != send(hislip_instr->socket, data + index,
				SCPI_OUT_DATA_SIZE, 0)) {
					len = 0;
				}
				index += SCPI_OUT_DATA_SIZE;
			}
			if (rest) {

				if (NX_SOC_ERROR
						!= send(hislip_instr->socket, data + index, rest, 0)) {
					len = 0;
				}
			}
		}
	}
	return len;
}

// --------------------------------------------------------------------------------------------------------------------

scpi_result_t SCPI_Flush(scpi_t *context) {
	return SCPI_RES_OK;
}

// -----------------------------------------------------------------------------------------------------------

int SCPI_Error(scpi_t *context, int_fast16_t err) {
	(void) context;
	// BEEP
	iprintf("**ERROR: %ld, \"%s\"\r\n", (int32_t) err,
			SCPI_ErrorTranslate(err));
	if (err != 0) {
		/* New error */
		/* Beep */
		/* Error LED ON */
	} else {
		/* No more errors in the queue */
		/* Error LED OFF */
	}
	return 0;
}

// -----------------------------------------------------------------------------------------------------------

scpi_result_t SCPI_Control(scpi_t *context, scpi_ctrl_name_t ctrl,
		scpi_reg_val_t val) {
	char b[16];

	if (SCPI_CTRL_SRQ == ctrl) {
		iprintf("**SRQ: 0x%X (%d)\r\n", val, val);
	} else {
		iprintf("**CTRL %02x: 0x%X (%d)\r\n", ctrl, val, val);
	}
	return SCPI_RES_OK;
}

// -----------------------------------------------------------------------------------------------------------

scpi_result_t SCPI_Reset(scpi_t *context) {
	(void) context;

	return SCPI_RES_OK;
}

// --------------------------------------------------------------------------------------------------------------------

scpi_result_t SCPI_SystemCommTcpipControlQ(scpi_t *context) {
	return SCPI_RES_OK;
}

// -----------------------------------------------------------------------------------------------------------

static void setEseReq(void) {
	SCPI_RegSetBits(&scpi_context, SCPI_REG_ESR, ESR_REQ);
}

// -----------------------------------------------------------------------------------------------------------

static void setError(int16_t err) {
	SCPI_ErrorPush(&scpi_context, err);
}

// -----------------------------------------------------------------------------------------------------------

void SCPI_RequestControl(void) {
	/* Avoid sending evtQueue message if ESR_REQ is already set
	 if((SCPI_RegGet(&scpi_context, SCPI_REG_ESR) & ESR_REQ) == 0) {
	 xQueueSend(user_data.evtQueue, &msg, 1000);
	 }
	 */

	// xQueueSend(user_data.evtQueue, &msg, 1000);
}

// -----------------------------------------------------------------------------------------------------------

void SCPI_AddError(int16_t err) {
}
