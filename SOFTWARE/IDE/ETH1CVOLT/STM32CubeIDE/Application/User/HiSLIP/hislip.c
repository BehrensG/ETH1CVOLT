/*
 * HiSLIP.c
 *
 *  Created on: Jun 3, 2024
 *      Author: grzegorz
 */

#include "main.h"
#include "hislip.h"
#include "nxd_bsd.h"
//#include "SCPI_Def.h"

#define HISLIP_THREAD_STACKSIZE		2*HISLIP_TCP_BUFFER

#define MAX_CLIENTS 2

hislip_instr_t hislip_instr;

TX_THREAD server_thread;
TX_THREAD sync_thread;
TX_THREAD async_thread;
uint8_t client_count = 0;

// __attribute__((aligned(4)))  is critical to get it running
// also that all arrays have the same size
__attribute__((aligned(4)))     int8_t server_thread_stack[HISLIP_THREAD_STACKSIZE];
__attribute__((aligned(4)))     int8_t sync_thread_stack[HISLIP_THREAD_STACKSIZE];
__attribute__((aligned(4)))     int8_t async_thread_stack[HISLIP_THREAD_STACKSIZE];

// ----------------------------------------------------------------------------

size_t hislip_SumSize(size_t *sizes, size_t len)
{
	size_t sum = 0;

	for (unsigned char i = 0; i < len; i++)
	{
		sum += sizes[i];
	}

	return sum;
}

void hislip_CopyMemory(char *destination, void **sources, size_t *sizes,
		unsigned int num_sources)
{
	size_t offset = 0;
	for (unsigned int i = 0; i < num_sources; i++)
	{
		memcpy(destination + offset, sources[i], sizes[i]);
		offset += sizes[i];
	}
}

// ----------------------------------------------------------------------------
void hislip_Init(hislip_instr_t *hislip_instr)
{
	memset(hislip_instr->end, 0, sizeof(hislip_instr->end));
}

// ----------------------------------------------------------------------------

void hislip_htonl(hislip_msg_t *hislip_msg)
{
	hislip_msg->msg_param = NX_CHANGE_ULONG_ENDIAN(hislip_msg->msg_param);
	hislip_msg->prologue = NX_CHANGE_USHORT_ENDIAN(hislip_msg->prologue);
	hislip_msg->payload_len.hi = NX_CHANGE_ULONG_ENDIAN(
			hislip_msg->payload_len.hi);
	hislip_msg->payload_len.lo = NX_CHANGE_ULONG_ENDIAN(
			hislip_msg->payload_len.lo);
}

hislip_msg_t hislip_MsgParser(hislip_instr_t *hislip_instr)
{
	hislip_msg_t hislip_msg;

	size_t hislip_msg_size = sizeof(hislip_msg_t);

	memcpy(&hislip_msg, hislip_instr->netbuf.data, hislip_msg_size);

	hislip_htonl(&hislip_msg);

	return hislip_msg;
}

err_t hislip_MsgSend(hislip_instr_t *hislip_instr, hislip_msg_t *msg_tx,
		size_t msg_size)
{
	err_t status;

	status = send(hislip_instr->socket, msg_tx, msg_size, 0);

	return status;
}

hislip_msg_type_t hislip_Recv(hislip_instr_t *hislip_instr)
{
	hislip_msg_t hislip_msg;
	hislip_instr->netbuf.len = 0;

	hislip_instr->netbuf.len = recv(hislip_instr->socket,
			hislip_instr->netbuf.data, sizeof(hislip_instr->netbuf.data) - 1,
			0);

	if (0 < hislip_instr->netbuf.len)
	{
		hislip_msg = hislip_MsgParser(hislip_instr);

	}
	else if (NX_SOC_ERROR == hislip_instr->netbuf.len)
	{

		return HISLIP_NOT_CONNECTED;
	}
	else
	{
		return HISLIP_NOT_CONNECTED;
	}

	return (hislip_msg_type_t) hislip_msg.msg_type;

}

void hislip_listen_callback(NX_TCP_SOCKET *socket_ptr, UINT port);

void async_thread_entry(ULONG thread_input)
{
	hislip_instr_t hislip_instr;
	hislip_Init(&hislip_instr);
	hislip_instr.socket = (int32_t) thread_input;
	TX_THREAD *current_thread_ptr;
	current_thread_ptr = tx_thread_identify();
	uint32_t status = 0;

	while (1)
	{
		switch (hislip_Recv(&hislip_instr))
		{
		case AsyncInitialize:
			hislip_AsyncInitialize(&hislip_instr);
			break;
		case AsyncMaximumMessageSize:
			hislip_AsyncMaximumMessageSize(&hislip_instr);
			break;
		case AsyncStatusQuery:
			hislip_AsyncStatusQuery(&hislip_instr);
			break;
		case AsyncDeviceClear:
			hislip_AsyncDeviceClearAcknowledge(&hislip_instr);
			break;
		case AsyncLock:
			hislip_AsyncLockResponse(&hislip_instr);
			break;
		case AsyncLockInfo:
			hislip_AsyncLockInfoResponse(&hislip_instr);
			break;
		case AsyncRemoteLocalControl:
			hislip_AsyncRemoteLocalResponse(&hislip_instr);
			break;
		case GetDescriptors:
			hislip_GetDescriptorsResponse(&hislip_instr);
			break;
		case HISLIP_NOT_CONNECTED:
			soc_close(hislip_instr.socket);
			client_count--;
			tx_thread_terminate(current_thread_ptr);
			break;
		default:
			tx_thread_sleep(2);
			break;
		}
		tx_thread_sleep(2);
	}

}

void sync_thread_entry(ULONG thread_input)
{
	hislip_instr_t hislip_instr;
	hislip_Init(&hislip_instr);
	hislip_instr.socket = (int32_t) thread_input;
	TX_THREAD *current_thread_ptr;
	current_thread_ptr = tx_thread_identify();
	uint32_t status = 0;

	SCPI_Init(&scpi_hislip_context, scpi_commands, &scpi_hislip_interface,
			scpi_units_def, SCPI_IDN1, SCPI_IDN2, SCPI_IDN3, SCPI_IDN4,
			scpi_input_buffer, SCPI_INPUT_BUFFER_LENGTH, scpi_error_queue_data,
			SCPI_ERROR_QUEUE_SIZE);

	scpi_hislip_context.user_context = (void*) &hislip_instr;

	while (1)
	{
		switch (hislip_Recv(&hislip_instr))
		{
		case Initialize:
			hislip_InitializeResponse(&hislip_instr);
			break;
		case DataEnd:
			hislip_DataEnd(&hislip_instr);
			break;
		case Data:
			hislip_Data(&hislip_instr);
			break;
		case Trigger:
			hislip_Trigger(&hislip_instr);
			break;
		case GetDescriptors:
			hislip_GetDescriptorsResponse(&hislip_instr);
			break;
		case DeviceClearComplete:
			hislip_DeviceClearAcknowledge(&hislip_instr);
			break;
		case HISLIP_NOT_CONNECTED:
			soc_close(hislip_instr.socket);
			client_count--;
			tx_thread_terminate(current_thread_ptr);
			break;
		default:
			tx_thread_sleep(2);
			break;
		}
		tx_thread_sleep(2);
	}

}

void server_thread_entry(ULONG thread_input)
{
	int32_t server_socket;
	int32_t status;
	int32_t test;
	struct sockaddr_in server_addr;
	struct sockaddr_in client_addr;
	int32_t addr_len = sizeof(client_addr);
	int32_t optval = 1;

	server_socket = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);

	server_addr.sin_family = AF_INET;
	server_addr.sin_port = htons(HISLIP_PORT);
	server_addr.sin_addr.s_addr = INADDR_ANY;

	status = bind(server_socket, (struct sockaddr*) &server_addr,
			sizeof(server_addr));
	status = listen(server_socket, MAX_CLIENTS + 1);

	while (1)
	{
		for (uint8_t i = 0; i < 2; i++)
		{

			// 3. accept() blocks until client 1, then client 2 arrives
			int32_t client_connection = accept(server_socket,
					(struct sockaddr*) &client_addr, &addr_len);

			if ( NX_SOC_ERROR != client_connection)
			{
				setsockopt(client_connection, SOL_SOCKET, SO_KEEPALIVE, &optval,
						sizeof(optval));

				if (0 == client_count)
				{
					tx_thread_terminate(&sync_thread);
					tx_thread_delete(&sync_thread);

					status = tx_thread_create(&sync_thread,
							"HiSLIP Sync Thread", sync_thread_entry,
							(ULONG )client_connection, sync_thread_stack,
							HISLIP_THREAD_STACKSIZE, 10, 1, TX_NO_TIME_SLICE,
							TX_AUTO_START);
					if (status == TX_SUCCESS)
					{
						client_count++;
					}
				}
				else if (1 == client_count)
				{
					tx_thread_terminate(&async_thread);
					tx_thread_delete(&async_thread);

					status = tx_thread_create(&async_thread,
							"HiSLIP Async Thread", async_thread_entry,
							(ULONG )client_connection, async_thread_stack,
							HISLIP_THREAD_STACKSIZE, 11, 1, TX_NO_TIME_SLICE,
							TX_AUTO_START);
					if (status == TX_SUCCESS)
					{
						client_count++;
					}
				}
			}
		}
		tx_thread_sleep(10);
	}
}

void hislip_CreateTask(void)
{

	tx_thread_create(&server_thread, "HiSLIP Server Thread",
			server_thread_entry, 0, server_thread_stack,
			HISLIP_THREAD_STACKSIZE, 10, 1, TX_NO_TIME_SLICE, TX_AUTO_START);

}

