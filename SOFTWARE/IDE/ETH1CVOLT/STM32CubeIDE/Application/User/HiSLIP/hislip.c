/*
 * HiSLIP.c
 *
 *  Created on: Jun 3, 2024
 *      Author: grzegorz
 */

#include "main.h"
#include "hislip.h"
//#include "SCPI_Def.h"

#define HISLIP_THREAD_STACKSIZE		2048

#define MAX_CLIENTS 5

extern NX_IP NetXDuoEthIpInstance;

NX_PACKET_POOL pool;
uint8_t pool_buffer[PACKET_POOL_SIZE];

hislip_instr_t hislip_instr;
ULONG client_index = 0;

static bool sync_conn_state = FALSE;
static bool async_conn_state = FALSE;

TX_THREAD server_thread;
TX_THREAD sync_thread;
TX_THREAD async_thread;

NX_TCP_SOCKET sync_socket;
NX_TCP_SOCKET async_socket;
UCHAR server_thread_stack[HISLIP_THREAD_STACKSIZE];
UCHAR sync_thread_stack[HISLIP_THREAD_STACKSIZE];
UCHAR async_thread_stack[HISLIP_THREAD_STACKSIZE];

TX_SEMAPHORE sem_client_conn;
TX_SEMAPHORE sem_async_thread;
TX_SEMAPHORE sync_disconn;
TX_SEMAPHORE async_disconn;

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

err_t hislip_MsgSend(hislip_instr_t *hislip_instr, hislip_msg_t *msg_tx, size_t msg_size)
{
	err_t err;

	NX_PACKET *packet;

	err = nx_packet_allocate(&pool, &packet, NX_TCP_PACKET,
	NX_WAIT_FOREVER);

	err = nx_packet_data_append(packet, msg_tx, msg_size, &pool,
	NX_WAIT_FOREVER);

	err = nx_tcp_socket_send(hislip_instr->socket, packet, NX_WAIT_FOREVER);

	return err;
}

hislip_msg_type_t hislip_Recv(hislip_instr_t *hislip_instr)
{

	uint32_t status;

	unsigned short len = 0;
	unsigned short offset = 0;
	hislip_msg_t hislip_msg;

	hislip_instr->netbuf.len = 0;
	status = nx_tcp_socket_receive(hislip_instr->socket, &hislip_instr->packet,
	NX_WAIT_FOREVER);

	if (NX_SUCCESS == status)
	{
		nx_packet_data_retrieve(hislip_instr->packet, hislip_instr->netbuf.data,
				&hislip_instr->netbuf.len);

		hislip_msg = hislip_MsgParser(hislip_instr);

	}
	else if (NX_NOT_CONNECTED)
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
	hislip_instr.socket = (NX_TCP_SOCKET*) thread_input;
	TX_THREAD *current_thread_ptr;
	current_thread_ptr = tx_thread_identify();
	uint32_t status = 0;

	while (1)
	{
		if (NX_SUCCESS == nx_tcp_server_socket_accept(hislip_instr.socket,
		NX_WAIT_FOREVER))
		{
			// Close the listening gate so no other connections hit the port
			//nx_tcp_server_socket_unlisten(&NetXDuoEthIpInstance, HISLIP_PORT);
			async_conn_state = TRUE;
			while (1)
			{
				// FIXED: Capture the return status from the receive function
				status = hislip_Recv(&hislip_instr);

				// FIXED: Explicitly break out of the processing while(1) loop on disconnect
				if (status == HISLIP_NOT_CONNECTED)
				{
					break;
				}

				switch (status)
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
				default:
					tx_thread_sleep(2);
					break;
				}
				tx_thread_sleep(2);
			}
		}
		if (async_conn_state)
		{
			// 4. Teardown and clean up the server socket state safely
			status = nx_tcp_socket_disconnect(hislip_instr.socket,
			NX_WAIT_FOREVER);
			status = nx_tcp_server_socket_unaccept(hislip_instr.socket);

			// FIXED: Removed aggressive relisten. Let the sync thread reset the port.

			// 5. Notify the system that the async connection has completely terminated
			tx_semaphore_put(&async_disconn);
			async_conn_state = FALSE;
		}
		tx_thread_sleep(2);
	}
}

void sync_thread_entry(ULONG thread_input)
{
	hislip_instr_t hislip_instr;
	hislip_Init(&hislip_instr);
	hislip_instr.socket = (NX_TCP_SOCKET*) thread_input;
	TX_THREAD *current_thread_ptr;
	current_thread_ptr = tx_thread_identify();
	uint32_t status = 0;

	status = nx_packet_pool_create(&pool, "Test pool", PAYLOAD_SIZE,
			pool_buffer, PAYLOAD_SIZE + sizeof(NX_PACKET));

	SCPI_Init(&scpi_hislip_context, scpi_commands, &scpi_hislip_interface,
			scpi_units_def, SCPI_IDN1, SCPI_IDN2, SCPI_IDN3, SCPI_IDN4,
			scpi_input_buffer, SCPI_INPUT_BUFFER_LENGTH, scpi_error_queue_data,
			SCPI_ERROR_QUEUE_SIZE);

	scpi_hislip_context.user_context = (void*) &hislip_instr;

	while (1)
	{

		if (NX_SUCCESS == nx_tcp_server_socket_accept(hislip_instr.socket,
		NX_WAIT_FOREVER))
		{
			// Close the listening gate so no other connections hit the port
			//nx_tcp_server_socket_unlisten(&NetXDuoEthIpInstance, HISLIP_PORT);
			sync_conn_state = TRUE;
			while (1)
			{
				status = hislip_Recv(&hislip_instr);

				if (status == HISLIP_NOT_CONNECTED)
				{
					break;
				}

				switch (status)
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

				default:
					break;
				}
				tx_thread_sleep(2);
			}
		}
		if (sync_conn_state)
		{
			status = nx_tcp_socket_disconnect(hislip_instr.socket,
			NX_WAIT_FOREVER);
			status = nx_tcp_server_socket_unaccept(hislip_instr.socket);
			status = nx_tcp_server_socket_relisten(&NetXDuoEthIpInstance,
			HISLIP_PORT, hislip_instr.socket);

			sync_conn_state = FALSE;
		}

		tx_thread_sleep(2);
	}
}

VOID hislip_listen_callback(NX_TCP_SOCKET *socket_ptr, UINT port)
{
	nx_tcp_server_socket_unaccept(socket_ptr);
	nx_tcp_server_socket_unlisten(&NetXDuoEthIpInstance, HISLIP_PORT);
	tx_semaphore_put(&sem_client_conn);
}

void server_thread_entry(ULONG thread_input)
{
	uint32_t status = TX_SUCCESS;

	tx_semaphore_create(&sem_client_conn, "sem_client_conn", 0);

// Create and bind the server socket
	nx_tcp_socket_create(&NetXDuoEthIpInstance, // ip_ptr
			&sync_socket,// socket_ptr
			"HiSLIP Sync Socket",// name_ptr
			NX_IP_NORMAL,// type_of_service
			NX_FRAGMENT_OKAY,// fragment_control
			NX_IP_TIME_TO_LIVE,// time_to_live
			HISLIP_TCP_WINDOW_SIZE,// window size
			NX_NULL,// tcp_disconnect_callback
			NX_NULL);// tcp_receive_callback

	nx_tcp_socket_create(&NetXDuoEthIpInstance, &async_socket,
			"HiSLIP Async Socket", NX_IP_NORMAL, NX_FRAGMENT_OKAY,
			NX_IP_TIME_TO_LIVE, HISLIP_TCP_WINDOW_SIZE, NX_NULL, NX_NULL);

	nx_tcp_server_socket_listen(&NetXDuoEthIpInstance, HISLIP_PORT,
			&sync_socket,
			MAX_CLIENTS, hislip_listen_callback);

	tx_thread_create(&sync_thread, "HiSLIP Sync Thread", sync_thread_entry,
			(ULONG )&sync_socket, sync_thread_stack, HISLIP_THREAD_STACKSIZE,
			10, 1, TX_NO_TIME_SLICE, TX_AUTO_START);


	status = tx_semaphore_get(&sem_client_conn, TX_WAIT_FOREVER);

	nx_tcp_server_socket_listen(&NetXDuoEthIpInstance, HISLIP_PORT,
			&async_socket,
			MAX_CLIENTS, hislip_listen_callback);


	tx_thread_create(&async_thread, "HiSLIP Async Thread", async_thread_entry,
			(ULONG )&async_socket, async_thread_stack, HISLIP_THREAD_STACKSIZE,
			11, 1, TX_NO_TIME_SLICE, TX_AUTO_START);


	while (1)
	{

		tx_thread_sleep(10);
	}
}

void hislip_CreateTask(void)
{

	tx_thread_create(&server_thread, "HiSLIP Server Thread",
			server_thread_entry, 0, server_thread_stack,
			HISLIP_THREAD_STACKSIZE, 2, 1, TX_NO_TIME_SLICE, TX_AUTO_START);

}

