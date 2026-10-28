/*
 * HiSLIP.c
 *
 *  Created on: Jun 3, 2024
 *      Author: grzegorz
 */


 #include <string.h>
#include <stdbool.h>
// #include "stddef.h"
#include "app_azure_rtos.h"
#include "app_netxduo.h"

#include "main.h"

#include "HiSLIP.h"
//#include "SCPI_Def.h"

#define HISLIP_THREAD_STACKSIZE		2048
#define HISLIP_PORT					4880

#define	NETCONN_ACCEPT_ON			(unsigned char)1
#define	NETCONN_ACCEPT_OFF			(unsigned char)0


#define SERVER_PORT 4880
#define MAX_CLIENTS 2

typedef struct
{
	char data[HISLIP_MAX_DATA_SIZE + sizeof(hislip_msg_t)];
	unsigned short len;
}hislip_netbuf_t;

typedef struct {
	hislip_msg_t msg;
	hislip_netbuf_t netbuf;
	NX_PACKET* packet;
	NX_TCP_SOCKET* socket;
	unsigned short session_id;
}hislip_instr_t;

#define err_t	unsigned int

static	unsigned char thread_count = 0;


TX_THREAD server_thread;
TX_THREAD sync_thread;
TX_THREAD async_thread;

NX_TCP_SOCKET socket_1;
NX_TCP_SOCKET socket_2;
UCHAR server_thread_stack[HISLIP_THREAD_STACKSIZE];
UCHAR sync_thread_stack[HISLIP_THREAD_STACKSIZE];
UCHAR async_thread_stack[HISLIP_THREAD_STACKSIZE];
extern NX_IP          NetXDuoEthIpInstance;

#define APP_TCP_INSTANT_ECHO	1

static hislip_msg_t hislip_MsgParser(hislip_instr_t* hislip_instr);

// ----------------------------------------------------------------------------

static void hislip_htonl(hislip_msg_t* hislip_msg)
{
    hislip_msg->msg_param = NX_CHANGE_ULONG_ENDIAN(hislip_msg->msg_param);
    hislip_msg->prologue = NX_CHANGE_USHORT_ENDIAN(hislip_msg->prologue);
    hislip_msg->payload_len.hi = NX_CHANGE_ULONG_ENDIAN(hislip_msg->payload_len.hi);
    hislip_msg->payload_len.lo = NX_CHANGE_ULONG_ENDIAN(hislip_msg->payload_len.lo);
}

size_t hislip_SumSize(size_t* sizes, size_t len)
{
	size_t sum = 0;

	for(unsigned char i = 0; i < len; i++)
	{
		sum += sizes[i];
	}

	return sum;
}

void hislip_CopyMemory(char* destination, void** sources, size_t* sizes, unsigned int num_sources)
{
    size_t offset = 0;
    for (unsigned int i = 0; i < num_sources; i++)
    {
        memcpy(destination + offset, sources[i], sizes[i]);
        offset += sizes[i];
    }
}

// ----------------------------------------------------------------------------

UCHAR  pool_buffer[1024];
UCHAR  pool_buffer1[1024];
UCHAR  pool_buffer2[1024];
NX_PACKET_POOL pool;
void hislip_DataEnd(hislip_instr_t* hislip_instr)
{

}

/*
err_t hislip_DataEnd(hislip_instr_t* hislip_instr)
{
	err_t err = 0;

	hislip_msg_t msg_rx, msg_tx;
	payload_len_t max_msg_size;

	char data[] = "ABCad,123123,432,123213\n";

	void* sources[] = {&msg_tx, &data};
	size_t sizes[] = {sizeof(hislip_msg_t), strlen(data)};

	msg_rx = hislip_MsgParser(hislip_instr);

	size_t sum = hislip_SumSize(sizes, 2);

	char payload[sum];

	msg_tx.prologue = HISLIP_PROLOGUE;
	msg_tx.msg_type = HISLIP_DATAEND;
	msg_tx.control_code = 0x00;
	msg_tx.msg_param = msg_rx.msg_param;
	msg_tx.payload_len.hi = 0;
	msg_tx.payload_len.lo = strlen(data);

	hislip_htonl(&msg_tx);

	hislip_CopyMemory(payload, sources, sizes, 2);

	vTaskDelay(pdMS_TO_TICKS(1));
	err = netconn_write(hislip_instr->netconn.newconn, payload, sum, NETCONN_NOFLAG);

	return err;
}
*/


err_t hislip_AsyncMaximumMessageSize(hislip_instr_t* hislip_instr)
{
	err_t err = 0;
	hislip_msg_t msg_tx;
	payload_len_t max_msg_size;
	ULONG status = 0;
	NX_PACKET* packet;
	void* sources[] = {&msg_tx, &max_msg_size};
	size_t sizes[] = {sizeof(hislip_msg_t), sizeof(payload_len_t)};

	size_t sum = hislip_SumSize(sizes, 2);

	char payload[sum];

	msg_tx.prologue = HISLIP_PROLOGUE;
	msg_tx.msg_type = HISLIP_ASYNCMAXIMUMMESSAGESIZERESPONSE;
	msg_tx.control_code = 0x00;
	msg_tx.msg_param = 0x00000000;
	msg_tx.payload_len.hi = 0;
	msg_tx.payload_len.lo = 8;

	max_msg_size.hi = 0;
	max_msg_size.lo = 1024;

	hislip_htonl(&msg_tx);

	hislip_CopyMemory(payload, sources, sizes, 2);

	tx_thread_sleep(1);
	status = nx_packet_pool_create(&pool, "Test pool",512, pool_buffer2,512  + sizeof(NX_PACKET));
	status = nx_packet_allocate(&pool, &packet, NX_TCP_PACKET, NX_WAIT_FOREVER);

	status = nx_packet_data_append(packet,&payload,sum,&pool,NX_WAIT_FOREVER);


	        /* immediately resend the same packet */
	status = nx_tcp_socket_send(hislip_instr->socket, packet, NX_WAIT_FOREVER);

	//nx_packet_release(packet);
	//nx_packet_pool_delete(pool);

	return err;
}


err_t hislip_AsyncInitialize(hislip_instr_t* hislip_instr)
{
	err_t err = 0;
	hislip_msg_t msg_rx, msg_tx;


	NX_PACKET* packet;
	unsigned int status;

	msg_rx = hislip_MsgParser(hislip_instr);

	hislip_instr->session_id = msg_rx.msg_param;

	msg_tx.prologue = HISLIP_PROLOGUE;
	msg_tx.msg_type = HISLIP_ASYNCINITIALIZERESPONSE;
	msg_tx.control_code = 0x00;
	msg_tx.msg_param = HISLIP_VENDOR_ID;
	msg_tx.payload_len.hi = 0;
	msg_tx.payload_len.lo = 0;

	hislip_htonl(&msg_tx);

	//vTaskDelay(pdMS_TO_TICKS(1));
	tx_thread_sleep(1);
	status = nx_packet_pool_create(&pool, "Test pool",512, pool_buffer1,512  + sizeof(NX_PACKET));
	status = nx_packet_allocate(&pool, &packet, NX_TCP_PACKET, NX_WAIT_FOREVER);

	status = nx_packet_data_append(packet,&msg_tx,sizeof(hislip_msg_t),&pool,NX_WAIT_FOREVER);


	        /* immediately resend the same packet */
	status = nx_tcp_socket_send(hislip_instr->socket, packet, NX_WAIT_FOREVER);

	//nx_packet_release(packet);
	//nx_packet_pool_delete(pool);

	return err;
}




err_t hislip_Initialize(hislip_instr_t* hislip_instr)
{
	err_t err = 0;
	char name_ref[] = "hislip0";
	char name_rx[12];
	char* data;
	hislip_msg_t msg_rx, msg_tx;

	NX_PACKET* packet;

	memset(name_rx, 0, 12);

	msg_rx = hislip_MsgParser(hislip_instr);

	data = &hislip_instr->netbuf.data;

	memcpy(name_rx, data + sizeof(hislip_msg_t), msg_rx.payload_len.lo);

	if(!strcmp(name_ref, name_rx))
	{
		msg_tx.prologue = HISLIP_PROLOGUE;
		msg_tx.msg_type = HISLIP_INITIALIZERESPONSE;
		msg_tx.control_code = 0x00;
		msg_tx.msg_param = 0x01000001;
		msg_tx.payload_len.hi = 0;
		msg_tx.payload_len.lo = 0;

		hislip_htonl(&msg_tx);

		unsigned int status;
		//vTaskDelay(pdMS_TO_TICKS(1));
		tx_thread_sleep(1);
		status = nx_packet_pool_create(&pool, "Test pool",512, pool_buffer,512  + sizeof(NX_PACKET));
		status = nx_packet_allocate(&pool, &packet, NX_TCP_PACKET, NX_WAIT_FOREVER);

		status = nx_packet_data_append(packet,&msg_tx,sizeof(hislip_msg_t),&pool,NX_WAIT_FOREVER);


		        /* immediately resend the same packet */
		status = nx_tcp_socket_send(hislip_instr->socket, packet, NX_WAIT_FOREVER);

		//nx_packet_release(packet);
		//nx_packet_pool_delete(pool);
		//err = netconn_write(hislip_instr->netconn.newconn, &msg_tx, sizeof(hislip_msg_t), NETCONN_NOFLAG);
	}

	return err;
}

// ----------------------------------------------------------------------------


static hislip_msg_t hislip_MsgParser(hislip_instr_t* hislip_instr)
{
	hislip_msg_t hislip_msg;


	size_t hislip_msg_size = sizeof(hislip_msg_t);

	memcpy(&hislip_msg, hislip_instr->netbuf.data, hislip_msg_size);


	hislip_htonl(&hislip_msg);

	return hislip_msg;
}

static hislip_msg_type_t hislip_Recv(hislip_instr_t* hislip_instr)
{

	unsigned int status;

	unsigned short len = 0;
	unsigned short offset = 0;
	hislip_msg_t hislip_msg;

	hislip_instr->netbuf.len = 0;
	status = nx_tcp_socket_receive(hislip_instr->socket, &hislip_instr->packet, NX_WAIT_FOREVER);

	if(NX_SUCCESS == status)
	{
		nx_packet_data_retrieve(hislip_instr->packet, hislip_instr->netbuf.data, &hislip_instr->netbuf.len);

		hislip_msg = hislip_MsgParser(hislip_instr);

    //   nx_tcp_socket_disconnect(hislip_instr->socket, NX_WAIT_FOREVER);
     //  nx_tcp_server_socket_unaccept(&hislip_instr->socket);
     //   nx_tcp_server_socket_relisten(&NetXDuoEthIpInstance, HISLIP_PORT, &hislip_instr->socket);
	}
	else
	{
        if(thread_count)
        {
        	//thread_count--;
        }

		return HISLIP_CONN_ERR;
	}

	return (hislip_msg_type_t)hislip_msg.msg_type;

}


void async_thread_entry(ULONG thread_input)
{
	hislip_instr_t hislip_instr;
	hislip_instr.socket = (NX_TCP_SOCKET *)thread_input;

	ULONG status =0;
	status = nx_tcp_server_socket_accept(&socket_2, NX_WAIT_FOREVER);
    while (1)
    {
    	switch(hislip_Recv(&hislip_instr))
		{
		//	case Initialize : hislip_Initialize(&hislip_instr);break;
			case AsyncInitialize : hislip_AsyncInitialize(&hislip_instr); break;
			case AsyncMaximumMessageSize : hislip_AsyncMaximumMessageSize(&hislip_instr);break;
			case DataEnd : hislip_DataEnd(&hislip_instr); break;

			case HISLIP_CONN_ERR :
				{
			        nx_tcp_socket_disconnect(hislip_instr.socket, NX_WAIT_FOREVER);
			        nx_tcp_server_socket_unaccept(&hislip_instr.socket);
			        nx_tcp_server_socket_relisten(&NetXDuoEthIpInstance, HISLIP_PORT, &hislip_instr.socket);
					//vTaskDelete(NULL);

				}; break;

			default : tx_thread_sleep(2); break;
		}
    	tx_thread_sleep(2);
    }
}

VOID hislip_listen_callback(NX_TCP_SOCKET *socket_ptr, UINT port);
void sync_thread_entry(ULONG thread_input)
{
	hislip_instr_t hislip_instr;
	hislip_instr.socket = (NX_TCP_SOCKET *)thread_input;
ULONG status =0;
	status = nx_tcp_server_socket_accept(&socket_1, NX_WAIT_FOREVER);
		//status = nx_tcp_server_socket_listen(&NetXDuoEthIpInstance, SERVER_PORT, &socket_2, MAX_CLIENTS, hislip_listen_callback);
    while (1)
    {
    	switch(hislip_Recv(&hislip_instr))
		{
			case Initialize : hislip_Initialize(&hislip_instr);
			//status = nx_tcp_socket_disconnect(&socket_1, NX_NO_WAIT);
			status = nx_tcp_server_socket_unaccept(&socket_1);
			status = nx_tcp_server_socket_unlisten(&NetXDuoEthIpInstance, SERVER_PORT);
			status = nx_tcp_server_socket_listen(&NetXDuoEthIpInstance, SERVER_PORT, &socket_2, MAX_CLIENTS, hislip_listen_callback);
			tx_thread_sleep(2);break;
			case AsyncInitialize : hislip_AsyncInitialize(&hislip_instr); break;
			case AsyncMaximumMessageSize : hislip_AsyncMaximumMessageSize(&hislip_instr);break;
			case DataEnd : hislip_DataEnd(&hislip_instr); break;

			case HISLIP_CONN_ERR :
				{
			        nx_tcp_socket_disconnect(hislip_instr.socket, NX_WAIT_FOREVER);
			        nx_tcp_server_socket_unaccept(&hislip_instr.socket);
			        nx_tcp_server_socket_relisten(&NetXDuoEthIpInstance, HISLIP_PORT, &hislip_instr.socket);

					//vTaskDelete(NULL);

				}; break;

			default : tx_thread_sleep(2); break;
		}
    	tx_thread_sleep(2);
    }
}
#define APP_TCP_SOCKET_NUM 2

static NX_TCP_SOCKET g_tcp_sck[APP_TCP_SOCKET_NUM];

static ULONG g_not_listening = TX_FALSE;




extern NX_IP NetXDuoEthIpInstance;
NX_PACKET_POOL my_pool;
TX_THREAD client_threads[MAX_CLIENTS];
NX_TCP_SOCKET client_sockets[MAX_CLIENTS];



hislip_instr_t hislip_instr;
ULONG client_index = 0;
ULONG status = 0;
VOID hislip_listen_callback(NX_TCP_SOCKET *socket_ptr, UINT port)
{
    // Important: Do NOT use blocking functions or nx_tcp_server_socket_accept inside here!

    if (client_index == 0)
    {
        tx_thread_create(&client_threads[client_index], "Sync Thread", sync_thread_entry, (ULONG)&socket_1,
        		sync_thread_stack, HISLIP_THREAD_STACKSIZE, 10, 1, TX_NO_TIME_SLICE, TX_AUTO_START);
        client_index++;
    }
    else if (client_index == 1)
    {
          tx_thread_create(&client_threads[client_index], "ASync Thread", async_thread_entry, (ULONG)&socket_2,
        		async_thread_stack, HISLIP_THREAD_STACKSIZE, 11, 1, TX_NO_TIME_SLICE, TX_AUTO_START);
        client_index++;
    }
   // client_index = (client_index + 1) % MAX_CLIENTS;
}

void server_thread_entry(ULONG thread_input)
{
    UINT s1, s2, s3;
    ULONG client_index = 0;

    // Create and bind the server socket
    nx_tcp_socket_create(&NetXDuoEthIpInstance, &socket_1, "Server Socket 1", NX_IP_NORMAL, NX_FRAGMENT_OKAY, NX_IP_TIME_TO_LIVE, 512, NX_NULL, NX_NULL);
    nx_tcp_socket_create(&NetXDuoEthIpInstance, &socket_2, "Server Socket 2", NX_IP_NORMAL, NX_FRAGMENT_OKAY, NX_IP_TIME_TO_LIVE, 512, NX_NULL, NX_NULL);

    nx_tcp_server_socket_listen(&NetXDuoEthIpInstance, SERVER_PORT, &socket_1, MAX_CLIENTS, hislip_listen_callback);

    while (1)
    {
        // Accept a client connection

        tx_thread_sleep(10);
    }
}

void hislip_CreateTask(void)
{

	tx_thread_create(&server_thread, "HiSLIP Server Thread", server_thread_entry, 0,
	                     server_thread_stack, HISLIP_THREAD_STACKSIZE, 2, 1, TX_NO_TIME_SLICE, TX_AUTO_START);


}


