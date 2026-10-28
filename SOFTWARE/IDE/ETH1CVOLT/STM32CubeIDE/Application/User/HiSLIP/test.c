/*
 * test.c
 *
 *  Created on: Oct 28, 2026
 *      Author: grzegorz
 */


// Sockets
static NX_TCP_SOCKET master_listen_socket;
static NX_TCP_SOCKET hislip_sync_socket;
static NX_TCP_SOCKET hislip_async_socket;

// Semaphores or Flags to wake up worker threads
static TX_SEMAPHORE  sync_thread_sem;
static TX_SEMAPHORE  async_thread_sem;

static UINT task_count = 0;

// --- 1. The Listen Callback ---
// This runs inside the NetX IP thread context whenever a new SYN packet hits HISLIP_PORT.
VOID hislip_listen_callback(NX_TCP_SOCKET *socket_ptr, UINT port)
{
    // Important: Do NOT use blocking functions or nx_tcp_server_socket_accept inside here!

    if (task_count == 0)
    {
        // Hand the incoming connection structure off to the Sync socket
        hislip_sync_socket.nx_tcp_socket_passive_listen_ptr = master_listen_socket.nx_tcp_socket_passive_listen_ptr;

        // Signal the Sync thread to call nx_tcp_server_socket_accept
        tx_semaphore_put(&sync_thread_sem);
        task_count++;
    }
    else if (task_count == 1)
    {
        // Hand the second incoming connection structure off to the Async socket
        hislip_async_socket.nx_tcp_socket_passive_listen_ptr = master_listen_socket.nx_tcp_socket_passive_listen_ptr;

        // Signal the Async thread to call nx_tcp_server_socket_accept
        tx_semaphore_put(&async_thread_sem);
        task_count++;
    }
}

// --- 2. Master Listen Initialization ---
void start_hislip_server(void)
{
    // Create the Master listening socket
    nx_tcp_socket_create(&NetXDuoEthIpInstance, &master_listen_socket, "HiSLIP Master",
                          NX_IP_NORMAL, NX_FRAGMENT_OK, NX_IP_TIME_TO_LIVE, 2048, NX_NULL, NX_NULL);

    // Create the worker sockets (Do NOT call listen on these!)
    nx_tcp_socket_create(&NetXDuoEthIpInstance, &hislip_sync_socket, "HiSLIP Sync",
                          NX_IP_NORMAL, NX_FRAGMENT_OK, NX_IP_TIME_TO_LIVE, 2048, NX_NULL, NX_NULL);
    nx_tcp_socket_create(&NetXDuoEthIpInstance, &hislip_async_socket, "HiSLIP Async",
                          NX_IP_NORMAL, NX_FRAGMENT_OK, NX_IP_TIME_TO_LIVE, 2048, NX_NULL, NX_NULL);

    // Initialize Semaphores
    tx_semaphore_create(&sync_thread_sem, "Sync Sem", 0);
    tx_semaphore_create(&async_thread_sem, "Async Sem", 0);

    // Tell the Master socket to listen, and attach the callback.
    // Set listen queue depth to 2 so it can queue both handshakes.
    nx_tcp_server_socket_listen(&NetXDuoEthIpInstance, HISLIP_PORT, &master_listen_socket, 2, hislip_listen_callback);
}

// --- 3. The Worker Tasks ---
VOID hislip_SyncTask(ULONG thread_input)
{
    NX_PACKET *data_packet;
    UCHAR my_buffer[512]; // Buffer where your raw data will be stored
    ULONG bytes_received;
    while(1)
    {
        // Wait until the listen callback hands a connection to us
        tx_semaphore_get(&sync_thread_sem, TX_WAIT_FOREVER);

        // Finalize the connection handshake safely in our own thread context
        if (nx_tcp_server_socket_accept(&hislip_sync_socket, NX_WAIT_FOREVER) == NX_SUCCESS)
        {
            // --- Connection Active! ---
            // Run your data loop here...
        	/* --- STEP 2: Extract raw bytes into your C buffer --- */
        	                nx_packet_data_retrieve(data_packet, my_buffer, &bytes_received);

        	                // Add a null terminator if you are expecting a string (like HTTP headers)
        	                if (bytes_received < sizeof(my_buffer)) {
        	                    my_buffer[bytes_received] = '\0';
        	                }

        	                /* --- STEP 3: Use your data --- */
        	                printf("Successfully read %lu bytes from client!\n", bytes_received);
        	                printf("Data: %s\n", my_buffer);

        	                /* --- STEP 4: Crucial memory cleanup --- */

            // If client disconnects or an error occurs:
            nx_tcp_socket_disconnect(&hislip_sync_socket, NX_WAIT_FOREVER);
            nx_tcp_server_socket_unaccept(&hislip_sync_socket);

            // Decrement the global client tracking counter so the listener can re-route next time
            task_count--;
        }
    }
}
