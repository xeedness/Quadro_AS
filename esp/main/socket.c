#include "socket.h"

#include <string.h>
#include <sys/param.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_event.h"
#include "nvs.h"
#include "nvs_flash.h"

#include "lwip/err.h"
#include "lwip/sockets.h"
#include "lwip/sys.h"
#include <lwip/netdb.h>

#define PORT 12345

static const char *TAG = "socket";
static int listen_sock = -1;

int open_listening_socket(void) {

    char addr_str[128];
    int addr_family;
    int ip_protocol;

    struct sockaddr_in destAddr;
    destAddr.sin_addr.s_addr = htonl(INADDR_ANY);
    destAddr.sin_family = AF_INET;
    destAddr.sin_port = htons(PORT);
    addr_family = AF_INET;
    ip_protocol = IPPROTO_IP;
    inet_ntoa_r(destAddr.sin_addr, addr_str, sizeof(addr_str) - 1);

    listen_sock = socket(addr_family, SOCK_STREAM, ip_protocol);
    if (listen_sock < 0) {
        ESP_LOGI(TAG, "Unable to create socket: errno %d", errno);
        return listen_sock;
    }

    int err = bind(listen_sock, (struct sockaddr *)&destAddr, sizeof(destAddr));
    if (err != 0) {
        ESP_LOGI(TAG, "Socket unable to bind: errno %d", errno);
        return err;
    }

    err = listen(listen_sock, 1);
    if (err != 0) {
        ESP_LOGI(TAG, "Error occured during listen: errno %d", errno);
        return err;
    }
    return err;
}

int accept_worker_socket(void) {
    ESP_LOGI(TAG,"Accepting conenctions...");
    struct sockaddr_in sourceAddr;
    uint addrLen = sizeof(sourceAddr);
    int sock = accept(listen_sock, (struct sockaddr *)&sourceAddr, &addrLen);
    if (sock < 0) {
        ESP_LOGI(TAG, "Unable to accept connection: errno %d", errno);
    }
    return sock;
        /*while (1) {
            int len = recv(sock, rx_buffer, sizeof(rx_buffer) - 1, 0);
            // Error occured during receiving
            if (len < 0) {
                ESP_LOGI(TAG, "recv failed: errno %d", errno);
                break;
            }
            // Connection closed
            else if (len == 0) {
                ESP_LOGI(TAG, "Connection closed");
                break;
            }
            // Data received
            else {
                inet_ntoa_r(((struct sockaddr_in *)&sourceAddr)->sin_addr.s_addr, addr_str, sizeof(addr_str) - 1);

                rx_buffer[len] = 0; // Null-terminate whatever we received and treat like a string
                ESP_LOGI(TAG, "Received %d bytes from %s:", len, addr_str);
                ESP_LOGI(TAG, "%s", rx_buffer);

                int err = send(sock, rx_buffer, len, 0);
                if (err < 0) {
                    ESP_LOGI(TAG, "Error occured during sending: errno %d", errno);
                    break;
                }
            }
        }*/

        // TODO Do sth with socket errors in the caller of this function
        /*if (sock != -1) {
            ESP_LOGI(TAG, "Shutting down socket and restarting...");
            shutdown(sock, 0);
            close(sock);
        }*/
    
    // TODO May be no task
    // vTaskDelete(NULL);
}

int socket_kill(int socket) {
    if (socket > 0) {
        ESP_LOGI(TAG, "Killing socket %d.", socket);
        shutdown(socket, 0);
        close(socket);
    }
    return 0;
}

int socket_send(int socket, uint8_t* data, int length)  {
    return send(socket, data, length, 0);
}

int socket_recv(int socket, uint8_t* data, int length)  {
    return recv(socket, data, length, 0);
}


//int socket_handle_send = 0;
//int socket_handle_recv = 0;

//static int connect_socket(int* handle, int port);

/*int connect_send_socket(void) {
    return connect_socket(&socket_handle_send, PORT_SEND);
}


int connect_recv_socket(void) {
    return connect_socket(&socket_handle_recv, PORT_RECV);
}

static int connect_socket(int* handle, int port) {
    char addr_str[128];
    int addr_family;
    int ip_protocol;
    struct sockaddr_in destAddr;
    destAddr.sin_addr.s_addr = inet_addr(HOST_IP_ADDR);
    destAddr.sin_family = AF_INET;
    destAddr.sin_port = htons(port);
    addr_family = AF_INET;
    ip_protocol = IPPROTO_IP;
    inet_ntoa_r(destAddr.sin_addr, addr_str, sizeof(addr_str) - 1);

    *handle = socket(addr_family, SOCK_STREAM, ip_protocol);
    if (*handle < 0) {
        ESP_LOGW(TAG, "Unable to create socket: errno %d", errno);
        return -1;
    }
    ESP_LOGI(TAG, "Socket created");
    int err = connect(*handle, (struct sockaddr *)&destAddr, sizeof(destAddr));
    if (err != 0) {
        ESP_LOGW(TAG, "Socket unable to connect: errno %d", errno);
    }
    ESP_LOGI(TAG, "Successfully connected");
    return err;
}

void kill_send_socket(void) {
    if (socket_handle_send != -1) {
        ESP_LOGW(TAG, "Shutting down send socket... Need restart routine.");
        shutdown(socket_handle_send, 0);
        close(socket_handle_send);
    }
}

*/
