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

#define HOST_IP_ADDR "192.168.2.116"
#define PORT_SEND 12345
#define PORT_RECV 12346

static const char *TAG = "socket";

int socket_handle_send = 0;
int socket_handle_recv = 0;

static int connect_socket(int* handle, int port);

int connect_send_socket(void) {
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

void kill_recv_socket(void) {
    if (socket_handle_recv != -1) {
        ESP_LOGW(TAG, "Shutting down socket... Need restart routine.");
        shutdown(socket_handle_recv, 0);
        close(socket_handle_recv);
    }
}

int tcp_socket_send(uint8_t* data, int length)  {
    int err = send(socket_handle_send, data, length, 0);
    if (err < 0) {
        ESP_LOGW(TAG, "Error occured during sending: errno %d", errno);
    }
    return err;
}

int tcp_socket_recv(uint8_t* data, int length)  {
    int err = recv(socket_handle_recv, data, length, 0);
    if (err < 0) {
        ESP_LOGW(TAG, "Error occured during receiving: errno %d", errno);
    }
    return err;
}
