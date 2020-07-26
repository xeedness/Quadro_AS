#include "uart_to_tcp.h"

#include "socket.h"
#include "uart.h"
#include "server.h"

#include <stdlib.h>
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_log.h"

#define RD_BUF_SIZE (BUF_SIZE)

static const char *TAG = "uart_to_tcp";

void uart_to_tcp_task(void *pvParameters) {
    int socket = *((int*) pvParameters);
    ESP_LOGI(TAG, "Starting UART2TCP Task with socket %d.", socket);

    uint8_t *data = (uint8_t *) malloc(RD_BUF_SIZE);
    for (;;) {
        int received_bytes = uart_recv(data, RD_BUF_SIZE);
        if(received_bytes <= 0) {
            ESP_LOGI(TAG, "Uart Recv returned error: %d", received_bytes);
        } else {
            int err = socket_send(socket, data, received_bytes);
            if (err < 0) {
                ESP_LOGI(TAG, "Error occured during sending: errno %d", err);
                break;
            }
        }
    }

    free(data);
    data = NULL;
    uart_tcp_task_dead();
    vTaskDelete(NULL);
}