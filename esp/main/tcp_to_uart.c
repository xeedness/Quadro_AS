#include "tcp_to_uart.h"

#include "socket.h"
#include "uart.h"
#include "server.h"

#include <stdlib.h>
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_log.h"

#define WR_BUF_SIZE (BUF_SIZE)

static const char *TAG = "tcp_to_uart";
static uint8_t init[3] = "BEG";

/*void tcp_to_uart_init(int socket) {
    xTaskCreate(tcp_to_uart_task, "tcp_to_uart", 4096, NULL, 5, NULL);
}*/

void tcp_to_uart_task(void *pvParameters) {
    int socket = *((int*) pvParameters);
    ESP_LOGI(TAG, "Starting TCP2UART Task with socket %d.", socket);

    uint8_t *data = (uint8_t *) malloc(WR_BUF_SIZE);

    // Send startup sequence to mark the point where no debug output is send anymore
    int err = uart_send(init, sizeof(init));

    if(err < 0) {
        ESP_LOGI("Could not send uart init sequence: %d", err);
    } else {
        for (;;) {
            int received_bytes = socket_recv(socket, data, WR_BUF_SIZE);
            if(received_bytes < 0) {
                ESP_LOGI(TAG, "TCP Recv returned error: %d", received_bytes);
                break;
            } else {
                err = uart_send(data, received_bytes);
                if(err < 0) {
                    ESP_LOGI(TAG, "Uart Error occured: %d.", err);
                }
            }
            
        }
    }
    free(data);
    data = NULL;
    tcp_uart_task_dead();
    vTaskDelete(NULL);
}