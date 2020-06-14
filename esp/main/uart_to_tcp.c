#include "uart_to_tcp.h"

#include "socket.h"
#include "uart.h"

#include <stdlib.h>
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_log.h"

#define RD_BUF_SIZE (BUF_SIZE)

static const char *TAG = "uart_to_tcp";

void uart_to_tcp_init(void) {
    xTaskCreate(uart_to_tcp_task, "uart_to_tcp", 4096, NULL, 5, NULL);
}

void uart_to_tcp_task(void *pvParameters) {
    uint8_t *data = (uint8_t *) malloc(RD_BUF_SIZE);
    for (;;) {
        connect_send_socket();
        

        for (;;) {
            int received_bytes = uart_recv(data, RD_BUF_SIZE);
            if(received_bytes < 0) {
                ESP_LOGI(TAG, "Uart Recv returned error: %d", received_bytes);
            } else {
                int err = tcp_socket_send(data, received_bytes);
                if(err < 0) {
                    ESP_LOGI(TAG, "Socket error occured. Shutting down uart to tcp task");
                    break;
                }
            }
        }
        kill_send_socket();
    }
    free(data);
    data = NULL;
    vTaskDelete(NULL);
}