#include "tcp_to_uart.h"

#include "socket.h"
#include "uart.h"

#include <stdlib.h>
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_log.h"

#define WR_BUF_SIZE (BUF_SIZE)

static const char *TAG = "tcp_to_uart";
static uint8_t init[3] = "BEG";

void tcp_to_uart_init(void) {
    xTaskCreate(tcp_to_uart_task, "tcp_to_uart", 4096, NULL, 5, NULL);
}

void tcp_to_uart_task(void *pvParameters) {
    uint8_t *data = (uint8_t *) malloc(WR_BUF_SIZE);

    // Send startup sequence to mark the point where no debug output is send anymore
    int err = uart_send(init, sizeof(init));

    for (;;) {
        connect_recv_socket();
        if(err >= 0) {
            for (;;) {
                int received_bytes = tcp_socket_recv(data, WR_BUF_SIZE);
                if(received_bytes < 0) {
                    ESP_LOGI(TAG, "TCP Recv returned error: %d", received_bytes);
                    break;
                } else {
                    int err = uart_send(data, received_bytes);
                    if(err < 0) {
                        ESP_LOGI(TAG, "Socket error occured. Shutting down uart to tcp task");
                        break;
                    }
                }
            }
        }
        kill_recv_socket();
    }

    free(data);
    data = NULL;
    vTaskDelete(NULL);
}