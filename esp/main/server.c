#include "server.h"

#include <freertos/FreeRTOS.h>
#include "freertos/task.h"
#include "esp_log.h"

#include "socket.h"
#include "tcp_to_uart.h"
#include "uart_to_tcp.h"

static const char *TAG = "server";

static void tcp_server_task(void);
static int worker_socket = -1;

static TaskHandle_t tcpUartTaskHandle = NULL;
static TaskHandle_t uartTcpTaskHandle = NULL;

void server_init(void)  {
    xTaskCreate(tcp_server_task, "tcp_server", 4096, NULL, 5, NULL);
}

void tcp_server_task(void) {
    tcpUartTaskHandle = NULL;
    uartTcpTaskHandle = NULL;

    open_listening_socket();
    while(1) {
        int new_socket = accept_worker_socket();
        if(new_socket < 0) {
            open_listening_socket();
            // TODO Maybe sleep a while
            continue;
        }
        ESP_LOGI(TAG, "Opened new worker socket %d.", new_socket);
        if(tcpUartTaskHandle != NULL) {
            ESP_LOGI(TAG, "Killing TCP2Uart Task.");
            // TODO What happens if the task is already gone?
            vTaskDelete(tcpUartTaskHandle);
            tcpUartTaskHandle = NULL;
        }

        if(uartTcpTaskHandle != NULL) {
            ESP_LOGI(TAG, "Killing UART2TCP Task.");
            // TODO What happens if the task is already gone?
            vTaskDelete(uartTcpTaskHandle);
            uartTcpTaskHandle = NULL;
        }

        ESP_LOGI(TAG, "Previously existing tasks deleted");

        if(worker_socket >= 0) {
            ESP_LOGI(TAG, "Killing Worker Socket %d.", worker_socket);
            socket_kill(worker_socket);
        }
        worker_socket = new_socket;

        // TODO Add Socket parameter
        xTaskCreate(tcp_to_uart_task, "tcp_to_uart", 4096, (void*)&worker_socket, 5, &tcpUartTaskHandle);
        xTaskCreate(uart_to_tcp_task, "uart_to_tcp", 4096, (void*)&worker_socket, 5, &uartTcpTaskHandle);
    }
}

void tcp_uart_task_dead(void) {
    ESP_LOGI(TAG, "TCP2UART Dead. Killing worker socket %d.", worker_socket);
    tcpUartTaskHandle = NULL;
    socket_kill(worker_socket);
    worker_socket = -1;
    
}

void uart_tcp_task_dead(void) {
    ESP_LOGI(TAG, "UART2TCP Dead. Killing worker socket %d.", worker_socket);
    uartTcpTaskHandle = NULL;
    socket_kill(worker_socket);
    worker_socket = -1;
}