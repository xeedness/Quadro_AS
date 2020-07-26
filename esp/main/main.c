/* Hello World Example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include <stdio.h>
#include <freertos/FreeRTOS.h>
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_spi_flash.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "nvs_flash.h"

#include "wifi.h"
#include "uart.h"
#include "server.h"

void app_main()
{
    ESP_ERROR_CHECK(nvs_flash_init());

    uart_init();
    wifi_init(); 
    server_init();
}