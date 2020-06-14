#include <stdio.h>
#include <freertos/FreeRTOS.h>
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_spi_flash.h"
#include "driver/gpio.h"

#include "wifi.h"
#include "uart.h"

void app_main()
{
    uart_init();
    wifi_init(); 
}