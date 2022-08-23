/* Hello World Example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_spi_flash.h"
#include "sensors.h"

#include "azure_iot_hub_client.h"


void app_main(void)
{
    esp_rom_printf("Hello world!\n");
    i2c_bus_init();
    esp_rom_printf("I2C Inited!\n");
    icm42688_init();
    mpl3115_init();
    sht40_init();
    esp_rom_printf("ICM42688 loaded\n");
    for (int i = 100; i >= 0; i--) {
        //esp_rom_printf("Restarting in %d seconds...\n", i);
        vTaskDelay(1000 / portTICK_PERIOD_MS);
        showAcce();
    }
    esp_rom_printf("Restarting now.\n");
    fflush(stdout);
    esp_restart();
}
