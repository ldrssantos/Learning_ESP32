#include <stdio.h>
#include "sdkconfig.h"

// FreeRTOS functions 
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

// Espressif functions 
#include "esp_system.h"
#include "esp_spi_flash.h"
#include "esp_log.h"

//Adding GPIOs configuration 
#include "driver/gpio.h"

//Global defines
#define BLINK_GPIO GPIO_NUM_13
#define TASK_BLINK_GPIO GPIO_NUM_12

//Variables defines
uint32_t counter = 0;

void blink_task(void *pvParameter)
{
    /* Configure the IOMUX register for pad BLINK_GPIO (some pads are
       muxed to GPIO on reset already, but some default to other
       functions and need to be switched to GPIO. Consult the
       Technical Reference for a list of pads and their default
       functions.)
    */
    gpio_pad_select_gpio(TASK_BLINK_GPIO);
    /* Set the GPIO as a push/pull output */
    gpio_set_direction(TASK_BLINK_GPIO, GPIO_MODE_OUTPUT);
    while(1) {
        /* Blink off (output low) */
        gpio_set_level(TASK_BLINK_GPIO, 0);
        vTaskDelay(1050 / portTICK_PERIOD_MS);
        /* Blink on (output high) */
        gpio_set_level(TASK_BLINK_GPIO, 1);
        vTaskDelay(1050 / portTICK_PERIOD_MS);
    }
}

/* 
    Main Application execution 
*/
void app_main(void){
    //xTaskCreate(&blink_task, "blink_task", configMINIMAL_STACK_SIZE, NULL, 5, NULL);

    printf("***** ESP32 INICIALIZATION (by Leandro Santos) ***** \n");
    
    // Espressif Chipset information
    esp_chip_info_t chip_info;
    esp_chip_info(&chip_info);

    printf("ESP32 - %s for %d CPU cores - Wifi %s %s \n",
        CONFIG_IDF_TARGET,
        chip_info.cores,
        (chip_info.features & CHIP_FEATURE_BT) ? "/BT" : "",
        (chip_info.features & CHIP_FEATURE_BLE) ? "/BLE" : "");

    printf("Silicon version: %d \n", chip_info.revision);
    printf("FLASH size: %d MB (%s)\n",  
        (spi_flash_get_chip_size()  / (1024*1024)),
        (chip_info.features & CHIP_FEATURE_EMB_FLASH) ? "Embeded" : "External");

    gpio_pad_select_gpio(BLINK_GPIO);
    /* Set the GPIO as a push/pull output */
    gpio_set_direction(BLINK_GPIO, GPIO_MODE_OUTPUT);

    while(1) {
        printf("counter = %d\n", counter);
        /* Blink off (output low) */
        gpio_set_level(BLINK_GPIO, 0);
        vTaskDelay(1000 / portTICK_PERIOD_MS);
        /* Blink on (output high) */
        gpio_set_level(BLINK_GPIO, 1);
        vTaskDelay(1000 / portTICK_PERIOD_MS);
        counter++;
    }
}