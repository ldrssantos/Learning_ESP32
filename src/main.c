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
#define TACTILE_GPIO GPIO_NUM_22

//Variables defines
uint32_t counter = 0;
uint32_t led_flag = 0;

// callback do botão 
static void IRAM_ATTR gpio_isr_handler(void *arg){
    if (TACTILE_GPIO == (uint32_t) arg){
        if (gpio_get_level(TACTILE_GPIO) == 0){
            counter++;
        }
        led_flag = 1;
    }
};


void blink_task(void *pvParameter)
{
    /* Configure the IOMUX register for pad BLINK_GPIO (some pads are
       muxed to GPIO on reset already, but some default to other
       functions and need to be switched to GPIO. Consult the
       Technical Reference for a list of pads and their default
       functions.)
    */
    while(1) {
        /* Blink off (output low) */
        gpio_set_level(TASK_BLINK_GPIO, 0);
        vTaskDelay(1050 / portTICK_PERIOD_MS);
        /* Blink on (output high) */
        gpio_set_level(TASK_BLINK_GPIO, 1);
        vTaskDelay(1050 / portTICK_PERIOD_MS);
    }
};

// Identify ESP32 Device type and memory
void ESP32_IDENTIFY_DEVICE_TYPE(void)
{
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
};


// Configure all GPIOs 
void USER_GPIO_INIT(void)
{
    //insert atributes for tactile button 
    gpio_config_t gpio_user_config = {
        .intr_type = GPIO_INTR_NEGEDGE,
        .mode = GPIO_MODE_INPUT,
        .pin_bit_mask = (1ull << TACTILE_GPIO),
        .pull_down_en = 0,
        .pull_up_en = 1
    };
    //initilize tactile button 
    gpio_config(&gpio_user_config);

     //insert atributes for tactile button 
    gpio_config_t led_user_config = {
        .intr_type = GPIO_INTR_DISABLE,
        .mode = GPIO_MODE_OUTPUT,
        .pin_bit_mask = (1ULL << BLINK_GPIO) | (1ULL << TASK_BLINK_GPIO),
        .pull_down_en = 0
    };
    //initilize tactile button 
    gpio_config(&led_user_config);

    // create interrupt function with low level priority
    gpio_install_isr_service(ESP_INTR_FLAG_LEVEL1);
    gpio_isr_handler_add(TACTILE_GPIO, gpio_isr_handler, (void*) TACTILE_GPIO);
};


/* 
    Main Application execution 
*/
void app_main(void){
    ESP32_IDENTIFY_DEVICE_TYPE();

    USER_GPIO_INIT();

    xTaskCreate(&blink_task, "blink_task", configMINIMAL_STACK_SIZE, NULL, 5, NULL);

    while(1) { 
        vTaskDelay(1000 / portTICK_PERIOD_MS);
        if (led_flag){
            printf("counter = %d\n", counter);
            led_flag = 0;
        }
    }
};