/**
 * ESP32-Exercicio-Timer - 
 * 
 */
#include <stdio.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_spi_flash.h"
#include "esp_log.h"
#include "driver/gpio.h"

#include "driver/timer.h"

#include <driver/adc.h>


//Global defines
#define ESP32_BLINK_GPIO GPIO_NUM_13
#define RED_LED_GPIO GPIO_NUM_21

#define ADC_POT ADC1_CHANNEL_4
#define GREEN_LED_PWM_GPIO GPIO_NUM_19

#define TACTILE_GPIO GPIO_NUM_22

#define TIMER_DIVIDER (16)
#define TIMER_SCALE (TIMER_BASE_CLK / TIMER_DIVIDER)

//Variables defines
uint32_t counter = 0;
uint32_t led_flag = 0;
bool led_ctrl = 0;

uint8_t Counter_values_options[4] = {5, 10, 15, 20};

// TIMER Structure for our code
typedef struct{
    int timer_group;
    int timer_idx;
    int alarm_interval;
    bool auto_reload;
} esp_timer_info_t;

// Timer event structure 
typedef struct{
    esp_timer_info_t info;
    uint64_t timer_counter_value;
}esp_timer_event_t;


/**
 * Tactile button callback function
 */
static void IRAM_ATTR gpio_isr_handler(void *arg){
    if (TACTILE_GPIO == (uint32_t) arg){
        if (gpio_get_level(TACTILE_GPIO) == 0){
            if (counter == (sizeof(Counter_values_options) - 1)){
                counter = 0;
            } else {
                counter++;
            }
        }
        led_flag = 1;
    }
}



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
        gpio_set_level(ESP32_BLINK_GPIO, 0);
        vTaskDelay(500 / portTICK_PERIOD_MS);
        /* Blink on (output high) */
        gpio_set_level(ESP32_BLINK_GPIO, 1);
        vTaskDelay(500 / portTICK_PERIOD_MS);
    }
};

/**
 * Identify ESP32 Device type and memory function
 */
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


/**
 * GPIOs configuration and initialize 
 */
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
        .pin_bit_mask = (1ULL << ESP32_BLINK_GPIO) | (1ULL << GREEN_LED_GPIO) | (1ULL << RED_LED_GPIO),
        .pull_down_en = 0
    };
    //initilize tactile button 
    gpio_config(&led_user_config);

    // create interrupt function with low level priority
    gpio_install_isr_service(ESP_INTR_FLAG_LEVEL1);
    gpio_isr_handler_add(TACTILE_GPIO, gpio_isr_handler, (void*) TACTILE_GPIO);
};


/**
 * TIMER callback function
 */
static bool IRAM_ATTR timer_group_isr_callback(void *args){
    BaseType_t high_task_awoken = pdFALSE;
    esp_timer_info_t *info = (esp_timer_event_t *) args;
    
    uint64_t timer_counter_value = timer_group_get_counter_value_in_isr(info->timer_group, info->timer_idx);

    esp_timer_event_t evt = {
        .info.timer_group = info->timer_group,
        .info.timer_idx = info->timer_idx,
        .info.auto_reload = info->auto_reload,
        .info.alarm_interval = info->alarm_interval,
        .timer_counter_value = timer_counter_value
    };
    
    if (!info->auto_reload){
        timer_counter_value += info->alarm_interval * TIMER_SCALE;
        timer_group_set_alarm_value_in_isr (info->timer_group, info->timer_idx, timer_counter_value);
    }

    gpio_set_level(RED_LED_GPIO, led_ctrl);
    led_ctrl = !led_ctrl;

    return high_task_awoken == pdTRUE;
}
/**
 * Timer initialize
 */
void USER_TIMER_INIT(void){
    timer_config_t config = {
        .divider = TIMER_DIVIDER,        
        .counter_dir = TIMER_COUNT_UP,   
        .counter_en = TIMER_PAUSE,       
        .alarm_en = TIMER_ALARM_EN,      
        .auto_reload = true,             
    };
    timer_init(TIMER_GROUP_0, TIMER_0, &config);

    uint32_t intervalo_em_segundos = 5;

    timer_set_counter_value(TIMER_GROUP_0, TIMER_0, 0); 
    timer_set_alarm_value(TIMER_GROUP_0, TIMER_0, intervalo_em_segundos * TIMER_SCALE);
    timer_enable_intr(TIMER_GROUP_0, TIMER_0); 

    esp_timer_info_t *timer_info = calloc(1, sizeof(esp_timer_info_t));
    timer_info->timer_group = TIMER_GROUP_0;
    timer_info->timer_idx = TIMER_0;
    timer_info->auto_reload = true;
    timer_info->alarm_interval = intervalo_em_segundos; 
    timer_isr_callback_add(TIMER_GROUP_0, TIMER_0, timer_group_isr_callback, timer_info, 0);

    timer_start(TIMER_GROUP_0, TIMER_0);
}

/**
 * Timer initialize
 */
void USER_TIMER_INIT(void){
    adc1_config_width(ADC_WIDTH_BIT_12);
    adc1_config_channel_atten(ADC1_CHANNEL_0,ADC_ATTEN_DB_0);
    int val = adc1_get_raw(ADC1_CHANNEL_0);
}

/**
 *  Main Application execution 
*/
void app_main(void){
    ESP32_IDENTIFY_DEVICE_TYPE();

    USER_GPIO_INIT();

    USER_TIMER_INIT();

    xTaskCreate(&blink_task, "blink_task", configMINIMAL_STACK_SIZE, NULL, 5, NULL);

    while(1) { 
        vTaskDelay(1000 / portTICK_PERIOD_MS);
        if (led_flag){
            printf("counter = %d\n", counter);
            printf("Counter_values_options = %d\n", Counter_values_options[counter]);
            
            led_flag = 0;
        }
    }
};