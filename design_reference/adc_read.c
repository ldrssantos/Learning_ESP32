// Includes gerais de codigo

#include "stdio.h"

#include "stdlib.h"

#include "string.h"

#include "math.h"



// Includes da plataforma ESP-IDF

#include "freertos/FreeRTOS.h"

#include "freertos/task.h"

#include "driver/gpio.h"

#include "driver/adc.h"

#include "driver/ledc.h"



// Defines e referencias

#define BUFFER_SIZE     1024

#define LED_PWM         GPIO_NUM_19     // GPIO19

#define ADC_POT         ADC1_CHANNEL_4  // GPIO32



// Tarefa "principal - main"

void app_main(void) {

    // Setup do modulo ADC1 para Canal4 (onde ligamos Pot.)

    adc1_config_width(ADC_WIDTH_12Bit);

    // Atenuacao para escala total - 0 a 3.3V

    adc1_config_channel_atten(ADC_POT, ADC_ATTEN_DB_11);



    while(1) {



        uint32_t adc_raw_read = 0;

        float adc_voltage_read = 0.0f;

        // Le sinal do ADC e adapta pra tensao

        adc_raw_read = adc1_get_raw(ADC_POT); // leitura numerica do ADC

        adc_voltage_read = adc_raw_read * 0.8;  // adaptacao com fator de escala

        // Formata mensagem para ser enviada em buffer

        printf("ADC RAW: %04d | ADC VOLT: %04.2f mV \n", adc_raw_read, adc_voltage_read);



        uint32_t ctrl_value = 0; // valor a ser controlado de pwm

        uint32_t duty_value = 0; // valor de duty cicle do pwm



        // Configuracao do timer para controle de PWM com lib LEDC

        ledc_timer_config_t ledc_timer = {

            .duty_resolution = LEDC_TIMER_12_BIT,

            .freq_hz = 5000,

            .speed_mode = LEDC_HIGH_SPEED_MODE,

            .timer_num = LEDC_TIMER_0

        };

		// Configura o timer de controle do LEDC de acordo com o struct de configuracao

        ledc_timer_config(&ledc_timer);

        // Configuracao do canal de controle PWM com Lib LEDC

        ledc_channel_config_t ledc_channel = {

            .channel = LEDC_CHANNEL_0,

            .duty = 0,

            .gpio_num = LED_PWM,

            .speed_mode = LEDC_HIGH_SPEED_MODE,

            .hpoint = 0,

            .timer_sel = LEDC_TIMER_0

        };

		// Configura o canal de LEDC de aocrdo com o stuct de configuracao

        ledc_channel_config(&ledc_channel);

		

		// escala leitura do AD de 0-100

        ctrl_value = (adc_raw_read * 100) / 4096; 

        // equiparacao de duty cicle para pwm (vide documentacao esp-idf para pwm)

        duty_value = (ctrl_value * pow(2, ledc_timer.duty_resolution)) / 100;

        // Seta novo duty cicle de PWM

        ledc_set_duty(ledc_channel.speed_mode,

            ledc_channel.channel, duty_value);

        // Atualiza novo duty cicle de pwm

        ledc_update_duty(ledc_channel.speed_mode,

            ledc_channel.channel);

        printf("PWM SET: %d \n", ctrl_value);

		// delay basico de 500ms

        vTaskDelay(500 / portTICK_PERIOD_MS);

    } // fim do while(1)

} // fim app_main