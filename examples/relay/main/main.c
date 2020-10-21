/* Hello World Example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include <stdio.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "string.h"
#include "esp_log.h"
#include "esp_spi_flash.h"
#include "driver/gpio.h"
#include "driver/adc.h"
#include "esp_adc_cal.h"



#define TAG "RELAY"


#define RELAY_CTRL_1_PIN GPIO_NUM_40
#define RELAY_CTRL_2_PIN GPIO_NUM_39
#define RELAY_CTRL_3_PIN GPIO_NUM_38
#define LED_WIFI GPIO_NUM_33
#define LED_JOB GPIO_NUM_35
#define GPIO_OUTPUT_PIN_SEL  ((1ULL<<RELAY_CTRL_1_PIN) | (1ULL<<RELAY_CTRL_2_PIN) | (1ULL<<RELAY_CTRL_3_PIN) | (1ULL<<LED_WIFI) | (1ULL<<LED_JOB))

#define ADC_DET_1 ADC_CHANNEL_0
#define ADC_DET_2 ADC_CHANNEL_1
#define ADC_DET_3 ADC_CHANNEL_2


static esp_adc_cal_characteristics_t *adc_chars;


static void check_efuse(void)
{
    if (esp_adc_cal_check_efuse(ESP_ADC_CAL_VAL_EFUSE_TP) == ESP_OK) {
        printf("eFuse Two Point: Supported\n");
    } else {
        printf("Cannot retrieve eFuse Two Point calibration values. Default calibration values will be used.\n");
    }
}

static void adc_init(){
    adc1_config_width(ADC_WIDTH_BIT_13);
    adc1_config_channel_atten(ADC_CHANNEL_0, ADC_ATTEN_0db );
    adc1_config_channel_atten(ADC_CHANNEL_1, ADC_ATTEN_0db );
    adc1_config_channel_atten(ADC_CHANNEL_2, ADC_ATTEN_0db );
    adc_chars = calloc(1, sizeof(esp_adc_cal_characteristics_t));
    esp_adc_cal_value_t val_type = esp_adc_cal_characterize(ADC_UNIT_1, ADC_ATTEN_0db, ADC_WIDTH_BIT_13, 1100, adc_chars);
    //Check type of calibration value used to characterize ADC
    if (val_type == ESP_ADC_CAL_VAL_EFUSE_VREF) {
        printf("eFuse Vref");
    } else if (val_type == ESP_ADC_CAL_VAL_EFUSE_TP) {
        printf("Two Point");
    } else {
        printf("Default");
    }
}

static void gpio_task_example(void* arg)
{
    uint32_t cnt=0;
    for(;;) {
        cnt++;
        vTaskDelay(5000 / portTICK_RATE_MS);
        gpio_set_level(RELAY_CTRL_1_PIN, cnt % 2);
        gpio_set_level(RELAY_CTRL_2_PIN, cnt % 2);
        gpio_set_level(RELAY_CTRL_3_PIN, cnt % 2);
        gpio_set_level(LED_JOB, cnt % 2);
        gpio_set_level(LED_WIFI, cnt % 2);
    }
}



void app_main(void)
{
    uint32_t adc_reading = 0;
    uint32_t voltage = 0;
    /*gpio*/
    gpio_config_t io_conf;
    io_conf.intr_type = GPIO_INTR_DISABLE;
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pin_bit_mask = GPIO_OUTPUT_PIN_SEL;
    io_conf.pull_down_en = 1;
    io_conf.pull_up_en = 0;
    gpio_config(&io_conf);
    xTaskCreate(gpio_task_example, "gpio_task_example", 2048, NULL, 10, NULL);

    check_efuse();
    adc_init();
    //Continuously sample ADC1
    while (1) {
        //1
        
        adc_reading = adc1_get_raw((adc1_channel_t)ADC_DET_1);
        //Convert adc_reading to voltage in mV
        voltage = esp_adc_cal_raw_to_voltage(adc_reading, adc_chars);
        //printf("ADC1:Raw: %d\tVoltage: %dmV\n", adc_reading, voltage);
        //2
        adc_reading = adc1_get_raw((adc1_channel_t)ADC_DET_2);
        //Convert adc_reading to voltage in mV
        voltage = esp_adc_cal_raw_to_voltage(adc_reading, adc_chars);
        //printf("ADC2:Raw: %d\tVoltage: %dmV\n", adc_reading, voltage);
        //3
        adc_reading = adc1_get_raw((adc1_channel_t)ADC_DET_3);
        //Convert adc_reading to voltage in mV
        voltage = esp_adc_cal_raw_to_voltage(adc_reading, adc_chars);
        //printf("ADC3:Raw: %d\tVoltage: %dmV\n", adc_reading, voltage);
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
