/* Camera Example

    This example code is in the Public Domain (or CC0 licensed, at your option.)
    Unless required by applicable law or agreed to in writing, this
    software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
    CONDITIONS OF ANY KIND, either express or implied.
*/
#include <stdio.h>
#include <stdint.h>
#include <stddef.h>
#include <string.h>
#include "esp_wifi.h"
#include "esp_system.h"
#include "nvs_flash.h"
#include "esp_event.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_heap_caps.h"
#include "esp_log.h"
#include "esp_spiffs.h"
#include "lcd.h"
#include "jpeg.h"
#include "board.h"
#include "lvgl/lvgl.h"
#include "esp32_lvgl.h"
#include "lv_demo_benchmark.h"
#include "i2s_parallel.h"
#include "jd5858.h"

#define TAG "main"
//gpio
#define  CS   GPIO_NUM_45
#define  RST  GPIO_NUM_40
#define  RS   GPIO_NUM_42

#define  CLK  GPIO_NUM_41
#define  D0   GPIO_NUM_39
#define  D1   GPIO_NUM_38
#define  D2   GPIO_NUM_37
#define  D3   GPIO_NUM_36
#define  D4   GPIO_NUM_35
#define  D5   GPIO_NUM_34
#define  D6   GPIO_NUM_33
#define  D7   GPIO_NUM_21
#define  D8   GPIO_NUM_20
#define  D9   GPIO_NUM_19
#define  D10   GPIO_NUM_18
#define  D11   GPIO_NUM_17
#define  D12   GPIO_NUM_16
#define  D13   GPIO_NUM_15
#define  D14   GPIO_NUM_14
#define  D15   GPIO_NUM_13
#define  RD    GPIO_NUM_2

void app_main()
{   
    gpio_config_t io_conf;
    io_conf.intr_type = GPIO_PIN_INTR_DISABLE;
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pin_bit_mask = (1ULL << 9|1ULL<<12);
    io_conf.pull_down_en = 0;
    io_conf.pull_up_en = 0;
    gpio_config(&io_conf);
    gpio_set_level(9, 1);
    gpio_set_level(12, 0);

    uint8_t pin[] = {D0, D1, D2, D3, D4, D5, D6, D7, D8, D9, D10, D11, D12, D13, D14, D15};//D16, D17, D18, D19, D20, D21, D22, D23};
    i2s_parallel_config_t i2s_parallel_bus = {
        .bit_width = 16,
        .pin_clk = CLK,
        .pin_num = pin,
        .ws_clk_div  = 2 , //50/(2*2)
        .pin_cs = CS,
        .pin_rst = RST,
        .pin_rd = RD,
        .pin_rs =RS 
    };

    ESP_ERROR_CHECK(i2s_parallel_init(&i2s_parallel_bus));
    jd5858_init();
    ESP_LOGI(TAG,"i2s parallel init ok");
    lvgl_task_init();
    th_init();
#if 0
    lv_demo_benchmark();
    vTaskSuspend(NULL);
#else
    lv_obj_t * img = lv_img_create(lv_scr_act(), NULL);
    // lv_img_set_angle(img,900);
    LV_IMG_DECLARE(page1);
    LV_IMG_DECLARE(page2);
	/*From variable*/
    uint8_t cnt=0;
	while (1)
    {
        if(cnt%2==0)
            lv_img_set_src(img, &page1);
        else if(cnt%2==1)
            lv_img_set_src(img, &page2);
        cnt++;
        vTaskDelay(500);
    }
#endif
}

