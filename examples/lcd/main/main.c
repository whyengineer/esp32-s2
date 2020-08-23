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



void app_main()
{
    xTaskCreate(lvgl_task,"lvgl",8096,NULL,5,NULL);
    vTaskDelay(1000);
#if 1
    lv_demo_benchmark();
    vTaskSuspend(NULL);
#else
    lv_obj_t * img = lv_img_create(lv_scr_act(), NULL);
    // lv_img_set_angle(img,900);
    LV_IMG_DECLARE(test1);
    LV_IMG_DECLARE(test2);
    LV_IMG_DECLARE(test3);
	/*From variable*/
    uint8_t cnt=0;
	while (1)
    {
        
        
        if(cnt%3==0)
            lv_img_set_src(img, &test1);
        else if(cnt%3==1)
            lv_img_set_src(img, &test2);
        else
            lv_img_set_src(img, &test3);
        cnt++;
        vTaskDelay(500);
    }
#endif
	

}

