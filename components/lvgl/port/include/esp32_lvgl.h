#ifndef ESP32_LVGL_H
#define ESP32_LVGL_H


#include "esp_err.h"
#include "lvgl/lvgl.h"
#include "driver/i2c.h"

void lvgl_task_init();
esp_err_t th_init();



#endif