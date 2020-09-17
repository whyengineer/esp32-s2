#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "lvgl/lvgl.h"
#include "lcd.h"
#include "board.h"
#include "esp_freertos_hooks.h"
#include "esp_log.h"
#include "i2s_parallel.h"
#include "jd5858.h"

#define TAG "lvgl"

#define PARALLEL

static lv_disp_buf_t disp_buf;
static lv_disp_drv_t* s_disp_driver=NULL;

static void lvgl_tick(){
    lv_tick_inc(1);
}


static void flush(lv_disp_drv_t * disp_drv, const lv_area_t * area, lv_color_t * color_p)
{
    s_disp_driver=disp_drv;
    /*The most simple case (but also the slowest) to put all pixels to the screen one-by-one*/
    uint32_t len=(area->x2-area->x1+1)*(area->y2-area->y1+1)*sizeof(lv_color_t);
    jd5858_set_index(area->x1, area->y1, area->x2, area->y2);
    lcdp_write_data((uint8_t *)color_p,len);
    /* IMPORTANT!!!
     * Inform the graphics library that you are ready with the flushing*/
    lv_disp_flush_ready(disp_drv);

}

void IRAM_ATTR lcd_write_done(){
    lv_disp_flush_ready(s_disp_driver);
}

static void lvgl_task (void* param){
    while(1) {
        lv_task_handler();
        vTaskDelay(5);
    }
}


void lvgl_task_init(){
    lv_init();
    esp_register_freertos_tick_hook_for_cpu(lvgl_tick,0);

    lv_color_t *buf1 = (lv_color_t *)heap_caps_calloc(LV_HOR_RES_MAX * 50, sizeof(lv_color_t), MALLOC_CAP_32BIT);
    lv_color_t *buf2 = (lv_color_t *)heap_caps_calloc(LV_HOR_RES_MAX * 50, sizeof(lv_color_t), MALLOC_CAP_32BIT);
    if((buf1==NULL)||(buf2==NULL)){
        ESP_LOGE(TAG,"calloc failed");
        abort();
    }
    lv_disp_buf_init(&disp_buf, buf1, buf2, LV_HOR_RES_MAX * 50);   /*Initialize the display buffer*/

    lv_disp_drv_t disp_drv;                         /*Descriptor of a display driver*/
    lv_disp_drv_init(&disp_drv);                    /*Basic initialization*/

    /*Set up the functions to access to your display*/

    /*Set the resolution of the display*/
    disp_drv.hor_res = LV_HOR_RES_MAX;
    disp_drv.ver_res = LV_VER_RES_MAX;

    /*Used to copy the buffer's content to the display*/
    disp_drv.flush_cb = flush;

    /*Set a display buffer*/
    disp_drv.buffer = &disp_buf;

    /*Finally register the driver*/
    lv_disp_drv_register(&disp_drv);
    xTaskCreate(lvgl_task,"lvgl",8096,NULL,5,NULL);
}


