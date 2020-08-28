#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "lvgl/lvgl.h"
#include "lcd.h"
#include "board.h"
#include "esp_freertos_hooks.h"
#include "esp_log.h"
#include "i2s_parallel.h"


#define TAG "lvgl"

#define PARALLEL

static lv_disp_buf_t disp_buf;
static lv_disp_drv_t* s_disp_driver=NULL;

static void lvgl_tick(){
    lv_tick_inc(1);
}
static void IRAM_ATTR lcd_write_done(){
    if(s_disp_driver!=NULL){
        lv_disp_flush_ready(s_disp_driver);
    }
}

static void flush(lv_disp_drv_t * disp_drv, const lv_area_t * area, lv_color_t * color_p)
{
    s_disp_driver=disp_drv;
    /*The most simple case (but also the slowest) to put all pixels to the screen one-by-one*/
    uint32_t len=(area->x2-area->x1+1)*(area->y2-area->y1+1)*sizeof(lv_color_t);
#ifndef PARALLEL
    lcd_set_index(area->x1, area->y1, area->x2, area->y2);
    lcd_write_data((uint8_t *)color_p,len);
#else
    lcdp_set_index(area->x1, area->y1, area->x2, area->y2);
    lcdp_write_data((uint8_t *)color_p,len);

#endif
    /* IMPORTANT!!!
     * Inform the graphics library that you are ready with the flushing*/
#ifdef PARALLEL
    lv_disp_flush_ready(disp_drv);
#endif
}



void lvgl_task(void* param){
    lv_init();
    esp_register_freertos_tick_hook_for_cpu(lvgl_tick,0);
#ifndef PARALLEL
    lcd_config_t lcd_config = {
#ifdef CONFIG_LCD_ST7789
        .clk_fre         = 80 * 1000 * 1000, /*!< ILI9341 Stable frequency configuration */
#endif
#ifdef CONFIG_LCD_ILI9341
        .clk_fre         = 80 * 1000 * 1000, /*!< ILI9341 Stable frequency configuration */
#endif
        .pin_clk         = LCD_CLK,
        .pin_mosi        = LCD_MOSI,
        .pin_dc          = LCD_DC,
        .pin_cs          = LCD_CS,
        .pin_rst         = LCD_RST,
        .pin_bk          = LCD_BK,
        .max_buffer_size = 4095*2,
        .horizontal      = 2, /*!< 2: UP, 3: DOWN */
        .lcd_write_done =lcd_write_done,
    };
    lcd_init(&lcd_config);
#endif
    
#if 0
    lv_color_t *buf1 = (lv_color_t *)heap_caps_calloc(LV_HOR_RES_MAX * LV_VER_RES_MAX, sizeof(lv_color_t), MALLOC_CAP_SPIRAM);
    lv_color_t *buf2 = (lv_color_t *)heap_caps_calloc(LV_HOR_RES_MAX * LV_VER_RES_MAX, sizeof(lv_color_t), MALLOC_CAP_SPIRAM);
    if((buf1==NULL)||(buf2==NULL)){
        ESP_LOGE(TAG,"calloc failed");
        vTaskDelete(NULL);
    }
    lv_disp_buf_init(&disp_buf, buf1, buf2, LV_HOR_RES_MAX * LV_VER_RES_MAX);   /*Initialize the display buffer*/
#else
    lv_color_t *buf1 = (lv_color_t *)heap_caps_calloc(LV_HOR_RES_MAX * 40, sizeof(lv_color_t), MALLOC_CAP_DMA);
    lv_color_t *buf2 = (lv_color_t *)heap_caps_calloc(LV_HOR_RES_MAX * 40, sizeof(lv_color_t), MALLOC_CAP_DMA);
    if((buf1==NULL)||(buf2==NULL)){
        ESP_LOGE(TAG,"calloc failed");
        vTaskDelete(NULL);
    }
    lv_disp_buf_init(&disp_buf, buf1, buf2, LV_HOR_RES_MAX * 40);   /*Initialize the display buffer*/


#endif

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

    while(1) {
        lv_task_handler();
        vTaskDelay(5);
    }
}


