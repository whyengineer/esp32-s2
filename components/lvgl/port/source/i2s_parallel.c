#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"

#include "soc/i2s_struct.h"
#include "soc/apb_ctrl_reg.h"
#include "soc/io_mux_reg.h"
#include "soc/gpio_periph.h"
#include "esp32s2/rom/lldesc.h"
#include "esp32s2/rom/cache.h"
#include "soc/dport_access.h"
#include "soc/dport_reg.h"
#include "i2s_parallel.h"
#include "driver/gpio.h"
#include "esp_attr.h"

#include "esp_log.h"
#include "esp_err.h"

static const char *TAG = "i2s_parallel";
#define DMA_LIST_SIZE       40
#define DMA_MAX_NODE_SIZE   4095
#define TX_BUFFER_SIZE   (DMA_LIST_SIZE * DMA_MAX_NODE_SIZE)

typedef struct {
    i2s_parallel_config_t conf;
    SemaphoreHandle_t i2s_tx_sem;
    uint8_t *i2s_tx_buf;
    lldesc_t *dma_list;
	bool sync;
} i2s_parallel_obj_t;

i2s_parallel_obj_t *g_i2s_para_obj;
i2s_dev_t * _i2s_dev = &I2S0;

extern void IRAM_ATTR lcd_write_done();

static void IRAM_ATTR _i2s_isr(void *arg)
{
    if(_i2s_dev->int_st.out_total_eof) {
        BaseType_t HPTaskAwoken;
		if(g_i2s_para_obj->sync==true){
			xSemaphoreGiveFromISR(g_i2s_para_obj->i2s_tx_sem, &HPTaskAwoken);
			if(HPTaskAwoken == pdTRUE) {
				portYIELD_FROM_ISR();
			}
		}else{
			lcd_write_done();
		}
    }

    _i2s_dev->int_clr.val = 0xffffffff;
}

static inline void _i2s_tx_stop(void)
{
    _i2s_dev->conf.tx_start = 0;
    _i2s_dev->fifo_conf.dscr_en = 0;
    _i2s_dev->conf.tx_reset = 1;
    _i2s_dev->conf.tx_fifo_reset = 1;
}

static inline void _i2s_tx_start(void)
{
    _i2s_dev->conf.tx_reset = 0;
    _i2s_dev->conf.tx_fifo_reset = 0;
    _i2s_dev->fifo_conf.dscr_en = 1;
    _i2s_dev->out_link.start = 1;
    _i2s_dev->conf.tx_start = 1;
}

static inline void _i2s_dma_start(void)
{
    _i2s_dev->conf.tx_start = 0;
    _i2s_dev->fifo_conf.dscr_en = 0;
    _i2s_dev->conf.tx_reset = 1;
    _i2s_dev->conf.tx_fifo_reset = 1;

    _i2s_dev->conf.tx_reset = 0;
    _i2s_dev->conf.tx_fifo_reset = 0;
    _i2s_dev->fifo_conf.dscr_en = 1;
    _i2s_dev->out_link.start = 1;
    _i2s_dev->conf.tx_start = 1;
    
    xSemaphoreTake(g_i2s_para_obj->i2s_tx_sem, portMAX_DELAY);
    while (!_i2s_dev->state.tx_idle);
}

void i2s_para_write_async(uint8_t *data, uint32_t len)
{	
	g_i2s_para_obj->sync=false;
    const static int dma_half_index = DMA_LIST_SIZE / 2 - 1;
    const static int dma_last_index = DMA_LIST_SIZE - 1;
    const static int txbuf_half_size = TX_BUFFER_SIZE / 2;
    for (int pos = 0; pos < DMA_LIST_SIZE; pos++) {
        // printf("ping pang cnt:%d\n",pos);
        g_i2s_para_obj->dma_list[pos].size = DMA_MAX_NODE_SIZE;
        g_i2s_para_obj->dma_list[pos].length = DMA_MAX_NODE_SIZE;
        g_i2s_para_obj->dma_list[pos].buf = data + DMA_MAX_NODE_SIZE * pos;
        if(pos == dma_half_index || pos == dma_last_index) {
            g_i2s_para_obj->dma_list[pos].eof = 1;
        } else {
            g_i2s_para_obj->dma_list[pos].eof = 0;
        }
        g_i2s_para_obj->dma_list[pos].empty = (uint32_t)&g_i2s_para_obj->dma_list[(pos + 1) % DMA_LIST_SIZE];
    }
    g_i2s_para_obj->dma_list[dma_half_index].empty = (uint32_t)NULL;
    g_i2s_para_obj->dma_list[dma_last_index].empty = (uint32_t)NULL;

    int cnt = len / txbuf_half_size;
    for (int bufpos = 0; bufpos < cnt; bufpos++) {
		ESP_LOGE(TAG,"dma small error");
        _i2s_tx_stop();
        // memcpy(g_i2s_para_obj->dma_list[(bufpos % 2) * (dma_half_index+1)].buf, data, txbuf_half_size);
        data += txbuf_half_size;
        _i2s_dev->out_link.addr = ((uint32_t)&g_i2s_para_obj->dma_list[(bufpos % 2) * (dma_half_index+1)]);
        _i2s_tx_start();
        xSemaphoreTake(g_i2s_para_obj->i2s_tx_sem, portMAX_DELAY);
        // while (!_i2s_dev->state.tx_idle);
    }
    int rest_data_size = len % txbuf_half_size;
    if (rest_data_size) {
        _i2s_tx_stop();
        int end_index = 0;
        int end_size = 0;
        g_i2s_para_obj->dma_list[0].buf=data;
        // memcpy(g_i2s_para_obj->dma_list[0].buf, data, rest_data_size);
        if(rest_data_size % DMA_MAX_NODE_SIZE != 0) {
            end_index = rest_data_size / DMA_MAX_NODE_SIZE;
            end_size = rest_data_size % DMA_MAX_NODE_SIZE;
        } else {
            end_index = rest_data_size / DMA_MAX_NODE_SIZE - 1;
            end_size = DMA_MAX_NODE_SIZE;
        }
        g_i2s_para_obj->dma_list[end_index].size = end_size;
        g_i2s_para_obj->dma_list[end_index].length = end_size;
        g_i2s_para_obj->dma_list[end_index].eof = 1;
        g_i2s_para_obj->dma_list[end_index].empty = (uint32_t)NULL;
        _i2s_dev->out_link.addr = ((uint32_t)&g_i2s_para_obj->dma_list[0]);
        _i2s_tx_start();
        // while (!_i2s_dev->state.tx_idle);
    }
}



void i2s_para_write(uint8_t *data, uint32_t len)
{
	g_i2s_para_obj->sync=true;
    const static int dma_half_index = DMA_LIST_SIZE / 2 - 1;
    const static int dma_last_index = DMA_LIST_SIZE - 1;
    const static int txbuf_half_size = TX_BUFFER_SIZE / 2;
    for (int pos = 0; pos < DMA_LIST_SIZE; pos++) {
        // printf("ping pang cnt:%d\n",pos);
        g_i2s_para_obj->dma_list[pos].size = DMA_MAX_NODE_SIZE;
        g_i2s_para_obj->dma_list[pos].length = DMA_MAX_NODE_SIZE;
        g_i2s_para_obj->dma_list[pos].buf = data + DMA_MAX_NODE_SIZE * pos;
        if(pos == dma_half_index || pos == dma_last_index) {
            g_i2s_para_obj->dma_list[pos].eof = 1;
        } else {
            g_i2s_para_obj->dma_list[pos].eof = 0;
        }
        g_i2s_para_obj->dma_list[pos].empty = (uint32_t)&g_i2s_para_obj->dma_list[(pos + 1) % DMA_LIST_SIZE];
    }
    g_i2s_para_obj->dma_list[dma_half_index].empty = (uint32_t)NULL;
    g_i2s_para_obj->dma_list[dma_last_index].empty = (uint32_t)NULL;

    int cnt = len / txbuf_half_size;
    for (int bufpos = 0; bufpos < cnt; bufpos++) {
        _i2s_tx_stop();
        // memcpy(g_i2s_para_obj->dma_list[(bufpos % 2) * (dma_half_index+1)].buf, data, txbuf_half_size);
        data += txbuf_half_size;
        _i2s_dev->out_link.addr = ((uint32_t)&g_i2s_para_obj->dma_list[(bufpos % 2) * (dma_half_index+1)]);
        _i2s_tx_start();
        xSemaphoreTake(g_i2s_para_obj->i2s_tx_sem, portMAX_DELAY);
        // while (!_i2s_dev->state.tx_idle);
    }
    int rest_data_size = len % txbuf_half_size;
    if (rest_data_size) {
        _i2s_tx_stop();
        int end_index = 0;
        int end_size = 0;
        g_i2s_para_obj->dma_list[0].buf=data;
        // memcpy(g_i2s_para_obj->dma_list[0].buf, data, rest_data_size);
        if(rest_data_size % DMA_MAX_NODE_SIZE != 0) {
            end_index = rest_data_size / DMA_MAX_NODE_SIZE;
            end_size = rest_data_size % DMA_MAX_NODE_SIZE;
        } else {
            end_index = rest_data_size / DMA_MAX_NODE_SIZE - 1;
            end_size = DMA_MAX_NODE_SIZE;
        }
        g_i2s_para_obj->dma_list[end_index].size = end_size;
        g_i2s_para_obj->dma_list[end_index].length = end_size;
        g_i2s_para_obj->dma_list[end_index].eof = 1;
        g_i2s_para_obj->dma_list[end_index].empty = (uint32_t)NULL;
        _i2s_dev->out_link.addr = ((uint32_t)&g_i2s_para_obj->dma_list[0]);
        _i2s_tx_start();
        xSemaphoreTake(g_i2s_para_obj->i2s_tx_sem, portMAX_DELAY);
        // while (!_i2s_dev->state.tx_idle);
    }
}

static inline void _i2s_gpio_init(void)
{
    PIN_FUNC_SELECT(GPIO_PIN_MUX_REG[g_i2s_para_obj->conf.pin_clk], PIN_FUNC_GPIO);
    gpio_set_direction(g_i2s_para_obj->conf.pin_clk, GPIO_MODE_OUTPUT);
    gpio_set_pull_mode(g_i2s_para_obj->conf.pin_clk, GPIO_PULLUP_ONLY);
    gpio_matrix_out(g_i2s_para_obj->conf.pin_clk, I2S0O_WS_OUT_IDX, true, 0);
    for(int i = 0; i < g_i2s_para_obj->conf.bit_width; i++) {
        PIN_FUNC_SELECT(GPIO_PIN_MUX_REG[g_i2s_para_obj->conf.pin_num[i]], PIN_FUNC_GPIO);
        gpio_set_direction(g_i2s_para_obj->conf.pin_num[i], GPIO_MODE_OUTPUT);
        gpio_set_pull_mode(g_i2s_para_obj->conf.pin_num[i],GPIO_PULLUP_ONLY);
        gpio_matrix_out(g_i2s_para_obj->conf.pin_num[i], 
                            I2S0O_DATA_OUT0_IDX + (24 - g_i2s_para_obj->conf.bit_width) + i, false, 0);
    }
}

static inline void _i2s_reg_init(void)
{
    DPORT_CLEAR_PERI_REG_MASK(DPORT_PERIP_CLK_EN_REG, DPORT_I2S0_CLK_EN);
    DPORT_SET_PERI_REG_MASK(DPORT_PERIP_CLK_EN_REG, DPORT_I2S0_CLK_EN);
    DPORT_SET_PERI_REG_MASK(DPORT_PERIP_RST_EN_REG, DPORT_I2S0_RST);
    DPORT_CLEAR_PERI_REG_MASK(DPORT_PERIP_RST_EN_REG, DPORT_I2S0_RST);
    //Configure pclk, max: 20M
    _i2s_dev->clkm_conf.val = 0;
    _i2s_dev->clkm_conf.clk_en = 1;
    _i2s_dev->clkm_conf.clk_sel = 2;
    //F_i2s = F_i2s_clk_s / (N + b/a)    N-->clkm_div_num,b-->clkm_div_b,a-->clkm_div_a
    _i2s_dev->clkm_conf.clkm_div_num = 3;
    _i2s_dev->clkm_conf.clkm_div_b = 4;
    _i2s_dev->clkm_conf.clkm_div_a = 20;
    _i2s_dev->sample_rate_conf.val = 0;
    _i2s_dev->sample_rate_conf.tx_bck_div_num = g_i2s_para_obj->conf.ws_clk_div;

    _i2s_dev->conf.tx_slave_mod = 0;
    _i2s_dev->conf.tx_right_first = 1;
    _i2s_dev->conf.tx_msb_right = 1;
    _i2s_dev->conf.tx_dma_equal = 1;

    _i2s_dev->int_ena.val = 0;
    _i2s_dev->int_clr.val = 0xffffffff;

    _i2s_dev->conf1.val = 0;
    _i2s_dev->conf1.tx_pcm_bypass = 1;
    _i2s_dev->conf1.tx_stop_en = 1;
    _i2s_dev->timing.val = 0;

    _i2s_dev->fifo_conf.val = 0;
    _i2s_dev->fifo_conf.tx_fifo_mod_force_en = 1;
    _i2s_dev->fifo_conf.tx_data_num = 32;
    _i2s_dev->fifo_conf.tx_fifo_mod = 4;

    _i2s_dev->conf_chan.tx_chan_mod = 0;
    _i2s_dev->sample_rate_conf.tx_bits_mod = g_i2s_para_obj->conf.bit_width;
    _i2s_dev->conf2.val = 0;
    _i2s_dev->conf2.lcd_en = 1;
}

static esp_err_t _i2s_para_driver_init(void)
{
    _i2s_gpio_init();
    _i2s_reg_init();
    esp_intr_alloc(ETS_I2S0_INTR_SOURCE, 0, _i2s_isr, NULL, NULL);
    _i2s_dev->int_ena.out_total_eof = 1;
    return ESP_OK;
}
void  lcdp_write_cmd(uint16_t data){
	g_i2s_para_obj->sync=true;
    gpio_set_level(g_i2s_para_obj->conf.pin_rs, 0);
#if 0
	i2s_para_write((uint8_t*)&data,2);
#else
	_i2s_dev->int_ena.out_total_eof = 0;
	_i2s_tx_stop(); 
	g_i2s_para_obj->dma_list[0].buf=(uint8_t*)&data;
	g_i2s_para_obj->dma_list[0].size = 2;
	g_i2s_para_obj->dma_list[0].length = 2;
	g_i2s_para_obj->dma_list[0].eof = 1;
	g_i2s_para_obj->dma_list[0].empty = (uint32_t)NULL;
	_i2s_dev->out_link.addr = ((uint32_t)&g_i2s_para_obj->dma_list[0]);
	_i2s_tx_start();
	while (!_i2s_dev->state.tx_idle);
	_i2s_dev->int_clr.val = 0xffffffff;
	_i2s_dev->int_ena.out_total_eof = 1;
#endif
}

void  lcdp_write_data_one(uint16_t data){
	g_i2s_para_obj->sync=true;
    gpio_set_level(g_i2s_para_obj->conf.pin_rs, 1);
#if 0
	i2s_para_write((uint8_t*)&data,2);
#else
	_i2s_dev->int_ena.out_total_eof = 0;
	_i2s_tx_stop(); 
	g_i2s_para_obj->dma_list[0].buf=(uint8_t*)&data;
	g_i2s_para_obj->dma_list[0].size = 2;
	g_i2s_para_obj->dma_list[0].length = 2;
	g_i2s_para_obj->dma_list[0].eof = 1;
	g_i2s_para_obj->dma_list[0].empty = (uint32_t)NULL;
	_i2s_dev->out_link.addr = ((uint32_t)&g_i2s_para_obj->dma_list[0]);
	_i2s_tx_start();
	while (!_i2s_dev->state.tx_idle);
	_i2s_dev->int_clr.val = 0xffffffff;
	_i2s_dev->int_ena.out_total_eof = 1;
    //xSemaphoreTake(g_i2s_para_obj->i2s_tx_sem, portMAX_DELAY);
#endif
}


esp_err_t i2s_parallel_init(i2s_parallel_config_t *config)
{
    if(config == NULL) {
        ESP_LOGE(TAG, "parameters configuration failed");
        return ESP_FAIL;
    }

    g_i2s_para_obj = (i2s_parallel_obj_t *)heap_caps_calloc(1, sizeof(i2s_parallel_obj_t), MALLOC_CAP_DMA);
    g_i2s_para_obj->dma_list = (lldesc_t *)heap_caps_calloc(DMA_LIST_SIZE, sizeof(lldesc_t), MALLOC_CAP_DMA);
    //g_i2s_para_obj->i2s_tx_buf = (uint8_t *)heap_caps_calloc(TX_BUFFER_SIZE, sizeof(uint8_t), MALLOC_CAP_DMA);

    g_i2s_para_obj->i2s_tx_sem = xSemaphoreCreateBinary();
    if(g_i2s_para_obj == NULL || g_i2s_para_obj->dma_list == NULL
        || g_i2s_para_obj->i2s_tx_sem == NULL) {
        ESP_LOGE(TAG, "have no enough memery");
        return ESP_ERR_NO_MEM;
    }

    g_i2s_para_obj->conf.bit_width = config->bit_width;
    g_i2s_para_obj->conf.pin_num = config->pin_num;
    g_i2s_para_obj->conf.pin_clk = config->pin_clk;
    g_i2s_para_obj->conf.ws_clk_div = config->ws_clk_div;
    g_i2s_para_obj->conf.pin_cs = config->pin_cs;
    g_i2s_para_obj->conf.pin_rs = config->pin_rs;
    g_i2s_para_obj->conf.pin_rst = config->pin_rst;
    g_i2s_para_obj->conf.pin_rd = config->pin_rd;

    gpio_config_t io_conf;
    io_conf.intr_type = GPIO_PIN_INTR_DISABLE;
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pin_bit_mask = (1ULL << config->pin_cs) | (1ULL << config->pin_rd) | (1ULL << config->pin_rs) | (1ULL << config->pin_rst); 
    io_conf.pull_down_en = 0;
    io_conf.pull_up_en = 0;
    gpio_config(&io_conf);
    gpio_set_level(config->pin_rst, 0);
    vTaskDelay(2);
    gpio_set_level(config->pin_rst, 1);
    gpio_set_level(config->pin_cs, 1);
    gpio_set_level(config->pin_rd, 1);
    gpio_set_level(config->pin_rs, 1);
    _i2s_para_driver_init();
    
    
    // gpio_set_pull_mode(g_i2s_para_obj->conf.pin_clk, GPIO_PULLUP_ONLY);
    
    gpio_set_level(config->pin_cs, 0);
    xSemaphoreTake(g_i2s_para_obj->i2s_tx_sem,0);
    return ESP_OK;
}






void lcdp_write_data(uint8_t *data, size_t len){
    gpio_set_level(g_i2s_para_obj->conf.pin_rs, 1);
    i2s_para_write(data,len);
}

void lcdp_write_data_async(uint8_t *data, size_t len){
    gpio_set_level(g_i2s_para_obj->conf.pin_rs, 1);
    i2s_para_write_async(data,len);
}