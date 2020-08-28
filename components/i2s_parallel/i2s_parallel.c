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

#include "esp_log.h"
#include "esp_err.h"

static const char *TAG = "i2s_parallel";
#define DMA_LIST_SIZE       20
#define DMA_MAX_NODE_SIZE   4095
#define TX_BUFFER_SIZE   (DMA_LIST_SIZE * DMA_MAX_NODE_SIZE)

typedef struct {
    i2s_parallel_config_t conf;
    SemaphoreHandle_t i2s_tx_sem;
    uint8_t *i2s_tx_buf;
    lldesc_t *dma_list;
} i2s_parallel_obj_t;

i2s_parallel_obj_t *g_i2s_para_obj;
i2s_dev_t * _i2s_dev = &I2S0;

static void IRAM_ATTR _i2s_isr(void *arg)
{
    if(_i2s_dev->int_st.out_total_eof) {
        BaseType_t HPTaskAwoken;
        xSemaphoreGiveFromISR(g_i2s_para_obj->i2s_tx_sem, &HPTaskAwoken);
        if(HPTaskAwoken == pdTRUE) {
            portYIELD_FROM_ISR();
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

void i2s_para_write(uint8_t *data, uint32_t len)
{
    const static int dma_half_index = DMA_LIST_SIZE / 2 - 1;
    const static int dma_last_index = DMA_LIST_SIZE - 1;
    const static int txbuf_half_size = TX_BUFFER_SIZE / 2;
    for (int pos = 0; pos < DMA_LIST_SIZE; pos++) {
        // printf("ping pang cnt:%d\n",pos);
        g_i2s_para_obj->dma_list[pos].size = DMA_MAX_NODE_SIZE;
        g_i2s_para_obj->dma_list[pos].length = DMA_MAX_NODE_SIZE;
        g_i2s_para_obj->dma_list[pos].buf = (g_i2s_para_obj->i2s_tx_buf + DMA_MAX_NODE_SIZE * pos);
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
        memcpy(g_i2s_para_obj->dma_list[(bufpos % 2) * (dma_half_index+1)].buf, data, txbuf_half_size);
        data += txbuf_half_size;
        // xSemaphoreTake(g_i2s_para_obj->i2s_tx_sem, portMAX_DELAY);
        _i2s_dev->out_link.addr = ((uint32_t)&g_i2s_para_obj->dma_list[(bufpos % 2) * (dma_half_index+1)]);
        _i2s_tx_start();
        while (!_i2s_dev->state.tx_idle);
    }
    int rest_data_size = len % txbuf_half_size;
    if (rest_data_size) {
        _i2s_tx_stop();
        int end_index = 0;
        int end_size = 0;
        memcpy(g_i2s_para_obj->dma_list[0].buf, data, rest_data_size);
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
        xSemaphoreTake(g_i2s_para_obj->i2s_tx_sem, portMAX_DELAY);
        _i2s_tx_start();
        while (!_i2s_dev->state.tx_idle);
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
    _i2s_dev->clkm_conf.clkm_div_num = 2;
    _i2s_dev->clkm_conf.clkm_div_b = 0;
    _i2s_dev->clkm_conf.clkm_div_a = 63;
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
static void lcd_write_cmd(uint8_t data){
    gpio_set_level(g_i2s_para_obj->conf.pin_rs, 0);
    i2s_para_write(&data,1);
}

static void lcd_write_data(uint8_t data){
    gpio_set_level(g_i2s_para_obj->conf.pin_rs, 1);
    i2s_para_write(&data,1);
}


static void jd5858_init(){
    /* RST */
    // gpio_set_level(g_i2s_para_obj->conf.pin_rst, 0);
    // vTaskDelay(1);
    // gpio_set_level(g_i2s_para_obj->conf.pin_rst, 1);
    //PASSWORD
    lcd_write_cmd(0xDF); //Password
    lcd_write_data(0x58);
    lcd_write_data(0x58);
    lcd_write_data(0xB0);

    //---------------- PAGE0 --------------
    lcd_write_cmd(0xDE);  	
    lcd_write_data(0x00);  


    //VCOM_SET
    lcd_write_cmd(0xB2);
    lcd_write_data(0x01);
    lcd_write_data(0x10); //VCOM  

    //Gamma_Set
    lcd_write_cmd(0xB7); 
    lcd_write_data(0x10); //VGMP = +5.3V 0x14A
    lcd_write_data(0x4A); 
    lcd_write_data(0x00); //VGSP = +0.0V
    lcd_write_data(0x10); //VGMN = -5.3V 0x14A
    lcd_write_data(0x4A); 
    lcd_write_data(0x00); //VGSN = -0.0V
    
    //DCDC_SEL
    lcd_write_cmd(0xBB); 
    lcd_write_data(0x01); //VGH = AVDD+VCI = 5.8V+3.1V= 8.9V ;VGL = -1*VGH = -8.9V; AVDD = 2xVCI = 3.1V*2 = 6.2V 
    lcd_write_data(0x1D); //AVDD_S = +5.8V (0x1D) ; AVEE = -1xAVDD_S = -5.8V
    lcd_write_data(0x43);
    lcd_write_data(0x43);
    lcd_write_data(0x21);
    lcd_write_data(0x21);

    //GATE_POWER
    lcd_write_cmd(0xCF); 
    lcd_write_data(0x20); //VGHO = +8V
    lcd_write_data(0x50); //VGLO = -8V


    //SET_R_GAMMA 
    lcd_write_cmd(0xC8);
    lcd_write_data(0x7F);
    lcd_write_data(0x52);
    lcd_write_data(0x3B);
    lcd_write_data(0x2A);
    lcd_write_data(0x22);
    lcd_write_data(0x12);
    lcd_write_data(0x17);
    lcd_write_data(0x04);
    lcd_write_data(0x21);
    lcd_write_data(0x26);
    lcd_write_data(0x29);
    lcd_write_data(0x4B);
    lcd_write_data(0x3A);
    lcd_write_data(0x45);
    lcd_write_data(0x3A);
    lcd_write_data(0x35);
    lcd_write_data(0x2C);
    lcd_write_data(0x1E);
    lcd_write_data(0x01);
    lcd_write_data(0x7F);
    lcd_write_data(0x52);
    lcd_write_data(0x3B);
    lcd_write_data(0x2A);
    lcd_write_data(0x22);
    lcd_write_data(0x12);
    lcd_write_data(0x17);
    lcd_write_data(0x04);
    lcd_write_data(0x21);
    lcd_write_data(0x26);
    lcd_write_data(0x29);
    lcd_write_data(0x4B);
    lcd_write_data(0x3A);
    lcd_write_data(0x45);
    lcd_write_data(0x3A);
    lcd_write_data(0x35);
    lcd_write_data(0x2C);
    lcd_write_data(0x1E);
    lcd_write_data(0x01);

    //-----------------------------
    // SET page4 TCON & GIP 
    //------------------------------ 
    lcd_write_cmd(0xDE);  	
    lcd_write_data(0x04);  // page4

    //SETSTBA
    lcd_write_cmd(0xB2);  	
    lcd_write_data(0x14); //GAP = 1 ;SAP= 4
    lcd_write_data(0x14);

    //SETRGBCYC1
    lcd_write_cmd(0xB8);  	
    lcd_write_data(0x74); //-	NEQ	PEQ[1:0] -	RGB_INV_NP[2:0]	
    lcd_write_data(0x44); //-	RGB_INV_PI[2:0] -	RGB_INV_I[2:0]
    lcd_write_data(0x00); //RGB_N_T2[11:8],RGB_N_T1[11:8] 
    lcd_write_data(0x01); //RGB_N_T1[7:0], 
    lcd_write_data(0x01); //RGB_N_T2[7:0], 
    lcd_write_data(0x00); //RGB_N_T4[11:8],RGB_N_T3[11:8] 
    lcd_write_data(0x01); //RGB_N_T3[7:0], 
    lcd_write_data(0x01); //RGB_N_T4[7:0], 
    lcd_write_data(0x00); //RGB_N_T6[11:8],RGB_N_T5[11:8] 
    lcd_write_data(0x09); //RGB_N_T5[7:0], 
    lcd_write_data(0x82); //RGB_N_T6[7:0], 
    lcd_write_data(0x10); //RGB_N_T8[11:8],RGB_N_T7[11:8] 
    lcd_write_data(0x8A); //RGB_N_T7[7:0], 
    lcd_write_data(0x03); //RGB_N_T8[7:0], 
    lcd_write_data(0x11); //RGB_N_T10[11:8],RGB_N_T9[11:8] 
    lcd_write_data(0x0B); //RGB_N_T9[7:0], 
    lcd_write_data(0x84); //RGB_N_T10[7:0], 
    lcd_write_data(0x21); //RGB_N_T12[11:8],RGB_N_T11[11:8] 
    lcd_write_data(0x8C); //RGB_N_T11[7:0], 
    lcd_write_data(0x05); //RGB_N_T12[7:0], 
    lcd_write_data(0x22); //RGB_N_T14[11:8],RGB_N_T13[11:8] 
    lcd_write_data(0x0D); //RGB_N_T13[7:0], 
    lcd_write_data(0x86); //RGB_N_T14[7:0], 
    lcd_write_data(0x32); //RGB_N_T16[11:8],RGB_N_T15[11:8] 
    lcd_write_data(0x8E); //RGB_N_T15[7:0], 
    lcd_write_data(0x07); //RGB_N_T16[7:0], 
    lcd_write_data(0x00); //RGB_N_T18[11:8],RGB_N_T17[11:8] 
    lcd_write_data(0x00); //RGB_N_T17[7:0], 
    lcd_write_data(0x00); //RGB_N_T18[7:0], 


    //SETRGBCYC2
    lcd_write_cmd(0xB9);  	
    lcd_write_data(0x40); //-,ENJDT,RGB_JDT2[2:0],ENP_LINE_INV,ENP_FRM_SEL[1:0],
    lcd_write_data(0x22); //RGB_N_T20[11:8],RGB_N_T19[11:8],
    lcd_write_data(0x08); //RGB_N_T19[7:0],
    lcd_write_data(0x3A); //RGB_N_T20[7:0],
    lcd_write_data(0x22); //RGB_N_T22[11:8],RGB_N_T21[11:8],
    lcd_write_data(0x4B); //RGB_N_T21[7:0],
    lcd_write_data(0x7D); //RGB_N_T22[7:0],
    lcd_write_data(0x22); //RGB_N_T24[11:8],RGB_N_T23[11:8],
    lcd_write_data(0x8D); //RGB_N_T23[7:0],
    lcd_write_data(0xBF); //RGB_N_T24[7:0],
    lcd_write_data(0x32); //RGB_N_T26[11:8],RGB_N_T25[11:8],
    lcd_write_data(0xD0); //RGB_N_T25[7:0],
    lcd_write_data(0x02); //RGB_N_T26[7:0],
    lcd_write_data(0x33); //RGB_N_T28[11:8],RGB_N_T27[11:8],
    lcd_write_data(0x12); //RGB_N_T27[7:0],
    lcd_write_data(0x44); //RGB_N_T28[7:0],
    lcd_write_data(0x00); //-,-,-,-,RGB_N_TA1[11:8],
    lcd_write_data(0x0A); //RGB_N_TA1[7:0],
    lcd_write_data(0x00); //RGB_N_TA3[11:8],RGB_N_TA2[11:8],
    lcd_write_data(0x0A); //RGB_N_TA2[7:0],
    lcd_write_data(0x0A); //RGB_N_TA3[7:0],
    lcd_write_data(0x00); //RGB_N_TA5[11:8],RGB_N_TA4[11:8],
    lcd_write_data(0x0A); //RGB_N_TA4[7:0],
    lcd_write_data(0x0A); //RGB_N_TA5[7:0],
    lcd_write_data(0x00); //RGB_N_TA7[11:8],RGB_N_TA6[11:8],
    lcd_write_data(0x0A); //RGB_N_TA6[7:0],
    lcd_write_data(0x0A); //RGB_N_TA7[7:0],


    //SETRGBCYC3
    lcd_write_cmd(0xBA);  	
    lcd_write_data(0x00);//-	-	-	-	-	-	-	-
    lcd_write_data(0x00);//RGB_N_TA9[11:8],RGB_N_TA8[11:8] 
    lcd_write_data(0x07);//RGB_N_TA8[7:0], 
    lcd_write_data(0x07);//RGB_N_TA9[7:0], 
    lcd_write_data(0x00);//RGB_N_TA11[11:8],RGB_N_TA10[11:8] 
    lcd_write_data(0x07);//RGB_N_TA10[7:0], 
    lcd_write_data(0x07);//RGB_N_TA11[7:0], 
    lcd_write_data(0x00);//RGB_N_TA13[11:8],RGB_N_TA12[11:8] 
    lcd_write_data(0x07);//RGB_N_TA12[7:0], 
    lcd_write_data(0x07);//RGB_N_TA13[7:0], 
    lcd_write_data(0x00);//RGB_N_TC[11:8],RGB_N_TB[11:8] 
    lcd_write_data(0x01);//RGB_N_TB[7:0],  
    lcd_write_data(0x01);//RGB_N_TC[7:0], 
    lcd_write_data(0x00);//RGB_N_TE[11:8],RGB_N_TD[11:8] 
    lcd_write_data(0x0A);//RGB_N_TD[7:0], 
    lcd_write_data(0x01);//RGB_N_TE[7:0], 
    lcd_write_data(0x00);//-	-	-	-	RGB_N_TF[11:8] 
    lcd_write_data(0x01);//RGB_N_TF[7:0], 
    lcd_write_data(0x30);//RGB_CHGEN_OFF[11:8],RGB_CHGEN_ON[11:8] 
    lcd_write_data(0x0A);//RGB_CHGEN_ON[7:0], 
    lcd_write_data(0x40);//RGB_CHGEN_OFF[7:0], 
    lcd_write_data(0x30);//RES_MUX_OFF[11:8],RES_MUX_ON[11:8] 
    lcd_write_data(0x01);//RES_MUX_ON[7:0], 
    lcd_write_data(0x3E);//RES_MUX_OFF[7:0], 
    lcd_write_data(0x00);//-	-	-	L2_COND1_INV[12:8],
    lcd_write_data(0x00);//-	-	-	L2_COND0_INV[12:8],
    lcd_write_data(0x00);//L2_COND0_INV[7:0], 
    lcd_write_data(0x00);//L2_COND1_INV[7:0], 


    //SET_TCON
    lcd_write_cmd(0xBC);  	
    lcd_write_data(0x1A);//1  MUX_SEL =1:6 ,RSO = 360H   
    lcd_write_data(0x00);//2  LN_NO_MUL2 = 0:Gate line number=LN[10:0]*2 ,LN[10:8] = 0
    lcd_write_data(0xB4);//3  LN[7:0] =180*2 = 360             
    lcd_write_data(0x03);//4  PANEL[2:0] = dancing type 2
    lcd_write_data(0x00);//5  VFP[11:8],SLT[11:8]
    lcd_write_data(0xD0);//6  SLT[7:0] = 1/(60*(360+10+6))/4OSC(19MHZ)
    lcd_write_data(0x08);//7  VFP[7:0] = 8
    lcd_write_data(0x00);//8  HBP[11:8], VBP[11:8]
    lcd_write_data(0x07);//9  VBP[7:0]
    lcd_write_data(0x2C);//10 HBP[7:0]  
    lcd_write_data(0x00);//11 VFP_I[11:8],SLT_I[11:8]
    lcd_write_data(0xD0);//12 SLT_I[7:0]
    lcd_write_data(0x08);//13 VFP_I[7:0]
    lcd_write_data(0x00);//14 HBP_I[11:8],VBP_I[11:8]
    lcd_write_data(0x07);//15 VBP_I[7:0]
    lcd_write_data(0x2C);//16 HBP_I[7:0]
    lcd_write_data(0x82);//17 HBP_NCK[3:0],HFP_NCK[3:0]
    lcd_write_data(0x00);//18 TCON_OPT1[15:8]
    lcd_write_data(0x03);//19 TCON_OPT1[7:0]
    lcd_write_data(0x00);//20 VFP_PI[11:8],SLT_PI[11:8]
    lcd_write_data(0xD0);//21 SLT_PI[7:0]
    lcd_write_data(0x08);//22 VFP_PI[7:0]
    lcd_write_data(0x00);//23 HBP_PI[11:8],VBP_PI[11:8]
    lcd_write_data(0x07);//24 VBP_PI[7:0]
    lcd_write_data(0x2C);//25 HBP_PI[7:0]


    //-------------------GIP----------------------
    //SET_GIP_EQ
    lcd_write_cmd(0xC4); 
    lcd_write_data(0x00);
    lcd_write_data(0x00);
    lcd_write_data(0x00);
    lcd_write_data(0x02);
    lcd_write_data(0x00);
    lcd_write_data(0x00);
    lcd_write_data(0x00);
    lcd_write_data(0x00);
    lcd_write_data(0x02);
    lcd_write_data(0x00);
    lcd_write_data(0x00);
    lcd_write_data(0x00);
    lcd_write_data(0x02);
    lcd_write_data(0x00);
    lcd_write_data(0x00);
    lcd_write_data(0x00);
    lcd_write_data(0x00);
    lcd_write_data(0x00);
    lcd_write_data(0x00);

    //SET_GIP_L
    lcd_write_cmd(0xC5);  	
    lcd_write_data(0x00);//DUMMY 
    lcd_write_data(0x1F);//0
    lcd_write_data(0x1F);//1 
    lcd_write_data(0x1F);//2   
    lcd_write_data(0x1E);//3 GAS  
    lcd_write_data(0xDF);//4 BGAS
    lcd_write_data(0x1F);//5 RSTV  
    lcd_write_data(0xC7);//6 CKV4
    lcd_write_data(0xC5);//7 CKV2  
    lcd_write_data(0x1F);//8 SB
    lcd_write_data(0x1F);//9  
    lcd_write_data(0x1F);//10  
    lcd_write_data(0x1F);//11 
    lcd_write_data(0x1F);//12 
    lcd_write_data(0x1F);//13 
    lcd_write_data(0x1F);//14 
    lcd_write_data(0x1F);//15 
    lcd_write_data(0x1F);//16 
    lcd_write_data(0x1F);//17 
    lcd_write_data(0x1F);//18 
    lcd_write_data(0x1F);//19 
    lcd_write_data(0x1F);//20 
    lcd_write_data(0x1F);//21 
    lcd_write_data(0x1F);//22 
    lcd_write_data(0x1F);//23 
    lcd_write_data(0x1F);//24  
    lcd_write_data(0x1F);//25  
    


    //SET_GIP_R
    lcd_write_cmd(0xC6);  	
    lcd_write_data(0x00);//DUMMY
    lcd_write_data(0x1F);//0  
    lcd_write_data(0x1F);//1 
    lcd_write_data(0x1F);//2 
    lcd_write_data(0x1F);//3 
    lcd_write_data(0x1F);//4 
    lcd_write_data(0x1F);//5 
    lcd_write_data(0x1F);//6 
    lcd_write_data(0x1F);//7 
    lcd_write_data(0x1F);//8 
    lcd_write_data(0x1F);//9   ASB
    lcd_write_data(0x00);//10  LSTV
    lcd_write_data(0xC4);//11  CKV1
    lcd_write_data(0xC6);//12  CKV3
    lcd_write_data(0xE0);//13  CKH1
    lcd_write_data(0xE1);//14  CKH2
    lcd_write_data(0xE2);//15  CKH3
    lcd_write_data(0xE3);//16  CKH4
    lcd_write_data(0xE4);//17  CKH5
    lcd_write_data(0xE5);//18  CKH6
    lcd_write_data(0x1F);//19
    lcd_write_data(0x1F);//20
    lcd_write_data(0x1F);//21
    lcd_write_data(0x1F);//22
    lcd_write_data(0x1F);//23
    lcd_write_data(0x1F);//24  
    lcd_write_data(0x1F);//25  



    //SET_GIP_L_GS
    lcd_write_cmd(0xC7);  	
    lcd_write_data(0x00);//DUMMY   
    lcd_write_data(0x1F);//0
    lcd_write_data(0x1F);//1 
    lcd_write_data(0x1F);//2 
    lcd_write_data(0xDE);//3 GAS 
    lcd_write_data(0x1F);//4 BGAS
    lcd_write_data(0x00);//5 RSTV
    lcd_write_data(0xC4);//6 CKV4
    lcd_write_data(0xC6);//7 CKV2
    lcd_write_data(0x1F);//8 SB
    lcd_write_data(0x1F);//9 
    lcd_write_data(0x1F);//10
    lcd_write_data(0x1F);//11
    lcd_write_data(0x1F);//12
    lcd_write_data(0x1F);//13
    lcd_write_data(0x1F);//14
    lcd_write_data(0x1F);//15
    lcd_write_data(0x1F);//16
    lcd_write_data(0x1F);//17
    lcd_write_data(0x1F);//18
    lcd_write_data(0x1F);//19
    lcd_write_data(0x1F);//20
    lcd_write_data(0x1F);//21
    lcd_write_data(0x1F);//22
    lcd_write_data(0x1F);//23
    lcd_write_data(0x1F);//24
    lcd_write_data(0x1F);//25
    
    

    //SET_GIP_R_GS
    lcd_write_cmd(0xC8);  	
    lcd_write_data(0x00);//DUMMY   
    lcd_write_data(0x1F);//0
    lcd_write_data(0x1F);//1   
    lcd_write_data(0x1F);//2 
    lcd_write_data(0x1F);//3 
    lcd_write_data(0x1F);//4 
    lcd_write_data(0x1F);//5 
    lcd_write_data(0x1F);//6 
    lcd_write_data(0x1F);//7 
    lcd_write_data(0x1F);//8 
    lcd_write_data(0x1F);//9   ASB
    lcd_write_data(0x1F);//10  LSTV
    lcd_write_data(0xC7);//11  CKV1
    lcd_write_data(0xC5);//12  CKV3
    lcd_write_data(0x20);//13  CKH1
    lcd_write_data(0x21);//14  CKH2
    lcd_write_data(0x22);//15  CKH3
    lcd_write_data(0x23);//16  CKH4
    lcd_write_data(0x24);//17  CKH5
    lcd_write_data(0x25);//18  CKH6
    lcd_write_data(0x1F);//19
    lcd_write_data(0x1F);//20
    lcd_write_data(0x1F);//21
    lcd_write_data(0x1F);//22
    lcd_write_data(0x1F);//23
    lcd_write_data(0x1F);//24
    lcd_write_data(0x1F);//25



    //SETGIP1
    lcd_write_cmd(0xC9);  	
    lcd_write_data(0x00);//0
    lcd_write_data(0x00);//1   
    lcd_write_data(0x00);//2 
    lcd_write_data(0x00);//3   L:GAS 
    lcd_write_data(0x10);//4   L:BGAS :VGH
    lcd_write_data(0x00);//5   L:RSTV
    lcd_write_data(0x10);//6   L:CKV4 :VGH
    lcd_write_data(0x10);//7   L:CKV2 :VGH
    lcd_write_data(0x00);//8   L:SB
    lcd_write_data(0x00);//9   R:ASB
    lcd_write_data(0x00);//10  R:LSTV
    lcd_write_data(0x20);//11  R:CKV1 :VGH
    lcd_write_data(0x20);//12  R:CKV3 :VGH
    lcd_write_data(0x20);//13  R:CKH1 :VGH
    lcd_write_data(0x20);//14  R:CKH2 :VGH
    lcd_write_data(0x20);//15  R:CKH3 :VGH
    lcd_write_data(0x20);//16  R:CKH4 :VGH
    lcd_write_data(0x20);//17  R:CKH5 :VGH
    lcd_write_data(0x20);//18  R:CKH6 :VGH
    lcd_write_data(0x00);//19
    lcd_write_data(0x00);//20
    lcd_write_data(0x00);//21
    lcd_write_data(0x00);//22
    lcd_write_data(0x00);//23       
    lcd_write_data(0x00);//24  
    lcd_write_data(0x00);//25  

    //SETGIP2
    lcd_write_cmd(0xCB);  	
    lcd_write_data(0x01);//1  INIT_PORCH  
    lcd_write_data(0x10);//2  INIT_W
    lcd_write_data(0x00);//3 
    lcd_write_data(0x00);//4 
    lcd_write_data(0x07);//5  STV_S0
    lcd_write_data(0x01);//6 
    lcd_write_data(0x00);//7 
    lcd_write_data(0x0A);//8 
    lcd_write_data(0x00);//9  STV_NUM = 1 , STV_S1
    lcd_write_data(0x02);//10
    lcd_write_data(0x00);//11 STV1/0_W
    lcd_write_data(0x00);//12 STV3/2_W
    lcd_write_data(0x00);//13
    lcd_write_data(0x03);//14
    lcd_write_data(0x00);//15
    lcd_write_data(0x00);//16
    lcd_write_data(0x00);//17
    lcd_write_data(0x21);//18
    lcd_write_data(0x23);//19
    lcd_write_data(0x30);//20 CKV_W
    lcd_write_data(0x00);//21 
    lcd_write_data(0x08);//22 CKV_S0
    lcd_write_data(0x04);//23 CKV0_DUM[7:0]
    lcd_write_data(0x00);//24
    lcd_write_data(0x00);//25
    lcd_write_data(0x05);//26
    lcd_write_data(0x10);//27
    lcd_write_data(0x01);//28 //END_W
    lcd_write_data(0x04);//29
    lcd_write_data(0x06);//30
    lcd_write_data(0x10);//31
    lcd_write_data(0x10);//32


    //SET_GIP_ONOFF
    lcd_write_cmd(0xD1);  	
    lcd_write_data(0x00);
    lcd_write_data(0x00);
    lcd_write_data(0x03);
    lcd_write_data(0x60);
    lcd_write_data(0x30);
    lcd_write_data(0x03);
    lcd_write_data(0x18);
    lcd_write_data(0x30);//CKV0_OFF[11:8]
    lcd_write_data(0x07);//CKV0_ON[7:0]
    lcd_write_data(0x3A);//CKV0_OFF[7:0] 
    lcd_write_data(0x30);
    lcd_write_data(0x03);
    lcd_write_data(0x18);
    lcd_write_data(0x30);
    lcd_write_data(0x03);
    lcd_write_data(0x18);
    lcd_write_data(0x30);
    lcd_write_data(0x03);
    lcd_write_data(0x18);

    //SET_GIP_ONOFF_WB
    lcd_write_cmd(0xD2);  	
    lcd_write_data(0x00);
    lcd_write_data(0x30);//STV_OFF[11:8]
    lcd_write_data(0x07);//STV_ON[7:0]
    lcd_write_data(0x3A);//STV_OFF[7:0]
    lcd_write_data(0x32);
    lcd_write_data(0xBC);
    lcd_write_data(0x20);
    lcd_write_data(0x32);
    lcd_write_data(0xBC);
    lcd_write_data(0x20);
    lcd_write_data(0x32);
    lcd_write_data(0xBC);
    lcd_write_data(0x20);
    lcd_write_data(0x32);
    lcd_write_data(0xBC);
    lcd_write_data(0x20);
    lcd_write_data(0x30);
    lcd_write_data(0x10);
    lcd_write_data(0x20);
    lcd_write_data(0x30);
    lcd_write_data(0x10);
    lcd_write_data(0x20);
    lcd_write_data(0x30);
    lcd_write_data(0x10);
    lcd_write_data(0x20);
    lcd_write_data(0x30);
    lcd_write_data(0x10);
    lcd_write_data(0x20);



    
    //SETGIP8_CKH1 CKH_ON/OFF_CKH0-CKH7_odd
    lcd_write_cmd(0xD4);  	
    lcd_write_data(0x00); 
    lcd_write_data(0x00); //CKH_T2_ODD[11:8],CKH_T1_ODD[11:8] 
    lcd_write_data(0x03); //CKH_T1_ODD[7:0], 
    lcd_write_data(0x14); //CKH_T2_ODD[7:0], 
    lcd_write_data(0x00); //CKH_T4_ODD[11:8],CKH_T3_ODD[11:8] 
    lcd_write_data(0x03); //CKH_T3_ODD[7:0], 
    lcd_write_data(0x20); //CKH_T4_ODD[7:0], 
    lcd_write_data(0x00); //CKH_T6_ODD[11:8],CKH_T5_ODD[11:8] 
    lcd_write_data(0x09); //CKH_T5_ODD[7:0], 
    lcd_write_data(0x82); //CKH_T6_ODD[7:0], 
    lcd_write_data(0x10); //CKH_T8_ODD[11:8],CKH_T7_ODD[11:8] 
    lcd_write_data(0x8A); //CKH_T7_ODD[7:0], 
    lcd_write_data(0x03); //CKH_T8_ODD[7:0], 
    lcd_write_data(0x11); //CKH_T10_ODD[11:8],CKH_T9_ODD[11:8] 
    lcd_write_data(0x0B); //CKH_T9_ODD[7:0], 
    lcd_write_data(0x84); //CKH_T10_ODD[7:0], 
    lcd_write_data(0x21); //CKH_T12_ODD[11:8],CKH_T11_ODD[11:8] 
    lcd_write_data(0x8C); //CKH_T11_ODD[7:0], 
    lcd_write_data(0x05); //CKH_T12_ODD[7:0], 
    lcd_write_data(0x22); //CKH_T14_ODD[11:8],CKH_T13_ODD[11:8] 
    lcd_write_data(0x0D); //CKH_T13_ODD[7:0], 
    lcd_write_data(0x86); //CKH_T14_ODD[7:0], 
    lcd_write_data(0x32); //CKH_T16_ODD[11:8],CKH_T15_ODD[11:8] 
    lcd_write_data(0x8E); //CKH_T15_ODD[7:0], 
    lcd_write_data(0x07); //CKH_T16_ODD[7:0], 
    lcd_write_data(0x00); //CKH_T18_ODD[11:8],CKH_T17_ODD[11:8] 
    lcd_write_data(0x00); //CKH_T17_ODD[7:0], 
    lcd_write_data(0x00); //CKH_T18_ODD[7:0], 
    lcd_write_data(0x00); //CKH_T20_ODD[11:8],CKH_T19_ODD[11:8] 
    lcd_write_data(0x00); //CKH_T19_ODD[7:0], 
    lcd_write_data(0x00); //CKH_T20_ODD[7:0], 

    
    ///-----------------------------------------------------------------------------------------
    //---------------- PAGE0 --------------
    lcd_write_cmd(0xDE);  	
    lcd_write_data(0x00); 
    // RAM_CTRL
    lcd_write_cmd(0xD7);  	
    lcd_write_data(0x20);  //GM=1;RP=0;RM=0;DM=00
    lcd_write_data(0x00);  
    lcd_write_data(0x00); 

    //---------------- PAGE1 --------------
    lcd_write_cmd(0xDE);  	
    lcd_write_data(0x01); 

    ////MCMD_CTRL
    lcd_write_cmd(0xCA);  	
    lcd_write_data(0x00);

    //---------------- PAGE2 --------------
    lcd_write_cmd(0xDE);  	
    lcd_write_data(0x02); 

    //OSC DIV
    lcd_write_cmd(0xC5);  	
    lcd_write_data(0x03); //FPS 60HZ (0x03) to 30HZ (0x0B) ,47HZ(0x0F),42HZ(0x0E)
    
    //---------------- PAGE0 --------------
    lcd_write_cmd(0xDE);  	
    lcd_write_data(0x00);  

    //Color Pixel Format
    lcd_write_cmd(0x3A);  	
    lcd_write_data(0x05); //666

    //TE ON
    lcd_write_cmd(0x35);  	
    lcd_write_data(0x00);  

    //SLP OUT
    lcd_write_cmd(0x11);  	// SLPOUT
    vTaskDelay(120);

    //DISP ON
    lcd_write_cmd(0x29);  	// DSPON
    vTaskDelay(50);
}

esp_err_t i2s_parallel_init(i2s_parallel_config_t *config)
{
    if(config == NULL) {
        ESP_LOGE(TAG, "parameters configuration failed");
        return ESP_FAIL;
    }

    g_i2s_para_obj = (i2s_parallel_obj_t *)heap_caps_calloc(1, sizeof(i2s_parallel_obj_t), MALLOC_CAP_DMA);
    g_i2s_para_obj->dma_list = (lldesc_t *)heap_caps_calloc(DMA_LIST_SIZE, sizeof(lldesc_t), MALLOC_CAP_DMA);
    g_i2s_para_obj->i2s_tx_buf = (uint8_t *)heap_caps_calloc(TX_BUFFER_SIZE, sizeof(uint8_t), MALLOC_CAP_DMA);

    g_i2s_para_obj->i2s_tx_sem = xSemaphoreCreateBinary();
    if(g_i2s_para_obj == NULL || g_i2s_para_obj->dma_list == NULL
        ||  g_i2s_para_obj->i2s_tx_buf == NULL || g_i2s_para_obj->i2s_tx_sem == NULL) {
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
    _i2s_para_driver_init();
    gpio_set_level(config->pin_cs, 0);
    xSemaphoreGive(g_i2s_para_obj->i2s_tx_sem);
    jd5858_init();
    return ESP_OK;
}




void IRAM_ATTR lcdp_set_index(uint16_t x_start, uint16_t y_start, uint16_t x_end, uint16_t y_end)
{
    uint16_t start_pos, end_pos;
    lcd_write_cmd(0x2a);    /*!< CASET (2Ah): Column Address Set */

    /*!< Must write byte than byte */
    
    start_pos = x_start;
    end_pos = x_end;
    

    lcd_write_data(start_pos >> 8);
    lcd_write_data(start_pos & 0xFF);
    lcd_write_data(end_pos >> 8);
    lcd_write_data(end_pos & 0xFF);

    lcd_write_cmd(0x2b);    /*!< RASET (2Bh): Row Address Set */

    
    start_pos = y_start;
    end_pos = y_end;
    

    lcd_write_data(start_pos >> 8);
    lcd_write_data(start_pos & 0xFF);
    lcd_write_data(end_pos >> 8);
    lcd_write_data(end_pos & 0xFF);
    lcd_write_cmd(0x2c);    /*!< RAMWR (2Ch): Memory Write */
}

void lcdp_write_data(uint8_t *data, size_t len){
    gpio_set_level(g_i2s_para_obj->conf.pin_rs, 1);
    i2s_para_write(data,len);
}