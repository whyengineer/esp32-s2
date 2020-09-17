#pragma once

#include <stdio.h>
#include <stdint.h>
#include "driver/i2s.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    uint8_t bit_width;
    uint8_t pin_clk;
    uint8_t *pin_num;
    uint8_t ws_clk_div;
    uint8_t pin_cs;
    uint8_t pin_rd;
    uint8_t pin_rst;
    uint8_t pin_rs;
} i2s_parallel_config_t;


void i2s_para_write(uint8_t *data, uint32_t len);
esp_err_t i2s_parallel_init(i2s_parallel_config_t *config);


void lcdp_write_data(uint8_t *data, size_t len);
void lcdp_write_data_async(uint8_t *data, size_t len);
void  lcdp_write_cmd(uint16_t data);
void  lcdp_write_data_one(uint16_t data);
#ifdef __cplusplus
}
#endif

