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
#include "mbcontroller.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "esp_modbus_master.h"

#define MASTER_TAG "modbus"

#define MB_PORT_NUM 1
#define MB_TX_PIN GPIO_NUM_39
#define MB_RX_PIN GPIO_NUM_40
#define MB_RTS_PIN GPIO_NUM_38

#define MB_DEVICE_ADDR1 1
#define STR(fieldname) ((const char*)( fieldname ))
#define MASTER_CHECK(a, ret_val, str, ...) \
    if (!(a)) { \
        ESP_LOGE(MASTER_TAG, "%s(%u): " str, __FUNCTION__, __LINE__, ##__VA_ARGS__); \
        return (ret_val); \
    }
#define OPTS(min_val, max_val, step_val) { .opt1 = min_val, .opt2 = max_val, .opt3 = step_val }

const mb_parameter_descriptor_t device_parameters[] = {
    // { CID, Param Name, Units, Modbus Slave Addr, Modbus Reg Type, Reg Start, Reg Size, Instance Offset, Data Type, Data Size, Parameter Options, Access Mode}
    { 0, STR("U"), STR("V"), MB_DEVICE_ADDR1, MB_PARAM_HOLDING, 0x2000, 2,
                   0, PARAM_TYPE_FLOAT, 4, OPTS( -1000000, 10000000, 1 ), PAR_PERMS_READ },
    { 1, STR("I"), STR("A"), MB_DEVICE_ADDR1, MB_PARAM_HOLDING, 0x2002, 2,
                   0, PARAM_TYPE_FLOAT, 4, OPTS( -1000000, 10000000, 1 ), PAR_PERMS_READ },
    { 2, STR("P"), STR("W"), MB_DEVICE_ADDR1, MB_PARAM_HOLDING, 0x2004, 2,
                   0, PARAM_TYPE_FLOAT, 4, OPTS( -1000000, 10000000, 1 ), PAR_PERMS_READ },
    { 3, STR("Q"), STR("W"), MB_DEVICE_ADDR1, MB_PARAM_HOLDING, 0x2006, 2,
                   0, PARAM_TYPE_FLOAT, 4, OPTS( -1000000, 10000000, 1 ), PAR_PERMS_READ },
    { 4, STR("S"), STR("W"), MB_DEVICE_ADDR1, MB_PARAM_HOLDING, 0x2008, 2,
                   0, PARAM_TYPE_FLOAT, 4, OPTS( -1000000, 10000000, 1 ), PAR_PERMS_READ },
    { 5, STR("PF"), STR(""), MB_DEVICE_ADDR1, MB_PARAM_HOLDING, 0x200a, 2,
                   0, PARAM_TYPE_FLOAT, 4, OPTS( -1000000, 10000000, 1 ), PAR_PERMS_READ },
    { 6, STR("Freq"), STR("Hz"), MB_DEVICE_ADDR1, MB_PARAM_HOLDING, 0x200e, 2,
                   0, PARAM_TYPE_FLOAT, 4, OPTS( -1000000, 10000000, 1 ), PAR_PERMS_READ },
    { 7, STR("Ep"), STR(""), MB_DEVICE_ADDR1, MB_PARAM_HOLDING, 0x4000, 2,
                   0, PARAM_TYPE_FLOAT, 4, OPTS( -1000000, 10000000, 1 ), PAR_PERMS_READ },
};

const uint16_t num_device_parameters = (sizeof(device_parameters)/sizeof(device_parameters[0]));

static esp_err_t modbus_init(){
    mb_communication_info_t comm = {
            .port = MB_PORT_NUM,
            .mode = MB_MODE_RTU,
            .baudrate = 9600,
            .parity = MB_PARITY_NONE
    };
    void* master_handler = NULL;

    esp_err_t err = mbc_master_init(MB_PORT_SERIAL_MASTER, &master_handler);
    MASTER_CHECK((master_handler != NULL), ESP_ERR_INVALID_STATE,
                                "mb controller initialization fail.");
    MASTER_CHECK((err == ESP_OK), ESP_ERR_INVALID_STATE,
                            "mb controller initialization fail, returns(0x%x).",
                            (uint32_t)err);
    err = mbc_master_setup((void*)&comm);
    MASTER_CHECK((err == ESP_OK), ESP_ERR_INVALID_STATE,
                            "mb controller setup fail, returns(0x%x).",
                            (uint32_t)err);

    // Set UART pin numbers
    err = uart_set_pin(MB_PORT_NUM, MB_TX_PIN, MB_RX_PIN,
                              MB_RTS_PIN, UART_PIN_NO_CHANGE);

    err = mbc_master_start();
    MASTER_CHECK((err == ESP_OK), ESP_ERR_INVALID_STATE,
                            "mb controller start fail, returns(0x%x).",
                            (uint32_t)err);

    MASTER_CHECK((err == ESP_OK), ESP_ERR_INVALID_STATE,
            "mb serial set pin failure, uart_set_pin() returned (0x%x).", (uint32_t)err);
    // Set driver mode to Half Duplex
    err = uart_set_mode(MB_PORT_NUM, UART_MODE_RS485_HALF_DUPLEX);
    MASTER_CHECK((err == ESP_OK), ESP_ERR_INVALID_STATE,
            "mb serial set mode failure, uart_set_mode() returned (0x%x).", (uint32_t)err);

    vTaskDelay(5);
    err = mbc_master_set_descriptor(&device_parameters[0], num_device_parameters);
    MASTER_CHECK((err == ESP_OK), ESP_ERR_INVALID_STATE,
                                "mb controller set descriptor fail, returns(0x%x).",
                                (uint32_t)err);
    ESP_LOGI(MASTER_TAG, "Modbus master stack initialized...");
    return err;
}

static void ddsu666_test(){
    uint8_t data[4];
    
    const mb_parameter_descriptor_t* param_descriptor = NULL;
    uint8_t type = 0;
    esp_err_t err = ESP_OK;

    for(int cid=0;cid<num_device_parameters;cid++){
        err = mbc_master_get_cid_info(cid, &param_descriptor);
        if ((err != ESP_ERR_NOT_FOUND) && (param_descriptor != NULL)) {
            err = mbc_master_get_parameter(cid, (char*)param_descriptor->param_key,(uint8_t*)data, &type);
            if (err == ESP_OK) {
                uint8_t fdata[4];
                fdata[0]=data[2];
                fdata[1]=data[3];
                fdata[2]=data[0];
                fdata[3]=data[1];
                
                
                ESP_LOGI(MASTER_TAG, "Characteristic #%d %s (%s) value = (%f) read successful.",
                                                 param_descriptor->cid,
                                                 (char*)param_descriptor->param_key,
                                                 (char*)param_descriptor->param_units,
                                                 *(float*)fdata);
            }else{
                ESP_LOGE(MASTER_TAG, "Characteristic #%d (%s) read fail, err = 0x%x (%s).",
                                                    param_descriptor->cid,
                                                    (char*)param_descriptor->param_key,
                                                    (int)err,
                                                    (char*)esp_err_to_name(err));
            }
        }
    }
}

void app_main(void)
{
    modbus_init();
    while (1)
    {
        /* code */
        ddsu666_test();
        vTaskDelay(200);
    }
    
}
