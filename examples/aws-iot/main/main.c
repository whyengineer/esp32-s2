
/*
 * Copyright 2010-2015 Amazon.com, Inc. or its affiliates. All Rights Reserved.
 * Additions Copyright 2016 Espressif Systems (Shanghai) PTE LTD
 *
 * Licensed under the Apache License, Version 2.0 (the "License").
 * You may not use this file except in compliance with the License.
 * A copy of the License is located at
 *
 *  http://aws.amazon.com/apache2.0
 *
 * or in the "license" file accompanying this file. This file is distributed
 * on an "AS IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either
 * express or implied. See the License for the specific language governing
 * permissions and limitations under the License.
 */
/**
 * @file subscribe_publish_sample.c
 * @brief simple MQTT publish and subscribe on the same topic
 *
 * This example takes the parameters from the build configuration and establishes a connection to the AWS IoT MQTT Platform.
 * It subscribes and publishes to the same topic - "test_topic/esp32"
 *
 * Some setup is required. See example README for details.
 *
 */
#include <stdio.h>
#include <stdlib.h>
#include <ctype.h>
#include <unistd.h>
#include <limits.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_vfs_fat.h"
#include "driver/gpio.h"
#include "driver/adc.h"
#include "esp_adc_cal.h"
#include "nvs.h"
#include "nvs_flash.h"

#include "aws_iot_config.h"
#include "aws_iot_log.h"
#include "aws_iot_version.h"
#include "aws_iot_mqtt_client_interface.h"

#include "cJSON.h"

static const char *TAG = "subpub";

/* The examples use simple WiFi configuration that you can set via
   'make menuconfig'.
   If you'd rather not, just change the below entries to strings with
   the config you want - ie #define EXAMPLE_WIFI_SSID "mywifissid"
*/
#define EXAMPLE_WIFI_SSID "flhome"
#define EXAMPLE_WIFI_PASS "71451085ZfLll"

/* FreeRTOS event group to signal when we are connected & ready to make a request */
static EventGroupHandle_t wifi_event_group;
static int s_retry_num = 0;
/* The event group allows multiple bits for each event,
   but we only care about one event - are we connected
   to the AP with an IP? */
#define WIFI_CONNECTED_BIT BIT0
#define WIFI_FAIL_BIT      BIT1

/* CA Root certificate, device ("Thing") certificate and device
 * ("Thing") key.
   Example can be configured one of two ways:
   "Embedded Certs" are loaded from files in "certs/" and embedded into the app binary.
   "Filesystem Certs" are loaded from the filesystem (SD card, etc.)
   See example README for more details.
*/


extern const uint8_t aws_root_ca_pem_start[] asm("_binary_aws_root_ca_pem_start");
extern const uint8_t aws_root_ca_pem_end[] asm("_binary_aws_root_ca_pem_end");
extern const uint8_t certificate_pem_crt_start[] asm("_binary_certificate_pem_crt_start");
extern const uint8_t certificate_pem_crt_end[] asm("_binary_certificate_pem_crt_end");
extern const uint8_t private_pem_key_start[] asm("_binary_private_pem_key_start");
extern const uint8_t private_pem_key_end[] asm("_binary_private_pem_key_end");


#define RELAY_CTRL_1_PIN GPIO_NUM_40
#define RELAY_CTRL_2_PIN GPIO_NUM_39
#define RELAY_CTRL_3_PIN GPIO_NUM_38
#define LED_WIFI GPIO_NUM_33
#define LED_JOB GPIO_NUM_35
#define GPIO_OUTPUT_PIN_SEL  ((1ULL<<RELAY_CTRL_1_PIN) | (1ULL<<RELAY_CTRL_2_PIN) | (1ULL<<RELAY_CTRL_3_PIN) | (1ULL<<LED_WIFI) | (1ULL<<LED_JOB))

#define ADC_DET_1 ADC_CHANNEL_0
#define ADC_DET_2 ADC_CHANNEL_1
#define ADC_DET_3 ADC_CHANNEL_2

#define NO_OF_SAMPLES 100


static esp_adc_cal_characteristics_t *adc_chars;
static AWS_IoT_Client client;
static bool lightChange=false;
static uint8_t slight[3]={0,0,0};
static uint8_t ioStatue[3]={0,0,0};

static char *create_state(uint8_t* lightVal,uint8_t size);

/**
 * @brief Default MQTT HOST URL is pulled from the aws_iot_config.h
 */
char HostAddress[255] = "a2kob4few2zw49-ats.iot.cn-northwest-1.amazonaws.com.cn";
char cPayload[1024];
/**
 * @brief Default MQTT port is pulled from the aws_iot_config.h
 */
uint32_t port = AWS_IOT_MQTT_PORT;

static void event_handler(void* arg, esp_event_base_t event_base,
                                int32_t event_id, void* event_data)
{
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        if (s_retry_num < 3) {
            esp_wifi_connect();
            s_retry_num++;
            ESP_LOGI(TAG, "retry to connect to the AP");
        } else {
            xEventGroupSetBits(wifi_event_group, WIFI_FAIL_BIT);
        }
        ESP_LOGI(TAG,"connect to the AP fail");
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;
        ESP_LOGI(TAG, "got ip:" IPSTR, IP2STR(&event->ip_info.ip));
        s_retry_num = 0;
        xEventGroupSetBits(wifi_event_group, WIFI_CONNECTED_BIT);
    }
}
static void publish_led(){
    
    IoT_Publish_Message_Params paramsQOS0;
    paramsQOS0.qos = QOS0;
    paramsQOS0.isRetained = 0;
    const char *PUB_TOPIC = "stepiot/state/1259014452121047040/1263376536367665152/5f69b964480ec95b35a28b42";
    const int PUB_TOPIC_LEN = strlen("stepiot/state/1259014452121047040/1263376536367665152/5f69b964480ec95b35a28b42");
    char* payload=create_state(slight,3);
    uint32_t len=strlen(payload);
    if(payload!=NULL){
        memcpy(cPayload,payload,len);
        free(payload);
        paramsQOS0.payloadLen = len;
        paramsQOS0.payload = (void *) cPayload;
        aws_iot_mqtt_publish(&client,PUB_TOPIC, PUB_TOPIC_LEN, &paramsQOS0);
        // ESP_LOGI(TAG,"%s",payload); 
    }
}


static void iot_subscribe_callback_handler(AWS_IoT_Client *pClient, char *topicName, uint16_t topicNameLen,
                                    IoT_Publish_Message_Params *params, void *pData) {
    ESP_LOGI(TAG, "Subscribe callback");
    ESP_LOGI(TAG, "%.*s\t%.*s", topicNameLen, topicName, (int) params->payloadLen, (char *)params->payload);
    cJSON *root=NULL;
    root= cJSON_Parse(params->payload);
    cJSON *cmd=cJSON_GetObjectItem(root,"cmd");
    uint8_t cmdSize=cJSON_GetArraySize(cmd);
    ESP_LOGI(TAG,"cmd size:%d",cmdSize);
    for(int i=0;i<cmdSize;i++){
        cJSON* cmdItem=cJSON_GetArrayItem(cmd,i);
        int circuit=cJSON_GetObjectItem(cmdItem,"circuit")->valueint;
        int light=cJSON_GetObjectItem(cmdItem,"light")->valueint;
        if(light!=slight[circuit-1]){
            slight[circuit-1]=light;
            if(circuit==1){
                gpio_set_level(RELAY_CTRL_1_PIN, ioStatue[circuit-1]%2);
            }else if(circuit==2){
                gpio_set_level(RELAY_CTRL_2_PIN, ioStatue[circuit-1]%2);
            }else if(circuit==3){
                gpio_set_level(RELAY_CTRL_3_PIN, ioStatue[circuit-1]%2);
            }else{
                ESP_LOGE(TAG,"error circuit:%d",circuit);
            }
            ioStatue[circuit-1]++;
        }
       
    }
    cJSON_Delete(root);
    publish_led();
}

void disconnectCallbackHandler(AWS_IoT_Client *pClient, void *data) {
    ESP_LOGW(TAG, "MQTT Disconnect");
    IoT_Error_t rc = FAILURE;

    if(NULL == pClient) {
        return;
    }

    if(aws_iot_is_autoreconnect_enabled(pClient)) {
        ESP_LOGI(TAG, "Auto Reconnect is enabled, Reconnecting attempt will start now");
    } else {
        ESP_LOGW(TAG, "Auto Reconnect not enabled. Starting manual reconnect...");
        rc = aws_iot_mqtt_attempt_reconnect(pClient);
        if(NETWORK_RECONNECTED == rc) {
            ESP_LOGW(TAG, "Manual Reconnect Successful");
        } else {
            ESP_LOGW(TAG, "Manual Reconnect Failed - %d", rc);
        }
    }
}




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

static void adc_task(void* arg)
{
    int status;
    uint8_t index;
    uint32_t voltage;
    uint32_t adc_reading;
    check_efuse();
    adc_init();
    int cnt=0;
    adc1_channel_t channel;
    //Continuously sample ADC1
    while (1) {
        index=cnt%3;
        if(index==0){
            channel=ADC_DET_1;
        }else if(index==1){
            channel=ADC_DET_2;
        }else{
            channel=ADC_DET_3;
        }
        adc_reading=0;
        for (int i = 0; i < NO_OF_SAMPLES; i++) {
                adc_reading += adc1_get_raw(channel);
                // vTaskDelay(pdMS_TO_TICKS(10));
        }
        adc_reading /= NO_OF_SAMPLES;
        
        //Convert adc_reading to voltage in mV
        voltage = esp_adc_cal_raw_to_voltage(adc_reading, adc_chars);
        if(voltage<846){
            status=1;
        }else{
            status=0;
        }
        if(slight[index]!=status){
            slight[index]=status;
            lightChange=true;
            ESP_LOGI(TAG,"status change,index:%d,light:%d",index,status);
        }
        printf("ADC%d:Raw: %d\tVoltage: %dmV\n", index,adc_reading, voltage);
        vTaskDelay(pdMS_TO_TICKS(500));
        cnt++;
    }
}


void aws_iot_task(void *param) {
    
    
    
    esp_log_level_set("aws_iot",ESP_LOG_VERBOSE);
    int32_t i = 0;

    IoT_Error_t rc = FAILURE;

    
    IoT_Client_Init_Params mqttInitParams = iotClientInitParamsDefault;
    IoT_Client_Connect_Params connectParams = iotClientConnectParamsDefault;

    

    ESP_LOGI(TAG, "AWS IoT SDK Version %d.%d.%d-%s", VERSION_MAJOR, VERSION_MINOR, VERSION_PATCH, VERSION_TAG);

    mqttInitParams.enableAutoReconnect = false; // We enable this later below
    mqttInitParams.pHostURL = HostAddress;
    mqttInitParams.port = port;

    mqttInitParams.pRootCALocation = (char *)aws_root_ca_pem_start;
    mqttInitParams.pDeviceCertLocation = (char *)certificate_pem_crt_start;
    mqttInitParams.pDevicePrivateKeyLocation = (char *)private_pem_key_start;


    mqttInitParams.mqttCommandTimeout_ms = 20000;
    mqttInitParams.tlsHandshakeTimeout_ms = 20000;
    mqttInitParams.isSSLHostnameVerify = true;
    mqttInitParams.disconnectHandler = disconnectCallbackHandler;
    mqttInitParams.disconnectHandlerData = NULL;


    rc = aws_iot_mqtt_init(&client, &mqttInitParams);
    if(SUCCESS != rc) {
        ESP_LOGE(TAG, "aws_iot_mqtt_init returned error : %d ", rc);
        abort();
    }

    /* Wait for WiFI to show as connected */
    xEventGroupWaitBits(wifi_event_group, WIFI_CONNECTED_BIT,
                        false, true, portMAX_DELAY);

    connectParams.keepAliveIntervalInSec = 10;
    connectParams.isCleanSession = true;
    connectParams.MQTTVersion = MQTT_3_1_1;
    /* Client ID is set in the menuconfig of the example */
    connectParams.pClientID = "5f69b964480ec95b35a28b42";
    connectParams.clientIDLen = (uint16_t) strlen("5f69b964480ec95b35a28b42");
    connectParams.isWillMsgPresent = false;

    ESP_LOGI(TAG, "Connecting to AWS...");
    do {
        rc = aws_iot_mqtt_connect(&client, &connectParams);
        if(SUCCESS != rc) {
            ESP_LOGE(TAG, "Error(%d) connecting to %s:%d", rc, mqttInitParams.pHostURL, mqttInitParams.port);
            vTaskDelay(1000 / portTICK_RATE_MS);
        }
    } while(SUCCESS != rc);


    //
    /*
     * Enable Auto Reconnect functionality. Minimum and Maximum time of Exponential backoff are set in aws_iot_config.h
     *  #AWS_IOT_MQTT_MIN_RECONNECT_WAIT_INTERVAL
     *  #AWS_IOT_MQTT_MAX_RECONNECT_WAIT_INTERVAL
     */
    rc = aws_iot_mqtt_autoreconnect_set_status(&client, true);
    if(SUCCESS != rc) {
        ESP_LOGE(TAG, "Unable to set Auto Reconnect to true - %d", rc);
        abort();
    }

    const char *TOPIC = "stepiot/data/1259014452121047040/1263376536367665152/5f69b964480ec95b35a28b42";
    const int TOPIC_LEN = strlen("stepiot/data/1259014452121047040/1263376536367665152/5f69b964480ec95b35a28b42");

    

    ESP_LOGI(TAG, "Subscribing...");
    rc = aws_iot_mqtt_subscribe(&client, TOPIC, TOPIC_LEN, QOS0, iot_subscribe_callback_handler, NULL);
    if(SUCCESS != rc) {
        ESP_LOGE(TAG, "Error subscribing : %d ", rc);
        abort();
    }

    // xTaskCreate(publish_task, "publish_task", 4096, NULL, 9, NULL);

    while((NETWORK_ATTEMPTING_RECONNECT == rc || NETWORK_RECONNECTED == rc || SUCCESS == rc)) {

        //Max time the yield function will wait for read messages
        rc = aws_iot_mqtt_yield(&client, 100);
        if(NETWORK_ATTEMPTING_RECONNECT == rc) {
            // If the client is attempting to reconnect we will skip the rest of the loop.
            continue;
        }
        if(lightChange){
            publish_led();
            lightChange=false;
        }
        // ESP_LOGI(TAG, "Stack remaining for task '%s' is %d bytes", pcTaskGetTaskName(NULL), uxTaskGetStackHighWaterMark(NULL));
        vTaskDelay(1000 / portTICK_RATE_MS);
        
    }

    ESP_LOGE(TAG, "An error occurred in the main loop.");
    abort();
}


static void initialise_wifi(void)
{
    wifi_event_group = xEventGroupCreate();

    ESP_ERROR_CHECK(esp_netif_init());

    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    esp_event_handler_instance_t instance_any_id;
    esp_event_handler_instance_t instance_got_ip;
    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT,
                                                        ESP_EVENT_ANY_ID,
                                                        &event_handler,
                                                        NULL,
                                                        &instance_any_id));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT,
                                                        IP_EVENT_STA_GOT_IP,
                                                        &event_handler,
                                                        NULL,
                                                        &instance_got_ip));

    wifi_config_t wifi_config = {
        .sta = {
            .ssid = EXAMPLE_WIFI_SSID,
            .password = EXAMPLE_WIFI_PASS,
            /* Setting a password implies station will connect to all security modes including WEP/WPA.
             * However these modes are deprecated and not advisable to be used. Incase your Access point
             * doesn't support WPA2, these mode can be enabled by commenting below line */
	     .threshold.authmode = WIFI_AUTH_WPA2_PSK,

            .pmf_cfg = {
                .capable = true,
                .required = false
            },
        },
    };
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA) );
    ESP_ERROR_CHECK(esp_wifi_set_config(ESP_IF_WIFI_STA, &wifi_config) );
    ESP_ERROR_CHECK(esp_wifi_start() );

    ESP_LOGI(TAG, "wifi_init_sta finished.");
}

static char *create_state(uint8_t* lightVal,uint8_t size)
{
    char* string=NULL;

    cJSON *thingname = NULL;
    cJSON *group = NULL;
    cJSON *topic = NULL;
    cJSON *timestamp = NULL;
    cJSON *cmd=NULL;
    cJSON *subcmd=NULL;
    cJSON *light=NULL;
    cJSON *circuit=NULL;

    cJSON *root = cJSON_CreateObject();
    if (root == NULL)
    {   
        ESP_LOGE(TAG,"create cJSON root failed");
        goto end;
    }

    thingname = cJSON_CreateString("5f69b964480ec95b35a28b42");
    if (thingname == NULL)
    {
        ESP_LOGE(TAG,"create cJSON thingname failed");
        goto end;
    }
    cJSON_AddItemToObject(root, "thingname", thingname);



    topic = cJSON_CreateString("stepiot/state/1259014452121047040/1263376536367665152/5f69b964480ec95b35a28b42");
    if (topic == NULL)
    {
        ESP_LOGE(TAG,"create cJSON topic failed");
        goto end;
    }
    cJSON_AddItemToObject(root, "topic", topic);

    group = cJSON_CreateString("1263376536367665152");
    if (group == NULL)
    {
        ESP_LOGE(TAG,"create cJSON group failed");
        goto end;
    }
    cJSON_AddItemToObject(root, "group", group);


    timestamp = cJSON_CreateNumber(xTaskGetTickCount());
    if (timestamp == NULL)
    {
        ESP_LOGE(TAG,"create cJSON timestamp failed");
        goto end;
    }
    cJSON_AddItemToObject(root, "timestamp", timestamp);


    cmd = cJSON_CreateArray();
    if (cmd == NULL)
    {
        ESP_LOGE(TAG,"create cJSON cmd failed");
        goto end;
    }
    cJSON_AddItemToObject(root, "cmd", cmd);

    for (int i = 0; i < size; ++i)
    {
        subcmd = cJSON_CreateObject();
        if (subcmd == NULL)
        {
            ESP_LOGE(TAG,"create cJSON subcmd failed");
            goto end;
        }
        cJSON_AddItemToArray(cmd, subcmd);

        circuit = cJSON_CreateNumber(i+1);
        if (circuit == NULL)
        {
            ESP_LOGE(TAG,"create cJSON subcmd[circuit] failed");
            goto end;
        }
        cJSON_AddItemToObject(subcmd, "circuit", circuit);

        light = cJSON_CreateNumber(lightVal[i]);
        if (light == NULL)
        {
            ESP_LOGE(TAG,"create cJSON subcmd[light] failed");
            goto end;
        }
        cJSON_AddItemToObject(subcmd, "light", light);
    }

    string = cJSON_Print(root);
    if (string == NULL)
    {
        ESP_LOGE(TAG,"failed to print root");
    }

end:
    cJSON_Delete(root);
    return string;
}



void app_main()
{
    // Initialize NVS.
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK( err );

    initialise_wifi();
    /*gpio*/
    gpio_config_t io_conf;
    io_conf.intr_type = GPIO_INTR_DISABLE;
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pin_bit_mask = GPIO_OUTPUT_PIN_SEL;
    io_conf.pull_down_en = 1;
    io_conf.pull_up_en = 0;
    gpio_config(&io_conf);
    xTaskCreate(&aws_iot_task, "aws_iot_task", 9216, NULL, 5, NULL);
    xTaskCreate(adc_task, "adc_task", 2048, NULL, 1, NULL);
    vTaskSuspend(NULL);
}

