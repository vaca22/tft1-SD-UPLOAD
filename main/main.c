/* SPI Master example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <esp_log.h>
#include <esp_vfs_fat.h>
#include <driver/sdmmc_host.h>
#include <sdmmc_cmd.h>
#include <nvs_flash.h>
#include <nimble/nimble_port_freertos.h>
#include <nimble/nimble_port.h>
#include <host/ble_gap.h>
#include <cJSON.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "myble.h"
#include "font.h"
#include "myScreen.h"
#include "esp_nimble_hci.h"
#include <string.h>
#include <esp_rrm.h>
#include <esp_wnm.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "nvs_flash.h"

char *ble_name = "lghGood";
static const char *TAG = "HTTP_CLIENT";
int haveSD=false;
int sdFailStatus=true;
static TaskHandle_t detect_task_h;
static TaskHandle_t ble_task_h;
xQueueHandle  ble_evt_queue = NULL;

uint32_t disp_msg;
uint32_t ble_msg;

uint8_t wifi_name[32];
uint8_t wifi_password[64];


#define LCD_HOST    SPI2_HOST

#define PIN_NUM_MISO -1
#define PIN_NUM_MOSI 11
#define PIN_NUM_CLK  12
#define PIN_NUM_CS   10

#define PIN_NUM_DC   9
#define PIN_NUM_RST  -1
#define PIN_NUM_BCKL 46


#define PARALLEL_LINES 60
spi_device_handle_t *mySpi;


int wifi_connect_flag=0;

#define MOUNT_POINT "/sdcard"

int sdcard_mount(void)
{
    esp_err_t ret;
    esp_vfs_fat_sdmmc_mount_config_t mount_config = {
            .format_if_mount_failed = false,
            .max_files = 5,
            .allocation_unit_size = 16 * 1024
    };
    sdmmc_card_t *card;
    const char mount_point[] = MOUNT_POINT;
    ESP_LOGI(TAG, "Initializing SD card");


    ESP_LOGI(TAG, "Using SDMMC peripheral");
    sdmmc_host_t host = SDMMC_HOST_DEFAULT();

    sdmmc_slot_config_t slot_config = SDMMC_SLOT_CONFIG_DEFAULT();

    slot_config.width = 1;

    slot_config.clk = GPIO_NUM_3;
    slot_config.cmd = GPIO_NUM_4;
    slot_config.d0 = GPIO_NUM_14;
    slot_config.d1 = GPIO_NUM_8;
    slot_config.d2 = GPIO_NUM_12;
    slot_config.d3 = GPIO_NUM_8;

    slot_config.flags |= SDMMC_SLOT_FLAG_INTERNAL_PULLUP;

    ESP_LOGI(TAG, "Mounting filesystem");
    ret = esp_vfs_fat_sdmmc_mount(mount_point, &host, &slot_config, &mount_config, &card);

    if (ret != ESP_OK) {

        return 1;
    }
    ESP_LOGI(TAG, "Filesystem mounted");


    sdmmc_card_print_info(stdout, card);
    return 0;


}


void ble_uart( const void *src, size_t size){
    cJSON *json = cJSON_Parse(src);
    if (json != NULL)
    {
        cJSON* receiveN1=cJSON_GetObjectItemCaseSensitive(json, "x1");
        cJSON* receiveN2=cJSON_GetObjectItemCaseSensitive(json, "x2");
        cJSON* receiveN3=cJSON_GetObjectItemCaseSensitive(json, "x3");
        cJSON* receiveN4=cJSON_GetObjectItemCaseSensitive(json, "x4");
        cJSON* receiveN5=cJSON_GetObjectItemCaseSensitive(json, "x5");
        ESP_LOGE("re", "%s   %s   %s  %s  %s",
                 receiveN1->valuestring,
                 receiveN2->valuestring,
                 receiveN3->valuestring,
                 receiveN4->valuestring,
                 receiveN5->valuestring);
        ble_msg=2;

        memcpy(wifi_name,receiveN1->valuestring, strlen(receiveN1->valuestring)+1);
        memcpy(wifi_password,receiveN2->valuestring, strlen(receiveN2->valuestring)+1);
        xQueueSend(ble_evt_queue, &ble_msg, NULL);
    }else{
        ESP_LOGE("re","no work parse");
    }
    nimble_port_freertos_deinit();

}


const int MIN_RSSI = -80;


const int MAX_RSSI = -55;

int calculateSignalLevel(int rssi, int numLevels) {
    if(rssi <= MIN_RSSI) {
        return 0;
    } else if (rssi >= MAX_RSSI) {
        return numLevels - 1;
    } else {
        float inputRange = (MAX_RSSI -MIN_RSSI);
        float outputRange = (numLevels - 1);
        return (int)((float)(rssi - MIN_RSSI) * outputRange / inputRange);
    }
}

static void detect1_task(void *pvParameters) {
    while (true){
        if(sdFailStatus){
            sdFailStatus=sdcard_mount();
            disp_msg=1;
            if(sdFailStatus){
                disp_msg=1;
            }else{
                disp_msg=2;
                ble_msg=1;
                xQueueSend(ble_evt_queue, &ble_msg, NULL);
            }
            xQueueSend(disp_evt_queue, &disp_msg, NULL);
        }
        vTaskDelay(500);
        if(wifi_connect_flag){
            wifi_ap_record_t ap_info;
            esp_wifi_sta_get_ap_info(&ap_info);
            int level= calculateSignalLevel(ap_info.rssi,5);
            disp_msg=level+11;
            xQueueSend(disp_evt_queue, &disp_msg, NULL);
        }

    }

}



static void initialize_nvs(void) {
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK(err);
}






static EventGroupHandle_t s_wifi_event_group;


#define WIFI_CONNECTED_BIT BIT0
#define WIFI_FAIL_BIT      BIT1



static int s_retry_num = 0;

static void event_handler(void* arg, esp_event_base_t event_base,
                          int32_t event_id, void* event_data)
{
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        wifi_connect_flag=0;
        if (s_retry_num < 10) {
            esp_wifi_connect();
            s_retry_num++;
            ESP_LOGI(TAG, "retry to connect to the AP");
        } else {
            xEventGroupSetBits(s_wifi_event_group, WIFI_FAIL_BIT);
        }
        ESP_LOGI(TAG,"connect to the AP fail");
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        wifi_connect_flag=1;
        ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;
        ESP_LOGI(TAG, "got ip:" IPSTR, IP2STR(&event->ip_info.ip));
        s_retry_num = 0;
        xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
    }
}




void wifi_init_sta(void)
{
    s_wifi_event_group = xEventGroupCreate();

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
                    .threshold.authmode = WIFI_AUTH_WPA2_PSK,
            },
    };
    for(int k=0;k<32;k++){
        wifi_config.sta.ssid[k]=wifi_name[k];
    }
    for(int k=0;k<64;k++){
        wifi_config.sta.password[k]=wifi_password[k];
    }


    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA) );
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config) );
    ESP_ERROR_CHECK(esp_wifi_start() );

    ESP_LOGI(TAG, "wifi_init_sta finished.");

    /* Waiting until either the connection is established (WIFI_CONNECTED_BIT) or connection failed for the maximum
     * number of re-tries (WIFI_FAIL_BIT). The bits are set by event_handler() (see above) */
    EventBits_t bits = xEventGroupWaitBits(s_wifi_event_group,
                                           WIFI_CONNECTED_BIT | WIFI_FAIL_BIT,
                                           pdFALSE,
                                           pdFALSE,
                                           portMAX_DELAY);

    /* xEventGroupWaitBits() returns the bits before the call returned, hence we can test which event actually
     * happened. */
    if (bits & WIFI_CONNECTED_BIT) {
        ESP_LOGI(TAG, "connected to ap SSID:%s password:%s",
                 wifi_name,wifi_password);
        wifi_connect_flag=1;


    } else if (bits & WIFI_FAIL_BIT) {
        ESP_LOGI(TAG, "Failed to connect to SSID:%s, password:%s",
                 wifi_name, wifi_password);
    } else {
        ESP_LOGE(TAG, "UNEXPECTED EVENT");
    }

    /* The event will not be processed after unregister */
//    ESP_ERROR_CHECK(esp_event_handler_instance_unregister(IP_EVENT, IP_EVENT_STA_GOT_IP, instance_got_ip));
//    ESP_ERROR_CHECK(esp_event_handler_instance_unregister(WIFI_EVENT, ESP_EVENT_ANY_ID, instance_any_id));
//    vEventGroupDelete(s_wifi_event_group);
}






uint32_t ble_num;
static void ble_task(void *pvParameters) {
    ble_evt_queue= xQueueCreate(10, sizeof(uint32_t));
    send_uart_callback *ble_uart_callback;
    ble_uart_callback=(send_uart_callback *) malloc(sizeof(send_uart_callback));
    ble_uart_callback->func_name=ble_uart;
    register_uart(ble_uart_callback);

    while (1){
        xQueueReceive(ble_evt_queue, &ble_num, portMAX_DELAY);
        switch(ble_num){
            case 1:
                init_ble();
                break;
            case 2:
                wifi_init_sta();
                break;
            default:
                break;
        }



    }
}

void app_main(void) {
    initialize_nvs();


    initScreen();



//    for(int k=0;k<=100;k++){
//        dispProgress(k);
//        vTaskDelay(10);
//    }

    xTaskCreatePinnedToCore(detect1_task, "detect", 4096, NULL, configMAX_PRIORITIES, &detect_task_h, 1);
    xTaskCreatePinnedToCore(ble_task, "ble", 4096, NULL, configMAX_PRIORITIES, &ble_task_h, 1);

//    disp_msg=11;
//    xQueueSend(disp_evt_queue, &disp_msg, NULL);

}
