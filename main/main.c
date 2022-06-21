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
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "myble.h"
#include "font.h"
#include "myScreen.h"
#include "esp_nimble_hci.h"

char *ble_name = "lghGood";
static const char *TAG = "HTTP_CLIENT";
int haveSD=false;
int sdFailStatus=true;
static TaskHandle_t detect_task_h;
static TaskHandle_t ble_task_h;
xQueueHandle  ble_evt_queue = NULL;




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
   ESP_LOGE("good","%d",size);


    nimble_port_freertos_deinit();
    nimble_port_deinit();
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
                esp_restart();
                break;
            default:
                break;
        }



    }
}

uint32_t num;
uint32_t num2;
static void detect1_task(void *pvParameters) {
    while (true){
        if(sdFailStatus){
            sdFailStatus=sdcard_mount();
            num=1;
            if(sdFailStatus){
                num=1;
            }else{
                num=2;
                num2=1;
                xQueueSend(ble_evt_queue, &num2, NULL);
            }
            xQueueSend(disp_evt_queue, &num, NULL);
        }
        vTaskDelay(500);
    }

}


void n1(){
    clearScreen(0xffff);
    drawString(k1,10,5,0,0xffff);
    dispLine(1);
}
static void initialize_nvs(void) {
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK(err);
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




}
