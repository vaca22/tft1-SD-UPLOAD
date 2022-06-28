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
#include <sys/dirent.h>
#include <esp_http_client.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "config.h"

char *ble_name = BLE_NAME;
static const char *TAG = "HTTP_CLIENT";
int haveSD = false;
int sdFailStatus = true;
static TaskHandle_t detect_task_h;
static TaskHandle_t ble_task_h;
xQueueHandle ble_evt_queue = NULL;

uint32_t disp_msg;
uint32_t ble_msg;

uint8_t wifi_name[32];
uint8_t wifi_password[64];
char    file_name[32]={0};
long fileLen=0;
int isUploading=0;
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


int wifi_connect_flag = 0;

char MOUNT_POINT[100]="/sdcard/";
sdmmc_card_t *card;
const char mount_point[] = "/sdcard";
sdmmc_host_t host= SDMMC_HOST_DEFAULT();;
int sdcard_mount(void) {
    esp_err_t ret;
    esp_vfs_fat_sdmmc_mount_config_t mount_config = {
            .format_if_mount_failed = true,
            .max_files = 5,
            .allocation_unit_size = 16 * 1024
    };


    ESP_LOGI(TAG, "Initializing SD card");


    ESP_LOGI(TAG, "Using SDMMC peripheral");



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


void ble_uart(const void *src, size_t size) {
    cJSON *json = cJSON_Parse(src);
    if (json != NULL) {
        cJSON *receiveN1 = cJSON_GetObjectItemCaseSensitive(json, "x1");
        cJSON *receiveN2 = cJSON_GetObjectItemCaseSensitive(json, "x2");
        cJSON *receiveN3 = cJSON_GetObjectItemCaseSensitive(json, "x3");
        cJSON *receiveN4 = cJSON_GetObjectItemCaseSensitive(json, "x4");
        cJSON *receiveN5 = cJSON_GetObjectItemCaseSensitive(json, "x5");
        ESP_LOGE("re", "%s   %s   %s  %s  %s",
                 receiveN1->valuestring,
                 receiveN2->valuestring,
                 receiveN3->valuestring,
                 receiveN4->valuestring,
                 receiveN5->valuestring);
        ble_msg = 2;

        memcpy(wifi_name, receiveN1->valuestring, strlen(receiveN1->valuestring) + 1);
        memcpy(wifi_password, receiveN2->valuestring, strlen(receiveN2->valuestring) + 1);
        xQueueSend(ble_evt_queue, &ble_msg, NULL);
    } else {
        ESP_LOGE("re", "no work parse");
    }


}


const int MIN_RSSI = -80;


const int MAX_RSSI = -55;

int calculateSignalLevel(int rssi, int numLevels) {
    if (rssi <= MIN_RSSI) {
        return 0;
    } else if (rssi >= MAX_RSSI) {
        return numLevels - 1;
    } else {
        float inputRange = (MAX_RSSI - MIN_RSSI);
        float outputRange = (numLevels - 1);
        return (int) ((float) (rssi - MIN_RSSI) * outputRange / inputRange);
    }
}
#define FILE_PATH_MAX 56
static void detect1_task(void *pvParameters) {
    while (true) {
        if (sdFailStatus) {
            sdFailStatus = sdcard_mount();
            disp_msg = 1;
            if (sdFailStatus) {
                disp_msg = 1;
            } else {
                char entrypath[FILE_PATH_MAX];
                struct dirent *entry;
                const char *entrytype;
                struct stat entry_stat;
                char *dirpath="/sdcard/";
                DIR *dir = opendir(dirpath);
                const size_t dirpath_len = strlen(dirpath);
                strlcpy(entrypath, dirpath, sizeof(entrypath));
                while ((entry = readdir(dir)) != NULL) {
                    entrytype = (entry->d_type == DT_DIR ? "directory" : "file");

                    strlcpy(entrypath + dirpath_len, entry->d_name, strlen(entry->d_name)+1);
                    if (stat(entrypath, &entry_stat) == -1) {
                        ESP_LOGE(TAG, "Failed to stat %s : %s", entrytype, entry->d_name);
                        continue;
                    }
                    if(entry->d_type==DT_DIR){
                        continue;
                    }
                    if(entry_stat.st_size<MIN_FILE_SIZE){
                        continue;
                    }
                    memcpy(file_name,entry->d_name, strlen(entry->d_name)+1);
                    fileLen=entry_stat.st_size;
                    ESP_LOGE("foile","%s   %s   %ld",entrytype,entry->d_name,entry_stat.st_size);
                }

                disp_msg = 2;
                ble_msg = 1;
                xQueueSend(ble_evt_queue, &ble_msg, NULL);


            }
            xQueueSend(disp_evt_queue, &disp_msg, NULL);
        }
        vTaskDelay(500);
        if (wifi_connect_flag) {
            wifi_ap_record_t ap_info;
            esp_wifi_sta_get_ap_info(&ap_info);
            int level = calculateSignalLevel(ap_info.rssi, 5);
            disp_msg = level + 11;
            xQueueSend(disp_evt_queue, &disp_msg, NULL);
        }else{
            disp_msg = 10;
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


static EventGroupHandle_t s_wifi_event_group=NULL;


#define WIFI_CONNECTED_BIT BIT0
#define WIFI_FAIL_BIT      BIT1


static int s_retry_num = 0;

static void event_handler(void *arg, esp_event_base_t event_base,
                          int32_t event_id, void *event_data) {
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {

        if (s_retry_num < 3) {
            esp_wifi_connect();
            s_retry_num++;
            ESP_LOGI(TAG, "retry to connect to the AP");
        } else {
            xEventGroupSetBits(s_wifi_event_group, WIFI_FAIL_BIT);
        }
        ESP_LOGI(TAG, "connect to the AP fail");
        if(wifi_connect_flag==1){
            s_retry_num=0;
            esp_wifi_connect();
        }
        wifi_connect_flag = 0;
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        wifi_connect_flag = 1;
        ip_event_got_ip_t *event = (ip_event_got_ip_t *) event_data;
        ESP_LOGI(TAG, "got ip:" IPSTR, IP2STR(&event->ip_info.ip));
        s_retry_num = 0;
        xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
        disp_msg = 5;
        xQueueSend(disp_evt_queue, &disp_msg, NULL);
    }
}


void wifi_init_sta(void) {
    int new=0;
    if(s_wifi_event_group==NULL){
        new=1;
        s_wifi_event_group = xEventGroupCreate();

        ESP_ERROR_CHECK(esp_netif_init());

        ESP_ERROR_CHECK(esp_event_loop_create_default());
        esp_netif_t* obj=esp_netif_create_default_wifi_sta();
        esp_netif_set_hostname(obj, "lghGood");
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

    }


    wifi_config_t wifi_config = {
            .sta = {
                    .threshold.authmode = WIFI_AUTH_WPA2_PSK,
            },
    };
    for (int k = 0; k < 32; k++) {
        wifi_config.sta.ssid[k] = wifi_name[k];
    }
    for (int k = 0; k < 64; k++) {
        wifi_config.sta.password[k] = wifi_password[k];
    }

    if(new==0){
        esp_wifi_stop();
        s_retry_num=0;
    }

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_LOGI(TAG, "wifi_init_sta finished.");
    EventBits_t bits = xEventGroupWaitBits(s_wifi_event_group,
                                           WIFI_CONNECTED_BIT | WIFI_FAIL_BIT,
                                           pdFALSE,
                                           pdFALSE,
                                           portMAX_DELAY);

    if (bits & WIFI_CONNECTED_BIT) {
        ESP_LOGE(TAG, "connected to ap SSID:%s password:%s",
                 wifi_name, wifi_password);
        wifi_connect_flag = 1;
        disp_msg = 5;
        xQueueSend(disp_evt_queue, &disp_msg, NULL);


    } else if (bits & WIFI_FAIL_BIT) {
        ESP_LOGE(TAG, "Failed to connect to SSID:%s, password:%s",
                 wifi_name, wifi_password);
        disp_msg = 6;
        xQueueSend(disp_evt_queue, &disp_msg, NULL);
    } else {
        ESP_LOGE(TAG, "UNEXPECTED EVENT");
    }
    xEventGroupClearBits(s_wifi_event_group,0xff);


}


uint32_t ble_num;

static void ble_task(void *pvParameters) {
    ble_evt_queue = xQueueCreate(10, sizeof(uint32_t));
    send_uart_callback *ble_uart_callback;
    ble_uart_callback = (send_uart_callback *) malloc(sizeof(send_uart_callback));
    ble_uart_callback->func_name = ble_uart;
    register_uart(ble_uart_callback);

    while (1) {
        xQueueReceive(ble_evt_queue, &ble_num, portMAX_DELAY);
        switch (ble_num) {
            case 1:
                init_ble();
                break;
            case 2:
                disp_msg = 4;
                xQueueSend(disp_evt_queue, &disp_msg, NULL);
                wifi_init_sta();
                break;
            default:
                break;
        }


    }
}







#define MAX_HTTP_OUTPUT_BUFFER 2048
long file_len;
long have_send;
FILE *fd = NULL;
#define Segment 16384
char fileTemp[Segment]={0};
static char card_buf[16384];
static void http_native_request(void)
{
    for(int k=0;k<32;k++){
        MOUNT_POINT[k+8]=file_name[k];
    }
    fd = fopen(MOUNT_POINT, "rb");
    setvbuf(fd, card_buf, _IOFBF, 16384);
    char output_buffer[MAX_HTTP_OUTPUT_BUFFER] = {0};   // Buffer to store response of http request
    int content_length = 0;
    esp_http_client_config_t config = {
            .url = "http://httpbin.org/get",
    };
    esp_http_client_handle_t client = esp_http_client_init(&config);



    esp_http_client_set_url(client, POST_URL);
    esp_http_client_set_method(client, HTTP_METHOD_POST);
    esp_http_client_set_header(client, "Content-Type", "application/json");
    esp_http_client_set_header(client, "filename", file_name);

    file_len=fileLen;
    have_send=0;
    char lenString[20];
    sprintf(lenString,"%ld",fileLen);

    esp_http_client_set_header(client, "len", lenString);
    esp_err_t  err = esp_http_client_open(client, file_len);
    disp_msg=8;

    if (err != ESP_OK) {
        disp_msg=7;
        ESP_LOGE(TAG, "Failed to open HTTP connection: %s", esp_err_to_name(err));
    } else {

        while(1){
            if(file_len-have_send<Segment){
                fread(fileTemp, file_len - have_send, 1, fd);
                int wlen = esp_http_client_write(client, fileTemp, file_len - have_send);
                if (wlen < 0) {
                    disp_msg=7;
                    ESP_LOGE(TAG, "Write failed");
                    break;
                }else{
                    disp_msg=100;
                    xQueueSend(disp_evt_queue, &disp_msg, NULL);
                }
                break;
            }else{
                fread(fileTemp, Segment, 1, fd);
                int wlen = esp_http_client_write(client, fileTemp, Segment);
                if (wlen < 0) {
                    disp_msg=7;
                    ESP_LOGE(TAG, "Write failed");
                    break;
                }else{
                    disp_msg=100;
                    xQueueSend(disp_evt_queue, &disp_msg, NULL);
                }
                have_send+=Segment;
                if(have_send==file_len){
                    break;
                }
            };
        }

        fclose(fd);

        content_length = esp_http_client_fetch_headers(client);
        if (content_length < 0) {
            disp_msg=7;
            ESP_LOGE(TAG, "HTTP client fetch headers failed");
        } else {
            int data_read = esp_http_client_read_response(client, output_buffer, MAX_HTTP_OUTPUT_BUFFER);
            if (data_read >= 0) {
                disp_msg=8;
                ESP_LOGI(TAG, "HTTP POST Status = %d, content_length = %d",
                         esp_http_client_get_status_code(client),
                         esp_http_client_get_content_length(client));
                ESP_LOG_BUFFER_HEX(TAG, output_buffer, strlen(output_buffer));
            } else {
                ESP_LOGE(TAG, "Failed to read response");
            }
        }



    }

    ESP_LOGE("comlipe","com  %d",disp_msg);
    xQueueSend(disp_evt_queue, &disp_msg, NULL);
    esp_http_client_cleanup(client);
    isUploading=0;
    esp_vfs_fat_sdcard_unmount(mount_point, card);
}









static void http_test_task(void *pvParameters)
{
    http_native_request();
    vTaskDelete(NULL);
}







#define GPIO_INPUT_IO_0     0
#define GPIO_INPUT_PIN_SEL  ((1ULL<<GPIO_INPUT_IO_0) )
#define ESP_INTR_FLAG_DEFAULT 0
static xQueueHandle gpio_evt_queue = NULL;

static void IRAM_ATTR gpio_isr_handler(void* arg)
{
    uint32_t gpio_num = (uint32_t) arg;
    xQueueSendFromISR(gpio_evt_queue, &gpio_num, NULL);
}









static void button_task_example(void* arg)
{
    uint32_t io_num;
    for(;;) {
        if(xQueueReceive(gpio_evt_queue, &io_num, portMAX_DELAY)) {
            if(gpio_get_level(io_num)==0){
                if(isUploading==0){
                    isUploading=1;
                    xTaskCreatePinnedToCore(&http_test_task, "http_test_task", 8192, NULL, 5, NULL,0);
                }
            }
        }
    }
}
void initButton(){
    gpio_config_t io_conf = {};

    //interrupt of rising edge
    io_conf.intr_type =GPIO_INTR_NEGEDGE;
    //bit mask of the pins, use GPIO4/5 here
    io_conf.pin_bit_mask = GPIO_INPUT_PIN_SEL;
    //set as input mode
    io_conf.mode = GPIO_MODE_INPUT;
    //enable pull-up mode
    io_conf.pull_up_en = 1;
    gpio_config(&io_conf);
    gpio_evt_queue = xQueueCreate(1, sizeof(uint32_t));
    xTaskCreate(button_task_example, "button_task", 2048, NULL, 10, NULL);
    gpio_install_isr_service(ESP_INTR_FLAG_DEFAULT);
    gpio_isr_handler_add(GPIO_INPUT_IO_0, gpio_isr_handler, (void*) GPIO_INPUT_IO_0);

}








void app_main(void) {
    initialize_nvs();
    initButton();
    initScreen();


    xTaskCreatePinnedToCore(detect1_task, "detect", 4096, NULL, configMAX_PRIORITIES, &detect_task_h, 1);
    xTaskCreatePinnedToCore(ble_task, "ble", 4096, NULL, configMAX_PRIORITIES, &ble_task_h, 1);

}
