//
// Created by vaca on 6/21/22.
//

#include <driver/spi_master.h>
#include <string.h>
#include <driver/gpio.h>
#include <freertos/task.h>
#include <freertos/queue.h>
#include "myScreen.h"
#include "font.h"
#include "decode_image.h"

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

extern const unsigned char s1_start[] asm("_binary_s1_jpg_start");
extern const unsigned char s2_start[] asm("_binary_s2_jpg_start");
extern const unsigned char s3_start[] asm("_binary_s3_jpg_start");
extern const unsigned char s4_start[] asm("_binary_s4_jpg_start");
extern const unsigned char s5_start[] asm("_binary_s5_jpg_start");


typedef struct {
    uint8_t cmd;
    uint8_t data[16];
    uint8_t databytes;
} lcd_init_cmd_t;

DRAM_ATTR static const lcd_init_cmd_t st_init_cmds[] = {
        {0x11, {0},                                                                                  0x80},
        /* Memory Data Access Control, MX=MV=1, MY=ML=MH=0, RGB=0 */
        {0x36, {0x70},                                                                               1},
        /* Interface Pixel Format, 16bits/pixel for RGB/MCU interface */
        {0x3A, {0x05},                                                                               1},
        /* Porch Setting */
        {0xB2, {0x0c, 0x0c, 0x00, 0x33, 0x33},                                                       5},
        /* Gate Control, Vgh=13.65V, Vgl=-10.43V */
        {0xB7, {0x35},                                                                               1},
        /* VCOM Setting, VCOM=1.175V */
        {0xBB, {0x19},                                                                               1},
        /* LCM Control, XOR: BGR, MX, MH */
        {0xC0, {0x2C},                                                                               1},
        /* VDV and VRH Command Enable, enable=1 */
        {0xC2, {0x01},                                                                               1},
        /* VRH Set, Vap=4.4+... */
        {0xC3, {0x12},                                                                               1},
        /* VDV Set, VDV=0 */
        {0xC4, {0x20},                                                                               1},
        /* Frame Rate Control, 60Hz, inversion=0 */
        {0xC6, {0x0f},                                                                               1},
        /* Power Control 1, AVDD=6.8V, AVCL=-4.8V, VDDS=2.3V */
        {0xD0, {0xA4, 0xA1},                                                                         1},
        /* Positive Voltage Gamma Control */
        {0xE0, {0xD0, 0x04, 0x0D, 0x11, 0x13, 0x2B, 0x3F, 0x54, 0x4C, 0x18, 0x0D, 0x0B, 0x1F, 0x23}, 14},
        /* Negative Voltage Gamma Control */
        {0xE1, {0xD0, 0x04, 0x0C, 0x11, 0x13, 0x2C, 0x3F, 0x44, 0x51, 0x2F, 0x1F, 0x1F, 0x20, 0x23}, 14},
        /* Sleep Out */
        {0x21, {0},                                                                                  0x80},
        /* Display On */
        {0x29, {0},                                                                                  0x80},
        {0,    {0},                                                                                  0xff}
};


void lcd_cmd(spi_device_handle_t spi, const uint8_t cmd) {
    esp_err_t ret;
    spi_transaction_t t;
    memset(&t, 0, sizeof(t));       //Zero out the transaction
    t.length = 8;                     //Command is 8 bits
    t.tx_buffer = &cmd;               //The data is the cmd itself
    t.user = (void *) 0;                //D/C needs to be set to 0
    ret = spi_device_polling_transmit(spi, &t);  //Transmit!
    assert(ret == ESP_OK);            //Should have had no issues.
}


void lcd_data(spi_device_handle_t spi, const uint8_t *data, int len) {
    esp_err_t ret;
    spi_transaction_t t;
    if (len == 0) return;             //no need to send anything
    memset(&t, 0, sizeof(t));       //Zero out the transaction
    t.length = len * 8;                 //Len is in bytes, transaction length is in bits.
    t.tx_buffer = data;               //Data
    t.user = (void *) 1;                //D/C needs to be set to 1
    ret = spi_device_polling_transmit(spi, &t);  //Transmit!
    assert(ret == ESP_OK);            //Should have had no issues.
}


void lcd_spi_pre_transfer_callback(spi_transaction_t *t) {
    int dc = (int) t->user;
    gpio_set_level(PIN_NUM_DC, dc);
}


//Initialize the display
void lcd_init(spi_device_handle_t spi) {
    int cmd = 0;
    const lcd_init_cmd_t *lcd_init_cmds;

    //Initialize non-SPI GPIOs
    gpio_set_direction(PIN_NUM_DC, GPIO_MODE_OUTPUT);
    gpio_set_direction(PIN_NUM_BCKL, GPIO_MODE_OUTPUT);


    lcd_init_cmds = st_init_cmds;

    //Send all the commands
    while (lcd_init_cmds[cmd].databytes != 0xff) {
        lcd_cmd(spi, lcd_init_cmds[cmd].cmd);
        lcd_data(spi, lcd_init_cmds[cmd].data, lcd_init_cmds[cmd].databytes & 0x1F);
        if (lcd_init_cmds[cmd].databytes & 0x80) {
            vTaskDelay(100 / portTICK_RATE_MS);
        }
        cmd++;
    }

    gpio_set_level(PIN_NUM_BCKL, 1);
}


static void send_lines(spi_device_handle_t spi, int ypos, uint16_t *linedata) {
    esp_err_t ret;
    int x;

    static spi_transaction_t trans[6];

    for (x = 0; x < 6; x++) {
        memset(&trans[x], 0, sizeof(spi_transaction_t));
        if ((x & 1) == 0) {
            trans[x].length = 8;
            trans[x].user = (void *) 0;
        } else {
            trans[x].length = 8 * 4;
            trans[x].user = (void *) 1;
        }
        trans[x].flags = SPI_TRANS_USE_TXDATA;
    }
    trans[0].tx_data[0] = 0x2A;           //Column Address Set
    trans[1].tx_data[0] = (0) >> 8;              //Start Col High
    trans[1].tx_data[1] = (0) & 0XFF;              //Start Col Low
    trans[1].tx_data[2] = (240 - 1) >> 8;       //End Col High
    trans[1].tx_data[3] = (240 - 1) & 0xff;     //End Col Low
    trans[2].tx_data[0] = 0x2B;           //Page address set
    trans[3].tx_data[0] = (ypos) >> 8;        //Start page high
    trans[3].tx_data[1] = (ypos) & 0xff;      //start page low
    trans[3].tx_data[2] = (ypos + PARALLEL_LINES - 1) >> 8;    //end page high
    trans[3].tx_data[3] = (ypos + PARALLEL_LINES - 1) & 0xff;  //end page low
    trans[4].tx_data[0] = 0x2C;           //memory write
    trans[5].tx_buffer = linedata;        //finally send the line data
    trans[5].length = 240 * 2 * 8 * PARALLEL_LINES;          //Data length, in bits
    trans[5].flags = 0; //undo SPI_TRANS_USE_TXDATA flag

    for (x = 0; x < 6; x++) {
        ret = spi_device_queue_trans(spi, &trans[x], portMAX_DELAY);
        assert(ret == ESP_OK);
    }

}


static void send_line_finish(spi_device_handle_t spi) {
    spi_transaction_t *rtrans;
    esp_err_t ret;
    for (int x = 0; x < 6; x++) {
        ret = spi_device_get_trans_result(spi, &rtrans, portMAX_DELAY);
        assert(ret == ESP_OK);
    }
}


typedef unsigned short int u16;
typedef unsigned char u8;
#define USE_HORIZONTAL 2
#define LCD_W 240
#define LCD_H 240
#define WHITE             0xFFFF


uint16_t scr[14400];
//请插入设备
uint8_t k1[] = {5, 0, 1, 2, 3, 4};

//已检测到设备，请链接蓝牙
uint8_t k2[] = {12, 5, 6, 7, 8, 3, 4, 9, 0, 10, 11, 17, 18};

//设备错误，请重新插拔
uint8_t k3[] = {10, 3, 4, 12, 13, 9, 0, 14, 15, 16, 1};

//蓝牙链接成功，等待设置
uint8_t k4[] = {11, 17, 18, 10, 11, 19, 20, 9, 21, 22, 3, 23};

//传输失败
uint8_t k5[] = {4, 26, 27, 24, 25};

//传输成功，请拔除设备
uint8_t k6[] = {10, 26, 27, 19, 20, 9, 0, 16, 28, 3, 4};


void drawChar(int x1, int y1, uint8_t *z1, uint16_t frontColor, uint16_t backColor) {
    for (int k = 0; k < 16; k++) {
        for (int j = 0; j < 2; j++) {
            int x = 2 * k + j;
            int y = z1[x];
            for (int i = 0; i < 8; i++) {
                if (y & (1 << (7 - i))) {
                    scr[k * 240 + i + j * 8 + x1 + y1 * 240] = frontColor;
                } else {
                    scr[k * 240 + i + j * 8 + x1 + y1 * 240] = backColor;
                }
            }
        }
    }
}

void drawString(uint8_t *ss, int x1, int y1, uint16_t frontColor, uint16_t backColor) {
    int len = ss[0] + 1;
    for (int k = 1; k < len; k++) {
        drawChar(x1 + k * 16, y1, &myFont[ss[k] * 32], frontColor, backColor);
    }

}

void clearScreen(uint16_t color) {
    for (int k = 0; k < 14400; k++) {
        scr[k] = color;
    }
}

void drawRect(int x, int y, int w, int h, uint16_t color) {
    for (int k = 0; k < w; k++) {
        scr[x + k + y * LCD_W] = color;
        scr[x + k + (y + h) * LCD_W] = color;
    }
    for (int k = 0; k < h; k++) {
        scr[x + (y + k) * LCD_W] = color;
        scr[x + w + (y + k) * LCD_W] = color;
    }
}

void fillRect(int x, int y, int w, int h, uint16_t color) {
    for (int j = 0; j < w; j++) {
        for (int k = 0; k < h; k++) {
            scr[x + j + (y + k) * LCD_W] = color;
        }
    }
}


void dispLine(int x) {
    if (x >= 0 && x < 4) {
        send_lines(*mySpi, x * PARALLEL_LINES, scr);
        send_line_finish(*mySpi);
    }
}

void dispAll() {
    for (int k = 0; k < 4; k++) {
        dispLine(k);
    }
}

void dispProgress(int k) {
    clearScreen(0xffff);
    drawRect(20, 10, 200, 20, 0x1F00);
    fillRect(20, 10, k * 2, 20, 0x1F00);
    dispLine(3);
}



uint16_t **pixels;

void dispImg(int index){
    switch (index) {
        case 1:    decode_image(&pixels,s1_start);break;
        case 2:    decode_image(&pixels,s2_start);break;
        case 3:    decode_image(&pixels,s3_start);break;
        case 4:    decode_image(&pixels,s4_start);break;
        case 5:    decode_image(&pixels,s5_start);break;
        default:break;
    }

    for(int k=0;k<48;k++){
        for(int j=0;j<48;j++){
            scr[k*240+j+190]=pixels[k][j];
        }
    }
    dispLine(0);
}



static TaskHandle_t disp_task_h;
xQueueHandle disp_evt_queue = NULL;

static void disp_task(void *pvParameters) {
    disp_evt_queue = xQueueCreate(10, sizeof(uint32_t));
    esp_err_t ret;
    spi_device_handle_t spi;
    spi_bus_config_t buscfg = {
            .miso_io_num=PIN_NUM_MISO,
            .mosi_io_num=PIN_NUM_MOSI,
            .sclk_io_num=PIN_NUM_CLK,
            .quadwp_io_num=-1,
            .quadhd_io_num=-1,
            .max_transfer_sz=PARALLEL_LINES * 240 * 2 + 8
    };
    spi_device_interface_config_t devcfg = {
            .clock_speed_hz=26 * 1000 * 1000,           //Clock out at 10 MHz
            .mode=3,                                //SPI mode 0
            .spics_io_num=PIN_NUM_CS,               //CS pin
            .queue_size=7,                          //We want to be able to queue 7 transactions at a time
            .pre_cb=lcd_spi_pre_transfer_callback,  //Specify pre-transfer callback to handle D/C line
    };
    //Initialize the SPI bus
    ret = spi_bus_initialize(LCD_HOST, &buscfg, SPI_DMA_CH_AUTO);
    ESP_ERROR_CHECK(ret);
    //Attach the LCD to the SPI bus
    ret = spi_bus_add_device(LCD_HOST, &devcfg, &spi);
    ESP_ERROR_CHECK(ret);
    mySpi = &spi;
    lcd_init(spi);

    clearScreen(0xffff);
    dispAll();
    int n=1;
    while (1){
        dispImg(n);
        n++;
        if(n>5){
            n=1;
        }
        vTaskDelay(20);
    }

    uint32_t io_num;
    while (1) {
        xQueueReceive(disp_evt_queue, &io_num, portMAX_DELAY);
        switch (io_num) {
            case 1:
                clearScreen(0xffff);
                drawString(k1, 50, 10, 0x0, 0xffff);
                dispLine(1);

                break;
            case 2:
                clearScreen(0xffff);
                drawString(k2, 10, 10, 0x0, 0xffff);
                dispLine(1);

                break;
            case 3:
                clearScreen(0xffff);
                drawString(k4, 10, 10, 0x0, 0xffff);
                dispLine(1);

                break;
            default:
                break;
        }
    }
}

void initScreen() {


    xTaskCreatePinnedToCore(disp_task, "disp", 4096, NULL, configMAX_PRIORITIES, &disp_task_h, 1);

}