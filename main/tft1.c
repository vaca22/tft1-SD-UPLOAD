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
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"
static const char *TAG = "HTTP_CLIENT";
int haveSD=false;
int sdFailStatus=true;
static TaskHandle_t detect_task_h;





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

typedef struct {
    uint8_t cmd;
    uint8_t data[16];
    uint8_t databytes; //No of data in data; bit 7 = delay after set; 0xFF = end of cmds.
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
    trans[1].tx_data[0] = (0)>>8;              //Start Col High
    trans[1].tx_data[1] = (0)&0XFF;              //Start Col Low
    trans[1].tx_data[2] = (240  - 1) >> 8;       //End Col High
    trans[1].tx_data[3] = (240  - 1) & 0xff;     //End Col Low
    trans[2].tx_data[0] = 0x2B;           //Page address set
    trans[3].tx_data[0] = (ypos ) >> 8;        //Start page high
    trans[3].tx_data[1] = (ypos ) & 0xff;      //start page low
    trans[3].tx_data[2] = (ypos + PARALLEL_LINES  - 1) >> 8;    //end page high
    trans[3].tx_data[3] = (ypos + PARALLEL_LINES  - 1) & 0xff;  //end page low
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


void LCD_WR_REG(u8 dat) {
    lcd_cmd(*mySpi, dat);
}

void LCD_WR_DATA(u16 dat) {
    uint8_t data[] = {dat >> 8, dat};
    lcd_data(*mySpi, data, 1);
    lcd_data(*mySpi, data + 1, 1);
}

void LCD_Address_Set(u16 x1, u16 y1, u16 x2, u16 y2) {
    LCD_WR_REG(0x2a);
    LCD_WR_DATA(x1 + 0);
    LCD_WR_DATA(x2 + 0);
    LCD_WR_REG(0x2b);
    LCD_WR_DATA(y1 + 0);
    LCD_WR_DATA(y2 + 0);
    LCD_WR_REG(0x2c);
}

void LCD_Fill(u16 xsta, u16 ysta, u16 xend, u16 yend, u16 color) {
    u16 i, j;
    LCD_Address_Set(xsta, ysta, xend - 1, yend - 1);
    for (i = ysta; i < yend; i++) {
        for (j = xsta; j < xend; j++) {
            LCD_WR_DATA(color);
        }
    }
}


uint16_t scr[14400];
const uint8_t gaga[]={
        //qing  0
        0x00,0x40,0x40,0x40,0x27,0xFC,0x20,0x40,0x03,0xF8,0x00,0x40,0xE7,0xFE,0x20,0x00,
        0x23,0xF8,0x22,0x08,0x23,0xF8,0x22,0x08,0x2B,0xF8,0x32,0x08,0x22,0x28,0x02,0x10,

        //cha  1

        0x20,0x08,0x20,0x3C,0x27,0xC0,0x20,0x40,0xF8,0x40,0x2F,0xFE,0x20,0x40,0x29,0x40,
        0x36,0x5C,0xE4,0x44,0x24,0x44,0x27,0x5C,0x24,0x44,0x24,0x44,0xA7,0xFC,0x44,0x04,

        //ru  2
        0x04,0x00,0x02,0x00,0x01,0x00,0x01,0x00,0x01,0x00,0x02,0x80,0x02,0x80,0x02,0x80,
        0x04,0x40,0x04,0x40,0x08,0x20,0x08,0x20,0x10,0x10,0x20,0x10,0x40,0x08,0x80,0x06,

        //she  3
        0x00,0x00,0x21,0xF0,0x11,0x10,0x11,0x10,0x01,0x10,0x02,0x0E,0xF4,0x00,0x13,0xF8,
        0x11,0x08,0x11,0x10,0x10,0x90,0x14,0xA0,0x18,0x40,0x10,0xA0,0x03,0x18,0x0C,0x06,

        //bei  4
        0x04,0x00,0x04,0x00,0x0F,0xF0,0x18,0x20,0x64,0x40,0x03,0x80,0x1C,0x70,0xE0,0x0E,
        0x1F,0xF0,0x11,0x10,0x11,0x10,0x1F,0xF0,0x11,0x10,0x11,0x10,0x1F,0xF0,0x10,0x10,
        //yi  5
        0x00,0x00,0x3F,0xF0,0x00,0x10,0x00,0x10,0x00,0x10,0x20,0x10,0x20,0x10,0x3F,0xF0,
        0x20,0x00,0x20,0x00,0x20,0x00,0x20,0x04,0x20,0x04,0x20,0x04,0x1F,0xFC,0x00,0x00,

        //jian  6
        0x10,0x40,0x10,0x40,0x10,0xA0,0x10,0xA0,0xFD,0x10,0x12,0x08,0x35,0xF6,0x38,0x00,
        0x54,0x88,0x50,0x48,0x92,0x48,0x11,0x50,0x11,0x10,0x10,0x20,0x17,0xFE,0x10,0x00,

        //ce   7
        0x00,0x04,0x27,0xC4,0x14,0x44,0x14,0x54,0x85,0x54,0x45,0x54,0x45,0x54,0x15,0x54,
        0x15,0x54,0x25,0x54,0xE5,0x54,0x21,0x04,0x22,0x84,0x22,0x44,0x24,0x14,0x08,0x08,
        //dao  8
        0x00,0x04,0xFF,0x84,0x08,0x04,0x10,0x24,0x22,0x24,0x41,0x24,0xFF,0xA4,0x08,0xA4,
        0x08,0x24,0x08,0x24,0x7F,0x24,0x08,0x24,0x08,0x04,0x0F,0x84,0xF8,0x14,0x40,0x08,

        //,   9
        0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
        0x00,0x00,0x00,0x00,0x00,0x00,0x30,0x00,0x30,0x00,0x10,0x00,0x20,0x00,0x00,0x00,
        //lian  10
        0x00,0x40,0x20,0x40,0x17,0xFE,0x10,0x80,0x00,0xA0,0x01,0x20,0xF3,0xFC,0x10,0x20,
        0x10,0x20,0x10,0x20,0x17,0xFE,0x10,0x20,0x10,0x20,0x28,0x20,0x47,0xFE,0x00,0x00,

        //jie  11
        0x10,0x80,0x10,0x40,0x13,0xFC,0x10,0x00,0xFD,0x08,0x10,0x90,0x17,0xFE,0x10,0x40,
        0x18,0x40,0x37,0xFE,0xD0,0x88,0x11,0x08,0x10,0x90,0x10,0x60,0x51,0x98,0x26,0x04,

        //cuo  12
        0x21,0x10,0x21,0x10,0x39,0x10,0x27,0xFC,0x41,0x10,0x79,0x10,0xAF,0xFE,0x20,0x00,
        0xFB,0xF8,0x22,0x08,0x22,0x08,0x23,0xF8,0x2A,0x08,0x32,0x08,0x23,0xF8,0x02,0x08,

        //wu  13
        0x00,0x00,0x43,0xF8,0x22,0x08,0x22,0x08,0x03,0xF8,0x00,0x00,0xE0,0x00,0x27,0xFC,
        0x20,0x40,0x20,0x40,0x2F,0xFE,0x20,0x40,0x28,0xA0,0x31,0x10,0x22,0x08,0x0C,0x06,

        //chong   14
        0x00,0x10,0x00,0xF8,0x3F,0x00,0x01,0x00,0xFF,0xFE,0x01,0x00,0x1F,0xF0,0x11,0x10,
        0x1F,0xF0,0x11,0x10,0x1F,0xF0,0x01,0x00,0x3F,0xF8,0x01,0x00,0xFF,0xFE,0x00,0x00,

        //xin   15

        0x10,0x00,0x08,0x04,0x7F,0x78,0x00,0x40,0x22,0x40,0x14,0x40,0xFF,0x7E,0x08,0x48,
        0x08,0x48,0x7F,0x48,0x08,0x48,0x2A,0x48,0x49,0x48,0x88,0x88,0x28,0x88,0x11,0x08,

        //ba   16
        0x10,0x50,0x10,0x48,0x10,0x48,0x10,0x40,0xFB,0xFE,0x10,0x80,0x14,0x80,0x18,0xFC,
        0x31,0x44,0xD1,0x44,0x11,0x28,0x11,0x28,0x12,0x10,0x12,0x28,0x54,0x44,0x21,0x82,

        //lan   17
        0x08,0x20,0x08,0x20,0xFF,0xFE,0x08,0x20,0x04,0x80,0x24,0x80,0x24,0xFC,0x24,0xA0,
        0x25,0x10,0x00,0x00,0x3F,0xF8,0x24,0x48,0x24,0x48,0x24,0x48,0xFF,0xFE,0x00,0x00,

        //ya   18
        0x00,0x00,0x3F,0xFC,0x00,0x40,0x00,0x40,0x10,0x40,0x10,0x40,0x20,0x40,0x3F,0xFE,
        0x01,0x40,0x02,0x40,0x04,0x40,0x08,0x40,0x10,0x40,0x60,0x40,0x01,0x40,0x00,0x80,
        //cheng   19
        0x00,0x50,0x00,0x48,0x00,0x40,0x3F,0xFE,0x20,0x40,0x20,0x40,0x20,0x44,0x3E,0x44,
        0x22,0x44,0x22,0x28,0x22,0x28,0x22,0x12,0x2A,0x32,0x44,0x4A,0x40,0x86,0x81,0x02,

        //gong   20
        0x00,0x40,0x00,0x40,0x00,0x40,0xFE,0x40,0x11,0xFC,0x10,0x44,0x10,0x44,0x10,0x44,
        0x10,0x44,0x10,0x84,0x10,0x84,0x1E,0x84,0xF1,0x04,0x41,0x04,0x02,0x28,0x04,0x10,

        //deng   21
        0x20,0x40,0x3F,0x7E,0x48,0x90,0x85,0x08,0x01,0x00,0x3F,0xF8,0x01,0x00,0x01,0x00,
        0xFF,0xFE,0x00,0x00,0x00,0x20,0x7F,0xFC,0x08,0x20,0x04,0x20,0x04,0xA0,0x00,0x40,

        //dai   22
        0x08,0x40,0x08,0x40,0x10,0x40,0x23,0xFC,0x48,0x40,0x08,0x40,0x17,0xFE,0x30,0x10,
        0x50,0x10,0x97,0xFE,0x10,0x10,0x12,0x10,0x11,0x10,0x11,0x10,0x10,0x50,0x10,0x20,

        //zhi  23
        0x7F,0xFC,0x44,0x44,0x7F,0xFC,0x01,0x00,0x7F,0xFC,0x01,0x00,0x1F,0xF0,0x10,0x10,
        0x1F,0xF0,0x10,0x10,0x1F,0xF0,0x10,0x10,0x1F,0xF0,0x10,0x10,0xFF,0xFE,0x00,0x00,
        //shi 24
        0x01,0x00,0x11,0x00,0x11,0x00,0x11,0x00,0x3F,0xF8,0x21,0x00,0x41,0x00,0x01,0x00,
        0xFF,0xFE,0x02,0x80,0x04,0x40,0x04,0x40,0x08,0x20,0x10,0x10,0x20,0x08,0xC0,0x06,
//bai  25
        0x00,0x40,0x7C,0x40,0x44,0x40,0x54,0x80,0x54,0xFE,0x55,0x08,0x56,0x88,0x54,0x88,
        0x54,0x88,0x54,0x50,0x54,0x50,0x10,0x20,0x28,0x50,0x24,0x88,0x45,0x04,0x82,0x02,
//chuan  26
        0x08,0x40,0x08,0x40,0x08,0x40,0x13,0xF8,0x10,0x40,0x30,0x80,0x37,0xFE,0x50,0x80,
        0x91,0x00,0x13,0xF8,0x10,0x08,0x11,0x10,0x10,0xA0,0x10,0x40,0x10,0x20,0x10,0x20,
//shu  27
        0x20,0x40,0x20,0xA0,0x21,0x10,0xFA,0x08,0x25,0xF6,0x40,0x00,0x53,0xC4,0x92,0x54,
        0xFA,0x54,0x13,0xD4,0x1A,0x54,0xF2,0x54,0x53,0xD4,0x12,0x44,0x12,0x54,0x12,0xC8,
//chu 28
        0x00,0x40,0x78,0x40,0x48,0xA0,0x51,0x10,0x52,0x08,0x65,0xF6,0x50,0x40,0x48,0x40,
        0x4F,0xFC,0x48,0x40,0x6A,0x50,0x52,0x48,0x44,0x44,0x48,0x44,0x41,0x40,0x40,0x80,
};


uint8_t k1[]={5,0,1,2,3,4};
uint8_t k2[]={12,5,6,7,8,3,4,9,0,10,11,17,18};
uint8_t k3[]={10,3,4,12,13,9,0,14,15,16,1};
uint8_t k4[]={11,17,18,10,11,19,20,9,21,22,3,23};

uint8_t k5[]={4,26,27,24,25};
uint8_t k6[]={10,26,27,19,20,9,0,16,28,3,4};


void drawChar(int x1, int y1,uint8_t *z1,uint16_t frontColor, uint16_t backColor){
    for(int k=0;k<16;k++){
        for(int j=0;j<2;j++){
            int x=2*k+j;
            int y=z1[x];
            for(int i=0;i<8;i++){
                if(y&(1<<(7-i))){
                    scr[k*240+i+j*8+x1+y1*240]=frontColor;
                }else{
                    scr[k*240+i+j*8+x1+y1*240]=backColor;
                }
            }
        }
    }
}

void drawString(uint8_t* ss,int x1,int y1,uint16_t frontColor, uint16_t backColor){
    int len=ss[0]+1;
    for(int k=1;k<len;k++){
        drawChar(x1+k*16,y1,&gaga[ss[k]*32],frontColor,backColor);
    }

}

void clearScreen(uint16_t color){
    for (int k = 0; k < 14400; k++) {
        scr[k] =color;
    }
}

void drawRect(int x,int y, int w,int h,uint16_t color){
    for(int k=0;k<w;k++){
        scr[x+k+y*LCD_W]=color;
        scr[x+k+(y+h)*LCD_W]=color;
    }
    for(int k=0;k<h;k++){
        scr[x+(y+k)*LCD_W]=color;
        scr[x+w+(y+k)*LCD_W]=color;
    }
}

void fillRect(int x,int y, int w,int h,uint16_t color){
    for(int j=0;j<w;j++){
        for(int k=0;k<h;k++){
            scr[x+j+(y+k)*LCD_W]=color;
        }
    }
}



void dispLine(int x){
    if(x>=0&&x<4){
        send_lines(*mySpi, x*PARALLEL_LINES, scr);
        send_line_finish(*mySpi);
    }
}

void dispAll(){
    for(int k=0;k<4;k++){
        dispLine(k);
    }
}
void dispProgress(int k){
    clearScreen(0xffff);
    drawRect(20,10,200,20,0x1F00);
    fillRect(20,10,k*2,20,0x1F00);
    dispLine(3);
}





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
        if (ret == ESP_FAIL) {
            ESP_LOGE(TAG, "Failed to mount filesystem. "
                          "If you want the card to be formatted, set the EXAMPLE_FORMAT_IF_MOUNT_FAILED menuconfig option.");
        } else {
            ESP_LOGE(TAG, "Failed to initialize the card (%s). "
                          "Make sure SD card lines have pull-up resistors in place.", esp_err_to_name(ret));
        }
        return 1;
    }
    ESP_LOGI(TAG, "Filesystem mounted");


    sdmmc_card_print_info(stdout, card);
    return 0;


}







static void detect1_task(void *pvParameters) {
    while (true){
        if(sdFailStatus){
            sdFailStatus=sdcard_mount();
        }
        vTaskDelay(300);
    }

}


void app_main(void) {
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

    drawString(k1,10,5,0,0xffff);
    dispLine(0);
    clearScreen(0xffff);
    drawString(k2,10,5,0,0xffff);
    dispLine(1);
    clearScreen(0xffff);
    drawString(k5,10,5,0,0xffff);
    dispLine(2);


//    for(int k=0;k<=100;k++){
//        dispProgress(k);
//        vTaskDelay(10);
//    }

//    sdcard_mount();

    xTaskCreatePinnedToCore(detect1_task, "detect", 4096, NULL, configMAX_PRIORITIES, &detect_task_h, 1);


}
