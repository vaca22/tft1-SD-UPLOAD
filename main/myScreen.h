//
// Created by vaca on 6/21/22.
//

#ifndef TFT1_MYSCREEN_H
#define TFT1_MYSCREEN_H

#include <stdint-gcc.h>
extern xQueueHandle  disp_evt_queue;
extern uint8_t k1[];
extern uint8_t k2[];
extern uint8_t k3[];
extern uint8_t k4[];
extern uint8_t k5[];

void drawString(uint8_t* ss,int x1,int y1,uint16_t frontColor, uint16_t backColor);
void clearScreen(uint16_t color);
void drawRect(int x,int y, int w,int h,uint16_t color);
void fillRect(int x,int y, int w,int h,uint16_t color);
void dispLine(int x);
void dispAll();
void dispProgress(int k);
void initScreen();
#endif //TFT1_MYSCREEN_H
