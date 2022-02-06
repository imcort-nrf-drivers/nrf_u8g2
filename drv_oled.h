#ifndef __DRV_OLED_H
#define __DRV_OLED_H

#include "u8g2.h"
#include "u8x8.h"

//New PCB of NFC card
//#define OLED_PIN_CS  		3//2
//#define OLED_PIN_VDISP  2//3  
//#define OLED_PIN_RESET  4
//#define OLED_PIN_DC     5
//#define OLED_PIN_SCL    6
//#define OLED_PIN_SDA    7

//EINK
//#define OLED_PIN_CS  		15
//#define OLED_PIN_VDISP  28  
//#define OLED_PIN_RESET  6
//#define OLED_PIN_DC     8
//#define OLED_PIN_SCL    16
//#define OLED_PIN_SDA    18



void drv_oled_begin(void);
void drv_oled_on(void);
void drv_oled_off(void);

#endif
