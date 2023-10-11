#ifndef __LCD_INIT_H
#define __LCD_INIT_H

#include "main.h"                   

extern uint8_t LCDINIT;

#define USE_HORIZONTAL LCDINIT  //���ú�������������ʾ 0��1Ϊ���� 2��3Ϊ����


#if USE_HORIZONTAL == 0||USE_HORIZONTAL == 1
#define LCD_W 80
#define LCD_H 160

#else
#define LCD_W 160
#define LCD_H 80
#endif



//SCL PA05
//SDA PA04
//RES PA03
//DC  PA02
//CS  PA08
//BLK VCC ����



//-----------------LCD�˿ڶ���----------------

#define LCD_SCLK_Clr() PA05_SETLOW()//SCL=SCLK
#define LCD_SCLK_Set() PA05_SETHIGH() 

#define LCD_MOSI_Clr() PA04_SETLOW()//SDA=MOSI
#define LCD_MOSI_Set() PA04_SETHIGH() 

#define LCD_RES_Clr()  PA03_SETLOW()//RES
#define LCD_RES_Set()  PA03_SETHIGH() 

#define LCD_DC_Clr()   PA02_SETLOW()//DC
#define LCD_DC_Set()   PA02_SETHIGH() 

#define LCD_CS_Clr()   PA08_SETLOW()//CS
#define LCD_CS_Set()   PA08_SETHIGH() 

//#define LCD_BLK_Clr()  //BLK
//#define LCD_BLK_Set()  

void Lcd_Reset(void);//LCD��λ����

void LCD_Init(void);//LCD��ʼ��
void LCD_GPIO_Init(void);//��ʼ��GPIO
void LCD_Writ_Bus(uint8_t dat);//ģ��SPIʱ��
void Lcd_WriteData(uint8_t dat);//д��һ���ֽ�
void LCD_WR_DATA(uint16_t dat);//д�������ֽ�
void Lcd_WriteIndex(uint8_t dat);//д��һ��ָ��
void LCD_Address_Set(uint16_t x1,uint16_t y1,uint16_t x2,uint16_t y2);//�������꺯��
void LCD_Initt(void);
#endif




