#ifndef __LCD_H
#define __LCD_H
#include "main.h"



void LCD_Fill(uint16_t xsta,uint16_t ysta,uint16_t xend,uint16_t yend,uint16_t color);//指定区域填充颜色
void LCD_DrawPoint(uint16_t x,uint16_t y,uint16_t color);//在指定位置画一个点


void LCD_ShowChar(uint16_t x,uint16_t y,uint8_t num,uint16_t fc,uint16_t bc,uint8_t sizey,uint8_t mode);//显示一个字符
void LCD_ShowString(uint16_t x,uint16_t y,const uint8_t *p,uint16_t fc,uint16_t bc,uint8_t sizey,uint8_t mode);//显示字符串
uint32_t mypow(uint8_t m,uint8_t n);//求幂
void LCD_ShowIntNum(uint16_t x,uint16_t y,uint16_t num,uint8_t mark);//显示整数变量
void LCD_ShowFloatNum(uint16_t x,uint16_t y,float num,uint16_t fc,uint16_t bc,uint8_t sizey);//显示两位小数变量

void LCD_ShowBattey(uint16_t x,uint16_t y,uint8_t mode);  // 显示电池框
void LCD_Show_Power(uint16_t x,uint16_t y,uint8_t mode,uint16_t power);  // 显示电池电量
void LCD_ShowData(uint16_t x,uint16_t y,uint16_t num,uint16_t fc,uint16_t bc,uint8_t mode); // 显示血氧，心率数据
void LCD_ShowBoxing(float boxing[],uint8_t mode); // 画波形

void transverse_UI_init(void);  // 横屏UI初始化
void Vertical_UI_init(void);    // 竖屏UI初始化
//画笔颜色
//采用RGB565格式，如需要增加颜色，百度搜索RGB565颜色表既可
#define WHITE            0xFFFF
#define BLACK            0x0000
#define BLUE             0x001F
#define BRED             0XF81F
#define GRED             0XFFE0
#define GBLUE            0X07FF
#define RED              0xF800
#define MAGENTA          0xF81F
#define GREEN            0x07E0
#define CYAN             0x7FFF
#define YELLOW           0xFFE0
#define BROWN            0XBC40 //棕色
#define BRRED            0XFC07 //棕红色
#define GRAY             0X8430 //灰色
#define DARKBLUE         0X01CF //深蓝色
#define LIGHTBLUE        0X7D7C //浅蓝色
#define GRAYBLUE         0X5458 //灰蓝色
#define LIGHTGREEN       0X841F //浅绿色
#define LGRAY            0XC618 //浅灰色(PANNEL),窗体背景色
#define LGRAYBLUE        0XA651 //浅灰蓝色(中间层颜色)
#define LBBLUE           0X2B12 //浅棕蓝色(选择条目的反色)

#endif





