#ifndef __LCD_H
#define __LCD_H
#include "main.h"



void LCD_Fill(uint16_t xsta,uint16_t ysta,uint16_t xend,uint16_t yend,uint16_t color);//ָ�����������ɫ
void LCD_DrawPoint(uint16_t x,uint16_t y,uint16_t color);//��ָ��λ�û�һ����


void LCD_ShowChar(uint16_t x,uint16_t y,uint8_t num,uint16_t fc,uint16_t bc,uint8_t sizey,uint8_t mode);//��ʾһ���ַ�
void LCD_ShowString(uint16_t x,uint16_t y,const uint8_t *p,uint16_t fc,uint16_t bc,uint8_t sizey,uint8_t mode);//��ʾ�ַ���
uint32_t mypow(uint8_t m,uint8_t n);//����
void LCD_ShowIntNum(uint16_t x,uint16_t y,uint16_t num,uint8_t mark);//��ʾ��������
void LCD_ShowFloatNum(uint16_t x,uint16_t y,float num,uint16_t fc,uint16_t bc,uint8_t sizey);//��ʾ��λС������

void LCD_ShowBattey(uint16_t x,uint16_t y,uint8_t mode);  // ��ʾ��ؿ�
void LCD_Show_Power(uint16_t x,uint16_t y,uint8_t mode,uint16_t power);  // ��ʾ��ص���
void LCD_ShowData(uint16_t x,uint16_t y,uint16_t num,uint16_t fc,uint16_t bc,uint8_t mode); // ��ʾѪ������������
void LCD_ShowBoxing(float boxing[],uint8_t mode); // ������

void transverse_UI_init(void);  // ����UI��ʼ��
void Vertical_UI_init(void);    // ����UI��ʼ��
//������ɫ
//����RGB565��ʽ������Ҫ������ɫ���ٶ�����RGB565��ɫ��ȿ�
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
#define BROWN            0XBC40 //��ɫ
#define BRRED            0XFC07 //�غ�ɫ
#define GRAY             0X8430 //��ɫ
#define DARKBLUE         0X01CF //����ɫ
#define LIGHTBLUE        0X7D7C //ǳ��ɫ
#define GRAYBLUE         0X5458 //����ɫ
#define LIGHTGREEN       0X841F //ǳ��ɫ
#define LGRAY            0XC618 //ǳ��ɫ(PANNEL),���屳��ɫ
#define LGRAYBLUE        0XA651 //ǳ����ɫ(�м����ɫ)
#define LBBLUE           0X2B12 //ǳ����ɫ(ѡ����Ŀ�ķ�ɫ)

#endif





