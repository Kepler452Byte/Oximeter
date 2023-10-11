#include "LCD_INIT.h"

/******************************************************************************
      函数说明：LCD复位函数
      入口数据：无
      返回值：  无
******************************************************************************/
void Lcd_Reset(void)
{
	LCD_RES_Clr();
	FirmwareDelay(100);
	LCD_RES_Set();
	FirmwareDelay(100);
}


/******************************************************************************
      函数说明：LCD_GPIO初始化函数
      入口数据：dat  要写入的串行数据
      返回值：  无
******************************************************************************/
void LCD_GPIO_Init(void)
{	
	GPIO_InitTypeDef GPIO_InitStruct;
	
	__RCC_GPIOA_CLK_ENABLE();

	GPIO_InitStruct.IT = GPIO_IT_NONE; 
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	
	GPIO_InitStruct.Pins = GPIO_PIN_2| GPIO_PIN_3| GPIO_PIN_4| GPIO_PIN_5|GPIO_PIN_8;
	GPIO_Init(CW_GPIOA, &GPIO_InitStruct);
}


/******************************************************************************
      函数说明：LCD串行数据写入函数
      入口数据：dat  要写入的串行数据
      返回值：  无
******************************************************************************/
void LCD_Writ_Bus(uint8_t dat)
{
    uint8_t i;
    LCD_CS_Clr();
    for(i=0;i<8;i++)
    {
        LCD_SCLK_Clr();
        if(dat&0x80)
        {
           LCD_MOSI_Set();
        }
        else
        {
           LCD_MOSI_Clr();
        }
        LCD_SCLK_Set();
        dat<<=1;
    }
  LCD_CS_Set();
}


/******************************************************************************
      函数说明：LCD写入8位数据
      入口数据：dat 写入的数据
      返回值：  无
******************************************************************************/
void Lcd_WriteData(uint8_t dat)
{
    LCD_Writ_Bus(dat);
}


/******************************************************************************
      函数说明：LCD写入16位数据
      入口数据：dat 写入的数据
      返回值：  无
******************************************************************************/
void LCD_WR_DATA(uint16_t dat)
{
    LCD_Writ_Bus(dat>>8);
    LCD_Writ_Bus(dat);
}


/******************************************************************************
      函数说明：LCD写入命令
      入口数据：dat 写入的命令
      返回值：  无
******************************************************************************/
void Lcd_WriteIndex(uint8_t dat)
{
    LCD_DC_Clr();//写命令
    LCD_Writ_Bus(dat);
    LCD_DC_Set();//写数据
}


/******************************************************************************
      函数说明：设置起始和结束地址
      入口数据：x1,x2 设置列的起始和结束地址
                y1,y2 设置行的起始和结束地址
      返回值：  无
******************************************************************************/
void LCD_Address_Set(uint16_t x1,uint16_t y1,uint16_t x2,uint16_t y2)
{
    if(USE_HORIZONTAL==0)
    {
        Lcd_WriteIndex(0x2a);//列地址设置
        LCD_WR_DATA(x1+26);
        LCD_WR_DATA(x2+26);
        Lcd_WriteIndex(0x2b);//行地址设置
        LCD_WR_DATA(y1+1);
        LCD_WR_DATA(y2+1);
        Lcd_WriteIndex(0x2c);//储存器写
    }
    else if(USE_HORIZONTAL==1)
    {
        Lcd_WriteIndex(0x2a);//列地址设置
        LCD_WR_DATA(x1+26);
        LCD_WR_DATA(x2+26);
        Lcd_WriteIndex(0x2b);//行地址设置
        LCD_WR_DATA(y1+1);
        LCD_WR_DATA(y2+1);
        Lcd_WriteIndex(0x2c);//储存器写
    }
    else if(USE_HORIZONTAL==2)
    {
        Lcd_WriteIndex(0x2a);//列地址设置
        LCD_WR_DATA(x1+1);
        LCD_WR_DATA(x2+1);
        Lcd_WriteIndex(0x2b);//行地址设置
        LCD_WR_DATA(y1+26);
        LCD_WR_DATA(y2+26);
        Lcd_WriteIndex(0x2c);//储存器写
    }
    else
    {
        Lcd_WriteIndex(0x2a);//列地址设置
        LCD_WR_DATA(x1+1);
        LCD_WR_DATA(x2+1);
        Lcd_WriteIndex(0x2b);//行地址设置
        LCD_WR_DATA(y1+26);
        LCD_WR_DATA(y2+26);
        Lcd_WriteIndex(0x2c);//储存器写
    }
}

/******************************************************************************
      函数说明：LCD初始化代码
      入口数据：无
      返回值：  无
******************************************************************************/



void LCD_Init(void)
{
	LCD_GPIO_Init();//初始化GPIO
	
	LCD_RES_Clr();//复位
	FirmwareDelay(1);
	LCD_RES_Set();
	//FirmwareDelay(1);
	
	//LCD_BLK_Set();//打开背光
 // FirmwareDelay(1);
	
	Lcd_WriteIndex(0x11);     //Sleep out
	//FirmwareDelay(1);                //Delay 120ms
	Lcd_WriteIndex(0xB1);     //Normal mode
	Lcd_WriteData(0x05);   
	Lcd_WriteData(0x3C);   
	Lcd_WriteData(0x3C);   
	Lcd_WriteIndex(0xB2);     //Idle mode
	Lcd_WriteData(0x05);   
	Lcd_WriteData(0x3C);   
	Lcd_WriteData(0x3C);   
	Lcd_WriteIndex(0xB3);     //Partial mode
	Lcd_WriteData(0x05);   
	Lcd_WriteData(0x3C);   
	Lcd_WriteData(0x3C);   
	Lcd_WriteData(0x05);   
	Lcd_WriteData(0x3C);   
	Lcd_WriteData(0x3C);   
	Lcd_WriteIndex(0xB4);     //Dot inversion
	Lcd_WriteData(0x03);   
	Lcd_WriteIndex(0xC0);     //AVDD GVDD
	Lcd_WriteData(0xAB);   
	Lcd_WriteData(0x0B);   
	Lcd_WriteData(0x04);   
	Lcd_WriteIndex(0xC1);     //VGH VGL
	Lcd_WriteData(0xC5);   //C0
	Lcd_WriteIndex(0xC2);     //Normal Mode
	Lcd_WriteData(0x0D);   
	Lcd_WriteData(0x00);   
	Lcd_WriteIndex(0xC3);     //Idle
	Lcd_WriteData(0x8D);   
	Lcd_WriteData(0x6A);   
	Lcd_WriteIndex(0xC4);     //Partial+Full
	Lcd_WriteData(0x8D);   
	Lcd_WriteData(0xEE);   
	Lcd_WriteIndex(0xC5);     //VCOM
	Lcd_WriteData(0x0F);   
	Lcd_WriteIndex(0xE0);     //positive gamma
	Lcd_WriteData(0x07);   
	Lcd_WriteData(0x0E);   
	Lcd_WriteData(0x08);   
	Lcd_WriteData(0x07);   
	Lcd_WriteData(0x10);   
	Lcd_WriteData(0x07);   
	Lcd_WriteData(0x02);   
	Lcd_WriteData(0x07);   
	Lcd_WriteData(0x09);   
	Lcd_WriteData(0x0F);   
	Lcd_WriteData(0x25);   
	Lcd_WriteData(0x36);   
	Lcd_WriteData(0x00);   
	Lcd_WriteData(0x08);   
	Lcd_WriteData(0x04);   
	Lcd_WriteData(0x10);   
	Lcd_WriteIndex(0xE1);     //negative gamma
	Lcd_WriteData(0x0A);   
	Lcd_WriteData(0x0D);   
	Lcd_WriteData(0x08);   
	Lcd_WriteData(0x07);   
	Lcd_WriteData(0x0F);   
	Lcd_WriteData(0x07);   
	Lcd_WriteData(0x02);   
	Lcd_WriteData(0x07);   
	Lcd_WriteData(0x09);   
	Lcd_WriteData(0x0F);   
	Lcd_WriteData(0x25);   
	Lcd_WriteData(0x35);   
	Lcd_WriteData(0x00);   
	Lcd_WriteData(0x09);   
	Lcd_WriteData(0x04);   
	Lcd_WriteData(0x10);
		 
	Lcd_WriteIndex(0xFC);    
	Lcd_WriteData(0x80);  
		
	Lcd_WriteIndex(0x3A);     
	Lcd_WriteData(0x05);   
	Lcd_WriteIndex(0x36);
	if(USE_HORIZONTAL==0)Lcd_WriteData(0x08);
	else if(USE_HORIZONTAL==1)Lcd_WriteData(0xC8);
	else if(USE_HORIZONTAL==2)Lcd_WriteData(0x78);
	else Lcd_WriteData(0xA8);   
	Lcd_WriteIndex(0x21);     //Display inversion
	Lcd_WriteIndex(0x29);     //Display on
	Lcd_WriteIndex(0x2A);     //Set Column Address
	Lcd_WriteData(0x00);   
	Lcd_WriteData(0x1A);  //26  
	Lcd_WriteData(0x00);   
	Lcd_WriteData(0x69);   //105 
	Lcd_WriteIndex(0x2B);     //Set Page Address
	Lcd_WriteData(0x00);   
	Lcd_WriteData(0x01);    //1
	Lcd_WriteData(0x00);   
	Lcd_WriteData(0xA0);    //160
	Lcd_WriteIndex(0x2C); 
}
