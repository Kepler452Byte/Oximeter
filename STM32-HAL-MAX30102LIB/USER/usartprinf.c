#include "stdio.h"
#include "main.h"
#include "usart.h"

uint8_t ch;
uint8_t ch_r;

//重写这个函数,重定向printf函数到串口
/*fputc*/
int fputc(int c, FILE * f)
{
  ch=c;
  HAL_UART_Transmit(&huart1,&ch,1,1000);//发送串口
  return c;
}



//重定向scanf函数到串口 意思就是说接受串口发过来的数据
/*fgetc*/
int fgetc(FILE * F)
{
  HAL_UART_Receive (&huart1,&ch_r,1,0xffff);//接收
  return ch_r;
}


