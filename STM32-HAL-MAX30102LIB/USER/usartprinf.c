#include "stdio.h"
#include "main.h"
#include "usart.h"

uint8_t ch;
uint8_t ch_r;

//��д�������,�ض���printf����������
/*fputc*/
int fputc(int c, FILE * f)
{
  ch=c;
  HAL_UART_Transmit(&huart1,&ch,1,1000);//���ʹ���
  return c;
}



//�ض���scanf���������� ��˼����˵���ܴ��ڷ�����������
/*fgetc*/
int fgetc(FILE * F)
{
  HAL_UART_Receive (&huart1,&ch_r,1,0xffff);//����
  return ch_r;
}


