#include "LCD.h"
#include "LCD_INIT.h"
#include "LCD_FONT.h"

/******************************************************************************
      ����˵������ָ�����������ɫ
      ������ݣ�xsta,ysta   ��ʼ����
                xend,yend   ��ֹ����
                                color       Ҫ������ɫ
      ����ֵ��  ��
******************************************************************************/
void LCD_Fill(uint16_t xsta,uint16_t ysta,uint16_t xend,uint16_t yend,uint16_t color)
{
    uint16_t i = ysta;
    uint16_t j = xsta;
    LCD_Address_Set(xsta,ysta,xend-1,yend-1);//������ʾ��Χ
    for(i=ysta;i<yend;i++)
    {
        for(j=xsta;j<xend;j++)
        {
            LCD_WR_DATA(color);
        }
    }
}

/******************************************************************************
      ����˵������ָ��λ�û���
      ������ݣ�x,y ��������
                color �����ɫ
      ����ֵ��  ��
******************************************************************************/
void LCD_DrawPoint(uint16_t x,uint16_t y,uint16_t color)
{
    LCD_Address_Set(x,y,x,y);//���ù��λ��
    LCD_WR_DATA(color);
}




/******************************************************************************
      ����˵������ʾ�����ַ�
      ������ݣ�x,y��ʾ����
                num Ҫ��ʾ���ַ�
                fc �ֵ���ɫ
                bc �ֵı���ɫ
                sizey �ֺ�
                mode:  0�ǵ���ģʽ  1����ģʽ
      ����ֵ��  ��
******************************************************************************/
void LCD_ShowChar(uint16_t x,uint16_t y,uint8_t num,uint16_t fc,uint16_t bc,uint8_t sizey,uint8_t mode)
{
    uint8_t temp,sizex,t,m=0;
    uint16_t i,TypefaceNum;//һ���ַ���ռ�ֽڴ�С
    uint16_t x0=x;
    sizex=sizey/2;
		if(sizey==30)sizex=19;
		else num=num-' ';    //�õ�ƫ�ƺ��ֵ
    TypefaceNum=(sizex/8+((sizex%8)?1:0))*sizey;
    LCD_Address_Set(x,y,x+sizex-1,y+sizey-1);  //���ù��λ��
    for(i=0;i<TypefaceNum;i++)
    {
        if(sizey==24)temp=ascii_2412[num][i];       //����12x24����
        else if(sizey==16)temp=ascii_1608[num][i];       //����16x32����
				else if(sizey==30)temp=int_1930[(num+1)*90+i];       //����16x32����
        else return;
        for(t=0;t<8;t++)
        {
            if(!mode)//�ǵ���ģʽ
            {
                if(temp&(0x01<<t))LCD_WR_DATA(fc);
                else LCD_WR_DATA(bc);
                m++;
                if(m%sizex==0)
                {
                    m=0;
                    break;
                }
            }
            else//����ģʽ
            {
                if(temp&(0x01<<t))LCD_DrawPoint(x,y,fc);//��һ����
                x++;
                if((x-x0)==sizex)
                {
                    x=x0;
                    y++;
                    break;
                }
            }
        }
    }
}


/******************************************************************************
      ����˵������ʾ�ַ���
      ������ݣ�x,y��ʾ����
                *p Ҫ��ʾ���ַ���
                fc �ֵ���ɫ
                bc �ֵı���ɫ
                sizey �ֺ�
                mode:  0�ǵ���ģʽ  1����ģʽ
      ����ֵ��  ��
******************************************************************************/
void LCD_ShowString(uint16_t x,uint16_t y,const uint8_t *p,uint16_t fc,uint16_t bc,uint8_t sizey,uint8_t mode)
{
    while(*p!='\0')
    {
        LCD_ShowChar(x,y,*p,fc,bc,sizey,mode);
        x+=sizey/2;
        p++;
    }
}


/******************************************************************************
      ����˵������ʾ�������õĸ�������
      ������ݣ�m������nָ��
      ����ֵ��  ��
******************************************************************************/
uint32_t mypow(uint8_t m,uint8_t n)
{
    uint32_t result=1;
    while(n--)result*=m;
    return result;
}


/******************************************************************************
      ����˵����ר����ʾѪ����������ֵ ��Ѫ������93�Զ����ɫ
      ������ݣ�x,y��ʾ����
                num Ҫ��ʾ��������
                len Ҫ��ʾ��λ��
								mark 1��ʾҪ��ʾѪ����2 ��ʾ����  �������ʵ���90������ʾ��ɫ����
      ����ֵ��  ��
******************************************************************************/
void LCD_ShowIntNum(uint16_t x,uint16_t y,uint16_t num,uint8_t mark)
{
    uint8_t t,temp,len=3;
    uint8_t enshow=0;
    uint8_t sizex,sizey;
		uint16_t fc;
		sizey=30;sizex=19;
		fc=GREEN;
		LCD_Fill(x-9,y,x+48,y+30,BLACK);
	
		//if(num==0) len++;
		if(num/10==0) x-=28;
		else if(num/10!=0 && num/100==0)x-=18;
		else x-=9;
	
		if(mark==1)   // Ѫ����ʾ��ɫ
		{
			if(num>=93 && num<96) fc=BRRED;
			else if(num<93) fc=RED;
		}
		else   // ������ʾ��ɫ
		{
			if(num>=105 || num<54) fc=RED;
			else if(num<105 && num>=90) fc=BRRED;
		}
		
    for(t=0;t<len;t++)
    {
        temp=(num/mypow(10,len-t-1))%10;
        if(enshow==0&&t<(len-1))
        {
            if(temp==0)
            {
                //LCD_ShowChar(x+t*sizex,y,' ',fc,BLACK,16,0);
                continue;
            }else enshow=1;

        }
				LCD_ShowChar(x+t*sizex,y,temp,fc,BLACK,sizey,0);
    }
}


/******************************************************************************
      ����˵������ʾ��λС������
      ������ݣ�x,y��ʾ����
                num Ҫ��ʾС������
                len Ҫ��ʾ��λ��
                fc �ֵ���ɫ
                bc �ֵı���ɫ
                sizey �ֺ�
      ����ֵ��  ��
******************************************************************************/
void LCD_ShowFloatNum(uint16_t x,uint16_t y,float num,uint16_t fc,uint16_t bc,uint8_t sizey)
{
		
		uint8_t t,temp,sizex,len=2;
    uint16_t num1,integer;
		LCD_Fill(x,y,x+40,y+16,BLACK);
    sizex=sizey/2;
		num1=num*100;
		integer=num1/100;
		if(integer==0)len+=1;
		while(integer>0)
		{
			integer=integer/10;
			len++;
		}
		for(t=0;t<len;t++)
		{
			temp=(num1/mypow(10,len-t-1))%10;
			if(t==(len-2))
			{
				LCD_ShowChar(x+(len-2)*sizex,y,'.',WHITE,BLACK,sizey,0);
				t++;
				len+=1;
			}
			LCD_ShowChar(x+t*sizex,y,temp+48,WHITE,BLACK,sizey,0);
		}
}




/******************************************************************************
      ����˵������ʾ���ͼ�� ��ʣ�����
      ������ݣ�x,y�������
								fc ͼ����ɫ
								fb ͼ�걳��ɫ
								mode 1������ 2������
								power ���ʣ���������67 55
      ����ֵ��  ��
******************************************************************************/
void LCD_ShowBattey(uint16_t x,uint16_t y,uint8_t mode)
{
		uint8_t temp,width,length,t;
		uint16_t const_x=x;
		width=56;length=24;

			uint16_t i,TypefaceNum;//һ���ַ���ռ�ֽڴ�С
			uint16_t x0=x;
			TypefaceNum=(width/8+((width%8)?1:0))*length;
			//num=num-' ';    //�õ�ƫ�ƺ��ֵ
			LCD_Address_Set(x,y,x+width-1,y+length-1);  //���ù��λ��
			for(i=0;i<TypefaceNum;i++)
			{
					temp=battery[i];
					for(t=0;t<8;t++)
					{
						if(temp&(0x01<<t))
						{
							if(mode ==2) LCD_DrawPoint(y,((2*const_x)-x),YELLOW);//��һ����
							else LCD_DrawPoint(x,y,YELLOW);
						}
						x++;
						if((x-x0)==width)
						{
								x=x0;
								y++;
								break;
						}
					 }
			}
}


/******************************************************************************
      ����˵������ʾ���ʣ�����
      ������ݣ�x,y�������
								mode 1������ 2������
								power ���ʣ���������67 55
      ����ֵ��  ��
******************************************************************************/
void LCD_Show_Power(uint16_t x,uint16_t y,uint8_t mode,uint16_t power)
{
	uint16_t power_x_start,power_y_start,power_x_end,power_y_end;
	
	uint16_t fc=GREEN;
	if(power<=20) fc=RED;  // �͵����Զ���ɫ
	
	if(mode ==2)
	{
		power_x_start=y+6;power_x_end=y+19;power_y_start=x-10-((power*40)/100);power_y_end=x-10;
		if(power<=100 && power>0)
		{
			LCD_Fill(power_x_start-1,x-46,power_x_end,power_y_start,BLACK);
			LCD_Fill(power_x_start-1,power_y_start,power_x_end,power_y_end+1,fc);
		}
	}
	else
		{
			power_x_start=x+11;power_x_end=x+11+(power*40)/100;power_y_start=y+5;power_y_end=y+19;
			if(power<=100 && power>0)
			{
				LCD_Fill(power_x_end,power_y_start,x+46,power_y_end+1,BLACK);
				LCD_Fill(power_x_start-1,power_y_start,power_x_end,power_y_end+1,fc);
			}
		}
}

/******************************************************************************
      ����˵������ʾ��������
      ������ݣ�boxing[] ���η��ȱ�������Ҫ160����ֵ��������80��������ֵ��Χ��0-3.3����ADCת�����
								mode 1������ 2������
      ����ֵ��  ��
******************************************************************************/
void LCD_ShowBoxing(float boxing[],uint8_t mode)
{
	uint16_t i,width,length,hight;

	if(mode==1) {width=160;length=80;}
	else {width=80;length=160;}
	for(i=0;i<width;i++)
	{
		hight=boxing[i]*23/3.3;
		LCD_Fill(i,length-hight,i+2,length+2,BLACK);
		LCD_Fill(i,length-hight,i+2,length+2,GREEN);
	}
}


//  ���� UI ��ʼ��
void transverse_UI_init()
{

	LCD_Init();
	LCD_Fill(0,0,160,80,BLACK);
	LCD_ShowString(0,0,(const unsigned char*)"%Sp0",YELLOW,BLACK,24,1);
	LCD_ShowString(48,8,(const unsigned char*)"2",YELLOW,BLACK,16,1);
	LCD_ShowString(112,0,(const unsigned char*)"PR",YELLOW,BLACK,24,1);
	LCD_ShowString(136,8,(const unsigned char*)"bpm",YELLOW,BLACK,16,1);
	LCD_ShowBattey(56,0,1);
	LCD_ShowString(63,24,(const unsigned char*)"PI %:",WHITE,BLACK,16,1);
	LCD_Fill(12,40,24,46,GREEN);LCD_Fill(30,40,42,46,GREEN);
	LCD_Fill(119,40,132,46,GREEN);LCD_Fill(138,40,151,46,GREEN);
}


//  ���� UI ��ʼ��
void Vertical_UI_init()
{
	LCD_Init();
	LCD_Fill(0,0,80,160,BLACK);
	LCD_ShowString(0,0,(const unsigned char*)"Sp0",YELLOW,BLACK,24,1);
	LCD_ShowString(36,8,(const unsigned char*)"2",YELLOW,BLACK,16,1);
	LCD_ShowString(40,0,(const unsigned char*)"  %",YELLOW,BLACK,24,1);
	LCD_ShowString(0,54,(const unsigned char*)"PR ",YELLOW,BLACK,24,1);
	LCD_ShowString(24,62,(const unsigned char*)"bpm",YELLOW,BLACK,16,1);
	LCD_ShowBattey(113,58,2);
	LCD_ShowString(0,114,(const unsigned char*)"PI %:",WHITE,BLACK,16,1);
	LCD_Fill(12,36,26,42,GREEN);LCD_Fill(30,36,44,42,GREEN);
	LCD_Fill(12,90,26,96,GREEN);LCD_Fill(30,90,44,96,GREEN);
}
