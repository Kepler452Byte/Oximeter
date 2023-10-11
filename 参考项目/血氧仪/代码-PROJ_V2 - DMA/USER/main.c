#include "main.h"												//����ͷ�ļ�����main.h
#include "LCD.h"
#include "LCD_INIT.h"
#include "pic.h"
#include "FFT.h"
#include "math.h"

//uint16_t *adcvalue=0;    								        //����ADC��ֵ��ָ�룩
//uint16_t timecount=0;										    //��ʱ��
uint8_t SEND_status = 0; 									    //���������״̬����4��״̬����ʼΪ״̬0��
//uint8_t SEND_state = 0;										//�������ͣ״̬��0Ϊֹͣ��1Ϊ����
uint32_t BTIM1_counter1 = 0;							        //BTIM1������1�����ڿ��ư���ʱ��
//uint32_t BTIM1_counter2 = 39;							        //BTIM1������2�����ڿ��Ʒ��������״̬
uint32_t BTIM1_counter3 = 3;							        //BTIM1������3�����ڿ��Ʒ������ͣʱ��	
uint8_t  KEY_state = 0;								            //����״̬(0 = δ���£�1 = ���£�
uint16_t KEY_cnt = 0;                                           //����ʱ���ۼ���
uint8_t  sys_status = 0;								        //ϵͳ����״̬��0 = ������1 = �ػ���
uint8_t  ShowDirect_status = 0;                                 //��Ļ��ʾ����0 = ������1 = ������
//uint32_t  LCD_BKL = 0;										//LCD��������
uint16_t POWER = 80;                                            //�����ص�������ʼֵ=80��
uint16_t DAC1_PWM = 0;										    //����ܵ�������DAC1
uint16_t DAC2_PWM = 0;										    //����ܵ�������DAC2
uint16_t DAC_PWM_PLUS = 300;								    //����ܵ�����ǿֵ(���Ϊ699��
uint8_t  IRorRED = 0;											//�洢��ǰ������ź����ͱ�־��0��ʾ���⣬1��ʾ���
uint32_t DC_ADC_Result[256] = {0};				                //�洢DMA������ֱ������
uint32_t AC_ADC_Result[256] = {0};				                //�洢DMA��������������
uint8_t PULSE_number = 0;									    //�洢�����ź�����
uint8_t PULSE_No[128] = {0};							        //�洢�����ź�λ��
uint16_t PULSE_R[128] = {0};							        //�洢�����źŲ������
float PULSE_Result = 0.0;									    //�洢����������
extern signed short Fft_Real[128]; 				                //fftʵ����128����
extern signed short Fft_Image[128]; 			                //fft�鲿��128����
float DC_IR,AC_IR,DC_RED,AC_RED;					            //�洢����ͺ���ֱ������������
float  PI_IR,PI_RED,R,SPO2;								        //�洢����ͺ���PIֵ��Rֵ��SPO2ֵ
uint8_t R_Calt = 0;												//�洢Calt��������ֵ״̬
uint8_t IsCycleEnd = 0;										    //128�β������ڽ�����־��1Ϊ�����У�0Ϊ��������
uint8_t LCDINIT = 2;
// ģ���������ݣ�ģ��ADCת���������λС������Χ(0-3.3)
float boxing_moni[160]= {1.40,0.58,3.04,2.43,1.91,2.65,1.24,0.51,3.02,0.72,2.05,1.52,0.34,1.28,2.71,2.30,0.21,3.22,1.06,0.64,0.37,3.15,2.26,3.14,3.01,2.63,0.24,0.99,0.91,2.16,0.69,0.46,2.87,0.68,1.95,2.84,2.91,1.47,3.21,0.79,1.65,2.07,2.10,0.83,0.59,1.92,0.57,0.19,2.04,1.76,0.39,3.19,1.50,2.85,3.26,2.41,2.15,2.48,3.25,0.38,1.77,1.04,2.72,2.59,0.03,1.78,2.12,1.58,0.17,2.47,1.13,0.73,0.60,1.97,1.03,0.12,0.08,2.55,3.28,0.35,0.44,2.37,1.05,1.66,2.57,2.46,0.92,2.22,0.54,1.22,3.16,1.53,0.27,0.01,2.28,0.81,2.03,0.47,2.25,0.07,3.08,0.88,0.45,1.72,2.74,2.88,1.45,2.08,0.61,0.77,3.13,3.11,2.99,1.90,1.62,0.00,2.33,2.64,1.43,0.63,0.41,1.71,0.32,2.49,1.67,1.86,1.56,0.02,1.09,2.98,0.49,1.99,0.33,1.38,3.07,2.62,2.61,0.75,1.55,2.21,3.12,0.65,2.67,1.80,0.98,2.60,3.18,2.31,1.54,3.06,2.18,1.36,0.05,2.92,0.84,0.96,2.78,2.52,0.76,0.43};	;
		
int32_t main(void)
{
	SYS_init();													        //ϵͳ��ʼ��
    if(ShowDirect_status == 0)                                          //����
    {
        LCDINIT = 2;
        transverse_UI_init();                                           //������ʾ��ʼ��
    }
    else                                                                //����        
    {
        LCDINIT = 0;
        Vertical_UI_init();                                             //������ʾ��ʼ��
    }
    LCD_show(ShowDirect_status);                                        //��ֵ��ʾ
	FirmwareDelay(400000);                                              //��ʱ
    //GPIO_WritePin(bsp_LED01_port, bsp_LED01_pin, GPIO_Pin_RESET);		//����Ĭ�ϴ�LED01�������ã�	
//	printf("**************��������**************\r\n");	                //�������ã�
	IsCycleEnd = 1;

	while(1)
	{
//		LCD_ShowBoxing(boxing_moni,2);  			                    // ��������
//		LCD_ShowBoxing(boxing_moni,1);    	   	                        // ��������		
//		printf("R_Calt:%d\r\n",R_Calt);
		if((1 == IsCycleEnd)	&& (0 == (CW_BTIM1->BCR & 0x0001)))	    // �������ڿ�ʼ��BTIM1��״̬Ϊ��ʹ��
		{
			SEND_status = 0; 																														//���������״̬����4��״̬����ʼΪ״̬0��
//			SEND_state = 0;																															//�������ͣ״̬��0Ϊֹͣ��1Ϊ����
//			BTIM1_counter1 = 27;																												//BTIM1������1�����ڿ��Ʒ����ֹͣ״̬
//			BTIM1_counter2 = 39;																												//BTIM1������2�����ڿ��Ʒ��������״̬
			BTIM1_counter3 = 3;																													//BTIM1������3�����ڿ��Ʒ������ͣʱ��	
//			ADC_Enable();																																//ADCʹ��
			BTIM_Cmd(CW_BTIM1, ENABLE);									//BTIM1ʹ��
		}
		else if((0 == IsCycleEnd)	&& (1 == (CW_BTIM1->BCR & 0x0001)))					// �������ڽ�����BTIM1��״̬Ϊʹ��
		{
			BTIM_Cmd(CW_BTIM1, DISABLE);								//BTIM1��ʹ��
			Calt();
		}

		if(R_Calt)
		{
			R_Calt = 0;
            LCD_show(ShowDirect_status);                                //��ֵ��ʾ
		}

        if(sys_status == 1)                                             //ϵͳ����״̬Ϊ1���ػ�
        {
            GPIO_WritePin(bsp_PWM_port, bsp_PWM_pin, GPIO_Pin_SET);
        }
        else                                                            //���򣬿���
        {
            GPIO_WritePin(bsp_PWM_port, bsp_PWM_pin, GPIO_Pin_RESET);
        }

	}
}

void SYS_init(void)
{
	RCC_init();														    //ʱ�ӳ�ʼ��
    GPIO_init();													    //GPIO��ʼ��
	BTIM_init();           								                //������ʱ����ʼ�� 
	GTIM_init();													    //ͨ�ö�ʱ����ʼ��
    NVIC_init();													    //�ж�������ʼ��
	transverse_UI_init();									            //����UI ��ʼ��
	ADC_Configuration();   								                //ADC��ʼ��
	UART2_init();													    //���ڳ�ʼ��
	DMA_Configration();										            //DMA��ʼ��
    GPIO_WritePin(bsp_PWM_port, bsp_PWM_pin, GPIO_Pin_RESET);		    //����Ĭ�ϴ�LCD���͵�ƽ��
}


void RCC_init(void)
{
	//����ϵͳʱ��
	__RCC_FLASH_CLK_ENABLE();							                
	FLASH_SetLatency(FLASH_Latency_1);                                  //< ��ʹ�õ�ʱ��ԴHCLKС��24M������FLASH ���ȴ�����Ϊ1 cycle    
	RCC_HSI_Enable(RCC_HSIOSC_DIV2);				                    // 0. HSIʹ�ܲ�У׼ 
	RCC_HCLKPRS_Config(RCC_HCLK_DIV1);			                        // 1. ����HCLK�ķ�Ƶϵ����
	RCC_PCLKPRS_Config(RCC_PCLK_DIV1);			                        // 2. ����PCLK�ķ�Ƶϵ��			
	RCC_SystemCoreClockUpdate(24000000);		                        // 3. ����ϵͳ��ƵΪ24MHz
	//����GPIOʱ��
	__RCC_GPIOA_CLK_ENABLE();	
	__RCC_GPIOB_CLK_ENABLE();	
	__RCC_GPIOF_CLK_ENABLE();	
	//������ʱ��ʱ��
	__RCC_BTIM_CLK_ENABLE();
//	__RCC_GTIM1_CLK_ENABLE();
	__RCC_GTIM2_CLK_ENABLE();
	//����ADCʱ��
	__RCC_ADC_CLK_ENABLE();	
	//����DMAʱ��
	__RCC_DMA_CLK_ENABLE();
	//��������ʱ��
	__RCC_UART2_CLK_ENABLE();
}
	

void GPIO_init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct;
    //��ʼ��LED_GPIO
    GPIO_InitStruct.IT = GPIO_IT_NONE; 
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pins = bsp_LED01_pin;
    GPIO_Init(bsp_LED01_port, &GPIO_InitStruct);
    GPIO_InitStruct.IT = GPIO_IT_NONE; 
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pins = bsp_LED02_pin;
    GPIO_Init(bsp_LED02_port, &GPIO_InitStruct);
    //��ʼ��KEY_GPIO
    GPIO_InitStruct.IT = GPIO_IT_FALLING; 
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pins = bsp_KEY01_pin;
    GPIO_Init(bsp_KEY01_port, &GPIO_InitStruct);
	GPIOB_INTFLAG_CLR(bv3);
	//��ʼ��BEEP_GPIO
//	PA07_AFx_GTIM1CH2();
//	PA07_DIGTAL_ENABLE();
//	PA07_DIR_OUTPUT();
//	PA07_PUSHPULL_ENABLE();
	//��ʼ��LCD_GPIO
	GPIO_InitStruct.IT = GPIO_IT_NONE; 
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pins = GPIO_PIN_2| GPIO_PIN_3| GPIO_PIN_4| GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_8;
	GPIO_Init(CW_GPIOA, &GPIO_InitStruct);
//	PA06_AFx_GTIM1CH1();
//	PA06_DIGTAL_ENABLE();
//	PA06_DIR_OUTPUT();
//	PA06_PUSHPULL_ENABLE();
	//��ʼ��SEND_GPIO
	GPIO_InitStruct.IT = GPIO_IT_NONE; 
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pins = bsp_IN1_pin;
    GPIO_Init(bsp_IN1_port, &GPIO_InitStruct);	
	GPIO_InitStruct.IT = GPIO_IT_NONE; 
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pins = bsp_IN2_pin;
    GPIO_Init(bsp_IN2_port, &GPIO_InitStruct);	
	PB14_AFx_GTIM2CH1();
	PB14_DIGTAL_ENABLE();
	PB14_DIR_OUTPUT();
	PB14_PUSHPULL_ENABLE();
	PB15_AFx_GTIM2CH2();
	PB15_DIGTAL_ENABLE();
	PB15_DIR_OUTPUT();
	PB15_PUSHPULL_ENABLE();	
	//��ʼ��ADC_GPIO
	PA00_ANALOG_ENABLE();
	PA01_ANALOG_ENABLE();
	//��ʼ������
	PB00_AFx_UART2RXD();
	PB00_DIGTAL_ENABLE();
	PB00_DIR_INPUT();
	PB00_PUR_ENABLE();
	PB01_AFx_UART2TXD();
	PB01_DIGTAL_ENABLE();
	PB01_DIR_OUTPUT();
	PB01_PUSHPULL_ENABLE();
}

void NVIC_init(void)
{
	__disable_irq(); 
	//�����ж����ȼ�
	NVIC_SetPriority(BTIM1_IRQn, 0);
//	NVIC_SetPriority(GTIM1_IRQn, 0);
//	NVIC_SetPriority(GTIM2_IRQn, 0);
	NVIC_SetPriority(GPIOA_IRQn, 3);
	NVIC_SetPriority(GPIOB_IRQn, 0);
//	NVIC_SetPriority(ADC_IRQn, 0);
	NVIC_SetPriority(DMACH1_IRQn, 1);
	NVIC_SetPriority(DMACH23_IRQn, 1);
	//�����ж�ʹ��
    NVIC_EnableIRQ(BTIM1_IRQn); 
//  NVIC_EnableIRQ(GTIM1_IRQn); 
//  NVIC_EnableIRQ(GTIM2_IRQn); 
	NVIC_EnableIRQ(GPIOA_IRQn);
	NVIC_EnableIRQ(GPIOB_IRQn);
//	NVIC_EnableIRQ(ADC_IRQn);
	NVIC_EnableIRQ(DMACH1_IRQn);
	NVIC_EnableIRQ(DMACH23_IRQn);
  __enable_irq();
}

void BTIM_init(void)
{
	BTIM_TimeBaseInitTypeDef BTIM_InitStruct;
	
	BTIM_InitStruct.BTIM_Mode = BTIM_Mode_TIMER;
    BTIM_InitStruct.BTIM_OPMode = BTIM_OPMode_Repetitive;
    BTIM_InitStruct.BTIM_Period = 999;
    BTIM_InitStruct.BTIM_Prescaler = 23;  															//T = ((PSC+1)/PCLK)��(ARR+1)
    BTIM_TimeBaseInit(CW_BTIM1, &BTIM_InitStruct);
	BTIM_ClearITPendingBit(CW_BTIM1, BTIM_IT_OV);
    BTIM_ITConfig(CW_BTIM1, BTIM_IT_OV, ENABLE);
//  BTIM_Cmd(CW_BTIM1, ENABLE);
}

void GTIM_init(void)
{
	GTIM_InitTypeDef GTIM_InitStruct;
//	//GTIM1		���ڿ��Ʒ���������
//	GTIM_InitStruct.Mode = GTIM_MODE_TIME;
//  GTIM_InitStruct.OneShotMode = GTIM_COUNT_CONTINUE;
//  GTIM_InitStruct.ReloadValue = 487;																//����Ϊ0.5ms��Ƶ��Ϊ2KHz
//  GTIM_InitStruct.Prescaler = 23;  											
//	GTIM_InitStruct.ToggleOutState = DISABLE;
//  GTIM_TimeBaseInit(CW_GTIM1, &GTIM_InitStruct);

////	GTIM_OCInit(CW_GTIM1, GTIM_CHANNEL1, GTIM_OC_OUTPUT_PWM_HIGH);		                        //����ͨ��1��OCģʽΪ�������
//	GTIM_OCInit(CW_GTIM1, GTIM_CHANNEL2, GTIM_OC_OUTPUT_PWM_HIGH);		                            //����ͨ��2��OCģʽΪ�������
////	GTIM_OCInit(CW_GTIM1, GTIM_CHANNEL3, GTIM_OC_OUTPUT_PWM_HIGH);		                        //����ͨ��3��OCģʽΪ�������
////	GTIM_OCInit(CW_GTIM1, GTIM_CHANNEL4, GTIM_OC_OUTPUT_PWM_HIGH);		                        //����ͨ��4��OCģʽΪ�������
////	GTIM_SetCompare1(CW_GTIM1, LCD_BKL);													    //����CCRΪLCD_BKL������LCD��������
//	GTIM_SetCompare2(CW_GTIM1, 100);																//����CCRΪ100�����Ʒ�������������

//	GTIM_ITConfig(CW_GTIM1,  GTIM_IT_OV,  ENABLE);
	//GTIM_Cmd(CW_GTIM1, ENABLE);

	//GTIM2		���ڿ��Ʒ����ʱ��
	GTIM_InitStruct.Mode = GTIM_MODE_TIME;
    GTIM_InitStruct.OneShotMode = GTIM_COUNT_CONTINUE;
    GTIM_InitStruct.ReloadValue = 999;																//����Ϊ1ms��Ƶ��Ϊ1KHz
    GTIM_InitStruct.Prescaler = 23;  											
	GTIM_InitStruct.ToggleOutState = DISABLE;
    GTIM_TimeBaseInit(CW_GTIM2, &GTIM_InitStruct);

	GTIM_OCInit(CW_GTIM2, GTIM_CHANNEL1, GTIM_OC_OUTPUT_PWM_HIGH);		                            //����ͨ��1��OCģʽΪ�������
	GTIM_OCInit(CW_GTIM2, GTIM_CHANNEL2, GTIM_OC_OUTPUT_PWM_HIGH);		                            //����ͨ��2��OCģʽΪ�������
//	GTIM_OCInit(CW_GTIM2, GTIM_CHANNEL3, GTIM_OC_OUTPUT_PWM_HIGH);		                            //����ͨ��3��OCģʽΪ�������
//	GTIM_OCInit(CW_GTIM2, GTIM_CHANNEL4, GTIM_OC_OUTPUT_PWM_HIGH);		                            //����ͨ��4��OCģʽΪ�������
	GTIM_SetCompare1(CW_GTIM2, DAC1_PWM);															//���ó�ʼռ�ձ�Ϊ0
	GTIM_SetCompare2(CW_GTIM2, DAC2_PWM);															//���ó�ʼռ�ձ�Ϊ0
//	GTIM_SetCompare3(CW_GTIM2, DAC1_PWM);															//���ó�ʼռ�ձ�Ϊ0%
//	GTIM_SetCompare4(CW_GTIM2, DAC2_PWM);															//���ó�ʼռ�ձ�Ϊ0%
//	GTIM_ITConfig(CW_GTIM2,  GTIM_IT_OV,  ENABLE);
	//GTIM_Cmd(CW_GTIM2, ENABLE);
}

void ADC_Configuration(void)
{
	ADC_InitTypeDef ADC_InitStruct;
	ADC_SerialChTypeDef ADC_SerialChStruct;
	
	ADC_InitStruct.ADC_Align = ADC_AlignRight;  								                    // ��������Ҷ��룬���������bit11~bit0
	ADC_InitStruct.ADC_ClkDiv = ADC_Clk_Div1;  									                    // ADC�Ĳ���ʱ��ΪPCLK��1��Ƶ,��ADCCLK=24MHz
	ADC_InitStruct.ADC_DMASOCEn = ADC_DMASOCDisable;  					                            // ADC����ת����ɴ���DMA��ʹ��
	ADC_InitStruct.ADC_InBufEn = ADC_BufEnable;  								                    // ���ٲ�����ADC�ڲ���ѹ������ʹ��
	ADC_InitStruct.ADC_OpMode = ADC_SerialChScanMode;  					                            // ����ɨ��ת��ģʽ
	ADC_InitStruct.ADC_SampleTime = ADC_SampTime10Clk;  				                            // ����Ϊ10����������
	ADC_InitStruct.ADC_TsEn = ADC_TsDisable;  									                    // �ڲ��¶ȴ�������ֹ
	ADC_InitStruct.ADC_VrefSel = ADC_Vref_VDDA;  								                    // �����ο���ѹѡ��ΪVDDA
	//ADC_Init(&ADC_InitStruct);																	// ADC�ṹ���ʼ��
	//CW_ADC->CR1_f.DISCARD = FALSE;                                                                // �������ݸ��²���:���Ǿ�����
	ADC_SerialChStruct.ADC_InitStruct = ADC_InitStruct;  				                            // ���в����Ļ���������
	ADC_SerialChStruct.ADC_Sqr0Chmux = ADC_SqrCh0;  						                        // ����0����Ϊ�ⲿ����ͨ��0
	ADC_SerialChStruct.ADC_Sqr1Chmux = ADC_SqrCh1;  						                        // ����1����Ϊ�ⲿ����ͨ��1
	ADC_SerialChStruct.ADC_SqrEns = ADC_SqrEns01;  							                        // ʹ������ 0~1
	ADC_SerialChStruct.ADC_DMASOFEn = ADC_DMASOFEnable;					                            // ADC����ת����ɴ���DMAʹ��
	ADC_SerialChScanModeCfg(&ADC_SerialChStruct);  							                        // ADC����ɨ��ת��ģʽ��ʼ��
	ADC_AutoStop(ADC_AutoStopDisable);													            // ADCת����ɺ��������ADCʹ��
	//ADC_ClearITPendingAll();																		// ADC��������ж�
	//ADC_ITConfig(ADC_IT_EOC | ADC_IT_EOS, ENABLE);							                    // ADC�ж�ʹ��
	ADC_Enable();																					// ADCʹ��
	//ADC_SoftwareStartConvCmd(ENABLE);


}

void DMA_Configration(void)
{
	DMA_InitTypeDef   DMA_InitStruct;
	//DMAͨ��1��ʼ��
	DMA_InitStruct.DMA_Mode = DMA_MODE_BLOCK;														//����DMAģʽ��BLOCK
	DMA_InitStruct.TrigMode = DMA_HardTrig;															//����DMA����ģʽ��Ӳ������
	DMA_InitStruct.HardTrigSource = DMA_HardTrig_ADC_TRANSCOMPLETE;			                        //����DMAӲ������Դ��ADC����ת�����
	DMA_InitStruct.DMA_TransferCnt = 256;															//����DMA���������256��
	DMA_InitStruct.DMA_TransferWidth = DMA_TRANSFER_WIDTH_32BIT;				                    //����DMA����λ��16λ
	DMA_InitStruct.DMA_SrcAddress = (uint32_t) &(CW_ADC->RESULT0);			                        //����DMAԴ��ַ��ADC����Ĵ���0
	DMA_InitStruct.DMA_SrcInc = DMA_SrcAddress_Fix;											        //����DMAԴ��ַģʽ���̶���ַ
	DMA_InitStruct.DMA_DstAddress = (uint32_t) &DC_ADC_Result[0];				                    //����DMAĿ�ĵ�ַ���ڴ��������
	DMA_InitStruct.DMA_DstInc = DMA_DstAddress_Increase;								            //����DMAĿ�ĵ�ַģʽ���Զ�����
	DMA_Init(CW_DMACHANNEL1, &DMA_InitStruct);													    //��ʼ��DMAͨ��1
	//DMAͨ��2��ʼ��
	DMA_InitStruct.DMA_SrcAddress = (uint32_t) &(CW_ADC->RESULT1);			                        //����DMAԴ��ַ��ADC����Ĵ���1
	DMA_InitStruct.DMA_DstAddress = (uint32_t) &AC_ADC_Result[0];				                    //����DMAĿ�ĵ�ַ���ڴ��������
	DMA_Init(CW_DMACHANNEL2, &DMA_InitStruct);													    //��ʼ��DMAͨ��2
	
	DMA_ClearITPendingBit(DMA_IT_ALL);																//���DMA�жϱ�־
	DMA_ITConfig(CW_DMACHANNEL1, DMA_IT_TC | DMA_IT_TE, ENABLE);   			                        //ʹ��DMA_CHANNEL1�ж�
	DMA_Cmd(CW_DMACHANNEL1, ENABLE);  																//ʹ��DMAͨ��1
	DMA_ITConfig(CW_DMACHANNEL2, DMA_IT_TC | DMA_IT_TE, ENABLE);   			                        //ʹ��DMA_CHANNEL2�ж�
	DMA_Cmd(CW_DMACHANNEL2, ENABLE);  																//ʹ��DMAͨ��2

}

void UART2_init(void)
{
	USART_InitTypeDef USART_InitStruct;
	USART_InitStruct.USART_BaudRate = 9600;
	USART_InitStruct.USART_Over = USART_Over_16;
	USART_InitStruct.USART_Source = USART_Source_PCLK;
	USART_InitStruct.USART_UclkFreq = 24000000;
	USART_InitStruct.USART_StartBit = USART_StartBit_FE;
	USART_InitStruct.USART_StopBits = USART_StopBits_1;
	USART_InitStruct.USART_Parity = USART_Parity_No ;
	USART_InitStruct.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStruct.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	USART_Init(CW_UART2, &USART_InitStruct);
	
}

//�жϻص�����
void BTIM1_IRQHandlerCallback(void)
{
/*����ʱ��˵����ÿ�η�������ĸ��׶Σ�IR���䡢ֹͣ���䡢RED���䡢ֹͣ���䣩,ÿ�׶�3ms������12ms��֮��Ϊ27ms���ӳ٣�ֹͣ���䣩��
	            ����Ϊһ����������ѭ����ÿ��ѭ��Ϊ39ms�������������ڰ���128�η���ѭ��������4.992�롣
����ܿ���ʱ��	
                        |IR����|ֹͣ����|RED����|ֹͣ����| �ӳ� |IR����|ֹͣ����|RED����|ֹͣ����| �ӳ� |......
    ---------------------------------------------------------------------------------------------------------
        ʱ��(ms)        0      3        6       9        12     39     42       45      48       51     78......
	---------------------------------------------------------------------------------------------------------
		SEND_state 	    |  1   |   1    |   1   |   1    |   0  |  1   |   1    |   1   |   1    |   0  |......
    ---------------------------------------------------------------------------------------------------------
        SEND_status     |  0   |   1    |   2   |   3    |   0  |  0   |   1    |   2   |   3    |   0  |......   	
    ---------------------------------------------------------------------------------------------------------
        BTIM1_counter1 ��								 ��	                                     �� 
    ---------------------------------------------------------------------------------------------------------
        BTIM1_counter2                                   ��                                      ��    
    ---------------------------------------------------------------------------------------------------------
        BTIM1_counter3 ��     ��       ��      ��        �� ... ��     ��       ��      ��       �� ... ��   
    ---------------------------------------------------------------------------------------------------------
		
*/	
//	static uint8_t BEEP_status = 0;													//����������״̬����2��״̬����ʼΪ״̬0��
//	static uint32_t BTIM1_counter4 = 0;												//BTIM1������4�����ڿ��Ʒ�����	
	if(SET == BTIM_GetITStatus(CW_BTIM1, BTIM_IT_OV))
	{
		BTIM_ClearITPendingBit(CW_BTIM1, BTIM_IT_OV);                               //����жϱ�־λ
		if(IsCycleEnd == 1)															//128�β������ڽ�����־��1Ϊ�����У�0Ϊ��������
		{
			if(BTIM1_counter3 > 2)			                                        //��ʱ�ﵽ3ms
			{
				BTIM1_counter3 = 0;
				switch(SEND_status)
				{
					case 0:															//��������źţ�3ms��
					{
						SEND_status ++;
						GPIO_WritePin(bsp_IN1_port, bsp_IN1_pin, GPIO_Pin_RESET);
						GPIO_WritePin(bsp_IN2_port, bsp_IN2_pin, GPIO_Pin_SET);
						DAC1_PWM = 0;
						DAC2_PWM = 300 + DAC_PWM_PLUS;
						GTIM_SetCompare1(CW_GTIM2, DAC1_PWM);						//����DAC1ռ�ձ�Ϊ0
						GTIM_SetCompare2(CW_GTIM2, DAC2_PWM);						//����DAC2ռ�ձ�Ϊ300+����ֵ
						GTIM_Cmd(CW_GTIM2, ENABLE);
						IRorRED = 0;																											//���ú�������־������
//						__NOP;
						ADC_SoftwareStartConvCmd(ENABLE);						    //����ADCת��
						break;
					}
					case 1:														    //�ر��źŷ��䣨3ms��
					{
						SEND_status ++;
						GPIO_WritePin(bsp_IN1_port, bsp_IN1_pin, GPIO_Pin_RESET);
						GPIO_WritePin(bsp_IN2_port, bsp_IN2_pin, GPIO_Pin_RESET);
						DAC1_PWM = 0;
						DAC2_PWM = 0;
						GTIM_SetCompare1(CW_GTIM2, DAC1_PWM);						//����ռ�ձ�Ϊ0
						GTIM_SetCompare2(CW_GTIM2, DAC2_PWM);						//����ռ�ձ�Ϊ0
						GTIM_Cmd(CW_GTIM2, DISABLE);
						//ADC_SoftwareStartConvCmd(DISABLE);
						break;
					}
					case 2:															//�������źţ�3ms��
					{
						SEND_status ++;
						GPIO_WritePin(bsp_IN1_port, bsp_IN1_pin, GPIO_Pin_SET);
						GPIO_WritePin(bsp_IN2_port, bsp_IN2_pin, GPIO_Pin_RESET);
						DAC1_PWM = 300 + DAC_PWM_PLUS;
						DAC2_PWM = 0;
						GTIM_SetCompare1(CW_GTIM2, DAC1_PWM);					    //����DAC1ռ�ձ�Ϊ300+����ֵ
						GTIM_SetCompare2(CW_GTIM2, DAC2_PWM);						//����DAC2ռ�ձ�Ϊ0
						GTIM_Cmd(CW_GTIM2, ENABLE);
						IRorRED = 1;																											//���ú�������־�����
//						__NOP;
						ADC_SoftwareStartConvCmd(ENABLE);							//����ADCת��
						break;
					}
					case 3:															//�ر��źŷ��䣨4ms��
					{
						SEND_status = 0;
						GPIO_WritePin(bsp_IN1_port, bsp_IN1_pin, GPIO_Pin_RESET);
						GPIO_WritePin(bsp_IN2_port, bsp_IN2_pin, GPIO_Pin_RESET);
						DAC1_PWM = 0;
						DAC2_PWM = 0;
						GTIM_SetCompare1(CW_GTIM2, DAC1_PWM);						//����ռ�ձ�Ϊ0
						GTIM_SetCompare2(CW_GTIM2, DAC2_PWM);						//����ռ�ձ�Ϊ0
						GTIM_Cmd(CW_GTIM2, DISABLE);
						//ADC_SoftwareStartConvCmd(DISABLE);
						break;
					}
				}
			}
			else
			{
				BTIM1_counter3++;                                                   //������3�ۼ�
			}
		}
        //��������
        if(KEY_state == 1)                                                          //����״̬Ϊ�����¡�
        {
            
            if(BTIM1_counter1 >10)                                                  //ÿ10ms���һ��
            {
                BTIM1_counter1 = 0;                                                 //BTIM1_counter1����������
                if(GPIO_Pin_RESET == GPIO_ReadPin(bsp_KEY01_port, bsp_KEY01_pin))   //�����¡�״̬
                {
                    KEY_cnt++;                                                      //����ʱ���ۼ�������
                }
                else                                                                //���ɿ���״̬
                {
                    if(KEY_cnt < 200)                                               //����ʱ��С��2�룬�̰�
                    {
                        KEY_cnt = 0;                                                //����ʱ���ۼ�������
                        BTIM1_counter1 = 0;                                         //BTIM1_counter1����������
                        KEY_state =0;                                               //�������״̬��־                    
                        if(ShowDirect_status == 0)                                  //��ʾ״̬Ϊ0���л�Ϊ������ʾ
                        {
                            ShowDirect_status = 1;
                            LCDINIT = 0;
                            Vertical_UI_init();
                        }
                        else                                                        //�����л�Ϊ������ʾ
                        {
                            ShowDirect_status = 0;
                            LCDINIT = 2;
                            transverse_UI_init();
                        }
                    }
                }
                if(KEY_cnt >= 200)                                                  //����ʱ�����2�룬����
                {
                    KEY_cnt = 0;                                                    //����ʱ���ۼ�������
                    BTIM1_counter1 = 0;                                             //BTIM1_counter1����������
                    KEY_state =0;                                                   //�������״̬��־                    
                    if(sys_status == 0)                                             //ϵͳ����״̬Ϊ0���ػ�
                    {
                        sys_status = 1;
                    }
                    else                                                            //���򣬿���
                    {
                        sys_status = 0;
                    }
                }
                
            }
            else
            {
                BTIM1_counter1++;                                                   //BTIM1_counter1�������ۼ�
            }
        }
        //����������
//		if((BTIM1_counter4 > 500) && (KEY01_status == 1))						    //��ʱ�ﵽ0.5s
//		{
//			BTIM1_counter4 = 0;
//			if(BEEP_status == 0)
//			{
//				BEEP_status = 1;
//				GTIM_Cmd(CW_GTIM1, ENABLE);
//			}
//			else
//			{
//				BEEP_status = 0;
//				GTIM_Cmd(CW_GTIM1, DISABLE);
//			}
//		}
	}
}

//void GTIM1_IRQHandlerCallback(void)
//{
//	if(SET == GTIM_GetITStatus(CW_GTIM1, GTIM_IT_OV))
//  {
//    GTIM_ClearITPendingBit(CW_GTIM1, GTIM_IT_OV);
//	}
//}

//void GTIM2_IRQHandlerCallback(void)
//{
//	if(SET == GTIM_GetITStatus(CW_GTIM2, GTIM_IT_OV))
//  {
//    GTIM_ClearITPendingBit(CW_GTIM2, GTIM_IT_OV);
//	}
//}

void KEY_IRQHandlerCallback(void)
{
    if (CW_GPIOB->ISR_f.PIN3)										//KEY01����GPIOB3�ж�
    {
        GPIOB_INTFLAG_CLR(bv3);                                     //����жϱ�־
        KEY_state = 1;                                              //����״̬����Ϊ����
    }

    
//    if (CW_GPIOB->ISR_f.PIN4)										//KEY02���أ�ѭ������LCD��������
//    {
//        GPIOB_INTFLAG_CLR(bv4);
//				if(LCD_BKL >= 487)
//				{
//					LCD_BKL = 0;
//				}
//				else
//				{
//					LCD_BKL += 50;
//				}
//    }

}

void DMACH1_IRQHandlerCallback(void)
{
	if( DMA_GetITStatus(DMA_IT_TC1) )
  {
		DMA_ClearITPendingBit(DMA_IT_TC1);														//����жϱ�־
		DMA_Cmd(CW_DMACHANNEL1, DISABLE);  														//�ر�DMAͨ��1
		CW_DMACHANNEL1->CNT      = bv16 | 0x100;                              		            //����CNT����
		CW_DMACHANNEL1->DSTADDR  = (uint32_t) &DC_ADC_Result[0];   								//����Ŀ�ĵ�ַ
		DMA_Cmd(CW_DMACHANNEL1, ENABLE);  														//ʹ��DMAͨ��1
//		printf("DMA1 Complete!\r\n");
		IsCycleEnd = 0;																			//���256�β��������ò������ڽ�����־Ϊ0
	}
	if( DMA_GetITStatus(DMA_IT_TE1) )
	{
		DMA_ClearITPendingBit(DMA_IT_TE1);
		printf("DMA1 Error!DMA STATUS:%d \r\n",DMA_GetFlagStatus(CW_DMACHANNEL1));
		while(1);
	}
}

void DMACH23_IRQHandlerCallback(void)
{
	if( DMA_GetITStatus(DMA_IT_TC2) )
  {
		DMA_ClearITPendingBit(DMA_IT_TC2);														//����жϱ�־
		DMA_Cmd(CW_DMACHANNEL2, DISABLE);  														//�ر�DMAͨ��2
		CW_DMACHANNEL2->CNT      = bv16 | 0x100;                              		            //����CNT����
		CW_DMACHANNEL2->DSTADDR  = (uint32_t) &AC_ADC_Result[0];   								//����Ŀ�ĵ�ַ
		DMA_Cmd(CW_DMACHANNEL2, ENABLE);  														//ʹ��DMAͨ��2
//		printf("DMA2 Complete!\r\n");
	}	
	if( DMA_GetITStatus(DMA_IT_TE2) )
	{
		DMA_ClearITPendingBit(DMA_IT_TE2);
		printf("DMA2 Error!DMA STATUS:%d \r\n",DMA_GetFlagStatus(CW_DMACHANNEL2));
		while(1);
	}
}

//void ADC_IRQHandlerCallback(void)
//{
//	if(ADC_GetITStatus(ADC_IT_EOC) != RESET)					//ADC����ת������жϱ�־
//	{
//	}
//	if(ADC_GetITStatus(ADC_IT_EOS) != RESET)					//ADC����ת������жϱ�־
//	{
//	}
//	ADC_ClearITPendingAll();
//}

void Calt(void)
{
	//����
	for(uint16_t i=0;i<128;i++)																				//�����е�ADCͨ��0����ֵ��ֵ��FFTʵ������
	{
		Fft_Real[i] = (uint16_t) (DC_ADC_Result[i*2] & 0x0000FFFF);
	}
	Fft_Imagclear();																						//FFT�鲿��������
	FFT();																									//���ٸ���Ҷ�任
	DC_IR = sqrt(Fft_Real[0]*Fft_Real[0]+Fft_Image[0]*Fft_Image[0]);										//�����ֱ������
	for(uint16_t i=0;i<128;i++)																				//�����е�ADCͨ��1����ֵ��ֵ��FFTʵ������
	{
		Fft_Real[i] = (uint16_t) (AC_ADC_Result[i*2] & 0x0000FFFF);
	}
	Fft_Imagclear();																						//FFT�鲿��������
	FFT();																								    //���ٸ���Ҷ�任
	AC_IR = sqrt(Fft_Real[1]*Fft_Real[1]+Fft_Image[1]*Fft_Image[1]);										//����⽻������
	PI_IR = (float) (AC_IR / DC_IR);																		//�����PI��ֵ

	//���
	for(uint16_t i=0;i<128;i++)																				//�����е�ADCͨ��0����ֵ��ֵ��FFTʵ������
	{
		Fft_Real[i] = (uint16_t) (DC_ADC_Result[i*2+1] & 0x0000FFFF);
	}
	Fft_Imagclear();																						//FFT�鲿��������
	FFT();																									//���ٸ���Ҷ�任
	DC_RED = sqrt(Fft_Real[0]*Fft_Real[0]+Fft_Image[0]*Fft_Image[0]);										//����ֱ������
	for(uint16_t i=0;i<128;i++)																				//�����е�ADCͨ��1����ֵ��ֵ��FFTʵ������
	{
		Fft_Real[i] = (uint16_t) (AC_ADC_Result[i*2+1] & 0x0000FFFF);
	}
	Fft_Imagclear();																						//FFT�鲿��������
	FFT();																									//���ٸ���Ҷ�任
	AC_RED = sqrt(Fft_Real[1]*Fft_Real[1]+Fft_Image[1]*Fft_Image[1]);										//���⽻������
	PI_RED = (float) (AC_RED / DC_RED);																		//����PI��ֵ
	
	//����Ѫ�����Ͷ�
	R = (float) (PI_RED / PI_IR);																			//��Rֵ
	SPO2 = (float) (110.0 - R * 25.0);																	    //��SPO2ֵ
    if(SPO2 < 80)
    {
        SPO2 = 80;
    }
    if(SPO2 > 100)
    {
        SPO2 = 100;
    }
	R_Calt = 1;																								//����Calt��������ֵ״̬Ϊ1

	//��������
	PULSE_number = 0;																						//�ɼ��������ź�������0
	PULSE_Result = 0.0;																				        //�ɼ������������0
	for(uint16_t i=0;i<128;i++)																				//�ں�⽻�������й��˳������ź�
	{
		if(((uint16_t) (AC_ADC_Result[i*2+1] & 0x0000FFFF) > 1000) && ((i*2+1)-PULSE_No[PULSE_number-1]>13))		            //�����źţ�ADC�ɼ����>1000�Ҿ�����һ��λ�ü������500
		{
			PULSE_R[PULSE_number] = (uint16_t) (AC_ADC_Result[i*2+1] & 0x0000FFFF);    			                                //���������ź�
			PULSE_No[PULSE_number] = i * 2 + 1;																					//���������ź�λ��
			PULSE_number += 1;                                                                                                  //�ɼ��������ź�������1
		}
	}
	if(PULSE_number >1)																											//��������ź�����Ϊ2��������
	{
		for(uint16_t i=0;i<PULSE_number-1 ;i++)																					//�ں�⽻�������й��˳������ź�
		{
			PULSE_Result += (float) (60000.0 / ((PULSE_No[i+1] - PULSE_No[i]) * 39.0));					                        //�������㣺ͨ���������������źŵ�λ������60�������������ۼ�
		}
		PULSE_Result = (float) (PULSE_Result / (PULSE_number-1));											                    //�þ�ֵ����������������
	}
	else
	{
		PULSE_Result = 0.0;																										//��������ź�����Ϊ0��1����������Ϊ0
	}
	IsCycleEnd = 1;																												//������һ�ּ������

//	printf("**************ADC�ɼ�����**************\r\n");
//	for(uint16_t i=0;i<128;i++)
//	{
//		printf("No:%d  DC_ADC_IR:%d\r\n",i,DC_ADC_Result[i*2]);
//		printf("No:%d  AC_ADC_IR:%d\r\n",i,AC_ADC_Result[i*2]);
//	}
//	for(uint16_t i=0;i<128;i++)
//	{
//		printf("No:%d  DC_ADC_RED:%d\r\n",i,DC_ADC_Result[i*2+1]);
//		printf("No:%d  AC_ADC_RED:%d\r\n",i,AC_ADC_Result[i*2+1]);
//	}
//	printf("**************�м�������**************\r\n");
//	printf("DC_IR:%f\r\n",DC_IR);
//	printf("AC_IR:%f\r\n",AC_IR);
//	printf("DC_RED:%f\r\n",DC_RED);
//	printf("AC_RED:%f\r\n",AC_RED);
//	printf("PI_IR:%f\r\n",PI_IR);
//	printf("PI_RED:%f\r\n",PI_RED);
//  printf("R:%f\r\n",R);
//  printf("PULSE_number:%d\r\n",PULSE_number);
//  printf("PULSE_Result:%f\r\n",PULSE_Result);
//	printf("**************���ս������**************\r\n");
//	printf("SPO2:%f\r\n",SPO2);
	
//	//���ݹ���
//	for(uint8_t i= 0;i<100;i++)
//	{
//		acc += SPO2[i];
//	}
//	avg = acc / 100;
//	for(uint8_t i= 0;i<100;i++)
//	{
//		sd += pow((double)(SPO2[i]-avg) ,(double)2.0);
//	}
//	sd = sqrt((double)sd);
//	for(uint8_t i= 0;i<100;i++)
//	{
//		if(!(((SPO2[i] - avg)> (3.0 * sd)) || ((avg - SPO2[i])> (3.0 * sd))))
//		{
//			counter += SPO2[i];
//			number +=1;
//		}
//	}
//	SPO2_AVG = (float) (counter / number);
}

void LCD_show(uint8_t tmp_Direct)
{
    if(tmp_Direct == 0)                                                 //����
    {
        LCD_ShowIntNum(9,28,(uint16_t)SPO2,1);  					    //Ѫ��
        LCD_ShowIntNum(117,28,(uint16_t)PULSE_Result,2);  			    //����
        LCD_ShowFloatNum(64,40,PI_RED,WHITE,BLACK,16);  		 	    //PI
        LCD_Show_Power(56,0,1,POWER);  			                        //����
        LCD_ShowBoxing(boxing_moni,1);                                  //��������
    }
    else                                                                //����        
    {
        LCD_ShowIntNum(9,24,(uint16_t)SPO2,1);  					    //Ѫ��
        LCD_ShowIntNum(9,78,(uint16_t)PULSE_Result,2);  			    //����
        LCD_ShowFloatNum(40,114,PI_RED,WHITE,BLACK,16);  		 	    //PI
        LCD_Show_Power(113,58,2,POWER);  			                    //����
        LCD_ShowBoxing(boxing_moni,2);                                  //��������
    }
}


int fputc(int ch, FILE *f)
{
    USART_SendData_8bit(CW_UART2, (uint8_t)ch);

    while (USART_GetFlagStatus(CW_UART2, USART_FLAG_TXE) == RESET);

    return ch;
}

/******************************************************************************
 * EOF (not truncated)
 ******************************************************************************/
#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
    /* USER CODE BEGIN 6 */
    /* User can add his own implementation to report the file name and line number,
       tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
    /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */


