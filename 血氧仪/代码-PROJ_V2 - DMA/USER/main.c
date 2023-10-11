#include "main.h"												//常用头文件放置main.h
#include "LCD.h"
#include "LCD_INIT.h"
#include "pic.h"
#include "FFT.h"
#include "math.h"

//uint16_t *adcvalue=0;    								        //保存ADC的值（指针）
//uint16_t timecount=0;										    //计时器
uint8_t SEND_status = 0; 									    //发射管引脚状态（共4种状态，初始为状态0）
//uint8_t SEND_state = 0;										//发射管启停状态：0为停止，1为启用
uint32_t BTIM1_counter1 = 0;							        //BTIM1计数器1，用于控制按键时序
//uint32_t BTIM1_counter2 = 39;							        //BTIM1计数器2，用于控制发射管启动状态
uint32_t BTIM1_counter3 = 3;							        //BTIM1计数器3，用于控制发射管启停时序	
uint8_t  KEY_state = 0;								            //按键状态(0 = 未按下；1 = 按下）
uint16_t KEY_cnt = 0;                                           //按键时长累加器
uint8_t  sys_status = 0;								        //系统运行状态（0 = 开机；1 = 关机）
uint8_t  ShowDirect_status = 0;                                 //屏幕显示方向（0 = 横屏；1 = 竖屏）
//uint32_t  LCD_BKL = 0;										//LCD背光亮度
uint16_t POWER = 80;                                            //保存电池电量（初始值=80）
uint16_t DAC1_PWM = 0;										    //发射管电流控制DAC1
uint16_t DAC2_PWM = 0;										    //发射管电流控制DAC2
uint16_t DAC_PWM_PLUS = 300;								    //发射管电流增强值(最大为699）
uint8_t  IRorRED = 0;											//存储当前处理的信号类型标志：0表示红外，1表示红光
uint32_t DC_ADC_Result[256] = {0};				                //存储DMA传输结果直流分量
uint32_t AC_ADC_Result[256] = {0};				                //存储DMA传输结果交流分量
uint8_t PULSE_number = 0;									    //存储脉搏信号数量
uint8_t PULSE_No[128] = {0};							        //存储脉搏信号位置
uint16_t PULSE_R[128] = {0};							        //存储脉搏信号测量结果
float PULSE_Result = 0.0;									    //存储脉搏计算结果
extern signed short Fft_Real[128]; 				                //fft实部，128数组
extern signed short Fft_Image[128]; 			                //fft虚部，128数组
float DC_IR,AC_IR,DC_RED,AC_RED;					            //存储红外和红光的直流、交流分量
float  PI_IR,PI_RED,R,SPO2;								        //存储红外和红光的PI值、R值、SPO2值
uint8_t R_Calt = 0;												//存储Calt函数返回值状态
uint8_t IsCycleEnd = 0;										    //128次采样周期结束标志：1为采样中，0为采样结束
uint8_t LCDINIT = 2;
// 模拟脉搏数据，模拟ADC转换结果，两位小数，范围(0-3.3)
float boxing_moni[160]= {1.40,0.58,3.04,2.43,1.91,2.65,1.24,0.51,3.02,0.72,2.05,1.52,0.34,1.28,2.71,2.30,0.21,3.22,1.06,0.64,0.37,3.15,2.26,3.14,3.01,2.63,0.24,0.99,0.91,2.16,0.69,0.46,2.87,0.68,1.95,2.84,2.91,1.47,3.21,0.79,1.65,2.07,2.10,0.83,0.59,1.92,0.57,0.19,2.04,1.76,0.39,3.19,1.50,2.85,3.26,2.41,2.15,2.48,3.25,0.38,1.77,1.04,2.72,2.59,0.03,1.78,2.12,1.58,0.17,2.47,1.13,0.73,0.60,1.97,1.03,0.12,0.08,2.55,3.28,0.35,0.44,2.37,1.05,1.66,2.57,2.46,0.92,2.22,0.54,1.22,3.16,1.53,0.27,0.01,2.28,0.81,2.03,0.47,2.25,0.07,3.08,0.88,0.45,1.72,2.74,2.88,1.45,2.08,0.61,0.77,3.13,3.11,2.99,1.90,1.62,0.00,2.33,2.64,1.43,0.63,0.41,1.71,0.32,2.49,1.67,1.86,1.56,0.02,1.09,2.98,0.49,1.99,0.33,1.38,3.07,2.62,2.61,0.75,1.55,2.21,3.12,0.65,2.67,1.80,0.98,2.60,3.18,2.31,1.54,3.06,2.18,1.36,0.05,2.92,0.84,0.96,2.78,2.52,0.76,0.43};	;
		
int32_t main(void)
{
	SYS_init();													        //系统初始化
    if(ShowDirect_status == 0)                                          //横屏
    {
        LCDINIT = 2;
        transverse_UI_init();                                           //横屏显示初始化
    }
    else                                                                //竖屏        
    {
        LCDINIT = 0;
        Vertical_UI_init();                                             //竖屏显示初始化
    }
    LCD_show(ShowDirect_status);                                        //数值显示
	FirmwareDelay(400000);                                              //延时
    //GPIO_WritePin(bsp_LED01_port, bsp_LED01_pin, GPIO_Pin_RESET);		//启动默认打开LED01（调试用）	
//	printf("**************启动程序**************\r\n");	                //（调试用）
	IsCycleEnd = 1;

	while(1)
	{
//		LCD_ShowBoxing(boxing_moni,2);  			                    // 竖屏波形
//		LCD_ShowBoxing(boxing_moni,1);    	   	                        // 横屏波形		
//		printf("R_Calt:%d\r\n",R_Calt);
		if((1 == IsCycleEnd)	&& (0 == (CW_BTIM1->BCR & 0x0001)))	    // 采样周期开始且BTIM1的状态为不使能
		{
			SEND_status = 0; 																														//发射管引脚状态（共4种状态，初始为状态0）
//			SEND_state = 0;																															//发射管启停状态：0为停止，1为启用
//			BTIM1_counter1 = 27;																												//BTIM1计数器1，用于控制发射管停止状态
//			BTIM1_counter2 = 39;																												//BTIM1计数器2，用于控制发射管启动状态
			BTIM1_counter3 = 3;																													//BTIM1计数器3，用于控制发射管启停时序	
//			ADC_Enable();																																//ADC使能
			BTIM_Cmd(CW_BTIM1, ENABLE);									//BTIM1使能
		}
		else if((0 == IsCycleEnd)	&& (1 == (CW_BTIM1->BCR & 0x0001)))					// 采样周期结束且BTIM1的状态为使能
		{
			BTIM_Cmd(CW_BTIM1, DISABLE);								//BTIM1不使能
			Calt();
		}

		if(R_Calt)
		{
			R_Calt = 0;
            LCD_show(ShowDirect_status);                                //数值显示
		}

        if(sys_status == 1)                                             //系统运行状态为1，关机
        {
            GPIO_WritePin(bsp_PWM_port, bsp_PWM_pin, GPIO_Pin_SET);
        }
        else                                                            //否则，开机
        {
            GPIO_WritePin(bsp_PWM_port, bsp_PWM_pin, GPIO_Pin_RESET);
        }

	}
}

void SYS_init(void)
{
	RCC_init();														    //时钟初始化
    GPIO_init();													    //GPIO初始化
	BTIM_init();           								                //基本定时器初始化 
	GTIM_init();													    //通用定时器初始化
    NVIC_init();													    //中断向量初始化
	transverse_UI_init();									            //横屏UI 初始化
	ADC_Configuration();   								                //ADC初始化
	UART2_init();													    //串口初始化
	DMA_Configration();										            //DMA初始化
    GPIO_WritePin(bsp_PWM_port, bsp_PWM_pin, GPIO_Pin_RESET);		    //启动默认打开LCD（低电平）
}


void RCC_init(void)
{
	//配置系统时钟
	__RCC_FLASH_CLK_ENABLE();							                
	FLASH_SetLatency(FLASH_Latency_1);                                  //< 当使用的时钟源HCLK小于24M：设置FLASH 读等待周期为1 cycle    
	RCC_HSI_Enable(RCC_HSIOSC_DIV2);				                    // 0. HSI使能并校准 
	RCC_HCLKPRS_Config(RCC_HCLK_DIV1);			                        // 1. 设置HCLK的分频系数　
	RCC_PCLKPRS_Config(RCC_PCLK_DIV1);			                        // 2. 设置PCLK的分频系数			
	RCC_SystemCoreClockUpdate(24000000);		                        // 3. 设置系统主频为24MHz
	//开启GPIO时钟
	__RCC_GPIOA_CLK_ENABLE();	
	__RCC_GPIOB_CLK_ENABLE();	
	__RCC_GPIOF_CLK_ENABLE();	
	//开启定时器时钟
	__RCC_BTIM_CLK_ENABLE();
//	__RCC_GTIM1_CLK_ENABLE();
	__RCC_GTIM2_CLK_ENABLE();
	//开启ADC时钟
	__RCC_ADC_CLK_ENABLE();	
	//开启DMA时钟
	__RCC_DMA_CLK_ENABLE();
	//开启串口时钟
	__RCC_UART2_CLK_ENABLE();
}
	

void GPIO_init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct;
    //初始化LED_GPIO
    GPIO_InitStruct.IT = GPIO_IT_NONE; 
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pins = bsp_LED01_pin;
    GPIO_Init(bsp_LED01_port, &GPIO_InitStruct);
    GPIO_InitStruct.IT = GPIO_IT_NONE; 
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pins = bsp_LED02_pin;
    GPIO_Init(bsp_LED02_port, &GPIO_InitStruct);
    //初始化KEY_GPIO
    GPIO_InitStruct.IT = GPIO_IT_FALLING; 
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pins = bsp_KEY01_pin;
    GPIO_Init(bsp_KEY01_port, &GPIO_InitStruct);
	GPIOB_INTFLAG_CLR(bv3);
	//初始化BEEP_GPIO
//	PA07_AFx_GTIM1CH2();
//	PA07_DIGTAL_ENABLE();
//	PA07_DIR_OUTPUT();
//	PA07_PUSHPULL_ENABLE();
	//初始化LCD_GPIO
	GPIO_InitStruct.IT = GPIO_IT_NONE; 
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pins = GPIO_PIN_2| GPIO_PIN_3| GPIO_PIN_4| GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_8;
	GPIO_Init(CW_GPIOA, &GPIO_InitStruct);
//	PA06_AFx_GTIM1CH1();
//	PA06_DIGTAL_ENABLE();
//	PA06_DIR_OUTPUT();
//	PA06_PUSHPULL_ENABLE();
	//初始化SEND_GPIO
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
	//初始化ADC_GPIO
	PA00_ANALOG_ENABLE();
	PA01_ANALOG_ENABLE();
	//初始化串口
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
	//设置中断优先级
	NVIC_SetPriority(BTIM1_IRQn, 0);
//	NVIC_SetPriority(GTIM1_IRQn, 0);
//	NVIC_SetPriority(GTIM2_IRQn, 0);
	NVIC_SetPriority(GPIOA_IRQn, 3);
	NVIC_SetPriority(GPIOB_IRQn, 0);
//	NVIC_SetPriority(ADC_IRQn, 0);
	NVIC_SetPriority(DMACH1_IRQn, 1);
	NVIC_SetPriority(DMACH23_IRQn, 1);
	//设置中断使能
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
    BTIM_InitStruct.BTIM_Prescaler = 23;  															//T = ((PSC+1)/PCLK)×(ARR+1)
    BTIM_TimeBaseInit(CW_BTIM1, &BTIM_InitStruct);
	BTIM_ClearITPendingBit(CW_BTIM1, BTIM_IT_OV);
    BTIM_ITConfig(CW_BTIM1, BTIM_IT_OV, ENABLE);
//  BTIM_Cmd(CW_BTIM1, ENABLE);
}

void GTIM_init(void)
{
	GTIM_InitTypeDef GTIM_InitStruct;
//	//GTIM1		用于控制蜂鸣器发声
//	GTIM_InitStruct.Mode = GTIM_MODE_TIME;
//  GTIM_InitStruct.OneShotMode = GTIM_COUNT_CONTINUE;
//  GTIM_InitStruct.ReloadValue = 487;																//周期为0.5ms，频率为2KHz
//  GTIM_InitStruct.Prescaler = 23;  											
//	GTIM_InitStruct.ToggleOutState = DISABLE;
//  GTIM_TimeBaseInit(CW_GTIM1, &GTIM_InitStruct);

////	GTIM_OCInit(CW_GTIM1, GTIM_CHANNEL1, GTIM_OC_OUTPUT_PWM_HIGH);		                        //设置通道1的OC模式为正向输出
//	GTIM_OCInit(CW_GTIM1, GTIM_CHANNEL2, GTIM_OC_OUTPUT_PWM_HIGH);		                            //设置通道2的OC模式为正向输出
////	GTIM_OCInit(CW_GTIM1, GTIM_CHANNEL3, GTIM_OC_OUTPUT_PWM_HIGH);		                        //设置通道3的OC模式为正向输出
////	GTIM_OCInit(CW_GTIM1, GTIM_CHANNEL4, GTIM_OC_OUTPUT_PWM_HIGH);		                        //设置通道4的OC模式为正向输出
////	GTIM_SetCompare1(CW_GTIM1, LCD_BKL);													    //设置CCR为LCD_BKL，控制LCD背光亮度
//	GTIM_SetCompare2(CW_GTIM1, 100);																//设置CCR为100，控制蜂鸣器声音最响

//	GTIM_ITConfig(CW_GTIM1,  GTIM_IT_OV,  ENABLE);
	//GTIM_Cmd(CW_GTIM1, ENABLE);

	//GTIM2		用于控制发射管时序
	GTIM_InitStruct.Mode = GTIM_MODE_TIME;
    GTIM_InitStruct.OneShotMode = GTIM_COUNT_CONTINUE;
    GTIM_InitStruct.ReloadValue = 999;																//周期为1ms，频率为1KHz
    GTIM_InitStruct.Prescaler = 23;  											
	GTIM_InitStruct.ToggleOutState = DISABLE;
    GTIM_TimeBaseInit(CW_GTIM2, &GTIM_InitStruct);

	GTIM_OCInit(CW_GTIM2, GTIM_CHANNEL1, GTIM_OC_OUTPUT_PWM_HIGH);		                            //设置通道1的OC模式为正向输出
	GTIM_OCInit(CW_GTIM2, GTIM_CHANNEL2, GTIM_OC_OUTPUT_PWM_HIGH);		                            //设置通道2的OC模式为正向输出
//	GTIM_OCInit(CW_GTIM2, GTIM_CHANNEL3, GTIM_OC_OUTPUT_PWM_HIGH);		                            //设置通道3的OC模式为正向输出
//	GTIM_OCInit(CW_GTIM2, GTIM_CHANNEL4, GTIM_OC_OUTPUT_PWM_HIGH);		                            //设置通道4的OC模式为正向输出
	GTIM_SetCompare1(CW_GTIM2, DAC1_PWM);															//设置初始占空比为0
	GTIM_SetCompare2(CW_GTIM2, DAC2_PWM);															//设置初始占空比为0
//	GTIM_SetCompare3(CW_GTIM2, DAC1_PWM);															//设置初始占空比为0%
//	GTIM_SetCompare4(CW_GTIM2, DAC2_PWM);															//设置初始占空比为0%
//	GTIM_ITConfig(CW_GTIM2,  GTIM_IT_OV,  ENABLE);
	//GTIM_Cmd(CW_GTIM2, ENABLE);
}

void ADC_Configuration(void)
{
	ADC_InitTypeDef ADC_InitStruct;
	ADC_SerialChTypeDef ADC_SerialChStruct;
	
	ADC_InitStruct.ADC_Align = ADC_AlignRight;  								                    // 采样结果右对齐，即结果存于bit11~bit0
	ADC_InitStruct.ADC_ClkDiv = ADC_Clk_Div1;  									                    // ADC的采样时钟为PCLK的1分频,即ADCCLK=24MHz
	ADC_InitStruct.ADC_DMASOCEn = ADC_DMASOCDisable;  					                            // ADC单次转换完成触发DMA不使能
	ADC_InitStruct.ADC_InBufEn = ADC_BufEnable;  								                    // 高速采样，ADC内部电压跟随器使能
	ADC_InitStruct.ADC_OpMode = ADC_SerialChScanMode;  					                            // 序列扫描转换模式
	ADC_InitStruct.ADC_SampleTime = ADC_SampTime10Clk;  				                            // 设置为10个采样周期
	ADC_InitStruct.ADC_TsEn = ADC_TsDisable;  									                    // 内部温度传感器禁止
	ADC_InitStruct.ADC_VrefSel = ADC_Vref_VDDA;  								                    // 采样参考电压选择为VDDA
	//ADC_Init(&ADC_InitStruct);																	// ADC结构体初始化
	//CW_ADC->CR1_f.DISCARD = FALSE;                                                                // 配置数据更新策略:覆盖旧数据
	ADC_SerialChStruct.ADC_InitStruct = ADC_InitStruct;  				                            // 序列采样的基本配置项
	ADC_SerialChStruct.ADC_Sqr0Chmux = ADC_SqrCh0;  						                        // 序列0配置为外部输入通道0
	ADC_SerialChStruct.ADC_Sqr1Chmux = ADC_SqrCh1;  						                        // 序列1配置为外部输入通道1
	ADC_SerialChStruct.ADC_SqrEns = ADC_SqrEns01;  							                        // 使能序列 0~1
	ADC_SerialChStruct.ADC_DMASOFEn = ADC_DMASOFEnable;					                            // ADC序列转换完成触发DMA使能
	ADC_SerialChScanModeCfg(&ADC_SerialChStruct);  							                        // ADC序列扫描转换模式初始化
	ADC_AutoStop(ADC_AutoStopDisable);													            // ADC转换完成后继续保持ADC使能
	//ADC_ClearITPendingAll();																		// ADC清除所有中断
	//ADC_ITConfig(ADC_IT_EOC | ADC_IT_EOS, ENABLE);							                    // ADC中断使能
	ADC_Enable();																					// ADC使能
	//ADC_SoftwareStartConvCmd(ENABLE);


}

void DMA_Configration(void)
{
	DMA_InitTypeDef   DMA_InitStruct;
	//DMA通道1初始化
	DMA_InitStruct.DMA_Mode = DMA_MODE_BLOCK;														//设置DMA模式：BLOCK
	DMA_InitStruct.TrigMode = DMA_HardTrig;															//设置DMA触发模式：硬件触发
	DMA_InitStruct.HardTrigSource = DMA_HardTrig_ADC_TRANSCOMPLETE;			                        //设置DMA硬件触发源：ADC序列转换完成
	DMA_InitStruct.DMA_TransferCnt = 256;															//设置DMA传输次数：256次
	DMA_InitStruct.DMA_TransferWidth = DMA_TRANSFER_WIDTH_32BIT;				                    //设置DMA传输位宽：16位
	DMA_InitStruct.DMA_SrcAddress = (uint32_t) &(CW_ADC->RESULT0);			                        //设置DMA源地址：ADC结果寄存器0
	DMA_InitStruct.DMA_SrcInc = DMA_SrcAddress_Fix;											        //设置DMA源地址模式：固定地址
	DMA_InitStruct.DMA_DstAddress = (uint32_t) &DC_ADC_Result[0];				                    //设置DMA目的地址：内存变量数组
	DMA_InitStruct.DMA_DstInc = DMA_DstAddress_Increase;								            //设置DMA目的地址模式：自动增长
	DMA_Init(CW_DMACHANNEL1, &DMA_InitStruct);													    //初始化DMA通道1
	//DMA通道2初始化
	DMA_InitStruct.DMA_SrcAddress = (uint32_t) &(CW_ADC->RESULT1);			                        //设置DMA源地址：ADC结果寄存器1
	DMA_InitStruct.DMA_DstAddress = (uint32_t) &AC_ADC_Result[0];				                    //设置DMA目的地址：内存变量数组
	DMA_Init(CW_DMACHANNEL2, &DMA_InitStruct);													    //初始化DMA通道2
	
	DMA_ClearITPendingBit(DMA_IT_ALL);																//清除DMA中断标志
	DMA_ITConfig(CW_DMACHANNEL1, DMA_IT_TC | DMA_IT_TE, ENABLE);   			                        //使能DMA_CHANNEL1中断
	DMA_Cmd(CW_DMACHANNEL1, ENABLE);  																//使能DMA通道1
	DMA_ITConfig(CW_DMACHANNEL2, DMA_IT_TC | DMA_IT_TE, ENABLE);   			                        //使能DMA_CHANNEL2中断
	DMA_Cmd(CW_DMACHANNEL2, ENABLE);  																//使能DMA通道2

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

//中断回调函数
void BTIM1_IRQHandlerCallback(void)
{
/*控制时序说明：每次发射包括四个阶段（IR发射、停止发射、RED发射、停止发射）,每阶段3ms，共计12ms；之后为27ms的延迟（停止发射）；
	            上述为一个完整发射循环，每个循环为39ms；完整发射周期包括128次发射循环，共计4.992秒。
发射管控制时序：	
                        |IR发射|停止发射|RED发射|停止发射| 延迟 |IR发射|停止发射|RED发射|停止发射| 延迟 |......
    ---------------------------------------------------------------------------------------------------------
        时序(ms)        0      3        6       9        12     39     42       45      48       51     78......
	---------------------------------------------------------------------------------------------------------
		SEND_state 	    |  1   |   1    |   1   |   1    |   0  |  1   |   1    |   1   |   1    |   0  |......
    ---------------------------------------------------------------------------------------------------------
        SEND_status     |  0   |   1    |   2   |   3    |   0  |  0   |   1    |   2   |   3    |   0  |......   	
    ---------------------------------------------------------------------------------------------------------
        BTIM1_counter1 △								 △	                                     △ 
    ---------------------------------------------------------------------------------------------------------
        BTIM1_counter2                                   △                                      △    
    ---------------------------------------------------------------------------------------------------------
        BTIM1_counter3 △     △       △      △        △ ... △     △       △      △       △ ... △   
    ---------------------------------------------------------------------------------------------------------
		
*/	
//	static uint8_t BEEP_status = 0;													//蜂鸣器引脚状态（共2种状态，初始为状态0）
//	static uint32_t BTIM1_counter4 = 0;												//BTIM1计数器4，用于控制蜂鸣器	
	if(SET == BTIM_GetITStatus(CW_BTIM1, BTIM_IT_OV))
	{
		BTIM_ClearITPendingBit(CW_BTIM1, BTIM_IT_OV);                               //清除中断标志位
		if(IsCycleEnd == 1)															//128次采样周期结束标志：1为采样中，0为采样结束
		{
			if(BTIM1_counter3 > 2)			                                        //计时达到3ms
			{
				BTIM1_counter3 = 0;
				switch(SEND_status)
				{
					case 0:															//发射红外信号（3ms）
					{
						SEND_status ++;
						GPIO_WritePin(bsp_IN1_port, bsp_IN1_pin, GPIO_Pin_RESET);
						GPIO_WritePin(bsp_IN2_port, bsp_IN2_pin, GPIO_Pin_SET);
						DAC1_PWM = 0;
						DAC2_PWM = 300 + DAC_PWM_PLUS;
						GTIM_SetCompare1(CW_GTIM2, DAC1_PWM);						//设置DAC1占空比为0
						GTIM_SetCompare2(CW_GTIM2, DAC2_PWM);						//设置DAC2占空比为300+调整值
						GTIM_Cmd(CW_GTIM2, ENABLE);
						IRorRED = 0;																											//设置红外或红光标志：红外
//						__NOP;
						ADC_SoftwareStartConvCmd(ENABLE);						    //启动ADC转换
						break;
					}
					case 1:														    //关闭信号发射（3ms）
					{
						SEND_status ++;
						GPIO_WritePin(bsp_IN1_port, bsp_IN1_pin, GPIO_Pin_RESET);
						GPIO_WritePin(bsp_IN2_port, bsp_IN2_pin, GPIO_Pin_RESET);
						DAC1_PWM = 0;
						DAC2_PWM = 0;
						GTIM_SetCompare1(CW_GTIM2, DAC1_PWM);						//设置占空比为0
						GTIM_SetCompare2(CW_GTIM2, DAC2_PWM);						//设置占空比为0
						GTIM_Cmd(CW_GTIM2, DISABLE);
						//ADC_SoftwareStartConvCmd(DISABLE);
						break;
					}
					case 2:															//发射红光信号（3ms）
					{
						SEND_status ++;
						GPIO_WritePin(bsp_IN1_port, bsp_IN1_pin, GPIO_Pin_SET);
						GPIO_WritePin(bsp_IN2_port, bsp_IN2_pin, GPIO_Pin_RESET);
						DAC1_PWM = 300 + DAC_PWM_PLUS;
						DAC2_PWM = 0;
						GTIM_SetCompare1(CW_GTIM2, DAC1_PWM);					    //设置DAC1占空比为300+调整值
						GTIM_SetCompare2(CW_GTIM2, DAC2_PWM);						//设置DAC2占空比为0
						GTIM_Cmd(CW_GTIM2, ENABLE);
						IRorRED = 1;																											//设置红外或红光标志：红光
//						__NOP;
						ADC_SoftwareStartConvCmd(ENABLE);							//启动ADC转换
						break;
					}
					case 3:															//关闭信号发射（4ms）
					{
						SEND_status = 0;
						GPIO_WritePin(bsp_IN1_port, bsp_IN1_pin, GPIO_Pin_RESET);
						GPIO_WritePin(bsp_IN2_port, bsp_IN2_pin, GPIO_Pin_RESET);
						DAC1_PWM = 0;
						DAC2_PWM = 0;
						GTIM_SetCompare1(CW_GTIM2, DAC1_PWM);						//设置占空比为0
						GTIM_SetCompare2(CW_GTIM2, DAC2_PWM);						//设置占空比为0
						GTIM_Cmd(CW_GTIM2, DISABLE);
						//ADC_SoftwareStartConvCmd(DISABLE);
						break;
					}
				}
			}
			else
			{
				BTIM1_counter3++;                                                   //计数器3累加
			}
		}
        //按键控制
        if(KEY_state == 1)                                                          //按键状态为“按下”
        {
            
            if(BTIM1_counter1 >10)                                                  //每10ms检测一次
            {
                BTIM1_counter1 = 0;                                                 //BTIM1_counter1计数器清零
                if(GPIO_Pin_RESET == GPIO_ReadPin(bsp_KEY01_port, bsp_KEY01_pin))   //“按下”状态
                {
                    KEY_cnt++;                                                      //按键时长累加器递增
                }
                else                                                                //“松开”状态
                {
                    if(KEY_cnt < 200)                                               //按下时间小于2秒，短按
                    {
                        KEY_cnt = 0;                                                //按键时长累加器清零
                        BTIM1_counter1 = 0;                                         //BTIM1_counter1计数器清零
                        KEY_state =0;                                               //清除按键状态标志                    
                        if(ShowDirect_status == 0)                                  //显示状态为0，切换为竖屏显示
                        {
                            ShowDirect_status = 1;
                            LCDINIT = 0;
                            Vertical_UI_init();
                        }
                        else                                                        //否则，切换为横屏显示
                        {
                            ShowDirect_status = 0;
                            LCDINIT = 2;
                            transverse_UI_init();
                        }
                    }
                }
                if(KEY_cnt >= 200)                                                  //按下时间大于2秒，长按
                {
                    KEY_cnt = 0;                                                    //按键时长累加器清零
                    BTIM1_counter1 = 0;                                             //BTIM1_counter1计数器清零
                    KEY_state =0;                                                   //清除按键状态标志                    
                    if(sys_status == 0)                                             //系统运行状态为0，关机
                    {
                        sys_status = 1;
                    }
                    else                                                            //否则，开机
                    {
                        sys_status = 0;
                    }
                }
                
            }
            else
            {
                BTIM1_counter1++;                                                   //BTIM1_counter1计数器累加
            }
        }
        //蜂鸣器控制
//		if((BTIM1_counter4 > 500) && (KEY01_status == 1))						    //计时达到0.5s
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
    if (CW_GPIOB->ISR_f.PIN3)										//KEY01引脚GPIOB3中断
    {
        GPIOB_INTFLAG_CLR(bv3);                                     //清除中断标志
        KEY_state = 1;                                              //按键状态设置为按下
    }

    
//    if (CW_GPIOB->ISR_f.PIN4)										//KEY02开关，循环控制LCD背光亮度
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
		DMA_ClearITPendingBit(DMA_IT_TC1);														//清除中断标志
		DMA_Cmd(CW_DMACHANNEL1, DISABLE);  														//关闭DMA通道1
		CW_DMACHANNEL1->CNT      = bv16 | 0x100;                              		            //重置CNT计数
		CW_DMACHANNEL1->DSTADDR  = (uint32_t) &DC_ADC_Result[0];   								//重置目的地址
		DMA_Cmd(CW_DMACHANNEL1, ENABLE);  														//使能DMA通道1
//		printf("DMA1 Complete!\r\n");
		IsCycleEnd = 0;																			//完成256次采样，设置采样周期结束标志为0
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
		DMA_ClearITPendingBit(DMA_IT_TC2);														//清除中断标志
		DMA_Cmd(CW_DMACHANNEL2, DISABLE);  														//关闭DMA通道2
		CW_DMACHANNEL2->CNT      = bv16 | 0x100;                              		            //重置CNT计数
		CW_DMACHANNEL2->DSTADDR  = (uint32_t) &AC_ADC_Result[0];   								//重置目的地址
		DMA_Cmd(CW_DMACHANNEL2, ENABLE);  														//使能DMA通道2
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
//	if(ADC_GetITStatus(ADC_IT_EOC) != RESET)					//ADC单次转换完成中断标志
//	{
//	}
//	if(ADC_GetITStatus(ADC_IT_EOS) != RESET)					//ADC序列转换完成中断标志
//	{
//	}
//	ADC_ClearITPendingAll();
//}

void Calt(void)
{
	//红外
	for(uint16_t i=0;i<128;i++)																				//数组中的ADC通道0采样值赋值给FFT实部数组
	{
		Fft_Real[i] = (uint16_t) (DC_ADC_Result[i*2] & 0x0000FFFF);
	}
	Fft_Imagclear();																						//FFT虚部数组清零
	FFT();																									//快速傅里叶变换
	DC_IR = sqrt(Fft_Real[0]*Fft_Real[0]+Fft_Image[0]*Fft_Image[0]);										//求红外直流分量
	for(uint16_t i=0;i<128;i++)																				//数组中的ADC通道1采样值赋值给FFT实部数组
	{
		Fft_Real[i] = (uint16_t) (AC_ADC_Result[i*2] & 0x0000FFFF);
	}
	Fft_Imagclear();																						//FFT虚部数组清零
	FFT();																								    //快速傅里叶变换
	AC_IR = sqrt(Fft_Real[1]*Fft_Real[1]+Fft_Image[1]*Fft_Image[1]);										//求红外交流分量
	PI_IR = (float) (AC_IR / DC_IR);																		//求红外PI的值

	//红光
	for(uint16_t i=0;i<128;i++)																				//数组中的ADC通道0采样值赋值给FFT实部数组
	{
		Fft_Real[i] = (uint16_t) (DC_ADC_Result[i*2+1] & 0x0000FFFF);
	}
	Fft_Imagclear();																						//FFT虚部数组清零
	FFT();																									//快速傅里叶变换
	DC_RED = sqrt(Fft_Real[0]*Fft_Real[0]+Fft_Image[0]*Fft_Image[0]);										//求红光直流分量
	for(uint16_t i=0;i<128;i++)																				//数组中的ADC通道1采样值赋值给FFT实部数组
	{
		Fft_Real[i] = (uint16_t) (AC_ADC_Result[i*2+1] & 0x0000FFFF);
	}
	Fft_Imagclear();																						//FFT虚部数组清零
	FFT();																									//快速傅里叶变换
	AC_RED = sqrt(Fft_Real[1]*Fft_Real[1]+Fft_Image[1]*Fft_Image[1]);										//求红光交流分量
	PI_RED = (float) (AC_RED / DC_RED);																		//求红光PI的值
	
	//计算血氧饱和度
	R = (float) (PI_RED / PI_IR);																			//求R值
	SPO2 = (float) (110.0 - R * 25.0);																	    //求SPO2值
    if(SPO2 < 80)
    {
        SPO2 = 80;
    }
    if(SPO2 > 100)
    {
        SPO2 = 100;
    }
	R_Calt = 1;																								//设置Calt函数返回值状态为1

	//计算脉搏
	PULSE_number = 0;																						//采集的脉搏信号数量清0
	PULSE_Result = 0.0;																				        //采集的脉搏结果清0
	for(uint16_t i=0;i<128;i++)																				//在红光交流分量中过滤出脉搏信号
	{
		if(((uint16_t) (AC_ADC_Result[i*2+1] & 0x0000FFFF) > 1000) && ((i*2+1)-PULSE_No[PULSE_number-1]>13))		            //脉搏信号：ADC采集结果>1000且距离上一次位置间隔大于500
		{
			PULSE_R[PULSE_number] = (uint16_t) (AC_ADC_Result[i*2+1] & 0x0000FFFF);    			                                //保存脉搏信号
			PULSE_No[PULSE_number] = i * 2 + 1;																					//保存脉搏信号位置
			PULSE_number += 1;                                                                                                  //采集的脉搏信号数量加1
		}
	}
	if(PULSE_number >1)																											//如果脉搏信号数量为2个及以上
	{
		for(uint16_t i=0;i<PULSE_number-1 ;i++)																					//在红光交流分量中过滤出脉搏信号
		{
			PULSE_Result += (float) (60000.0 / ((PULSE_No[i+1] - PULSE_No[i]) * 39.0));					                        //脉搏计算：通过相邻两个脉搏信号的位置折算60秒脉搏数量并累加
		}
		PULSE_Result = (float) (PULSE_Result / (PULSE_number-1));											                    //用均值法修正脉搏计算结果
	}
	else
	{
		PULSE_Result = 0.0;																										//如果脉搏信号数量为0或1个：计算结果为0
	}
	IsCycleEnd = 1;																												//开启新一轮检测周期

//	printf("**************ADC采集数据**************\r\n");
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
//	printf("**************中间结果数据**************\r\n");
//	printf("DC_IR:%f\r\n",DC_IR);
//	printf("AC_IR:%f\r\n",AC_IR);
//	printf("DC_RED:%f\r\n",DC_RED);
//	printf("AC_RED:%f\r\n",AC_RED);
//	printf("PI_IR:%f\r\n",PI_IR);
//	printf("PI_RED:%f\r\n",PI_RED);
//  printf("R:%f\r\n",R);
//  printf("PULSE_number:%d\r\n",PULSE_number);
//  printf("PULSE_Result:%f\r\n",PULSE_Result);
//	printf("**************最终结果数据**************\r\n");
//	printf("SPO2:%f\r\n",SPO2);
	
//	//数据规整
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
    if(tmp_Direct == 0)                                                 //横屏
    {
        LCD_ShowIntNum(9,28,(uint16_t)SPO2,1);  					    //血氧
        LCD_ShowIntNum(117,28,(uint16_t)PULSE_Result,2);  			    //心率
        LCD_ShowFloatNum(64,40,PI_RED,WHITE,BLACK,16);  		 	    //PI
        LCD_Show_Power(56,0,1,POWER);  			                        //电量
        LCD_ShowBoxing(boxing_moni,1);                                  //横屏波形
    }
    else                                                                //竖屏        
    {
        LCD_ShowIntNum(9,24,(uint16_t)SPO2,1);  					    //血氧
        LCD_ShowIntNum(9,78,(uint16_t)PULSE_Result,2);  			    //心率
        LCD_ShowFloatNum(40,114,PI_RED,WHITE,BLACK,16);  		 	    //PI
        LCD_Show_Power(113,58,2,POWER);  			                    //电量
        LCD_ShowBoxing(boxing_moni,2);                                  //竖屏波形
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


