/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 CW.
  * All rights reserved.</center></h2>
  *
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
//#include "base_types.h"
#include "cw32l031.h"
#include "system_cw32l031.h"
#include "interrupts_cw32l031.h"

#include "cw32l031_adc.h"	
#include "cw32l031_atim.h"	
#include "cw32l031_awt.h"	
#include "cw32l031_btim.h"	
#include "cw32l031_crc.h"	
#include "cw32l031_debug.h"	
#include "cw32l031_digitalsign.h"	
#include "cw32l031_dma.h"	
#include "cw32l031_flash.h"	
#include "cw32l031_gpio.h"	
#include "cw32l031_gtim.h"	
#include "cw32l031_i2c.h"
#include "cw32l031_iwdt.h"	
#include "cw32l031_lvd.h"	
#include "cw32l031_pwr.h"	
#include "cw32l031_ram.h"	
#include "cw32l031_rcc.h"
#include "cw32l031_rtc.h"	
#include "cw32l031_spi.h"	
#include "cw32l031_systick.h"	
#include "cw32l031_uart.h"	
#include "cw32l031_wwdt.h"

//#include "Debug.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
/* USER CODE END Includes */


/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */
/* USER CODE END ET */


/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */
/* USER CODE END EC */


/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */
//宏定义
//define bsp_LED_GPIO
#define bsp_LED01_port 		        CW_GPIOB
#define bsp_LED01_pin				GPIO_PIN_7
#define bsp_LED02_port  		    CW_GPIOB
#define bsp_LED02_pin				GPIO_PIN_8
#define bsp_LED03_port  		    CW_GPIOB
#define bsp_LED03_pin				GPIO_PIN_9
#define bsp_LED04_port  		    CW_GPIOB
#define bsp_LED04_pin				GPIO_PIN_10
//define bsp_KEY_GPIO
#define bsp_KEY01_port  		    CW_GPIOB
#define bsp_KEY01_pin				GPIO_PIN_3
#define bsp_KEY02_port  		    CW_GPIOB
#define bsp_KEY02_pin				GPIO_PIN_4
#define bsp_KEY03_port  		    CW_GPIOB
#define bsp_KEY03_pin				GPIO_PIN_5
#define bsp_KEY04_port 		        CW_GPIOB
#define bsp_KEY04_pin				GPIO_PIN_6
//define bsp_LCD_GPIO
#define bsp_RS_port  				CW_GPIOA
#define bsp_RS_pin					GPIO_PIN_2
#define bsp_RST_port  			    CW_GPIOA
#define bsp_RST_pin					GPIO_PIN_3
#define bsp_SDA_port  			    CW_GPIOA
#define bsp_SDA_pin					GPIO_PIN_4
#define bsp_SCL_port  			    CW_GPIOA
#define bsp_SCL_pin					GPIO_PIN_5
#define bsp_CS_port  				CW_GPIOA
#define bsp_CS_pin					GPIO_PIN_8
#define bsp_PWM_port  			    CW_GPIOA
#define bsp_PWM_pin					GPIO_PIN_6
//define bsp_BEEP_GPIO
#define bsp_BEEP_port  			    CW_GPIOA
#define bsp_BEEP_pin				GPIO_PIN_7
//define bsp_SEND_GPIO
#define bsp_IN1_port  			    CW_GPIOF
#define bsp_IN1_pin					GPIO_PIN_6
#define bsp_IN2_port  			    CW_GPIOF
#define bsp_IN2_pin					GPIO_PIN_7
#define bsp_DAC1_port  			    CW_GPIOB
#define bsp_DAC1_pin				GPIO_PIN_14
#define bsp_DAC2_port  			    CW_GPIOB
#define bsp_DAC2_pin				GPIO_PIN_15
//define bsp_RECEIVE_GPIO
#define bsp_ADCIN0_port			    CW_GPIOA
#define bsp_ADCIN0_pin			    GPIO_PIN_0
#define bsp_ADCIN1_port			    CW_GPIOA
#define bsp_ADCIN1_pin			    GPIO_PIN_1
//#define bsp_DAC3_port 
//#define bsp_DAC3_pin	
//define bsp_FLASH_GPIO
//#define bsp_SPICS_port 			CW_GPIOB
//#define bsp_SPICS_pin				GPIO_PIN_12
//#define bsp_SPISCK_port 		    CW_GPIOB
//#define bsp_SPISCK_pin			GPIO_PIN_13
//#define bsp_SPIMISO_port 		    CW_GPIOB
//#define bsp_SPIMISO_pin			GPIO_PIN_14
//#define bsp_SPIMOSI_port 		    CW_GPIOB
//#define bsp_SPIMOSI_pin			GPIO_PIN_15
//define bsp_RAM_GPIO
//#define bsp_I2CSCL_port 		    CW_GPIOA
//#define bsp_I2CSCL_pin			GPIO_PIN_9
//#define bsp_I2CSDA_port 		    CW_GPIOA
//#define bsp_I2CSDA_pin			GPIO_PIN_10

#define ADC_cnt  2       				//ADC连续采样次数

/* USER CODE END EM */


/* Exported functions prototypes ---------------------------------------------*/
/* USER CODE BEGIN EFP */
//void LED_Init(void);
void SYS_init(void);
void RCC_init(void);
void GPIO_init(void);
void BTIM_init(void);
void GTIM_init(void);
void NVIC_init(void);
void ADC_Configuration(void);
void DMA_Configration(void);
void UART2_init(void);
void BTIM1_IRQHandlerCallback(void);
//void GTIM1_IRQHandlerCallback(void);
//void GTIM2_IRQHandlerCallback(void);
void KEY_IRQHandlerCallback(void);
//void ADC_IRQHandlerCallback(void);
void Calt(void);
void DMACH1_IRQHandlerCallback(void);
void DMACH23_IRQHandlerCallback(void);
void LCD_show(uint8_t tmp_Direct);
/* USER CODE END EFP */


/* Private defines -----------------------------------------------------------*/
/* USER CODE BEGIN Private defines */
/* USER CODE END Private defines */


#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT CW *****END OF FILE****/


