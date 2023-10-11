/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    interrupts_cw32l031.h
  * @brief   This file contains the headers of the interrupt handlers.
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
#ifndef __INTERRUPTS_CW32L031_H
#define __INTERRUPTS_CW32L031_H

#ifdef __cplusplus
extern "C" {
#endif

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

/* USER CODE END EM */


/* Exported functions prototypes ---------------------------------------------*/

void NMI_Handler(void);
void HardFault_Handler(void);
void SVC_Handler(void);
void PendSV_Handler(void);
void WDT_IRQHandler(void);
void LVD_IRQHandler(void);
void RTC_IRQHandler(void);
void FLASHRAM_IRQHandler(void);
void RCC_IRQHandler(void);
void GPIOA_IRQHandler(void);
void GPIOB_IRQHandler(void);
void GPIOC_IRQHandler(void);
void GPIOF_IRQHandler(void);
void DMACH1_IRQHandler(void);
void DMACH23_IRQHandler(void);
void DMACH4_IRQHandler(void);
void ADC_IRQHandler(void);
void ATIM_IRQHandler(void);
void VC1_IRQHandler(void);
void VC2_IRQHandler(void);
void GTIM1_IRQHandler(void);
void GTIM2_IRQHandler(void);
void BTIM1_IRQHandler(void);
void BTIM2_IRQHandler(void);
void BTIM3_IRQHandler(void);
void I2C1_IRQHandler(void);
void SPI1_IRQHandler(void);
void UART1_IRQHandler(void);
void UART2_IRQHandler(void);
void UART3_IRQHandler(void);
void AWT_IRQHandler(void);
void FAULT_IRQHandler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */


#ifdef __cplusplus
}
#endif

#endif /* __INTERRUPTS_CW32L031_H */

/************************ (C) COPYRIGHT CW *****END OF FILE****/
