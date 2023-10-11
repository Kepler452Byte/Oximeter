#include "Debug.h"

//    使用printf需先配置UART
//    UART_Configuration();
//	  后面可使用printf函数
//    printf("\r\nCW32F030 UART Printf Example\r\n");



void UART_Configuration(void);

#ifdef __GNUC__
    /* With GCC/RAISONANCE, small printf (option LD Linker->Libraries->Small printf
    set to 'Yes') calls __io_putchar() */
    #define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
    #define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif /* __GNUC__ */





/**
 * @brief 配置UART
 *
 */
void UART_Configuration(void)
{
	//外设时钟使能
    RCC_AHBPeriphClk_Enable(DEBUG_USART_GPIO_CLK, ENABLE);
    DEBUG_USART_APBClkENx(DEBUG_USART_CLK, ENABLE);
	
	//配置GPIO口
	GPIO_InitTypeDef GPIO_InitStructure;

    //UART TX RX 复用
    DEBUG_USART_AFTX;
    DEBUG_USART_AFRX;

    GPIO_InitStructure.Pins = DEBUG_USART_TX_GPIO_PIN;
    GPIO_InitStructure.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStructure.IT = GPIO_IT_NONE;
    GPIO_Init(DEBUG_USART_TX_GPIO_PORT, &GPIO_InitStructure);

    GPIO_InitStructure.Pins = DEBUG_USART_RX_GPIO_PIN;
    GPIO_InitStructure.Mode = GPIO_MODE_INPUT_PULLUP;
    GPIO_Init(DEBUG_USART_RX_GPIO_PORT, &GPIO_InitStructure);
	
	
	//初始化USART
    USART_InitTypeDef USART_InitStructure;

    USART_InitStructure.USART_BaudRate = DEBUG_USART_BaudRate;
    USART_InitStructure.USART_Over = USART_Over_16;
    USART_InitStructure.USART_Source = USART_Source_PCLK;
    USART_InitStructure.USART_UclkFreq = DEBUG_USART_UclkFreq;
    USART_InitStructure.USART_StartBit = USART_StartBit_FE;
    USART_InitStructure.USART_StopBits = USART_StopBits_1;
    USART_InitStructure.USART_Parity = USART_Parity_No ;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
    USART_Init(DEBUG_USARTx, &USART_InitStructure);
}

/**
 * @brief Retargets the C library printf function to the USART.
 *
 */
PUTCHAR_PROTOTYPE
{
    USART_SendData_8bit(DEBUG_USARTx, (uint8_t)ch);

    while (USART_GetFlagStatus(DEBUG_USARTx, USART_FLAG_TXE) == RESET);

    return ch;
}


//下方代码不可删除，为系统报错函数。
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

