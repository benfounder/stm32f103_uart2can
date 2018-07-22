
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2018 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f1xx_hal.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
CAN_HandleTypeDef hcan;
UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_usart2_tx;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
uint8_t hUartRxBuf[20];
uint8_t hUartTxBuf[20];
CanRxMsgTypeDef hCanRxMsg;
CanTxMsgTypeDef hCanTxMsg;
__IO uint8_t uart_ready_for_next = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_CAN_Init(void);
static void MX_USART2_UART_Init(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  *
  * @retval None
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
  HAL_StatusTypeDef ret;
  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_CAN_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
  CAN_FilterConfTypeDef sFilterConfig;
  sFilterConfig.FilterNumber = 0;
  sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
  sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
  sFilterConfig.FilterIdHigh = 0x0000;
  sFilterConfig.FilterIdLow = 0x0000;
  sFilterConfig.FilterMaskIdHigh = 0;
  sFilterConfig.FilterMaskIdLow = 0;
  sFilterConfig.FilterFIFOAssignment = CAN_FIFO0;
  sFilterConfig.FilterActivation = ENABLE;
  sFilterConfig.BankNumber = 14;
  if (HAL_CAN_ConfigFilter(&hcan, &sFilterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  hcan.pRxMsg = &hCanRxMsg;
  hcan.pTxMsg = &hCanTxMsg;
  ret = HAL_CAN_Receive_IT(&hcan, CAN_FIFO0);
  if (ret != HAL_OK)
  {
    Error_Handler();
  }
  uart_ready_for_next = 1;
  /* USER CODE END 2 */

  /* Infinite loop */

  /* USER CODE BEGIN WHILE */
  while (1)
  { 
  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */
    if (1 == uart_ready_for_next) {
      HAL_UART_Receive_IT(&huart2, hUartRxBuf, 1);
      uart_ready_for_next = 0;
    }
  } 
  /* USER CODE END 3 */

}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL6;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* CAN init function */
static void MX_CAN_Init(void)
{
  hcan.Instance = CAN1;
  hcan.Init.Prescaler = 3;
  hcan.Init.Mode = CAN_MODE_NORMAL;
  hcan.Init.SJW = CAN_SJW_1TQ;
  hcan.Init.BS1 = CAN_BS1_9TQ;
  hcan.Init.BS2 = CAN_BS2_2TQ;
  hcan.Init.TTCM = DISABLE;
  hcan.Init.ABOM = ENABLE;
  hcan.Init.AWUM = DISABLE;
  hcan.Init.NART = DISABLE;
  hcan.Init.RFLM = DISABLE;
  hcan.Init.TXFP = DISABLE;
  if (HAL_CAN_Init(&hcan) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }
}

/* USART2 init function */
static void MX_USART2_UART_Init(void)
{

  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/** 
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void) 
{
  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel7_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel7_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel7_IRQn);

}

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
*/
static void MX_GPIO_Init(void)
{

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

}

/* USER CODE BEGIN 4 */
void HAL_CAN_RxCpltCallback(CAN_HandleTypeDef* hcan)
{
  HAL_StatusTypeDef ret;
  CanRxMsgTypeDef *pRxMsg = hcan->pRxMsg;
  uint32_t i;

  if ((CAN_ID_STD == pRxMsg->IDE) && (CAN_RTR_DATA == pRxMsg->RTR))
  {
    hUartTxBuf[0] = 0xB7;
    hUartTxBuf[1] = 0xB6;
    hUartTxBuf[2] = (pRxMsg->StdId >> 8) & 0xFF;
    hUartTxBuf[3] = pRxMsg->StdId & 0xFF;
    hUartTxBuf[4] = pRxMsg->DLC;
    for (i=0;i<pRxMsg->DLC;i++)
    {
      hUartTxBuf[5+i] = pRxMsg->Data[i];
    }
    ret = HAL_UART_Transmit_DMA(&huart2, hUartTxBuf, pRxMsg->DLC + 5);
    if (ret != HAL_OK)
    {
      Error_Handler();
    }
  }
	
	ret = HAL_CAN_Receive_IT(hcan, CAN_FIFO0);
	if (ret != HAL_OK)
	{
		Error_Handler();
	}
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
  huart->gState = HAL_UART_STATE_READY;
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  static uint8_t state_mach;
  static uint32_t index;
  uint8_t data;

  data = hUartRxBuf[0];
  if (0 == state_mach) {
    if (0xB7U == data)
      state_mach = 1;
  } else if (1 == state_mach) {
    if (0xB6U == data)
      state_mach = 2;
  } else if (2 == state_mach) {
    hCanTxMsg.StdId = ((uint32_t)data) << 8;
    state_mach = 3;
  } else if (3 == state_mach) {
    hCanTxMsg.StdId |= (uint32_t)data;
    state_mach = 4;
  } else if (4 == state_mach) {
    if (data > 8)
      state_mach = 0;
    else {
      hCanTxMsg.DLC = data;
      state_mach = 5;
      index = 0;
    }
  } else if (5 == state_mach) {
    hCanTxMsg.Data[index++] = data;
    if (hCanTxMsg.DLC <= index) {
      hCanTxMsg.IDE = CAN_ID_STD;
      hCanTxMsg.RTR = CAN_RTR_DATA;
      state_mach = 0;
      HAL_CAN_Transmit_IT(&hcan);
    }
  }
  uart_ready_for_next = 1;
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  file: The file name as string.
  * @param  line: The line in file as a number.
  * @retval None
  */
void _Error_Handler(char *file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
