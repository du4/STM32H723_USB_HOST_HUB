/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "usb_host.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "usbh_cdc.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
#ifdef __GNUC__
/* With GCC/RAISONANCE, small printf (option LD Linker->Libraries->Small printf
   set to 'Yes') calls __io_putchar() */
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif /* __GNUC__ */
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define USBHS_MAX_BULK_HS_PACKET_SIZE	512
#define USBHS_MAX_BULK_FS_PACKET_SIZE	64
#define BTN_DELAY	500000
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;

UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */
int packetSendCounter = 0;
int packetReceiveCounter = 0;
int bytesReceiveCounter = 0;
int btnCounter = 0;
const char* cdc_tx_buf = "I would like to share my experience to create test application and measure the USB performance for both FS and HS with external ULPI PHY (USB3300) on STM32F4 MCU series. To do that Olimex STM32-H405 board and USB3300 module was used as hardware. Because of high speed the connection between them was made with very short wires and using default pin connection case with USB3300 reset pin connected to PA6. In addition to use default USB FS Device pin-out USB_P was re-wired from PC4 to PA9. For debugging USART\r\n";
uint32_t TX_SIZE = 62;//sizeof(cdc_tx_buf);

uint8_t cdc_rx_buf[USBHS_MAX_BULK_FS_PACKET_SIZE];

PUTCHAR_PROTOTYPE{
  HAL_UART_Transmit(&huart3, (uint8_t *)&ch, 1, HAL_MAX_DELAY);
  return ch;
}


CDC_StateTypedef CDC_STATE = CDC_SEND;

extern USBH_HandleTypeDef hUsbHostHS;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
void MX_USB_HOST_Process(void);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void){
  /* USER CODE BEGIN 1 */
extern ApplicationTypeDef Appli_state;
  /* USER CODE END 1 */

  /* Enable I-Cache---------------------------------------------------------*/
  SCB_EnableICache();

  /* Enable D-Cache---------------------------------------------------------*/
  SCB_EnableDCache();

  /* MCU Configuration--------------------------------------------------------*/

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
  MX_USART3_UART_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_USB_HOST_Init();
  /* USER CODE BEGIN 2 */
//  if (HAL_TIM_Base_Start_IT(&htim3) != HAL_OK){
//      Error_Handler();
//  }

  printf("\n\r===========================================\n\rUSB HOST become to main loop.\r\n");

//  HAL_GPIO_WritePin(USB_FS_PWR_EN_GPIO_Port, USB_FS_PWR_EN_Pin, GPIO_PIN_RESET);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  HAL_TIM_Base_Start(&htim4);

  USBH_StatusTypeDef status = 0;
  while (1){
    /* USER CODE END WHILE */
    MX_USB_HOST_Process();

    /* USER CODE BEGIN 3 */
//    printf("MX_USB_HOST_Process duration = %d us\n\r", (int)__HAL_TIM_GET_COUNTER(&htim4));
//    HAL_TIM_Base_Stop(&htim4);
//    __HAL_TIM_SET_COUNTER(&htim4, 0);
//    btnCounter++;
//    if(btnCounter > BTN_DELAY){
//    	btnCounter = 0;
//	if (Appli_state == APPLICATION_READY){
//		switch (CDC_STATE){
//			case CDC_SEND:{
//				status = USBH_CDC_Transmit(&hUsbHostHS, (uint8_t *)cdc_tx_buf, 3);
//				CDC_STATE = CDC_BUSY;
//				break;
//			}
//			case CDC_RECEIVE:{
//				memset(cdc_rx_buf, 0, USBHS_MAX_BULK_PACKET_SIZE);
//				status = USBH_CDC_Receive(&hUsbHostHS, (uint8_t *)cdc_rx_buf, 62);
//				CDC_STATE = CDC_BUSY;
//			}
//			default:  break;
//			}
////    	if(status == USBH_OK) status = USBH_CDC_Receive(&hUsbHostHS, (uint8_t *)cdc_rx_buf, TX_SIZE);
////    	if(packetSendCounter != 0)	printf("send=%d.\n\r", packetSendCounter);
//	}
//    }

    if(HAL_GPIO_ReadPin(UserBtn_GPIO_Port, UserBtn_Pin) == GPIO_PIN_SET){
    	btnCounter++;
    	if(btnCounter >= BTN_DELAY){
			btnCounter = 0;
    		HAL_GPIO_TogglePin(YellowLed_GPIO_Port, YellowLed_Pin);
    		__HAL_TIM_SET_COUNTER(&htim4, 0);


    		status = USBH_CDC_Transmit(&hUsbHostHS, (uint8_t *)cdc_tx_buf, 1);
    		printf("USB packet has sent with status %d.\n\r", status);
    		memset(cdc_rx_buf, 0, USBHS_MAX_BULK_FS_PACKET_SIZE);
    		if(status == USBH_OK) USBH_CDC_Receive(&hUsbHostHS, (uint8_t *)cdc_rx_buf, TX_SIZE);
    	}
//    }else{
//    	HAL_GPIO_WritePin(ULPI_RES_GPIO_Port, ULPI_RES_Pin, RESET);
    }else{
    	btnCounter = 0;
    }


//    if(Appli_state == APPLICATION_READY){
//    	USBH_StatusTypeDef status   = USBH_CDC_Transmit(&hUsbHostHS, cdc_tx_buf, 64);
//    }
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Supply configuration update enable
  */
  HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE0);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI48|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.HSI48State = RCC_HSI48_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 68;
  RCC_OscInitStruct.PLL.PLLP = 1;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_3;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
  RCC_OscInitStruct.PLL.PLLFRACN = 6144;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 27499;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 10000;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 274;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 65535;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  huart3.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart3.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart3.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart3, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart3, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(ULPI_RES_GPIO_Port, ULPI_RES_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(RedLed_GPIO_Port, RedLed_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(USB_FS_PWR_EN_GPIO_Port, USB_FS_PWR_EN_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(YellowLed_GPIO_Port, YellowLed_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : UserBtn_Pin */
  GPIO_InitStruct.Pin = UserBtn_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(UserBtn_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : ULPI_RES_Pin RedLed_Pin */
  GPIO_InitStruct.Pin = ULPI_RES_Pin|RedLed_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : USB_FS_PWR_EN_Pin */
  GPIO_InitStruct.Pin = USB_FS_PWR_EN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(USB_FS_PWR_EN_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : YellowLed_Pin */
  GPIO_InitStruct.Pin = YellowLed_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(YellowLed_GPIO_Port, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void USBH_CDC_ReceiveCallback(USBH_HandleTypeDef *phost){
	int duration = (int)__HAL_TIM_GET_COUNTER(&htim4);
	int size = sizeof(cdc_rx_buf);
	printf("Got USB packet, duration=%d us; size=%d; %s\n\r", duration, size, cdc_rx_buf);
//	if(size == USBHS_MAX_BULK_PACKET_SIZE)

	CDC_HandleTypeDef *CDC_Handle = (CDC_HandleTypeDef *) hUsbHostHS.pActiveClass->pData;
	bytesReceiveCounter += CDC_Handle->RxDataLength;

	if (CDC_STATE == CDC_BUSY) CDC_STATE = CDC_SEND;
	packetReceiveCounter++;

}
/**
  * @brief  The function informs user that data have been received
  *  @param  pdev: Selected device
  * @retval None
  */
void USBH_CDC_TransmitCallback(USBH_HandleTypeDef *phost)
{
  /* Prevent unused argument(s) compilation warning */
	if (CDC_STATE == CDC_BUSY) CDC_STATE = CDC_RECEIVE;
	packetSendCounter++;
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
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
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
