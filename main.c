/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "can.h"
#include "rtc.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
uint32_t red_channel_value;
uint32_t green_channel_value;
uint32_t blue_channel_value;
int self_can_id;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

int can_id(void);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
static uint8_t recv_data;

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

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
  MX_RTC_Init();
  MX_CAN_Init();
  MX_TIM2_Init();
  MX_USART1_UART_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  /* USER CODE BEGIN 2 */

  CAN_FilterTypeDef can_filter; //defining the can filter type
  can_filter.FilterIdHigh = 0xFFFF; // define the filter identifcation number
  can_filter.FilterIdLow =0x000; // define the fileter identaificaiton number 
  can_filter.FilterFIFOAssignment = CAN_FILTER_FIFO0;
  can_filter.FilterMaskIdHigh = 0xFFF;
  can_filter.FilterBank = 3;
  can_filter.FilterMode = 0x00000001U;
  can_filter.FilterScale = CAN_FILTERSCALE_32BIT;
  can_filter.SlaveStartFilterBank = 8;
  can_filter.FilterActivation = CAN_FILTER_ENABLE;
  can_filter.SlaveStartFilterBank = 8;
  HAL_CAN_ConfigFilter(&hcan, &can_filter); //Configures the CAN reception filter according to the specified parameters in the CAN_FilterInitStruct.
  HAL_CAN_Start(&hcan); // starts the CAN module 
  // HAL_TIM_Base_Start(&htim2); //starts the TIM base generation in interrupt mode
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);
  htim2.Instance->CCR1 = 4095; //compare register 1. Contains the duration of the T-On period of the on-going pulse
  htim2.Instance->CCR2 = 4095; // compare register 2. Contains the duration of the t-on period of the on-going pulse
  htim2.Instance->CCR3 = 4095; // compare register 3. Contains the duration of the T- on period of teh on-going pulse
  uint8_t data[8]; //array to store the CAN frame message
  self_can_id = can_id();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
uint8_t intensity = 0;

  while (1)
    {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    htim2.Instance->CCR3 = 4095;
    htim2.Instance->CCR1 = 2048;
    HAL_Delay(1000);
    htim2.Instance->CCR1 = 4095;
    htim2.Instance->CCR2 = 2048;
    HAL_Delay(1000);
    htim2.Instance->CCR2 = 4095;
    htim2.Instance->CCR3 = 2048;
    HAL_Delay(1000);

    uint32_t can_frame = HAL_CAN_GetRxFifoFillLevel(&hcan, can_filter.FilterFIFOAssignment); //returns the Rx FIFO fill level
  
    if (can_frame > 0)
    {    
    CAN_RxHeaderTypeDef pHeader;
    HAL_StatusTypeDef CAN_status = HAL_CAN_GetRxMessage(&hcan, can_filter.FilterFIFOAssignment, &pHeader, data); //Gets a CAN frame from the Rx FIFO zone into the message RAM.

    if(CAN_status == HAL_OK // checks to see if the hcan is not null
    && pHeader.RTR == CAN_RTR_DATA //Specifies the type of frame for the message that will be transmitted.
    && pHeader.StdId == (self_can_id = can_id()) )  //Specifies theh standard identifier
      {
            red_channel_value = data[0] << 8;
            red_channel_value |= data[1];
            green_channel_value = data[2] << 8;
            green_channel_value |= data[3];
            blue_channel_value = data[4] << 8;
            blue_channel_value |= data[5];
            htim2.Instance->CCR1 = red_channel_value; //compare register 1. Contains the duration of the T-On period of the on-going pulse
            htim2.Instance->CCR2 = green_channel_value; // compare register 2. Contains the duration of the t-on period of the on-going pulse
            htim2.Instance->CCR3 = blue_channel_value; // compare register 3. Contains the duration of the T- on period of teh on-going pulse
          }
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
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSI|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_RTC;
  PeriphClkInit.RTCClockSelection = RCC_RTCCLKSOURCE_LSI;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
  HAL_RCC_MCOConfig(RCC_MCO, RCC_MCO1SOURCE_PLLCLK, RCC_MCODIV_1);
}

/* USER CODE BEGIN 4 */
int can_id(void)
{
 int retrieval = 0;
 if(HAL_GPIO_ReadPin(CAN_ID_1_GPIO_Port, CAN_ID_1_Pin) == 1){
  retrieval  += 1;
 }

 if(HAL_GPIO_ReadPin(CAN_ID_2_GPIO_Port, CAN_ID_2_Pin) == 1){
  retrieval  += 2;
 }

 if(HAL_GPIO_ReadPin(CAN_ID_4_GPIO_Port, CAN_ID_4_Pin) == 1){
  retrieval  += 4;
 }
 
 if(HAL_GPIO_ReadPin(CAN_ID_8_GPIO_Port, CAN_ID_8_Pin) == 1){
  retrieval  += 8;
 }

 if(HAL_GPIO_ReadPin(CAN_ID_16_GPIO_Port, CAN_ID_16_Pin) == 1){
  retrieval  += 16;
 }

 if(HAL_GPIO_ReadPin(CAN_ID_32_GPIO_Port, CAN_ID_32_Pin) == 1){
  retrieval  += 32;
 }

 if(HAL_GPIO_ReadPin(CAN_ID_64_GPIO_Port, CAN_ID_64_Pin) == 1){
  retrieval  += 64;
 }

 if(HAL_GPIO_ReadPin(CAN_ID_128_GPIO_Port, CAN_ID_128_Pin) == 1){
  retrieval  += 128;
 }

  return retrieval;
}

/* USER CODE END 4 */

 /**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM1 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM1) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
