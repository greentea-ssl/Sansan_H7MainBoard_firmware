/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
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
#include "adc.h"
#include "fdcan.h"
#include "i2c.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include <stdio.h>

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

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */


#ifdef __GNUC__
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif /* __GNUC__ */
void __io_putchar(uint8_t ch)
{
	HAL_UART_Transmit(&huart6, &ch, 1, 1);
}

/**
 *
 */

void putGPIOState(void)
{


	printf("DIPSW: [1]:%d, [2]:%d, [3]:%d, [4]:%d\n",
			HAL_GPIO_ReadPin(DIPSW_1_GPIO_Port, DIPSW_1_Pin),
			HAL_GPIO_ReadPin(DIPSW_2_GPIO_Port, DIPSW_2_Pin),
			HAL_GPIO_ReadPin(DIPSW_3_GPIO_Port, DIPSW_3_Pin),
			HAL_GPIO_ReadPin(DIPSW_4_GPIO_Port, DIPSW_4_Pin)
			);

	printf("USER-SW: [0]:%d, [1]:%d\n",
			HAL_GPIO_ReadPin(USER_SW0_GPIO_Port, USER_SW0_Pin),
			HAL_GPIO_ReadPin(USER_SW1_GPIO_Port, USER_SW1_Pin)
			);

	printf("EMO:%d, DONE:%d\n",
			HAL_GPIO_ReadPin(EMO_GPIO_Port, EMO_Pin),
			HAL_GPIO_ReadPin(DONE_GPIO_Port, DONE_Pin)
			);


	printf("\e[3A");

}



void driveMotor_speed(uint8_t channel, float omega)
{

	uint32_t fdcan2TxMailbox;
	FDCAN_TxHeaderTypeDef fdcan2TxHeader;
	uint8_t fdcan2TxData[8];

	union _ctrlRef{
		struct{
			float fval;
		};
		struct{
			uint8_t byte[4];
		};
	}controlRef;

	fdcan2TxHeader.Identifier = 0x200 | ((channel & 0x07) << 5) | (0x01 << 2);
	fdcan2TxHeader.IdType = FDCAN_STANDARD_ID;
	fdcan2TxHeader.TxFrameType = FDCAN_DATA_FRAME;
	fdcan2TxHeader.DataLength = FDCAN_DLC_BYTES_4;
	fdcan2TxHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
	fdcan2TxHeader.BitRateSwitch = FDCAN_BRS_OFF;
	fdcan2TxHeader.FDFormat = FDCAN_CLASSIC_CAN;
	fdcan2TxHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
	fdcan2TxHeader.MessageMarker = 0;

	controlRef.fval = omega;

	fdcan2TxData[0] = controlRef.byte[0];
	fdcan2TxData[1] = controlRef.byte[1];
	fdcan2TxData[2] = controlRef.byte[2];
	fdcan2TxData[3] = controlRef.byte[3];

	//HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan2, &fdcan2TxHeader, fdcan2TxData);

	HAL_FDCAN_AddMessageToTxBuffer(&hfdcan2, &fdcan2TxHeader, fdcan2TxData, &fdcan2TxMailbox);




	return;
}




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
  MX_FDCAN1_Init();
  MX_ADC1_Init();
  MX_I2C2_Init();
  MX_SPI4_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_USART3_UART_Init();
  MX_USART6_UART_Init();
  MX_FDCAN2_Init();
  MX_UART5_Init();
  /* USER CODE BEGIN 2 */


  if(HAL_FDCAN_Start(&hfdcan2) != HAL_OK)
  {
	  Error_Handler();
  }


  printf("oppai..\n");


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {


	  HAL_Delay(100);

	  // putGPIOState();

	  driveMotor_speed(1, 10.0f);



    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
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
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

  /** Supply configuration update enable 
  */
  HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);
  /** Configure the main internal regulator output voltage 
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE0);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}
  /** Macro to configure the PLL clock source 
  */
  __HAL_RCC_PLL_PLLSOURCE_CONFIG(RCC_PLLSOURCE_HSE);
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 480;
  RCC_OscInitStruct.PLL.PLLP = 2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_1;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
  RCC_OscInitStruct.PLL.PLLFRACN = 0;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_USART3|RCC_PERIPHCLK_USART2
                              |RCC_PERIPHCLK_USART6|RCC_PERIPHCLK_FDCAN
                              |RCC_PERIPHCLK_USART1|RCC_PERIPHCLK_UART5
                              |RCC_PERIPHCLK_SPI4|RCC_PERIPHCLK_I2C2
                              |RCC_PERIPHCLK_ADC;
  PeriphClkInitStruct.PLL2.PLL2M = 4;
  PeriphClkInitStruct.PLL2.PLL2N = 80;
  PeriphClkInitStruct.PLL2.PLL2P = 2;
  PeriphClkInitStruct.PLL2.PLL2Q = 2;
  PeriphClkInitStruct.PLL2.PLL2R = 2;
  PeriphClkInitStruct.PLL2.PLL2RGE = RCC_PLL2VCIRANGE_1;
  PeriphClkInitStruct.PLL2.PLL2VCOSEL = RCC_PLL2VCOMEDIUM;
  PeriphClkInitStruct.PLL2.PLL2FRACN = 0;
  PeriphClkInitStruct.Spi45ClockSelection = RCC_SPI45CLKSOURCE_D2PCLK1;
  PeriphClkInitStruct.FdcanClockSelection = RCC_FDCANCLKSOURCE_PLL2;
  PeriphClkInitStruct.Usart234578ClockSelection = RCC_USART234578CLKSOURCE_D2PCLK1;
  PeriphClkInitStruct.Usart16ClockSelection = RCC_USART16CLKSOURCE_D2PCLK2;
  PeriphClkInitStruct.I2c123ClockSelection = RCC_I2C123CLKSOURCE_D2PCLK1;
  PeriphClkInitStruct.AdcClockSelection = RCC_ADCCLKSOURCE_PLL2;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
