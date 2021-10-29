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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "LCD16x2/LCD16x2.h"
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
RTC_HandleTypeDef hrtc;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_RTC_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
uint8_t currentMode = 0;
#define MODE_NORMAL			0
#define MODE_CHNG_CLK		1
#define MODE_CHNG_MIN		2
#define MODE_CHNG_SEC		3
#define MODE_CHNG_YEAR	4
#define MODE_CHNG_MONTH	5
#define MODE_CHNG_DAY		6
#define MODE_CHNG_WEEK	7

uint8_t h,m,s,y,mon,d,wk;
RTC_TimeTypeDef tmptim = {.Hours=0, .Minutes=0, .Seconds=0};
RTC_DateTypeDef tmpdat = {.Year=0, .Month=0, .Date=0, .WeekDay=0};

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  if(GPIO_Pin == GPIO_PIN_1)
	{
		if(currentMode==MODE_NORMAL)
		{ 
			currentMode = MODE_CHNG_CLK;
		}
		else if(currentMode == MODE_CHNG_CLK)
		{
			currentMode = MODE_CHNG_MIN;
		}
		else if(currentMode == MODE_CHNG_MIN)
		{
			currentMode = MODE_CHNG_SEC;
		}
		else if(currentMode == MODE_CHNG_SEC)
		{
			currentMode = MODE_CHNG_DAY;
		}
		else if(currentMode == MODE_CHNG_DAY)
		{
			currentMode = MODE_CHNG_MONTH;
		}
		else if(currentMode == MODE_CHNG_MONTH)
		{
			currentMode = MODE_CHNG_YEAR;
		}
		else if(currentMode == MODE_CHNG_YEAR)
		/*{
			currentMode = MODE_CHNG_WEEK;
		}
		else if(currentMode == MODE_CHNG_WEEK)
		*/{
			currentMode = MODE_NORMAL;
			LCD_HideCursor();
		}
	}
	else if(GPIO_Pin == GPIO_PIN_2)
	{
		if(currentMode==MODE_NORMAL)
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, 0);	//turn off alarm
		else if(currentMode==MODE_CHNG_CLK)
		{
			h++;
			if(h==24) h=0;
			tmptim.Hours = h;
			tmptim.Minutes = m;
			tmptim.Seconds = s;
			HAL_RTC_SetTime(&hrtc,&tmptim,RTC_FORMAT_BIN);
		}
		else if(currentMode==MODE_CHNG_MIN)
		{
			m++;
			if(m==60) m=0;
			tmptim.Hours = h;
			tmptim.Minutes = m;
			tmptim.Seconds = s;
			HAL_RTC_SetTime(&hrtc,&tmptim,RTC_FORMAT_BIN);
		}
		else if(currentMode==MODE_CHNG_SEC)
		{
			s++;
			if(s==60) s=0;
			tmptim.Hours = h;
			tmptim.Minutes = m;
			tmptim.Seconds = s;
			HAL_RTC_SetTime(&hrtc,&tmptim,RTC_FORMAT_BIN);
		}
		else if(currentMode==MODE_CHNG_DAY) 
		{
			d++;
			if(d==32) d=1;
			tmpdat.Year = y;
			tmpdat.Month = mon;
			tmpdat.Date = d;
			//tmpdat.WeekDay = wk;
			HAL_RTC_SetDate(&hrtc,&tmpdat,RTC_FORMAT_BIN);
		}
		else if(currentMode==MODE_CHNG_MONTH) 
		{
			mon++;
			if(mon==13) mon=1;
			tmpdat.Year = y;
			tmpdat.Month = mon;
			tmpdat.Date = d;
			//tmpdat.WeekDay = wk;
			HAL_RTC_SetDate(&hrtc,&tmpdat,RTC_FORMAT_BIN);
		}
		else if(currentMode==MODE_CHNG_YEAR) 
		{
			y++;
			tmpdat.Year = y;
			tmpdat.Month = mon;
			tmpdat.Date = d;
			//tmpdat.WeekDay = wk;
			HAL_RTC_SetDate(&hrtc,&tmpdat,RTC_FORMAT_BIN);
		}
		/*else if(currentMode==MODE_CHNG_WEEK) 
		{
			wk++;
			if(wk==8) wk=1;
			tmpdat.Year = y;
			tmpdat.Month = mon;
			tmpdat.Date = d;
			tmpdat.WeekDay = wk;
			HAL_RTC_SetDate(&hrtc,&tmpdat,RTC_FORMAT_BIN);
		}*/
	}
}

void getTimeAndDate(uint8_t* h, uint8_t* m, uint8_t* s, uint8_t* y, uint8_t* mon, uint8_t* d, uint8_t* wk)
{
	RTC_TimeTypeDef timeh = {.Hours = 0, .Minutes= 0, .Seconds = 0};
	HAL_RTC_GetTime(&hrtc, &timeh, RTC_FORMAT_BIN);
	*h = timeh.Hours;
	*m = timeh.Minutes;
	*s = timeh.Seconds;
	
	RTC_DateTypeDef dateh;
	HAL_RTC_GetDate(&hrtc, &dateh, RTC_FORMAT_BIN);
	*y = dateh.Year;
	*mon = dateh.Month;
	*d = dateh.Date;
	*wk = dateh.WeekDay;
}

void cursorMove()
{
		if(currentMode == MODE_CHNG_CLK)
			LCD_Set_Cursor(1,2);
		else if(currentMode == MODE_CHNG_MIN)
			LCD_Set_Cursor(1,5);
		else if(currentMode == MODE_CHNG_SEC)
			LCD_Set_Cursor(1,8);
		else if(currentMode == MODE_CHNG_DAY)
			LCD_Set_Cursor(2,2);
		else if(currentMode == MODE_CHNG_MONTH)
			LCD_Set_Cursor(2,5);
		else if(currentMode == MODE_CHNG_YEAR)
			LCD_Set_Cursor(2,10);
		else if(currentMode == MODE_CHNG_WEEK)
			LCD_Set_Cursor(1,14);
		LCD_ShowCursor();
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
  MX_RTC_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  LCD_Init();
	LCD_Set_Cursor(1,1);
	char buf[16];
	const char* DayNames [] = {"", "MON", "TUE", "WED", "THU", "FRI", "SAT", "SUN"};
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
		
		//if(currentMode == 0)
		{
			getTimeAndDate(&h, &m, &s, &y, &mon, &d, &wk);
			sprintf(buf, "%02d:%02d:%02d     %s", h, m ,s, DayNames[wk]);
			LCD_Set_Cursor(1,1);
			LCD_Write_String(buf);
			sprintf(buf, "%02d/%02d/20%02d",d,mon,y);
			LCD_Set_Cursor(2,1);
			LCD_Write_String(buf);
			if(currentMode!=MODE_NORMAL)
				cursorMove();
			HAL_Delay(500);
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_LSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
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
}

/**
  * @brief RTC Initialization Function
  * @param None
  * @retval None
  */
static void MX_RTC_Init(void)
{

  /* USER CODE BEGIN RTC_Init 0 */

  /* USER CODE END RTC_Init 0 */

  RTC_TimeTypeDef sTime = {0};
  RTC_DateTypeDef DateToUpdate = {0};

  /* USER CODE BEGIN RTC_Init 1 */

  /* USER CODE END RTC_Init 1 */
  /** Initialize RTC Only
  */
  hrtc.Instance = RTC;
  hrtc.Init.AsynchPrediv = RTC_AUTO_1_SECOND;
  hrtc.Init.OutPut = RTC_OUTPUTSOURCE_ALARM;
  if (HAL_RTC_Init(&hrtc) != HAL_OK)
  {
    Error_Handler();
  }

  /* USER CODE BEGIN Check_RTC_BKUP */

  /* USER CODE END Check_RTC_BKUP */

  /** Initialize RTC and set the Time and Date
  */
  sTime.Hours = 0x0;
  sTime.Minutes = 0x0;
  sTime.Seconds = 0x0;

  if (HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BCD) != HAL_OK)
  {
    Error_Handler();
  }
  DateToUpdate.WeekDay = RTC_WEEKDAY_MONDAY;
  DateToUpdate.Month = RTC_MONTH_JANUARY;
  DateToUpdate.Date = 0x1;
  DateToUpdate.Year = 0x0;

  if (HAL_RTC_SetDate(&hrtc, &DateToUpdate, RTC_FORMAT_BCD) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN RTC_Init 2 */

  /* USER CODE END RTC_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_10|GPIO_PIN_11
                          |GPIO_PIN_12|GPIO_PIN_13, GPIO_PIN_RESET);

  /*Configure GPIO pins : PB1 PB2 */
  GPIO_InitStruct.Pin = GPIO_PIN_1|GPIO_PIN_2;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PB10 */
  GPIO_InitStruct.Pin = GPIO_PIN_10;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PA8 PA9 PA10 PA11
                           PA12 PA13 */
  GPIO_InitStruct.Pin = GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_10|GPIO_PIN_11
                          |GPIO_PIN_12|GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI1_IRQn, 2, 0);
  HAL_NVIC_EnableIRQ(EXTI1_IRQn);

  HAL_NVIC_SetPriority(EXTI2_IRQn, 3, 0);
  HAL_NVIC_EnableIRQ(EXTI2_IRQn);

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
