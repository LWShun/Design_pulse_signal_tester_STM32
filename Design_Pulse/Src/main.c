/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2023 STMicroelectronics.
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
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "lcd.h"
#include "stdio.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
__IO uint32_t keyslow = 0;
__IO uint32_t ledslow = 0;
__IO uint32_t lcdslow = 0;
__IO uint32_t uartslow = 0;
__IO uint32_t led_bling = 0;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
uint8_t uk_val,uk_down,uk_up,uk_old;
uint8_t Lcd_string[21];
uint8_t ucled;
//*ADC测量相关变量

float adc_voltage = 0;
uint16_t adc_cnt = 0;
uint16_t adc_max;

//*使用输入捕获测量频率相关变量
uint16_t PWM1_T_Count;
uint16_t PWM1_D_Count;
float PWM1_Duty;


uint8_t tim_flag=0;
uint32_t PWM_RisingCount;
uint32_t PWM_FallingCount;
float max=0;
float min=0.05;
float vpp=0;
float duty=0;

//*串口相关变量
uint8_t rx_buffer;
uint8_t str[40];
uint8_t counter;

//*外部计数器模式测量频率相关频率
uint32_t FQ; 			// 频率
float Time;				//周期
uint16_t CNT_2 = 0; 		// 超出65536位后 用这个记录超过的次数
uint16_t CNT_1 = 0; 		// 0-65536内用这个记录




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
	void KEY_Proc(void);
	void LCD_Proc(void);
	LCD_Init();
	LCD_Clear(White); 
	LCD_SetTextColor(Blue);
	LCD_SetBackColor(White);
	HAL_UART_Receive_IT(&huart2,(uint8_t *)(&rx_buffer),1);
	
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_ADC1_Init();
  MX_TIM2_Init();
  MX_TIM4_Init();
  /* USER CODE BEGIN 2 */
  /* 启动定时器 */

  HAL_TIM_Base_Start_IT(&htim4);
//  /* 启动定时器通道输入捕获并开启中断 */
    HAL_TIM_Base_Start(&htim2);
	HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_1);												 
	HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_2);


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	 adc_voltage = (((float)((get_ADC()*3.3)/4096)));
	 LED_Disp(0x00);
	 KEY_Proc();
	 LCD_Proc();
		if(tim_flag)
		{
			tim_flag=0;
			vpp=max-min;
			sprintf(str, "Fq:%uHz\r\n",72000000/PWM_RisingCount);
			HAL_UART_Transmit(&huart1,(unsigned char *)str, strlen(str), 50);
			sprintf(str, "Duty:%uHz\r\n",duty * 100);
			HAL_UART_Transmit(&huart1,(unsigned char *)str, strlen(str), 50);

         max=0;
		 min=0;
	
			
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

  /** Configure the main internal regulator output voltage
  */
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV2;
  RCC_OscInitStruct.PLL.PLLN = 12;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the peripherals clocks
  */
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1|RCC_PERIPHCLK_USART2
                              |RCC_PERIPHCLK_ADC12;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK2;
  PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
  PeriphClkInit.Adc12ClockSelection = RCC_ADC12CLKSOURCE_SYSCLK;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
  /** Enables the Clock Security System
  */
  HAL_RCC_EnableCSS();
}

/* USER CODE BEGIN 4 */

void KEY_Proc(void)
{
	if ((uwTick - keyslow) < 200) return;
	   keyslow = uwTick;
	
	uk_val = KEY_Scan();
	uk_down = uk_val&(uk_up^uk_old);
	uk_up = ~uk_val&(uk_up^uk_old);
	uk_old = uk_val;
	
	
	switch(uk_down)
	{
		case 1:   //按键1按下向设备发送幅度数据
			LCD_Clear(White);
			sprintf(str, "Vp:%4.2fV\r\n",adc_voltage);
			HAL_UART_Transmit(&huart2,(unsigned char *)str, strlen(str), 50);
		break;
		case 2://按键2按下向设备发送频率数据
			sprintf(str, "Freq:%uHz\r\n",72000000/PWM_RisingCount);
			HAL_UART_Transmit(&huart2,(unsigned char *)str, strlen(str), 50);			
		break;
		case 3://按键3按下向设备发送周期数据
			sprintf(str, " T:%7.6fs\r\n",(float)((float)PWM_RisingCount/72000000));
			HAL_UART_Transmit(&huart2,(unsigned char *)str, strlen(str), 50);				
		break;	
		case 4 ://按键2按下向设备发送占空比数据
			sprintf(str, " Duty:%.2f %%\r\n",duty * 100);
			HAL_UART_Transmit(&huart2,(unsigned char *)str, strlen(str), 50);					
		break;		
			
	}
}


void LCD_Proc(void)
{
	if ((uwTick - lcdslow) < 1000) return;
	   lcdslow = uwTick;
	

	
		sprintf((char *)Lcd_string," Parameter");
		LCD_DisplayStringLine(Line1,Lcd_string);
		

		sprintf((char *)Lcd_string," Vp:%4.2fV",adc_voltage);  //幅度
		LCD_DisplayStringLine(Line3,Lcd_string);
	
	    sprintf((char *)Lcd_string," Freq:%dHz      ",72000000/PWM_RisingCount);//频率
		LCD_DisplayStringLine(Line5,Lcd_string);
	
		sprintf((char *)Lcd_string," T:%7.6fs            ",(float)((float)PWM_RisingCount/72000000));//周期
		LCD_DisplayStringLine(Line7,Lcd_string);

		sprintf((char *)Lcd_string,"Duty = %.2f %%      ",duty * 100);//占空比
		LCD_DisplayStringLine(Line9,Lcd_string);


}


uint16_t get_ADC(void)
{
	uint16_t adc = 0;

	HAL_ADC_Start(&hadc1);
	adc = HAL_ADC_GetValue(&hadc1);
	if(adc > adc_max)	adc_max = adc;
	return adc_max;

}


void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
	if(htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1)    
	{
		PWM_RisingCount = HAL_TIM_ReadCapturedValue(&htim2,TIM_CHANNEL_1)+1;
		duty = (float)PWM_FallingCount / PWM_RisingCount;
	}
	if(htim->Channel == HAL_TIM_ACTIVE_CHANNEL_2)
	{
		PWM_FallingCount = HAL_TIM_ReadCapturedValue(&htim2,TIM_CHANNEL_2)+1;
	}
		

}

//回调函数，定时器中断服务函数调用
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	static uint16_t flag;
	static uint16_t num=0;
    if(htim==(&htim4))
    {
      if(++num==1000)
			{
				tim_flag=1;
				num=0;
				flag=!flag;
				if(flag){
				HAL_TIM_IC_Stop_IT(&htim2, TIM_CHANNEL_1);
			    HAL_TIM_IC_Stop_IT(&htim2, TIM_CHANNEL_2);
				}
				else{
						HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_1);												 
	                    HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_2);
				}
			}
    }
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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
