
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
#include "stm32l4xx_hal.h"
#include "adc.h"
#include "dma.h"
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* USER CODE BEGIN Includes */

//nclude <numeric>
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

// TO JEST LEGITNA WERSJA $$$$$$$$$$$$$$$$$$$$$$$$$$$



uint8_t Tx_flag = 0;
uint8_t Tx_flag_2 = 0;

uint8_t Rx_flag = 0;
uint8_t Rx_flag_2 = 0;

volatile float PomiarADC;
volatile float VoltageADC;

char* buffer="hello!\n\r";

const float V25 = 0.76; // [Volts]
const float SupplyVoltage = 3.3; // [Volts]
const float ADCResolution = 4096.0;

enum{ ADC_BUFFER_LENGTH = 8192 };
volatile uint16_t ADCBuffer[ADC_BUFFER_LENGTH];

int g_MeasurementNumber;


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

void LED_PWM(uint8_t timer_number, uint8_t timer_channel, uint8_t Duty);
void LED_Sweep(void);
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin);


void HAL_SYSTICK_Callback(void);
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart);
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim4);

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc){
	
	PomiarADC =0;
	
	for(uint16_t i = 0; i < ADC_BUFFER_LENGTH; i++){	
	PomiarADC += ADCBuffer[i];	
	}
	
	PomiarADC = PomiarADC/ADC_BUFFER_LENGTH;
	VoltageADC = (SupplyVoltage*PomiarADC)/(ADCResolution-1);
	
}

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
  MX_ADC1_Init();
  MX_ADC2_Init();
  MX_ADC3_Init();
  MX_I2C1_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_USART2_UART_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */


  
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);

	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_4);     // Timers initialization

	LED_Sweep();
	
	HAL_TIM_Base_Start_IT(&htim3);
	HAL_TIM_Base_Start_IT(&htim4);                // Timers start


	HAL_ADC_Start_DMA(&hadc1, (uint32_t*)ADCBuffer,ADC_BUFFER_LENGTH);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	while (1) {

		
		//HAL_UART_Transmit_IT(&huart1, (uint8_t *)buffer,8);
		//HAL_Delay(250);


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

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_PeriphCLKInitTypeDef PeriphClkInit;

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 40;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1|RCC_PERIPHCLK_USART2
                              |RCC_PERIPHCLK_I2C1|RCC_PERIPHCLK_ADC;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK2;
  PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_PCLK1;
  PeriphClkInit.AdcClockSelection = RCC_ADCCLKSOURCE_PLLSAI1;
  PeriphClkInit.PLLSAI1.PLLSAI1Source = RCC_PLLSOURCE_MSI;
  PeriphClkInit.PLLSAI1.PLLSAI1M = 1;
  PeriphClkInit.PLLSAI1.PLLSAI1N = 16;
  PeriphClkInit.PLLSAI1.PLLSAI1P = RCC_PLLP_DIV7;
  PeriphClkInit.PLLSAI1.PLLSAI1Q = RCC_PLLQ_DIV2;
  PeriphClkInit.PLLSAI1.PLLSAI1R = RCC_PLLR_DIV2;
  PeriphClkInit.PLLSAI1.PLLSAI1ClockOut = RCC_PLLSAI1_ADC1CLK;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the main internal regulator output voltage 
    */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
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

/* USER CODE BEGIN 4 */

// --------------------------------------------
void LED_PWM(uint8_t timer_number, uint8_t timer_channel, uint8_t Duty) {
	if (timer_number == 3) {
		switch (timer_channel) {
		case 1:
			TIM3->CCR1 = Duty;
			break;
		case 2:
			TIM3->CCR2 = Duty;
			break;
		case 3:
			TIM3->CCR3 = Duty;
			break;
		case 4:
			TIM3->CCR4 = Duty;
			break;
		default:
			break;

		}
	} else if (timer_number == 4) {
		switch (timer_channel) {
		case 1:
			TIM4->CCR1 = Duty;
			break;
		case 2:
			TIM4->CCR2 = Duty;
			break;
		case 3:
			TIM4->CCR3 = Duty;
			break;
		case 4:
			TIM4->CCR4 = Duty;
			break;
		default:
			break;
		}
	}
}


void LED_Sweep(void) {

	LED_PWM(3, 1, 100);
	HAL_Delay(150);
	LED_PWM(3, 2, 100);
	LED_PWM(3, 1, 0);
	HAL_Delay(150);
	LED_PWM(3, 3, 100);
	LED_PWM(3, 2, 0);
	HAL_Delay(150);
	LED_PWM(3, 4, 100);
	LED_PWM(3, 3, 0);
	HAL_Delay(150);

	LED_PWM(4, 4, 100);
	LED_PWM(3, 4, 0);
	HAL_Delay(150);
	LED_PWM(4, 3, 100);
	LED_PWM(4, 4, 0);
	HAL_Delay(150);

	LED_PWM(4, 4, 100);
	LED_PWM(4, 3, 0);
	HAL_Delay(150);
	LED_PWM(3, 4, 100);
	LED_PWM(4, 4, 0);
	HAL_Delay(150);
	LED_PWM(3, 3, 100);
	LED_PWM(3, 4, 0);
	HAL_Delay(150);
	LED_PWM(3, 2, 100);
	LED_PWM(3, 3, 0);
	HAL_Delay(150);
	LED_PWM(3, 1, 100);
	LED_PWM(3, 2, 0);
	HAL_Delay(150);
	LED_PWM(3, 1, 0);

}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {

	
	uint8_t button1_flag = 0;
	uint8_t button2_flag = 0;
	
	if (GPIO_Pin == Button1_Pin) {
		if (button1_flag == 0) {
			LED_PWM(4, 1, 100);
			button1_flag = 1;
		} else {
			LED_PWM(4, 1, 0);
			button1_flag = 0;
		}
	} else if (GPIO_Pin == Button2_Pin) {
		if (button2_flag == 0) {
			LED_PWM(4, 2, 40);
			button2_flag = 1;
		} else {
			LED_PWM(4, 2, 0);
			button2_flag = 0;
		}

	}

}

void HAL_SYSTICK_Callback(void) {
	uint8_t Duty = 0;
	static uint8_t InterruptPrescaler = 0; // licznik przerwan
	static uint8_t CzyRosnie = 1; // Flaga kierunku zliczania

	++InterruptPrescaler; // Inkrementacja numeru przerwania

	// Jezeli wywolalo sie 40 przerwanie z rzedu
	if (InterruptPrescaler == 40) {
		InterruptPrescaler = 0; // wyzeruj licznik przerwan

		if (Duty == 100) // Jezeli wypelnienie jest rowne 100
			CzyRosnie = 0; // Zmien kierunek zliczania w dol

		else if (Duty == 0) // Jezeli wypelnienie rowne 0
			CzyRosnie = 1; // Zmien kierunek zliczania w gore

		if (CzyRosnie) // Jezeli zliczamy w gore
			++Duty; // Inkrementuj wartosc wypelnienia
		else
			//Jezeli zliczamy w dol
			--Duty; // Dekrementuj wartosc wypelnienia

	


		// sprintf(Data, "Odebrana wiadomosc: %s\n\r", 10);
		// HAL_UART_Transmit(&huart1, Transmit, 10, 100); // Rozpoczecie nadawania danych z wykorzystaniem przerwan
	}
	TIM3->CCR3 = Duty;

}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{


}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim4) {



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
	while (1) {
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
