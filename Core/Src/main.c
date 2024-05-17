/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
#include "i2c.h"
#include "spi.h"
#include "usart.h"
#include "gpio.h"
#include "ds3231.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define ESCAPE_WHITE   "\033[37m"
#define ESCAPE_GREEN   "\033[32m"
#define ESCAPE_YELLOW  "\033[33m"
#define ESCAPE_RED     "\033[31m"
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
uint8_t tx_data[100] = ESCAPE_GREEN "\n\r<| Locksys |> - " ESCAPE_WHITE __TIME__ ESCAPE_GREEN "\n\r ˘˘˘˘˘˘˘˘ˇˇˇ \n\r";
uint8_t spi_RFID[100];
uint8_t last_pin[1] = "e";
uint8_t rx_data[4];
uint8_t code[4] = "1122";
uint8_t en_keypad = 0;

uint8_t time_set[7] = {0, 10, 13, 5, 25, 1, 21};
uint8_t time_current[7];
//* USER CODE END 0 */

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
  MX_USART1_UART_Init();
  MX_I2C1_Init();
  MX_SPI1_Init();
  /* USER CODE BEGIN 2 */

  // USART
  HAL_UART_Receive_IT(&huart1, rx_data, sizeof(rx_data));
  HAL_UART_Transmit(&huart1, tx_data, sizeof(tx_data), 1000);

  // RFID
  //MFRC522_Init();
  //HAL_GPIO_WritePin(uSPI1_SS_GPIO_Port, uSPI1_SS_Pin, GPIO_PIN_SET);
  //HAL_SPI_Receive_IT(&hspi1, spi_RFID, sizeof(spi_RFID));

  // LED
  HAL_GPIO_TogglePin(GPIOG, GPIO_PIN_14);

  // RTC
  ds3231_init(&time_set[0], CLOCK_HALT, FORCE_RESET);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  if (en_keypad == 1) {
		  keypad_read();
	  }
	  ds3231_read(TIME, time_current);
	  HAL_UART_Transmit(&huart1, time_current, sizeof(time_current), 1000);
	  HAL_Delay(1000);
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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
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
}

/* USER CODE BEGIN 4 */

void rtc_print(uint8_t *data_array, uint8_t array_length)
{
	for (int8_t index = array_length - 1; index >= 0; index--)
	{
		HAL_UART_Transmit(&huart1, data_array[index], sizeof(data_array[index]), 1000);
	}
}

void keypad_read() {
	HAL_GPIO_WritePin(GPIOF, MK_Output_2_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOF, MK_Output_3_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOF, MK_Output_1_Pin, GPIO_PIN_SET);

	uint8_t pin[1] = "e";

	if (HAL_GPIO_ReadPin(GPIOF, MK_Input_1_Pin) == GPIO_PIN_SET)
	{
		pin[0] = '1';
	} else
	if (HAL_GPIO_ReadPin(GPIOF, MK_Input_2_Pin) == GPIO_PIN_SET)
	{
		pin[0] = '4';

	} else
	if (HAL_GPIO_ReadPin(GPIOF, MK_Input_3_Pin) == GPIO_PIN_SET)
	{
		pin[0] = '7';
	} else
	if (HAL_GPIO_ReadPin(MK_Input_4_GPIO_Port, MK_Input_4_Pin) == GPIO_PIN_SET)
	{
		pin[0] = '*';
	}

	if (pin[0] != 'e' && pin[0] != last_pin[0]) {
		last_pin[0] = pin[0];
		uint8_t message[40] = ESCAPE_YELLOW "PIN: - " ESCAPE_WHITE;
		uint8_t messageEnd[40] = "\n\r";
		HAL_UART_Transmit(&huart1, message, sizeof(message), 1000);
		HAL_UART_Transmit(&huart1, pin, sizeof(pin), 1000);
		HAL_UART_Transmit(&huart1, messageEnd, sizeof(messageEnd), 1000);
	}
	HAL_GPIO_WritePin(GPIOF, MK_Output_1_Pin, GPIO_PIN_RESET);
	return;
}

void toggleLED() {
	HAL_GPIO_TogglePin(GPIOG, GPIO_PIN_13);
	HAL_GPIO_TogglePin(GPIOG, GPIO_PIN_14);
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	//COM
	HAL_UART_Receive_IT(&huart1, rx_data, sizeof(rx_data));
	//HAL_UART_Transmit(&huart1, rx_data, sizeof(rx_data), 1000);

	uint8_t message[40] = ESCAPE_YELLOW "Interrupt - COM - " ESCAPE_WHITE __TIME__ "\n\r";
	HAL_UART_Transmit(&huart1, message, sizeof(message), 1000);

	int en_keypad_code = 1;
	for (int i = 0; i < 4; i++) {
		if (rx_data[i] != '0') {
			en_keypad_code = 0;
		}
	}
	if (en_keypad_code == 1) {
		if (en_keypad == 1) {
			en_keypad = 0;
		}
		else en_keypad = 1;
		uint8_t response[20] = ESCAPE_GREEN "Keypad toggled\n\r";
		HAL_UART_Transmit(&huart1, response, sizeof(response), 1000);
		return;
	}

	int correctCode = 1;
	for (int i = 0; i < 4; i++) {
		if (rx_data[i] != code[i]) {
			correctCode = 0;
		}
	}

	if (correctCode == 1) {
		uint8_t response[20] = ESCAPE_GREEN "Correct\n\r";
		HAL_UART_Transmit(&huart1, response, sizeof(response), 1000);
		toggleLED();
	} else {
		uint8_t response[20] = ESCAPE_RED "Incorrect\n\r";
		HAL_UART_Transmit(&huart1, response, sizeof(response), 1000);

	}
}

void HAL_SPI_RxCpltCallback(SPI_HandleTypeDef * hspi)
{
	//RFID
	HAL_SPI_Receive_IT(&hspi1, spi_RFID, sizeof(spi_RFID));

	//HAL_UART_Transmit(&hspi1, spi_RFID, sizeof(spi_RFID), 1000);
	uint8_t message[40] = ESCAPE_YELLOW "Interrupt - RFID - " ESCAPE_WHITE __TIME__ "\n\r";
	HAL_UART_Transmit(&huart1, message, sizeof(message), 1000);
	toggleLED();
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
