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
#include "stm32f4xx_ll_gpio.h"
#include "stm32f4xx_ll_bus.h"

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

#define DS3231_ADDRESS 0xD0
#define ESP8266_ADDRESS 0X00

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

uint8_t tx_data[100] = ESCAPE_GREEN "\n\r<¨¨Locksys¨¨> - " ESCAPE_WHITE __TIME__ ESCAPE_GREEN "\n\r\n\r";
uint8_t spi_RFID[100];
uint8_t last_pin[1] = "e";
uint8_t rx_data[4];
uint8_t code[4] = "1122";
uint8_t code_keypad[4] = "    ";
int code_keypad_index = 0;
uint8_t en_keypad = 1;
uint8_t temperature;
/*
 * Structure for storing and managing the time value read from the RTC module
 */
typedef struct Time {
	uint8_t second;
	uint8_t minute;
	uint8_t hour;
	uint8_t day;
	uint8_t week;
	uint8_t month;
	uint8_t year;
} Time;

struct Time time;

/*
 * Set the current time in rtc module
 */
void set_time(uint8_t sec, uint8_t min, uint8_t h, uint8_t d, uint8_t w, uint8_t m, uint8_t y) {
	uint8_t set[7];
	set[0] = dec_to_bcd(sec);
	set[1] = dec_to_bcd(min);
	set[2] = dec_to_bcd(h);
	set[3] = dec_to_bcd(d);
	set[4] = dec_to_bcd(w);
	set[5] = dec_to_bcd(m);
	set[6] = dec_to_bcd(y);

	HAL_I2C_Mem_Write(&hi2c1, DS3231_ADDRESS, 0x00, 1, set, 7, 1000);
}

/*
 * Get current time from RTC module
 */
void get_time() {
	uint8_t get[7];
	HAL_I2C_Mem_Read(&hi2c1, DS3231_ADDRESS, 0x00, 1, get, 7, 1000);

	uint8_t t[1];
	HAL_I2C_Mem_Read(&hi2c1, DS3231_ADDRESS, 0x11, 1, t, 7, 1000);
	temperature = bcd_to_dec(t[1]);

	time.second = bcd_to_dec(get[1]);
	time.minute = bcd_to_dec(get[2]);
	time.hour = bcd_to_dec(get[3]);
	time.day = bcd_to_dec(get[4]);
	time.week = bcd_to_dec(get[5]);
	time.month = bcd_to_dec(get[6]);
	time.year = bcd_to_dec(get[7]);
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
  MX_USART1_UART_Init();
  MX_I2C1_Init();
  MX_SPI1_Init();
  MX_I2C2_Init();
  /* USER CODE BEGIN 2 */

  // USART
  HAL_UART_Receive_IT(&huart1, rx_data, sizeof(rx_data));
  HAL_UART_Transmit(&huart1, tx_data, sizeof(tx_data), 1000);

  // RFID
  //uint8_t reset = "15";
  //HAL_SPI_Transmit(&hspi1, &reset, sizeof(reset), 1000);
  uint8_t receive = 8;
  HAL_SPI_Transmit(&hspi1, &receive, sizeof(receive), 1000);


  // LED
  HAL_GPIO_TogglePin(GPIOG, GPIO_PIN_14);

  // RTC
  //set_time(0, 3, 19, 5, 17, 5, 24);

  // Keypad
  HAL_GPIO_WritePin(GPIOF, MK_Output_1_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(GPIOF, MK_Output_2_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(GPIOF, MK_Output_3_Pin, GPIO_PIN_SET);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  if (en_keypad == 1) {	// If enabled, read keypad by polling for a result
		  keypad_read();
	  }

	  get_time();

	  //MFRC522_read();

	  HAL_Delay(120);
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

/*
 * Function for conversion from (int) to (uint8_t)
 * @param1 - the value to be converted
 * @returns - the converted value
 */
uint8_t dec_to_bcd(int value) {
	return (uint8_t)((value / 10 * 16) + (value % 10));
}
/*
 * Function for conversion from (uint8_t) to (int)
 * @param1 - the value to be converted
 * @returns - the converted value
 */
int bcd_to_dec(uint8_t value) {
	return (int)((value / 16 * 10) + (value % 16));
}

/*
 * Reads which button is pressed on the keypad, by pulling each row high exclusively,
 * 		and reading whether we get an input signal from any of the connected input wires.
 */
void keypad_read() {
	LL_GPIO_SetPinMode(GPIOF, MK_Output_1_Pin, LL_GPIO_MODE_OUTPUT);
	LL_GPIO_SetPinMode(GPIOF, MK_Output_2_Pin, LL_GPIO_MODE_INPUT);
	LL_GPIO_SetPinMode(GPIOF, MK_Output_3_Pin, LL_GPIO_MODE_INPUT);

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
		//pin[0] = '*';
		check_code(code_keypad);
		for (int i = 0; i < 4; i++) {
			code_keypad[i] = " ";
		}
		code_keypad_index = 0;
		return;
	}

	LL_GPIO_SetPinMode(GPIOF, MK_Output_1_Pin, LL_GPIO_MODE_INPUT);
	LL_GPIO_SetPinMode(GPIOF, MK_Output_2_Pin, LL_GPIO_MODE_OUTPUT);
	LL_GPIO_SetPinMode(GPIOF, MK_Output_3_Pin, LL_GPIO_MODE_INPUT);

	if (HAL_GPIO_ReadPin(GPIOF, MK_Input_1_Pin) == GPIO_PIN_SET)
	{
		pin[0] = '2';
	} else
	if (HAL_GPIO_ReadPin(GPIOF, MK_Input_2_Pin) == GPIO_PIN_SET)
	{
		pin[0] = '5';
	} else
	if (HAL_GPIO_ReadPin(GPIOF, MK_Input_3_Pin) == GPIO_PIN_SET)
	{
		pin[0] = '8';
	} else
	if (HAL_GPIO_ReadPin(MK_Input_4_GPIO_Port, MK_Input_4_Pin) == GPIO_PIN_SET)
	{
		pin[0] = '0';
	}

	LL_GPIO_SetPinMode(GPIOF, MK_Output_1_Pin, LL_GPIO_MODE_INPUT);
	LL_GPIO_SetPinMode(GPIOF, MK_Output_2_Pin, LL_GPIO_MODE_INPUT);
	LL_GPIO_SetPinMode(GPIOF, MK_Output_3_Pin, LL_GPIO_MODE_OUTPUT);

	if (HAL_GPIO_ReadPin(GPIOF, MK_Input_1_Pin) == GPIO_PIN_SET)
	{
		pin[0] = '3';
	} else
	if (HAL_GPIO_ReadPin(GPIOF, MK_Input_2_Pin) == GPIO_PIN_SET)
	{
		pin[0] = '6';
	} else
	if (HAL_GPIO_ReadPin(GPIOF, MK_Input_3_Pin) == GPIO_PIN_SET)
	{
		pin[0] = '9';
	} else
	if (HAL_GPIO_ReadPin(MK_Input_4_GPIO_Port, MK_Input_4_Pin) == GPIO_PIN_SET)
	{
		//pin[0] = '#';
		for (int i = 0; i < 4; i++) {
			code_keypad[i] = " ";
		}
		code_keypad_index = 0;
		uint8_t response[40] = ESCAPE_GREEN " Keypad input deleted\n\r";
		HAL_UART_Transmit(&huart1, response, sizeof(response), 1000);
		return;
	}

	if (pin[0] != 'e') {
		last_pin[0] = pin[0];
		uint8_t message[40] = ESCAPE_YELLOW " -> PIN: - " ESCAPE_WHITE;
		uint8_t messageEnd[40] = "\n\r";
		HAL_UART_Transmit(&huart1, message, sizeof(message), 1000);
		HAL_UART_Transmit(&huart1, pin, sizeof(pin), 1000);
		HAL_UART_Transmit(&huart1, messageEnd, sizeof(messageEnd), 1000);

		code_keypad[code_keypad_index] = pin[0];
		code_keypad_index++;
		if (code_keypad_index > 3)
			code_keypad_index = 0;
		//HAL_UART_Transmit(&huart1, code_keypad, sizeof(code_keypad), 1000);
	}

	return;
}
/*
 * Toggles the open/closed status of the system (observe board for reference)
 * GREEN: open
 * RED: closed
 */
void toggleLock() {
	HAL_GPIO_TogglePin(GPIOG, GPIO_PIN_13);
	HAL_GPIO_TogglePin(GPIOG, GPIO_PIN_14);
	uint8_t message[7] = "toggled";
	HAL_I2C_Master_Transmit(&hi2c2, ESP8266_ADDRESS, message, sizeof(message), 1000);
}

void check_code(uint8_t * data)  {
	int correctCode = 1;		// Check whether the UART input is the correct keycode
	for (int i = 0; i < 4; i++) {
		if (data[i] != code[i]) {
			correctCode = 0;
		}
	}

	if (correctCode == 1) {
		uint8_t response[20] = ESCAPE_GREEN " Correct\n\r";
		HAL_UART_Transmit(&huart1, response, sizeof(response), 1000);
		toggleLock();
	} else {
		uint8_t response[20] = ESCAPE_RED " Incorrect\n\r";
		HAL_UART_Transmit(&huart1, response, sizeof(response), 1000);
	}
}

/*
 * Interrupt handler for UART RX
 * Processes input from COM, prints the current time by reading the RTC's correct register
 * @param1 - The handler of the defined UART entity
 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	HAL_UART_Receive_IT(&huart1, rx_data, sizeof(rx_data));		// Receive data via UART
	//HAL_UART_Transmit(&huart1, rx_data, sizeof(rx_data), 1000);

	uint8_t message[40] = ESCAPE_YELLOW " -> Interrupt - COM - " ESCAPE_WHITE;
	uint8_t messageEnd[10] = "\n\r";
	uint8_t demarcator[5] = ":";

	get_time();		// Get and print current time via RTC module
	HAL_UART_Transmit(&huart1, message, sizeof(message), 1000);
	//HAL_UART_Transmit(&huart1, time.hour, 10, 1000);
	HAL_UART_Transmit(&huart1, demarcator, sizeof(demarcator), 1000);
	//HAL_UART_Transmit(&huart1, &time.minute, 10, 1000);
	HAL_UART_Transmit(&huart1, demarcator, sizeof(demarcator), 1000);
	//HAL_UART_Transmit(&huart1, &time.second, 10, 1000);
	HAL_UART_Transmit(&huart1, messageEnd, sizeof(messageEnd), 1000);

	int en_keypad_code = 1;		// Check whether the UART input is the correct code for toggling input from keypad
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
		uint8_t response[40] = ESCAPE_GREEN " Keypad toggled - " ESCAPE_WHITE __TIME__ "\n\r" ;
		HAL_UART_Transmit(&huart1, response, sizeof(response), 1000);
		return;
	}

	check_code(rx_data);
	return;
}

/*
 * Interrupt handler for SPI RX
 * Processes input from RFID
 * @param1 - The handler of the defined SPI entity
 */
void HAL_SPI_RxCpltCallback(SPI_HandleTypeDef * hspi)
{
	HAL_SPI_Receive_IT(&hspi1, spi_RFID, sizeof(spi_RFID));		// Receive data via SPI

	//HAL_UART_Transmit(&hspi1, spi_RFID, sizeof(spi_RFID), 1000);
	uint8_t message[40] = ESCAPE_YELLOW " -> Interrupt - RFID - " ESCAPE_WHITE __TIME__ "\n\r";
	HAL_UART_Transmit(&huart1, message, sizeof(message), 1000);
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
