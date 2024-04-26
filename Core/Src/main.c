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
SPI_HandleTypeDef hspi2;

/* USER CODE BEGIN PV */

// Instructions for KP264 pressure sensor
// The reset bit for all commands is currently set to 0
const uint16_t request_pressure = 0b0010000000000000;
const uint16_t request_temperature = 0b0100000000000000;
const uint16_t request_diagnosis = 0b1000000000000000;
const uint16_t request_identifier = 0b1110000000000000;

// Bit masks for KP264 pressure sensor
const uint16_t data_mask = 0b0000011111111110;
const uint16_t com_error_mask = 0b0000000000000001;
const uint16_t FEC_error_mask = 0b1000000000000000;
const uint16_t Diag1_mask = 0b0100000000000000;
const uint16_t Diag2_mask = 0b0010000000000000;
const uint16_t pressure_over_max_mask = 0b0001000000000000;
const uint16_t pressure_under_min_mask = 0b0000100000000000;
const uint16_t no_error_mask = 0b0101000000000000;

// Pressure and temperature values in binary
uint16_t pressure_LSB;
uint16_t temperature_LSB;
// Pressure and temperature values in decimal
float pressure_Bar;
float temperature_C;
// Pressure transfer function parameters
float offs_p = -545.6; // LSB/kPa
float S_p = 13.64; // LSB

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI2_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void) {

	/*  CODE BEGIN 1 */
	// Variable for storing received byte;
	uint16_t receive_buffer;
	/*  CODE END 1 */

	/* MCU Configuration--------------------------------------------------------*/

	/* Reset of all peripherals, Initializes the Flash interface and the Systick. */
	HAL_Init();

	/*  CODE BEGIN Init */

	/* USER CODE END Init */

	/* Configure the system clock */
	SystemClock_Config();

	/* USER CODE BEGIN SysInit */

	/* USER CODE END SysInit */

	/* Initialize all configured peripherals */
	MX_GPIO_Init();
	MX_SPI2_Init();

	/* USER CODE BEGIN 2 */

	HAL_GPIO_WritePin(SPI_CS_GPIO_Port, SPI_CS_Pin, GPIO_PIN_SET);
	printf("Requesting SPI Identifier from Pressure Sensor... \r\n");
	HAL_GPIO_WritePin(SPI_CS_GPIO_Port, SPI_CS_Pin, GPIO_PIN_RESET);
	if (HAL_SPI_TransmitReceive(&hspi2, (uint8_t*) &request_identifier,
			(uint8_t*) &receive_buffer, 1, 100) != HAL_OK) {
		printf("ERROR: HAL_OK check failed \r\n");
	}
	HAL_GPIO_WritePin(SPI_CS_GPIO_Port, SPI_CS_Pin, GPIO_PIN_SET);
	// Check if received data is notifying about any errors
	// Check for communication error
	if (receive_buffer == com_error_mask) {
		printf("ERROR: Communication Error \r\n");
	}
	// Check for FEC Error
	else if ((receive_buffer & FEC_error_mask) != 0) {
		printf("ERROR: FEC Error \r\n");
	}
	// Check for Acquisition Chain Failure
	else if ((receive_buffer & no_error_mask) == Diag1_mask) {
		printf("ERROR: Acquisition chain failure \r\n");
	}
	// Check for Sensor Cell Failure
	else if ((receive_buffer & Diag2_mask) != 0) {
		printf("ERROR: Sensor cell failure \r\n");
	}
	// Check if pressure is above measuring range maximum
	else if ((receive_buffer & no_error_mask) == pressure_over_max_mask) {
		printf("ERROR: Pressure above measuring range maximum \r\n");
	}
	// Check if pressure is below measuring range minimum
	else if ((receive_buffer & pressure_under_min_mask) != 0) {
		printf("ERROR: Pressure below measuring range minimum \r\n");
	}
	// Check if no errors were detected
	else if ((receive_buffer & no_error_mask) == no_error_mask) {
		printf("No errors detected \r\n");
		printf("Identifier: %u \r\n", receive_buffer);
	}
	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1) {
		/* USER CODE END WHILE */

		/* USER CODE BEGIN 3 */

		// Toggle the LEDs
		HAL_GPIO_TogglePin(Valve_Enable_GPIO_Port, Valve_Enable_Pin);
		HAL_Delay(200);
		HAL_GPIO_TogglePin(Valve_Enable_GPIO_Port, Valve_Enable_Pin);
		HAL_GPIO_TogglePin(GPIOB, PRS_Ready_Pin);
		HAL_Delay(200);
		HAL_GPIO_TogglePin(GPIOB, PRS_Ready_Pin);
		HAL_GPIO_TogglePin(GPIOB, System_Ready_Pin);
		HAL_Delay(200);
		HAL_GPIO_TogglePin(GPIOB, System_Ready_Pin);

		// Measure Pressure
		HAL_GPIO_WritePin(SPI_CS_GPIO_Port, SPI_CS_Pin, GPIO_PIN_RESET);
		if (HAL_SPI_TransmitReceive(&hspi2, (uint8_t*) &request_pressure,
				(uint8_t*) &receive_buffer, 1, 100) != HAL_OK) {
			printf("ERROR: HAL_OK check failed \r\n");
		}
		HAL_GPIO_WritePin(SPI_CS_GPIO_Port, SPI_CS_Pin, GPIO_PIN_SET);
		// Check if received data is notifying about any errors
		// Check for communication error
		if (receive_buffer == com_error_mask) {
			printf("ERROR: Communication Error \r\n");
		}
		// Check for FEC Error
		else if ((receive_buffer & FEC_error_mask) != 0) {
			printf("ERROR: FEC Error \r\n");
		}
		// Check for Acquisition Chain Failure
		else if ((receive_buffer & no_error_mask) == Diag1_mask) {
			printf("ERROR: Acquisition chain failure \r\n");
			printf("Received bytes = %u", receive_buffer);
		}
		// Check for Sensor Cell Failure
		else if ((receive_buffer & Diag2_mask) != 0) {
			printf("ERROR: Sensor cell failure \r\n");
		}
		// Check if pressure is above measuring range maximum
		else if ((receive_buffer & no_error_mask) == pressure_over_max_mask)  {
			printf("ERROR: Pressure above measuring range maximum \r\n");
		}
		// Check if pressure is below measuring range minimum
		else if ((receive_buffer & pressure_under_min_mask) != 0) {
			printf("ERROR: Pressure below measuring range minimum \r\n");
		}
		// Check if no errors were detected
		else if ((receive_buffer & no_error_mask) == no_error_mask) {
			printf("No errors detected \r\n");
			// Get the pressure data using a bit mask and right shift operator to get rid of parity bit
			pressure_LSB = (receive_buffer & data_mask) >> 1;
			// Convert pressure from LSB to kPa using transfer function from data sheet
			pressure_Bar = ((float) pressure_LSB - offs_p) / S_p;
			// Convert pressure from Bar to kPa
			// pressure_Bar = pressure_Bar/100;
			// Print pressure value
			printf("Measured pressure: %i kPa \r\n", (int)pressure_Bar);

		}
		/* USER CODE END 3 */
	}
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void) {
	RCC_OscInitTypeDef RCC_OscInitStruct = { 0 };
	RCC_ClkInitTypeDef RCC_ClkInitStruct = { 0 };

	/** Configure the main internal regulator output voltage
	 */
	HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);

	/** Initializes the RCC Oscillators according to the specified parameters
	 * in the RCC_OscInitTypeDef structure.
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
	RCC_OscInitStruct.HSIState = RCC_HSI_ON;
	RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
	RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV4;
	RCC_OscInitStruct.PLL.PLLN = 75;
	RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
	RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
	RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
		Error_Handler();
	}

	/** Initializes the CPU, AHB and APB buses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
			| RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK) {
		Error_Handler();
	}
}

/**
 * @brief SPI2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_SPI2_Init(void) {

	/* USER CODE BEGIN SPI2_Init 0 */

	/* USER CODE END SPI2_Init 0 */

	/* USER CODE BEGIN SPI2_Init 1 */

	/* USER CODE END SPI2_Init 1 */
	/* SPI2 parameter configuration*/
	hspi2.Instance = SPI2;
	hspi2.Init.Mode = SPI_MODE_MASTER;
	hspi2.Init.Direction = SPI_DIRECTION_2LINES;
	hspi2.Init.DataSize = SPI_DATASIZE_16BIT;
	hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
	hspi2.Init.CLKPhase = SPI_PHASE_2EDGE;
	hspi2.Init.NSS = SPI_NSS_SOFT;
	hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_32;
	hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
	hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
	hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
	hspi2.Init.CRCPolynomial = 7;
	hspi2.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
	hspi2.Init.NSSPMode = SPI_NSS_PULSE_DISABLE;
	if (HAL_SPI_Init(&hspi2) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN SPI2_Init 2 */

	/* USER CODE END SPI2_Init 2 */

}

/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void) {
	GPIO_InitTypeDef GPIO_InitStruct = { 0 };
	/* USER CODE BEGIN MX_GPIO_Init_1 */
	/* USER CODE END MX_GPIO_Init_1 */

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(Valve_Enable_GPIO_Port, Valve_Enable_Pin, GPIO_PIN_SET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOB, SPI_CS_Pin | PRS_Ready_Pin | System_Ready_Pin,
			GPIO_PIN_SET);

	/*Configure GPIO pin : Valve_Enable_Pin */
	GPIO_InitStruct.Pin = Valve_Enable_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_PULLDOWN;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(Valve_Enable_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pin : SPI_CS_Pin */
	GPIO_InitStruct.Pin = SPI_CS_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(SPI_CS_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pins : PRS_Ready_Pin System_Ready_Pin */
	GPIO_InitStruct.Pin = PRS_Ready_Pin | System_Ready_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_PULLDOWN;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	/* USER CODE BEGIN MX_GPIO_Init_2 */
	/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
int _write(int file, char *ptr, int len) {
	(void) file;
	int DataIdx;

	for (DataIdx = 0; DataIdx < len; DataIdx++) {
		ITM_SendChar(*ptr++);
	}
	return len;
}
/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void) {
	/* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
	__disable_irq();
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
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
