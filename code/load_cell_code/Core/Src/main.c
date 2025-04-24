/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#include <string.h>
#include <stdbool.h>
// ADS1219 I²C address (7-bit <<1 for HAL)
#define ADS1219_I2C_ADDR        (0x40 << 1)

// ADS1219 command codes:
#define ADS1219_CMD_RESET       0x06  // soft reset
#define ADS1219_CMD_START_SYNC  0x08  // start (sync) conversion
#define ADS1219_CMD_READ_DATA   0x10  // read 3 data bytes
#define ADS1219_CMD_STATUS_READ   0x20


// Our 7-byte packet: 4-byte timestamp + 3-byte ADC reading, no padding
#pragma pack(push,1)
typedef struct {
  uint32_t timestamp;
  uint8_t  adc24[3];
} DataPacket;
#pragma pack(pop)

// Lantronix XPort reset on PA11
#define XPORT_RST_GPIO_Port   GPIOA
#define XPORT_RST_Pin         GPIO_PIN_11
/* USER CODE END Includes */
/* USER CODE END Includes */

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
I2C_HandleTypeDef hi2c1;

SPI_HandleTypeDef hspi1;

USART_HandleTypeDef husart1;

/* USER CODE BEGIN PV */
static DataPacket txPacket;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_SPI1_Init(void);
static void MX_USART1_Init(void);
/* USER CODE BEGIN PFP */
static int32_t ADS1219_ReadRaw(void);
static bool     ADS1219_dataReady(void);
static int32_t  ADS1219_readConversion(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/* USER CODE BEGIN 0 */
static int32_t ADS1219_ReadRaw(void)
{
    uint8_t cmd, raw[3];
    uint32_t u;

    // 1) START_SYNC
    cmd = ADS1219_CMD_START_SYNC;
    HAL_I2C_Master_Transmit(&hi2c1, ADS1219_I2C_ADDR, &cmd, 1, 100);

    // 2) wait for DRDY on PB4 (active low)
    while (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_4) == GPIO_PIN_SET) { }

    // 3) READ_DATA + fetch 3 bytes
    cmd = ADS1219_CMD_READ_DATA;
    HAL_I2C_Master_Transmit(&hi2c1, ADS1219_I2C_ADDR, &cmd, 1, 100);
    HAL_I2C_Master_Receive(&hi2c1, ADS1219_I2C_ADDR, raw, 3, 100);

    // 4) assemble signed 24→32-bit
    u = ((uint32_t)raw[0] << 16) |
        ((uint32_t)raw[1] <<  8) |
         (uint32_t)raw[2];
    if (u & 0x00800000) u |= 0xFF000000;  // sign-extend
    return (int32_t)u;
}
// Returns true once the ADS1219’s DRDY bit (bit7 of status reg) is set
static bool ADS1219_dataReady(void)
{
    uint8_t cmd = ADS1219_CMD_STATUS_READ;
    uint8_t status = 0;

    if (HAL_I2C_Master_Transmit(&hi2c1, ADS1219_I2C_ADDR, &cmd, 1, 100) != HAL_OK)
        return false;
    if (HAL_I2C_Master_Receive(&hi2c1, ADS1219_I2C_ADDR, &status, 1, 100) != HAL_OK)
        return false;

    return (status & 0x80) != 0;
}

// Reads 3 bytes via the “read data” command, sign-extends to 32 bits
static int32_t ADS1219_readConversion(void)
{
    uint8_t cmd = ADS1219_CMD_READ_DATA;
    uint8_t raw[3];
    uint32_t u = 0;

    HAL_I2C_Master_Transmit(&hi2c1, ADS1219_I2C_ADDR, &cmd, 1, 100);
    HAL_I2C_Master_Receive (&hi2c1, ADS1219_I2C_ADDR, raw, 3, 100);

    u = ((uint32_t)raw[0] << 16) |
        ((uint32_t)raw[1] <<  8) |
         (uint32_t)raw[2];

    // sign-extend 24→32 bits
    if (u & 0x00800000)
        u |= 0xFF000000;

    return (int32_t)u;
}

/* USER CODE END 0 */


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
  MX_I2C1_Init();
  MX_SPI1_Init();
  MX_USART1_Init();
  /* USER CODE BEGIN 2 */
  // reset ads1219 following arduino library

  {
    uint8_t cmd = ADS1219_CMD_RESET;
    HAL_I2C_Master_Transmit(&hi2c1, ADS1219_I2C_ADDR, &cmd, 1, 100);
    HAL_Delay(1);  // tRSSTA ≥100 µs
  }


  // Reset the Lantronix XPort on PA11
  HAL_GPIO_WritePin(XPORT_RST_GPIO_Port, XPORT_RST_Pin, GPIO_PIN_RESET);
  HAL_Delay(10);   // hold reset low ≥10 ms
  HAL_GPIO_WritePin(XPORT_RST_GPIO_Port, XPORT_RST_Pin, GPIO_PIN_SET);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */
	  /* USER CODE BEGIN 3 */
	    // 1) timestamp
	    txPacket.timestamp = HAL_GetTick();

	    // 2) trigger a conversion
	    {
	      uint8_t cmd = ADS1219_CMD_START_SYNC;
	      HAL_I2C_Master_Transmit(&hi2c1, ADS1219_I2C_ADDR, &cmd, 1, 100);
	    }

	    // 3) wait until the library‐style dataReady() returns true
	    while (!ADS1219_dataReady());

	    // 4) fetch & pack the signed 24-bit result
	    {
	      int32_t raw = ADS1219_readConversion();
	      txPacket.adc24[0] = (raw >> 16) & 0xFF;
	      txPacket.adc24[1] = (raw >>  8) & 0xFF;
	      txPacket.adc24[2] = (raw      ) & 0xFF;
	    }

	    // 5) send your 7-byte packet over UART1 → XPort
	    HAL_USART_Transmit(&husart1,
	                      (uint8_t *)&txPacket,
	                      sizeof(txPacket),
	                      100);
	  /* USER CODE END 3 */

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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI_DIV2;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_HARD_OUTPUT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_4;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  husart1.Instance = USART1;
  husart1.Init.BaudRate = 115200;
  husart1.Init.WordLength = USART_WORDLENGTH_8B;
  husart1.Init.StopBits = USART_STOPBITS_1;
  husart1.Init.Parity = USART_PARITY_NONE;
  husart1.Init.Mode = USART_MODE_TX_RX;
  husart1.Init.CLKPolarity = USART_POLARITY_LOW;
  husart1.Init.CLKPhase = USART_PHASE_1EDGE;
  husart1.Init.CLKLastBit = USART_LASTBIT_DISABLE;
  if (HAL_USART_Init(&husart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

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
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3|GPIO_PIN_11|GPIO_PIN_12, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_RESET);

  /*Configure GPIO pins : PA3 PA11 PA12 */
  GPIO_InitStruct.Pin = GPIO_PIN_3|GPIO_PIN_11|GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PB1 */
  GPIO_InitStruct.Pin = GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PB4 */
  GPIO_InitStruct.Pin = GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
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
