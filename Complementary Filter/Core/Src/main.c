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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>
#include <stdbool.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define MPU6050_ADDR 0x68
#define PWR_MGMT_1_REG 0x6B
#define SMPLRT_DIV_REG 0x19
#define GYRO_CNFG_REG 0x1B
#define ACC_CNFG_REG 0x1C
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
float offsetGyroX = 0, offsetGyroY = 0, offsetGyroZ = 0;
float qt[4] = {1,0,0,0};
float qw[4] = {0,0,0,0};
float qaWorld[4] = {0,0,0,0};

uint32_t loopTime;
uint32_t lastTimeGyro;



///
int eror = 0;
///
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_I2C1_Init(void);
/* USER CODE BEGIN PFP */
void initMPU6050(void);
void readAccelMPU6050(int16_t *rawAccelX, int16_t *rawAccelY, int16_t *rawAccelZ);
void readGyroMPU6050(int16_t *rawGyroX, int16_t *rawGyroY, int16_t *rawGyroZ);
void CalibrationGyro(float *offsetGyroX, float *offsetGyroY, float *offsetGyroZ);
void getCalibratedGyro(int16_t rawGyroX, int16_t rawGyroY, int16_t rawGyroZ, float *calibratedGyroX, float *calibratedGyroY, float *calibratedGyroZ);
void getCalibratedAccel(int16_t rawAccelX, int16_t rawAccelY, int16_t rawAccelZ, float *calibratedAccelX, float *calibratedAccelY, float *calibratedAccelZ);
void updateQuaternionFromGyro(float gx, float gy, float gz);
void updateQuaternionFromAccel(float ax, float ay, float az);
void updateTiltCorrectionWithComplementaryFilter(void);
void sendData(float qw, float qx, float qy, float qz);

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

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART2_UART_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */
  	initMPU6050();
  	CalibrationGyro(&offsetGyroX, &offsetGyroY, &offsetGyroZ);
  	loopTime = HAL_GetTick();
  	lastTimeGyro = HAL_GetTick();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {


		int16_t rawAccelX, rawAccelY, rawAccelZ;       // Ham ivmeölçer verileri
		float calibratedAccelX, calibratedAccelY, calibratedAccelZ; // Kalibre edilmiş ivmeölçer verileri

		int16_t rawGyroX, rawGyroY, rawGyroZ;           // Ham jiroskop verileri
		float calibratedGyroX, calibratedGyroY, calibratedGyroZ; // Kalibre edilmiş jiroskop verileri derece/saniye cinsinden
		readAccelMPU6050(&rawAccelX, &rawAccelY, &rawAccelZ);

	    readGyroMPU6050(&rawGyroX, &rawGyroY, &rawGyroZ);

	    getCalibratedAccel(rawAccelX, rawAccelY, rawAccelZ, &calibratedAccelX,
			  				&calibratedAccelY, &calibratedAccelZ);
	    getCalibratedGyro(rawGyroX, rawGyroY, rawGyroZ, &calibratedGyroX,
	    	  				&calibratedGyroY, &calibratedGyroZ);
	    // Gyro ile quaternion'ı güncelle
	    updateQuaternionFromGyro(calibratedGyroX, calibratedGyroY, calibratedGyroZ);

	    // İvmeölçer düzeltmesi (sadece statikse)
	    updateQuaternionFromAccel(calibratedAccelX, calibratedAccelY, calibratedAccelZ);

	    updateTiltCorrectionWithComplementaryFilter();



	    sendData(qt[0], qt[1], qt[2], qt[3]);
	    while(HAL_GetTick()-loopTime<100);
	    loopTime = HAL_GetTick();
	   // HAL_Delay(100);  // 100ms aralıklarla veri gönder
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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 180;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Activate the Over-Drive mode
  */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
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
  hi2c1.Init.ClockSpeed = 400000;
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
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

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
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void initMPU6050(void) {
	//int eror = 0;
	uint8_t buffer;
	//MPU6050 Uyku modundan çık ve başlat
	buffer = 0x00;
	if (HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR << 1, PWR_MGMT_1_REG, 1, &buffer,
			1,
			HAL_MAX_DELAY) != HAL_OK) {
		eror = 1;
		Error_Handler();
	}

	// SMPLRT_DIV_REG --> yazarak veri hızını 1KHz olarak ayarla 8KHZ Jiroskop çalışma frekansı için
	//ivme ölçer çıkış frekansı 1khz olarak ayarlanıyor
	//yani Sample Rate = Gyroscope Output Rate / (1 + SMPLRT_DIV) ->>>1KHz=8KHz /(1+ SMPLRT_DIV=7)
	buffer = 0x07;
	if (HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR << 1, SMPLRT_DIV_REG, 1, &buffer,
			1,
			HAL_MAX_DELAY) != HAL_OK) {
		eror = 2;
		Error_Handler();
	}
	// GYRO CNFG --> +-500 derece/saniye -->08 veya --> +-1000 derece/saniye -->10
	buffer = 0x10;
	if (HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR << 1, GYRO_CNFG_REG, 1, &buffer,
			1,
			HAL_MAX_DELAY) != HAL_OK) {
		eror = 2;
		Error_Handler();
	}

	// ACC CNFG --> +-8g -->10
	buffer = 0x10;
	if (HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR << 1, ACC_CNFG_REG, 1, &buffer,
			1,
			HAL_MAX_DELAY) != HAL_OK) {
		eror = 3;
		Error_Handler();
	}

}

void readAccelMPU6050(int16_t *rawAccelX, int16_t *rawAccelY, int16_t *rawAccelZ) {

	uint8_t bufferAccel[6];
	// belirttiğim adresten itibaren veri bana gönder emri
	bufferAccel[0] = 0x3B;
	HAL_I2C_Master_Transmit(&hi2c1, MPU6050_ADDR << 1, bufferAccel, 1,
	HAL_MAX_DELAY);
	// 6 tane byte al
	HAL_I2C_Master_Receive(&hi2c1, MPU6050_ADDR << 1, bufferAccel, 6,
	HAL_MAX_DELAY);

	*rawAccelX = (bufferAccel[0] << 8 | bufferAccel[1]);
	*rawAccelY = (bufferAccel[2] << 8 | bufferAccel[3]);
	*rawAccelZ = (bufferAccel[4] << 8 | bufferAccel[5]);

}
void readGyroMPU6050(int16_t *rawGyroX, int16_t *rawGyroY, int16_t *rawGyroZ) {
	// belirttiğim adresten itibaren veri bana gönder emri
	uint8_t bufferGyro[6];
	bufferGyro[0] = 0x43;
	HAL_I2C_Master_Transmit(&hi2c1, MPU6050_ADDR << 1, bufferGyro, 1,
	HAL_MAX_DELAY);
	// 6 tane byte al
	HAL_I2C_Master_Receive(&hi2c1, MPU6050_ADDR << 1, bufferGyro, 6,
	HAL_MAX_DELAY);

	*rawGyroX = (bufferGyro[0] << 8 | bufferGyro[1]);
	*rawGyroY = (bufferGyro[2] << 8 | bufferGyro[3]);
	*rawGyroZ = (bufferGyro[4] << 8 | bufferGyro[5]);
}

void CalibrationGyro(float *offsetGyroX, float *offsetGyroY, float *offsetGyroZ) {

	int16_t rawGyroX, rawGyroY, rawGyroZ;
	for (int i = 0; i < 4000; i++) {

		readGyroMPU6050(&rawGyroX, &rawGyroY, &rawGyroZ);
		*offsetGyroX += rawGyroX;
		*offsetGyroY += rawGyroY;
		*offsetGyroZ += rawGyroZ;

		HAL_Delay(1);
	}
	*offsetGyroX /= 4000.0;
	*offsetGyroY /= 4000.0;
	*offsetGyroZ /= 4000.0;
}

void getCalibratedGyro(int16_t rawGyroX, int16_t rawGyroY, int16_t rawGyroZ, float *calibratedGyroX, float *calibratedGyroY, float *calibratedGyroZ) {

	*calibratedGyroX = (rawGyroX - offsetGyroX) / 32.8f; // /32.8 derece/saniye cinsinden dönüşümü için gerekli adım
	*calibratedGyroY = (rawGyroY - offsetGyroY) / 32.8f;
	*calibratedGyroZ = (rawGyroZ - offsetGyroZ) / 32.8f;

	*calibratedGyroX = *calibratedGyroX * 3.14159265359/180.0; // radyan/saniye cinsinden
	*calibratedGyroY = *calibratedGyroY * 3.14159265359/180.0;
	*calibratedGyroZ = *calibratedGyroZ * 3.14159265359/180.0;
}

// Kalibre edilmiş ivmeölçer değerlerini hesapla
void getCalibratedAccel(int16_t rawAccelX, int16_t rawAccelY, int16_t rawAccelZ, float *calibratedAccelX, float *calibratedAccelY, float *calibratedAccelZ) {

	const float offsetAccelX = 132.00;
	  const float offsetAccelY = 432.00;
	  const float offsetAccelZ = -54.00;
	  const float scaleCorrectionFactorAccelX = 1.04364324;
	  const float scaleCorrectionFactorAccelY = 0.936369419;
	  const float scaleCorrectionFactorAccelZ = 1.02683783;

	// Kalibre edilmiş değerleri hesapla
	*calibratedAccelX = (rawAccelX - offsetAccelX)
			* scaleCorrectionFactorAccelX/4096.0;
	*calibratedAccelY = (rawAccelY - offsetAccelY)
			* scaleCorrectionFactorAccelY/4096.0;
	*calibratedAccelZ = (rawAccelZ - offsetAccelZ)
			* scaleCorrectionFactorAccelZ/4096.0;

}
// verileri seri port üzerinden gönder
void sendData(float qw, float qx, float qy, float qz) {
    char buffer[100];
    snprintf(buffer, sizeof(buffer), "%.6f, %.6f, %.6f, %.6f\n", qw, qx, qy, qz);
    HAL_UART_Transmit(&huart2, (uint8_t*)buffer, strlen(buffer), HAL_MAX_DELAY);
}

void updateQuaternionFromGyro(float gx, float gy, float gz) {

    // geçen zaman aralığı hesapla
	double deltaTime =(double)(HAL_GetTick()-lastTimeGyro)/1000.0f;
	lastTimeGyro = HAL_GetTick();

    float norm = sqrt(gx * gx + gy * gy + gz * gz);
    if (norm > 0) {
        float angle = norm * deltaTime;
        float sinHalfAngle = sin(angle / 2.0);
        float cosHalfAngle = cos(angle / 2.0);
        float qDelta[4] = {cosHalfAngle, sinHalfAngle * (gx / norm), sinHalfAngle * (gy / norm), sinHalfAngle * (gz / norm)};


        // Kuaterniyon güncelleme: q_t+1 = q_t * q_delta --- buraada sadece GYRO'dan Kuaterniyon hesabı
        qw[0] = qt[0] * qDelta[0] - qt[1] * qDelta[1] - qt[2] * qDelta[2] - qt[3] * qDelta[3];
        qw[1] = qt[0] * qDelta[1] + qt[1] * qDelta[0] + qt[2] * qDelta[3] - qt[3] * qDelta[2];
        qw[2] = qt[0] * qDelta[2] - qt[1] * qDelta[3] + qt[2] * qDelta[0] + qt[3] * qDelta[1];
        qw[3] = qt[0] * qDelta[3] + qt[1] * qDelta[2] - qt[2] * qDelta[1] + qt[3] * qDelta[0];
        float normq = sqrt(qw[0] * qw[0] + qw[1] * qw[1] + qw[2] * qw[2] + qw[3] * qw[3]);
        qw[0] = qw[0]/normq;
        qw[1] = qw[1]/normq;
        qw[2] = qw[2]/normq;
        qw[3] = qw[3]/normq;
        //memcpy(qt, qw, sizeof(qw)); //eğer sadece gyro'dan veriyi göncelleme yapmak istiyorsak bunu kullanırız ama drift hatası oluşacak
    }
}

void updateQuaternionFromAccel(float ax, float ay, float az) {
	float qa[4] = {0, ax ,ay, az};

	float norm = sqrt(qw[0] * qw[0] + qw[1] * qw[1] + qw[2] * qw[2] + qw[3] * qw[3]);
	float conjugateqw[4] = {qw[0], -qw[1] ,-qw[2], -qw[3]};
	float inverseqw[4] = {conjugateqw[0]/(norm*norm), conjugateqw[1]/(norm*norm), conjugateqw[2]/(norm*norm), conjugateqw[3]/(norm*norm)};
	float qTemp[4];


	//qa_world = qw * qa_body * inverseqw iki adımla Kuaterniyon çarpımı yapacağız

	//qTemp = qw * qa_body
	qTemp[0] = qw[0] * qa[0] - qw[1] * qa[1] - qw[2] * qa[2] - qw[3] * qa[3];
	qTemp[1] = qw[0] * qa[1] + qw[1] * qa[0] + qw[2] * qa[3] - qw[3] * qa[2];
	qTemp[2] = qw[0] * qa[2] - qw[1] * qa[3] + qw[2] * qa[0] + qw[3] * qa[1];
	qTemp[3] = qw[0] * qa[3] + qw[1] * qa[2] - qw[2] * qa[1] + qw[3] * qa[0];

	//qa_world = qTemp * inverseqw
	qaWorld[0] = qTemp[0] * inverseqw[0] - qTemp[1] * inverseqw[1] - qTemp[2] * inverseqw[2] - qTemp[3] * inverseqw[3];
	qaWorld[1] = qTemp[0] * inverseqw[1] + qTemp[1] * inverseqw[0] + qTemp[2] * inverseqw[3] - qTemp[3] * inverseqw[2];
	qaWorld[2] = qTemp[0] * inverseqw[2] - qTemp[1] * inverseqw[3] + qTemp[2] * inverseqw[0] + qTemp[3] * inverseqw[1];
	qaWorld[3] = qTemp[0] * inverseqw[3] + qTemp[1] * inverseqw[2] - qTemp[2] * inverseqw[1] + qTemp[3] * inverseqw[0];

    float normq = sqrt(qaWorld[0] * qaWorld[0] + qaWorld[1] * qaWorld[1] + qaWorld[2] * qaWorld[2] + qaWorld[3] * qaWorld[3]);
    qaWorld[0] /= normq;
    qaWorld[1] /= normq;
    qaWorld[2] /= normq;
    qaWorld[3] /= normq;

}
void updateTiltCorrectionWithComplementaryFilter() {


	    // Dönüş ekseni ve dönüş açısını hesapla
	    float psi = acos(qaWorld[3]);
	    float nx = qaWorld[2], ny = -qaWorld[1], nz = 0;
	    float norm = sqrt(nx * nx + ny * ny + nz * nz);

        const float alpha = 0.95;
	    float sinHalfPsi = sin((1 - alpha) * psi / 2.0);
	    float cosHalfPsi = cos((1 - alpha) * psi / 2.0);

	    float qTilt[4] = {cosHalfPsi, sinHalfPsi * nx/norm, sinHalfPsi * ny/norm, sinHalfPsi * nz/norm};

	    // Kuaterniyon güncelleme: qc = qTilt * qGyro
	    float qc[4];
	    qc[0] = qTilt[0] * qw[0] - qTilt[1] * qw[1] - qTilt[2] * qw[2] - qTilt[3] * qw[3];
	    qc[1] = qTilt[0] * qw[1] + qTilt[1] * qw[0] + qTilt[2] * qw[3] - qTilt[3] * qw[2];
	    qc[2] = qTilt[0] * qw[2] - qTilt[1] * qw[3] + qTilt[2] * qw[0] + qTilt[3] * qw[1];
	    qc[3] = qTilt[0] * qw[3] + qTilt[1] * qw[2] - qTilt[2] * qw[1] + qTilt[3] * qw[0];

	    float normq = sqrt(qc[0] * qc[0] + qc[1] * qc[1] + qc[2] * qc[2] + qc[3] * qc[3]);
	    qc[0] /= normq;
	    qc[1] /= normq;
	    qc[2] /= normq;
	    qc[3] /= normq;

	   memcpy(qt, qc, sizeof(qc));

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
