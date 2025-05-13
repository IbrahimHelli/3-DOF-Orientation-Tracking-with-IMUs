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

#define HMC5883L_ADDRESS 0x1E

// HMC5883L register adresleri
#define HMC5883L_REG_CONFIG_A 0x00
#define HMC5883L_REG_CONFIG_B 0x01
#define HMC5883L_REG_MODE 0x02
#define HMC5883L_REG_DATA_X_MSB 0x03
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

float offsetGyroX = 0, offsetGyroY = 0, offsetGyroZ = 0;

float scaleCorrectionFactorAccelX, scaleCorrectionFactorAccelY,
		scaleCorrectionFactorAccelZ;
float offsetAccelX, offsetAccelY, offsetAccelZ;

float scaleCorrectionFactorMagX, scaleCorrectionFactorMagY,
		scaleCorrectionFactorMagZ;
float offsetMagX, offsetMagY, offsetMagZ;

double roll;
double pitch;
double yaw;
double q[4];
// Kalibrasyon için sınır değerler
int16_t minMagX = 32767, maxMagX = -32768;
int16_t minMagY = 32767, maxMagY = -32768;
int16_t minMagZ = 32767, maxMagZ = -32768;

///
int eror = 0;
int state = 0;
///
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_I2C1_Init(void);
/* USER CODE BEGIN PFP */
void initMPU6050(void);
void readAccelMPU6050(int16_t *rawAccelX, int16_t *rawAccelY,
		int16_t *rawAccelZ);
void readGyroMPU6050(int16_t *rawGyroX, int16_t *rawGyroY, int16_t *rawGyroZ);
void CalibrationGyro(float *offsetGyroX, float *offsetGyroY, float *offsetGyroZ);
void getCalibratedGyro(int16_t rawGyroX, int16_t rawGyroY, int16_t rawGyroZ,
		float *calibratedGyroX, float *calibratedGyroY, float *calibratedGyroZ);
void CalibrationAccel(float *scaleCorrectionFactorX,
		float *scaleCorrectionFactorY, float *scaleCorrectionFactorZ,
		float *offsetX, float *offsetY, float *offsetZ);
void getCalibratedAccel(int16_t rawAccelX, int16_t rawAccelY, int16_t rawAccelZ,
		float *calibratedAccelX, float *calibratedAccelY,
		float *calibratedAccelZ);
void initHMC5883L(void);
void readHMC5883L(int16_t *rawMagX, int16_t *rawMagY, int16_t *rawMagZ);
void updateCalibrationMag(int16_t rawMagX, int16_t rawMagY, int16_t rawMagZ,
		int16_t *minMagX, int16_t *maxMagX, int16_t *minMagY, int16_t *maxMagY,
		int16_t *minMagZ, int16_t *maxMagZ);
void getCalibratedMag(int16_t rawMagX, int16_t rawMagY, int16_t rawMagZ,
		int16_t minMagX, int16_t maxMagX, int16_t minMagY, int16_t maxMagY,
		int16_t minMagZ, int16_t maxMagZ, float *calibratedMagX,
		float *calibratedMagY, float *calibratedMagZ);

void calculateRollPitchYaw(float accelX, float accelY, float accelZ, float magX, float magY, float magZ, double *rollRad, double *pitchRad, double *yawRad);
void eularToQuaternion( double rollRad, double pitchRad, double yawRad);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void) {

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
	initHMC5883L();
	initMPU6050();
	CalibrationGyro(&offsetGyroX, &offsetGyroY, &offsetGyroZ);
	CalibrationAccel(&scaleCorrectionFactorAccelX, &scaleCorrectionFactorAccelY,
			&scaleCorrectionFactorAccelZ, &offsetAccelX, &offsetAccelY,
			&offsetAccelZ);

	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1) {
		//HAL_Delay(100);
		//HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
		///HAL_Delay(100);

		int16_t rawAccelX, rawAccelY, rawAccelZ;       // Ham ivmeölçer verileri
		float calibratedAccelX, calibratedAccelY, calibratedAccelZ; // Kalibre edilmiş ivmeölçer verileri

		int16_t rawGyroX, rawGyroY, rawGyroZ;           // Ham jiroskop verileri
		float calibratedGyroX, calibratedGyroY, calibratedGyroZ; // Kalibre edilmiş jiroskop verileri derece/saniye cinsinden

		int16_t rawMagX, rawMagY, rawMagZ;          // Ham manyetometre verileri
		float calibratedMagX, calibratedMagY, calibratedMagZ; // Kalibre edilmiş manyetometre verileri

		double rollRad, pitchRad, yawRad;

		readAccelMPU6050(&rawAccelX, &rawAccelY, &rawAccelZ);

		readGyroMPU6050(&rawGyroX, &rawGyroY, &rawGyroZ);

		getCalibratedAccel(rawAccelX, rawAccelY, rawAccelZ, &calibratedAccelX,
				&calibratedAccelY, &calibratedAccelZ);
		getCalibratedGyro(rawGyroX, rawGyroY, rawGyroZ, &calibratedGyroX,
				&calibratedGyroY, &calibratedGyroZ);

		// HMC5883L manyetometre verilerini oku
		readHMC5883L(&rawMagX, &rawMagY, &rawMagZ);

		// Kalibrasyon verilerini güncelle
		updateCalibrationMag(rawMagX, rawMagY, rawMagZ, &minMagX, &maxMagX,
				&minMagY, &maxMagY, &minMagZ, &maxMagZ);

		// Kalibre edilmiş değerleri hesapla
		getCalibratedMag(rawMagX, rawMagY, rawMagZ, minMagX, maxMagX, minMagY,
				maxMagY, minMagZ, maxMagZ, &calibratedMagX, &calibratedMagY,
				&calibratedMagZ);

		calculateRollPitchYaw(calibratedAccelX, calibratedAccelY,
				calibratedAccelZ, calibratedMagX, calibratedMagZ,
				calibratedMagY, &rollRad, &pitchRad, &yawRad);

		eularToQuaternion(rollRad, pitchRad, yawRad);

		/* USER CODE END WHILE */

		/* USER CODE BEGIN 3 */
	}
	/* USER CODE END 3 */
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
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
		Error_Handler();
	}

	/** Activate the Over-Drive mode
	 */
	if (HAL_PWREx_EnableOverDrive() != HAL_OK) {
		Error_Handler();
	}

	/** Initializes the CPU, AHB and APB buses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
			| RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK) {
		Error_Handler();
	}
}

/**
 * @brief I2C1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_I2C1_Init(void) {

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
	if (HAL_I2C_Init(&hi2c1) != HAL_OK) {
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
static void MX_USART2_UART_Init(void) {

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
	if (HAL_UART_Init(&huart2) != HAL_OK) {
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
static void MX_GPIO_Init(void) {
	GPIO_InitTypeDef GPIO_InitStruct = { 0 };
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

	// GYRO CNFG --> +-500 derece/saniye -->08
	buffer = 0x08;
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
void readAccelMPU6050(int16_t *rawAccelX, int16_t *rawAccelY,
		int16_t *rawAccelZ) {

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
void getCalibratedGyro(int16_t rawGyroX, int16_t rawGyroY, int16_t rawGyroZ,
		float *calibratedGyroX, float *calibratedGyroY, float *calibratedGyroZ) {

	*calibratedGyroX = (rawGyroX - offsetGyroX) / 65.5f; // /65.5 derece/saniye cinsinden dönüşümü için gerekli adım
	*calibratedGyroY = (rawGyroY - offsetGyroY) / 65.5f;
	*calibratedGyroZ = (rawGyroZ - offsetGyroZ) / 65.5f;
}
void CalibrationAccel(float *scaleCorrectionFactorX,
		float *scaleCorrectionFactorY, float *scaleCorrectionFactorZ,
		float *offsetX, float *offsetY, float *offsetZ) {

	// Zamanlama
	uint32_t LoopTimer;

	int16_t minAccelX = 32767, maxAccelX = -32768;
	int16_t minAccelY = 32767, maxAccelY = -32768;
	int16_t minAccelZ = 32767, maxAccelZ = -32768;

	int16_t rawX, rawY, rawZ;
	LoopTimer = HAL_GetTick();
	state = 1;
	while (HAL_GetTick() - LoopTimer < 4000)
		;  // Bekle
	state = 2;

	for (int i = 0; i < 4000; i++) {
		readAccelMPU6050(&rawX, &rawY, &rawZ);
		if (rawX > maxAccelX)
			maxAccelX = rawX;
		if (rawX < minAccelX)
			minAccelX = rawX;

		HAL_Delay(1);
	}
	LoopTimer = HAL_GetTick();
	state = 3;

	while (HAL_GetTick() - LoopTimer < 4000)
		;  //Bekle
	state = 4;
	for (int i = 0; i < 4000; i++) {
		readAccelMPU6050(&rawX, &rawY, &rawZ);
		if (rawX > maxAccelX)
			maxAccelX = rawX;
		if (rawX < minAccelX)
			minAccelX = rawX;

		HAL_Delay(1);
	}
	LoopTimer = HAL_GetTick();
	state = 5;

	while (HAL_GetTick() - LoopTimer < 4000)
		;  //Bekle
	state = 6;
	for (int i = 0; i < 4000; i++) {
		readAccelMPU6050(&rawX, &rawY, &rawZ);
		if (rawY > maxAccelY)
			maxAccelY = rawY;
		if (rawY < minAccelY)
			minAccelY = rawY;

		HAL_Delay(1);
	}
	LoopTimer = HAL_GetTick();
	state = 7;

	while (HAL_GetTick() - LoopTimer < 4000)
		;  //Bekle
	state = 8;
	for (int i = 0; i < 4000; i++) {
		readAccelMPU6050(&rawX, &rawY, &rawZ);
		if (rawY > maxAccelY)
			maxAccelY = rawY;
		if (rawY < minAccelY)
			minAccelY = rawY;

		HAL_Delay(1);
	}
	LoopTimer = HAL_GetTick();
	state = 9;

	while (HAL_GetTick() - LoopTimer < 4000)
		;  //Bekle
	state = 10;
	for (int i = 0; i < 4000; i++) {
		readAccelMPU6050(&rawX, &rawY, &rawZ);
		if (rawZ > maxAccelZ)
			maxAccelZ = rawZ;
		if (rawZ < minAccelZ)
			minAccelZ = rawZ;

		HAL_Delay(1);
	}
	LoopTimer = HAL_GetTick();
	state = 11;

	while (HAL_GetTick() - LoopTimer < 4000)
		;  //Bekle
	state = 12;
	for (int i = 0; i < 4000; i++) {
		readAccelMPU6050(&rawX, &rawY, &rawZ);
		if (rawZ > maxAccelZ)
			maxAccelZ = rawZ;
		if (rawZ < minAccelZ)
			minAccelZ = rawZ;

		HAL_Delay(1);
	}

	// Offset hesapla (Hard Iron Düzeltmesi)
	*offsetX = (maxAccelX + minAccelX) / 2.0;
	*offsetY = (maxAccelY + minAccelY) / 2.0;
	*offsetZ = (maxAccelZ + minAccelZ) / 2.0;

	// Ölçek hesapla (Soft Iron Düzeltmesi)
	float scaleX = (maxAccelX - minAccelX) / 2.0;
	float scaleY = (maxAccelY - minAccelY) / 2.0;
	float scaleZ = (maxAccelZ - minAccelZ) / 2.0;
	float avgScale = (scaleX + scaleY + scaleZ) / 3.0;

	// ölçek düzeltme faktör değerleri hesapla
	*scaleCorrectionFactorX = avgScale / scaleX;
	*scaleCorrectionFactorY = avgScale / scaleY;
	*scaleCorrectionFactorZ = avgScale / scaleZ;

	state = 0;

}

// Kalibre edilmiş ivmeölçer değerlerini hesapla
void getCalibratedAccel(int16_t rawAccelX, int16_t rawAccelY, int16_t rawAccelZ,
		float *calibratedAccelX, float *calibratedAccelY,
		float *calibratedAccelZ) {

	// Kalibre edilmiş değerleri hesapla
	*calibratedAccelX = (rawAccelX - offsetAccelX)
			* scaleCorrectionFactorAccelX;
	*calibratedAccelY = (rawAccelY - offsetAccelY)
			* scaleCorrectionFactorAccelY;
	*calibratedAccelZ = (rawAccelZ - offsetAccelZ)
			* scaleCorrectionFactorAccelZ;

}

// // HMC5883L ayarları
void initHMC5883L(void) {

	//int eror = 0;
	uint8_t buffer;
	// Mag CNFG -->  Config A register adresi --> HMC5883L_REG_CONFIG_A ---- Örnekleme oranı: 8G range, 200Hz ölçüm modu -->  0x1C
	buffer = 0x1C;
	if (HAL_I2C_Mem_Write(&hi2c1, HMC5883L_ADDRESS << 1, HMC5883L_REG_CONFIG_A, 1, &buffer, 1,
	HAL_MAX_DELAY) != HAL_OK) {
		eror = 4;
		Error_Handler();
	}

	// Mag CNFG --> Config B register adresi --> HMC5883L_REG_CONFIG_B ---- Kazanç ayarı: ±1.3 Ga --> 0xA0
	buffer = 0xA0;
	if (HAL_I2C_Mem_Write(&hi2c1, HMC5883L_ADDRESS << 1, HMC5883L_REG_CONFIG_B, 1, &buffer, 1,
	HAL_MAX_DELAY) != HAL_OK) {
		eror = 5;
		Error_Handler();
	}

	// Mag CNFG --> Mode register adresi --> HMC5883L_REG_MODE ---- Sürekli ölçüm modu --> 0x00
	buffer = 0x00;
	if (HAL_I2C_Mem_Write(&hi2c1, HMC5883L_ADDRESS << 1, HMC5883L_REG_MODE, 1, &buffer, 1,
	HAL_MAX_DELAY) != HAL_OK) {
		eror = 6;
		Error_Handler();
	}

}

// HMC5883L manyetometre verilerini oku
void readHMC5883L(int16_t *rawMagX, int16_t *rawMagY, int16_t *rawMagZ) {
	// belirttiğim adresten itibaren veri bana gönder emri --- HMC5883L_REG_DATA_X_MSB = X ekseni MSB register adresi
	uint8_t bufferMag[6];
	bufferMag[0] = HMC5883L_REG_DATA_X_MSB;
	HAL_I2C_Master_Transmit(&hi2c1, HMC5883L_ADDRESS << 1, bufferMag, 1,
	HAL_MAX_DELAY);
	// 6 tane byte al
	HAL_I2C_Master_Receive(&hi2c1, HMC5883L_ADDRESS << 1, bufferMag, 6,
	HAL_MAX_DELAY);

	*rawMagX = (bufferMag[0] << 8 | bufferMag[1]);
	*rawMagY = (bufferMag[2] << 8 | bufferMag[3]);
	*rawMagZ = (bufferMag[4] << 8 | bufferMag[5]);
}

//  manyetometre kalibrasyon verilerini güncelle
void updateCalibrationMag(int16_t rawMagX, int16_t rawMagY, int16_t rawMagZ,
		int16_t *minMagX, int16_t *maxMagX, int16_t *minMagY, int16_t *maxMagY,
		int16_t *minMagZ, int16_t *maxMagZ) {
	if (rawMagX < *minMagX)
		*minMagX = rawMagX;
	if (rawMagX > *maxMagX)
		*maxMagX = rawMagX;
	if (rawMagY < *minMagY)
		*minMagY = rawMagY;
	if (rawMagY > *maxMagY)
		*maxMagY = rawMagY;
	if (rawMagZ < *minMagZ)
		*minMagZ = rawMagZ;
	if (rawMagZ > *maxMagZ)
		*maxMagZ = rawMagZ;
}

// Kalibre edilmiş Manyetometre değerlerini hesapla
void getCalibratedMag(int16_t rawMagX, int16_t rawMagY, int16_t rawMagZ,
		int16_t minMagX, int16_t maxMagX, int16_t minMagY, int16_t maxMagY,
		int16_t minMagZ, int16_t maxMagZ, float *calibratedMagX,
		float *calibratedMagY, float *calibratedMagZ) {
	// Offset hesapla (Hard Iron Düzeltmesi)
	offsetMagX = (maxMagX + minMagX) / 2.0;
	offsetMagY = (maxMagY + minMagY) / 2.0;
	offsetMagZ = (maxMagZ + minMagZ) / 2.0;

	// Ölçek hesapla (Soft Iron Düzeltmesi)
	float scaleX = (maxMagX - minMagX) / 2.0;
	float scaleY = (maxMagY - minMagY) / 2.0;
	float scaleZ = (maxMagZ - minMagZ) / 2.0;
	float avgScale = (scaleX + scaleY + scaleZ) / 3.0;

	scaleCorrectionFactorMagX = (avgScale / scaleX);
	scaleCorrectionFactorMagY = (avgScale / scaleY);
	scaleCorrectionFactorMagZ = (avgScale / scaleZ);

	// Kalibre edilmiş değerleri hesapla
	*calibratedMagX = (rawMagX - offsetMagX) * scaleCorrectionFactorMagX;
	*calibratedMagY = (rawMagY - offsetMagY) * scaleCorrectionFactorMagY;
	*calibratedMagZ = (rawMagZ - offsetMagZ) * scaleCorrectionFactorMagZ;

}


// Eğim telafisi (Tilt Compensation), pitch, roll ve Yaw hesaplama
void calculateRollPitchYaw(float accelX, float accelY, float accelZ, float magX, float magY, float magZ, double *rollRad, double *pitchRad, double *yawRad) {

	// değerleri normalleştir
	double normAccel =  sqrt(accelX*accelX + accelY*accelY + accelZ*accelZ);
	double	ax = accelX/normAccel;
	double	ay = accelY/normAccel;
	double	az = accelZ/normAccel;

	double normMag =  sqrt(magX*magX + magY*magY + magZ*magZ);
	double	mx = magX/normMag;
	double	my = magY/normMag;
	double	mz = magZ/normMag;
	/// Roll ve Pitch açılarını hesapla (Radyan)
	*rollRad = atan2(ay,sqrt(ax * ax + az * az));
	*pitchRad = atan2(-ax,sqrt(ay * ay + az * az)); //radyan cinsinden


	//Tilt Combinsation (Eğim telafisi hesapla)
	double bx = mx * cos(*pitchRad) + mz * sin(*pitchRad);
	double by = mx * sin(*rollRad) * sin(*pitchRad)+my * cos(*rollRad)-mz * sin(*rollRad) * cos(*pitchRad);

	//double bx = mx * cos(*rollRad) + sin(*rollRad)*(my * sin(*pitchRad) + mz * cos(*pitchRad));
	//double by = mz * sin(*pitchRad) - my * cos(*pitchRad);

	// X ve Y eksenlerinden Yaw (heading/azimut) açısını hesapla
	*yawRad = atan2(-by, -bx);

	roll = *rollRad* 180.0 / 3.141592654;
	pitch = *pitchRad* 180.0 / 3.141592654;
	yaw = *yawRad* 180.0 / 3.141592654;
	if (yaw < 0)
		yaw += 360;  // 0-360 derece aralığına taşı
}

void eularToQuaternion( double rollRad, double pitchRad, double yawRad){

	double sinPitch2 = sin(rollRad/2.0);
	double sinRoll2 = sin(rollRad/2.0);
	double sinYaw2 = sin(rollRad/2.0);

	double cosPitch2 = cos(rollRad/2.0);
	double cosRoll2 = cos(rollRad/2.0);
	double cosYaw2 = cos(rollRad/2.0);

	q[0]= cosPitch2*cosRoll2*cosYaw2 + sinPitch2*sinRoll2*sinYaw2;
	q[1]= sinPitch2*cosRoll2*cosYaw2 - cosPitch2*sinRoll2*sinYaw2;
	q[2]= cosPitch2*sinRoll2*cosYaw2 + sinPitch2*cosRoll2*sinYaw2;
	q[3]= cosPitch2*cosRoll2*sinYaw2 - sinPitch2*sinRoll2*cosYaw2;

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
