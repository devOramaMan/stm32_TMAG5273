/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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
#include "TMAG5273.h"

//SWO printf
#include "stdio.h"
//not to be used in mc api (for memset)
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct Sensor_Str
{
  char Bx[10];
  char BxRaw[10];
  char By[10];
  char ByRaw[10];
  char Bz[10];
  char BzRaw[10];
  char T[10];
  char angle[10];
  char magn[10];
}Sensor_Str_t;


/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define USE_TMAG5273_A1 1

#define getName(var)  #var
#define BYTE_TO_BINARY_PATTERN "%c%c%c%c%c%c%c%c\n"
#define BYTE_TO_BINARY(byte)  \
  (byte & 0x80 ? '1' : '0'), \
  (byte & 0x40 ? '1' : '0'), \
  (byte & 0x20 ? '1' : '0'), \
  (byte & 0x10 ? '1' : '0'), \
  (byte & 0x08 ? '1' : '0'), \
  (byte & 0x04 ? '1' : '0'), \
  (byte & 0x02 ? '1' : '0'), \
  (byte & 0x01 ? '1' : '0')
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
    
#if (USE_TMAG5273_A1 == 0) && (USE_TMAG5273_A2 == 0)
#error "No Sensor defined"
#endif

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;
I2C_HandleTypeDef hi2c2;
DMA_HandleTypeDef hdma_i2c1_rx;
DMA_HandleTypeDef hdma_i2c1_tx;
DMA_HandleTypeDef hdma_i2c2_rx;
DMA_HandleTypeDef hdma_i2c2_tx;

UART_HandleTypeDef hlpuart1;

/* USER CODE BEGIN PV */
#if (USE_TMAG5273_A1 == 1)
TMAG5273_Handle_t htmag5273_1 = 
{
  .pI2c = &hi2c1,
  .address = TMAG5273_DEFAULT_ADDR, //0x41,// 
  .magTempcoMode = TMAG5273_NO_MAG_TEMPCO,
  .convAvgMode = TMAG5273_CONV_AVG_32X,
  .readMode = TMAG5273_READ_MODE_STANDARD,
  .lplnMode = TMAG5273_LOW_NOISE,
  .operatingMode = TMAG5273_OPERATING_MODE_STANDBY,
  .magXYRange = TMAG5273_MAG_RANGE_40MT_133MT,
  .magZRange = TMAG5273_MAG_RANGE_80MT_266MT,
  .tempChEn = TMAG5273_TEMP_CH_DISABLED,
  .angEn = TMAG5273_ANG_X_Y,
  .magChEn = TMAG5276_MAG_X_Y,
  .crcEna = TMAG5273_CRC_EN,
  .sensor_id = 0
};
#endif

#if (USE_TMAG5273_A2 == 1)
TMAG5273_Handle_t htmag5273_2 = 
{
  .pI2c = &hi2c2,
  .address = TMAG5273_DEFAULT_ADDR,// 0x41
  .magTempcoMode = TMAG5273_NO_MAG_TEMPCO,
  .convAvgMode = TMAG5273_CONV_AVG_32X,
  .readMode = TMAG5273_READ_MODE_STANDARD,
  .lplnMode = TMAG5273_LOW_NOISE,
  .operatingMode = TMAG5273_OPERATING_MODE_STANDBY,
  .magXYRange = TMAG5273_MAG_RANGE_40MT_133MT,
  .magZRange = TMAG5273_MAG_RANGE_80MT_266MT,
  .tempChEn = TMAG5273_TEMP_CH_DISABLED,
  .angEn = TMAG5273_ANG_X_Y,
  .magChEn = TMAG5276_MAG_X_Y,
  .crcEna = TMAG5273_CRC_EN,
  .sensor_id = 1
};
#endif
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_I2C1_Init(void);
static void MX_LPUART1_UART_Init(void);
static void MX_I2C2_Init(void);
/* USER CODE BEGIN PFP */
uint8_t read_all_sensor_data(TMAG5273_Handle_t * pHandle, char * pBuff, uint32_t * len);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/**
  * @brief  Test CRC (ref 7.5.1.3.6 I2C Read CRC)
  * I2C Data 00h : CRC = F3h
  * I2C Data FFh : CRC = 00h
  * I2C Data 80h : CRC = 7Ah
  * I2C Data 4Ch : CRC = 10h
  * I2C Data E0h : CRC = 5Dh
  * I2C Data 00000000h : CRC = D1h
  * I2C Data FFFFFFFFh : CRC = 0Fh
  * @retval void
  */
void testCrc(void)
{
  uint8_t data[10];
  data[0] = 0x00;
    
  if(TMAG5273_calculateCRC( data, 1 ) != 0xF3)
    Error_Handler();
  
  data[0] = 0xFF;
  if(TMAG5273_calculateCRC( data, 1 ) != 0x00)
    Error_Handler();
  
  
  data[0] = 0xFF;
  data[1] = 0xFF;
  data[2] = 0xFF;
  data[3] = 0xFF;
  if(TMAG5273_calculateCRC( data, 4 ) != 0x0F)
    Error_Handler();
  
  
  data[0] = 0;
  data[1] = 0;
  data[2] = 0;
  data[3] = 0;
  if(TMAG5273_calculateCRC( data, 4 ) != 0xD1)
    Error_Handler();
  
}

/**
  * @brief  read sensor data, appends printout string and trigger conversion
  * @param TMAG5273_Handle_t the sensor handle
  * @param int8_t pointer the printout buffer
  * @param uint16_t pointer the Populated buffer length (appends)
  * @retval uint8_t 0 Success else failed
  */
uint8_t read_all_sensor_data(TMAG5273_Handle_t * pHandle, char * pBuff, uint32_t * len)
{
  uint8_t ret;
  
  TMAG5273_Sensor_Data_t sensor_data;
  Sensor_Str_t sensor_str;
  
  memset((void*)&sensor_data, 0, sizeof(sensor_data));

  ret = TMAG5273_ReadSensorData(pHandle, &sensor_data);
  
  
  sprintf(sensor_str.Bx,"%5.4f", sensor_data.axis.Bx);
  sprintf(sensor_str.BxRaw,"%d", htmag5273_1.raw[1]);
  sprintf(sensor_str.By,"%5.4f", sensor_data.axis.By);
  sprintf(sensor_str.ByRaw,"%d", htmag5273_1.raw[2]);
  sprintf(sensor_str.Bz,"%5.4f", sensor_data.axis.Bz);
  sprintf(sensor_str.BzRaw,"%d", htmag5273_1.raw[3]);
  sprintf(sensor_str.T,"%5.4f", sensor_data.temperature);
  sprintf(sensor_str.angle,"%3.2f", sensor_data.angle);
  sprintf(sensor_str.magn,"%5.4f", sensor_data.magnitude);
      
  *len += sprintf( pBuff,"%9s, %9s, %9s, %9s, %9s, %9s, %9s,     0x%02X, %8s, %9s,", \
          sensor_str.Bx, sensor_str.BxRaw, sensor_str.By, sensor_str.ByRaw, sensor_str.Bz, sensor_str.BzRaw,  sensor_str.T, sensor_data.status, sensor_str.angle, sensor_str.magn);
  
  TMAG5273_TriggerConversion(pHandle);
  
  return ret;
}


/**
  * @brief  read and print sensor configuration
  * @param TMAG5273_Handle_t the sensor handle
  * @retval void
  */
void print_sensor_config(TMAG5273_Handle_t * pHandle)
{
  uint8_t ret;
  uint8_t sensor_id = pHandle->sensor_id + 1;
  
  printf("*****************************************************************\n"  );
  printf("SENSOR %d Info: \n", sensor_id );
  printf("I2C_ADDRESS:            0x%02X\n", pHandle->address);
  printf("DEVICE_ID:              0x%02X\n", pHandle->deviceId);
  printf("MANUFACTURER_ID:      0x%04X\n", pHandle->manufactorerId);
  
  printf("*****************************************************************\n"  );
  printf("SENSOR %d Configuration: \n", sensor_id);
  TMAG5273_ReadRegister(pHandle, DEVICE_CONFIG_1, 1 , &ret);
  printf("DEVICE_CONFIG_1 (0x%X):      "BYTE_TO_BINARY_PATTERN, ret, BYTE_TO_BINARY(ret));
  
  TMAG5273_ReadRegister(pHandle, DEVICE_CONFIG_2, 1 , &ret);
  printf("DEVICE_CONFIG_2 (0x%2X):      "BYTE_TO_BINARY_PATTERN, ret, BYTE_TO_BINARY(ret));
  
  TMAG5273_ReadRegister(pHandle, SENSOR_CONFIG_1, 1 , &ret);
  printf("SENSOR_CONFIG_1 (0x%02X):      "BYTE_TO_BINARY_PATTERN, ret, BYTE_TO_BINARY(ret));
  
  TMAG5273_ReadRegister(pHandle, SENSOR_CONFIG_2, 1 , &ret);
  printf("SENSOR_CONFIG_2 (0x%02X):      "BYTE_TO_BINARY_PATTERN, ret, BYTE_TO_BINARY(ret));
  
  TMAG5273_ReadRegister(pHandle, X_THR_CONFIG, 1 , &ret);
  printf("X_THR_CONFIG (0x%02X):         "BYTE_TO_BINARY_PATTERN, ret, BYTE_TO_BINARY(ret));
  
  TMAG5273_ReadRegister(pHandle, Y_THR_CONFIG, 1 , &ret);
  printf("Y_THR_CONFIG (0x%02X):         "BYTE_TO_BINARY_PATTERN, ret, BYTE_TO_BINARY(ret));
  
  TMAG5273_ReadRegister(pHandle, Z_THR_CONFIG, 1 , &ret);
  printf("Z_THR_CONFIG (0x%02X):         "BYTE_TO_BINARY_PATTERN, ret, BYTE_TO_BINARY(ret));
  
  TMAG5273_ReadRegister(pHandle, T_CONFIG, 1 , &ret);
  printf("T_CONFIG (0x%02X):             "BYTE_TO_BINARY_PATTERN, ret, BYTE_TO_BINARY(ret));
  
  TMAG5273_ReadRegister(pHandle, INT_CONFIG_1, 1 , &ret);
  printf("INT_CONFIG_1 (0x%02X):         "BYTE_TO_BINARY_PATTERN, ret, BYTE_TO_BINARY(ret));
  
  TMAG5273_ReadRegister(pHandle, MAG_GAIN_CONFIG, 1 , &ret);
  printf("MAG_GAIN_CONFIG (0x%02X):      "BYTE_TO_BINARY_PATTERN, ret, BYTE_TO_BINARY(ret));
  
  TMAG5273_ReadRegister(pHandle, MAG_OFFSET_CONFIG_1, 1 , &ret);
  printf("MAG_OFFSET_CONFIG_1 (0x%2X):  "BYTE_TO_BINARY_PATTERN, ret, BYTE_TO_BINARY(ret));
  
  TMAG5273_ReadRegister(pHandle, MAG_OFFSET_CONFIG_2, 1 , &ret);
  printf("MAG_OFFSET_CONFIG_2 (0x%2X):  "BYTE_TO_BINARY_PATTERN, ret, BYTE_TO_BINARY(ret));
  
  TMAG5273_ReadRegister(pHandle, DEVICE_STATUS, 1 , &ret);
  printf("DEVICE_STATUS (0x%2X):        "BYTE_TO_BINARY_PATTERN, ret, BYTE_TO_BINARY(ret));
  
  printf("*****************************************************************\n\n"  );
}





/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
  uint8_t retA1 = 0 , retA2 = 0;
  uint32_t timeMs;
  char timeMsStr[25];
  uint8_t data[100];
  uint8_t cnt = 0;
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
  MX_DMA_Init();
  MX_I2C1_Init();
  MX_LPUART1_UART_Init();
  MX_I2C2_Init();
  /* USER CODE BEGIN 2 */
  
  testCrc();
  
  //Test Init
#if (USE_TMAG5273_A1 == 1)
  
  TMAG5273_Init(&htmag5273_1);
  
  HAL_Delay(20);
  //TMAG5273_ModifyI2CAddress(&htmag5273_1, 0x41); 
//  TMAG5273_ConfigOperatingMode(&htmag5273_1, TMAG5273_OPERATING_MODE_STANDBY);
//  TMAG5273_ConfigReadMode(&htmag5273_1, TMAG5273_READ_MODE_SENSOR8); 
//  //TMAG5273_ConfigXYMagRange(&htmag5273_1, TMAG5273_MAG_RANGE_40MT_133MT);
//  TMAG5273_ConfigZMagRange(&htmag5273_1, TMAG5273_MAG_RANGE_80MT_266MT); 
//  TMAG5273_ConfigLplnMode(&htmag5273_1, TMAG5273_LOW_NOISE); 
//  TMAG5273_ConfigMagTempcoMode(&htmag5273_1, TMAG5273_MAG_TEMPCO_NdBFe); 
//  TMAG5273_ConfigConvAvgMode(&htmag5273_1, TMAG5273_CONV_AVG_32X);
//  TMAG5273_ConfigTempChEnabled(&htmag5273_1, 1); 
//    TMAG5273_ConfigCrc(&htmag5273_1, TMAG5273_CRC_EN);
  
//  print_sensor_config(&htmag5273_1);
  
//  TMAG5273_TriggerConversion(&htmag5273_1);
  
  HAL_Delay(1000);
  TMAG5273_EnableTrigger( &htmag5273_1, 1);
  printf("Ret %d, reg 0x%x, Data [ 0x%x, 0x%x, 0x%x, 0x%x ], crc 0x%x\n",TMAG5273_ReadRegister( &htmag5273_1, 0, 4, data ), 0, data[0], data[1], data[2], data[3], htmag5273_1.lastCrc );
    
//  printf("Test Read Temperature %f\n", TMAG5273_ReadTemperature(&htmag5273_1));
  
//  TMAG5273_ConfigCrc(&htmag5273_1, TMAG5273_CRC_EN);
// TMAG5273_EnableTrigger( &htmag5273_1, 1);
//  TMAG5273_TriggerConversion(&htmag5273_1);
//  while( cnt++ < (DEVICE_STATUS -4))
//  { 
//    HAL_Delay(1000);
//    printf("Ret %d, reg 0x%x, Data [ 0x%x, 0x%x, 0x%x, 0x%x ], crc 0x%x\n",TMAG5273_ReadRegister( &htmag5273_1, cnt, 4, data ), cnt, data[0], data[1], data[2], data[3], data[4] );
//  }
  
  cnt = 0;
  while( cnt++ < 10)
  { 
    TMAG5273_TriggerConversion(&htmag5273_1);
    HAL_Delay(1000);
    printf("Ret %d, reg 0x%x, Data [ 0x%x, 0x%x, 0x%x, 0x%x ], crc 0x%x\n",TMAG5273_ReadRegister( &htmag5273_1, X_MSB_RESULT, 4, data ), X_MSB_RESULT, data[0], data[1], data[2], data[3], data[4] );
  }
  
  TMAG5273_EnableTrigger( &htmag5273_1, 0);
  
  TMAG5273_ConfigReadMode(&htmag5273_1, TMAG5273_READ_MODE_SENSOR8); 
  
  TMAG5273_ReadRegister( &htmag5273_1, T_MSB_RESULT, 20, data );
  
  TMAG5273_TriggerConversion(&htmag5273_1);
  
  HAL_Delay(50);
  
  TMAG5273_ConfigReadMode(&htmag5273_1, TMAG5273_READ_MODE_SENSOR16); 
  
  TMAG5273_ReadRegister( &htmag5273_1, T_MSB_RESULT, 20, data );
  
  TMAG5273_ConfigReadMode(&htmag5273_1, TMAG5273_READ_MODE_STANDARD); 
  
  
  
  
#endif
  
#if (USE_TMAG5273_A2 == 1)
  
  TMAG5273_Init(&htmag5273_2);
  TMAG5273_ModifyI2CAddress(&htmag5273_2, 0x42); 
  TMAG5273_ConfigOperatingMode(&htmag5273_2, TMAG5273_OPERATING_MODE_STANDBY); 
//  TMAG5273_ConfigReadMode(&htmag5273_2, TMAG5273_READ_MODE_SENSOR8); 
  TMAG5273_ConfigZMagRange(&htmag5273_2, TMAG5273_MAG_RANGE_80MT_266MT);
  //TMAG5273_ConfigXYMagRange(&htmag5273_2, TMAG5273_MAG_RANGE_40MT_133MT);
  TMAG5273_ConfigLplnMode(&htmag5273_2, TMAG5273_LOW_NOISE);
  TMAG5273_ConfigMagTempcoMode(&htmag5273_2, TMAG5273_MAG_TEMPCO_NdBFe);
  TMAG5273_ConfigConvAvgMode(&htmag5273_2, TMAG5273_CONV_AVG_32X);
  TMAG5273_ConfigTempChEnabled(&htmag5273_2, 1);
  
  print_sensor_config(&htmag5273_2);
  
  TMAG5273_TriggerConversion(&htmag5273_2);
  
  HAL_Delay(50);
  
  printf("Test Read Temperature %f\n", TMAG5273_ReadTemperature(&htmag5273_1));
  TMAG5273_ReadRegister( &htmag5273_2, T_MSB_RESULT, 4, data );
  
#endif
  
  
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  
  HAL_Delay(500);
#if (USE_TMAG5273_A1 == 1) && (USE_TMAG5273_A2 == 1)
  printf("%20s, %9s, %9s, %9s, %9s, %9s, %9s, %9s, %6s, %8s, %9s, \
%9s, %9s, %9s, %9s, %9s, %9s, %9s, %6s, %8s, %9s, %s\n","Time ms", "Dev1 X mT", "X Raw", "Y mT", "Y Raw", "Z mT", "Z Raw", "Temp", "ConvStat", "Angle", "Magn mT", \
         "Dev2 X mT", "X Raw", "Y mT", "Y Raw", "Z mT", "Z Raw", "Temp", "ConvStat", "Angle", "Magn mT", "Stat");
#else
  printf("%20s, %9s, %9s, %9s, %9s, %9s, %9s, %9s, %6s, %8s, %9s, %s\n"\
    ,"Time ms", "Dev X mT", "X Raw", "Y mT", "Y Raw", "Z mT", "Z Raw", "Temp", "ConvStat", "Angle", "Magn mT", "Stat");
#endif
  
#if (USE_TMAG5273_A1 == 1)
  TMAG5273_TriggerConversion(&htmag5273_1);
#endif
  
#if (USE_TMAG5273_A2 == 1)
  TMAG5273_TriggerConversion(&htmag5273_2);
#endif
  
  HAL_Delay(50);
  
  while (1)
  {
    char print_buffer[512];
    
    uint32_t size = 0;
    timeMs = HAL_GetTick();
    
    memset(print_buffer, 0 , sizeof(print_buffer));
    
      
    sprintf(timeMsStr,"%d", timeMs);
    size = sprintf(print_buffer,"%20s, ", timeMsStr);    
    
#if ( USE_TMAG5273_A1 == 1 )
    retA1 = read_all_sensor_data(&htmag5273_1, &print_buffer[size], &size);
#endif
    
#if ( USE_TMAG5273_A2 == 1 )
    retA2 = read_all_sensor_data(&htmag5273_2, &print_buffer[size], &size);
#endif
    
    if(!retA1 && !retA2)
      size += sprintf(&print_buffer[size]," OK\n");
    else if(!retA1 && retA2)
      size += sprintf(&print_buffer[size]," Sensor 2 Failed\n");
    else if(retA1 && !retA2)
      size += sprintf(&print_buffer[size]," Sensor 1 Failed\n");
    else if(retA1 && retA2)
      size += sprintf(&print_buffer[size]," Sensor 1 and 2 Failed\n");
    
    printf(print_buffer);
      
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    HAL_Delay(50);
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
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV3;
  RCC_OscInitStruct.PLL.PLLN = 40;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
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
  hi2c1.Init.Timing = 0x00701F6B;
  hi2c1.Init.OwnAddress1 = TMAG5273_ADDRESS;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }

  /** I2C Fast mode Plus enable
  */
  __HAL_SYSCFG_FASTMODEPLUS_ENABLE(I2C_FASTMODEPLUS_I2C1);
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.Timing = 0x00C01F67;
  hi2c2.Init.OwnAddress1 = 106;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c2, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c2, 0) != HAL_OK)
  {
    Error_Handler();
  }

  /** I2C Fast mode Plus enable
  */
  __HAL_SYSCFG_FASTMODEPLUS_ENABLE(I2C_FASTMODEPLUS_I2C2);
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

}

/**
  * @brief LPUART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_LPUART1_UART_Init(void)
{

  /* USER CODE BEGIN LPUART1_Init 0 */

  /* USER CODE END LPUART1_Init 0 */

  /* USER CODE BEGIN LPUART1_Init 1 */

  /* USER CODE END LPUART1_Init 1 */
  hlpuart1.Instance = LPUART1;
  hlpuart1.Init.BaudRate = 115200;
  hlpuart1.Init.WordLength = UART_WORDLENGTH_8B;
  hlpuart1.Init.StopBits = UART_STOPBITS_1;
  hlpuart1.Init.Parity = UART_PARITY_NONE;
  hlpuart1.Init.Mode = UART_MODE_TX_RX;
  hlpuart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  hlpuart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  hlpuart1.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  hlpuart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&hlpuart1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&hlpuart1, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&hlpuart1, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&hlpuart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN LPUART1_Init 2 */

  /* USER CODE END LPUART1_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMAMUX1_CLK_ENABLE();
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);
  /* DMA1_Channel2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel2_IRQn);
  /* DMA1_Channel3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel3_IRQn);
  /* DMA1_Channel4_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel4_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel4_IRQn);

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
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, TMAG5273A1_INT_Pin|TMAG5273A2_INT_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : TMAG5273A1_INT_Pin TMAG5273A2_INT_Pin */
  GPIO_InitStruct.Pin = TMAG5273A1_INT_Pin|TMAG5273A2_INT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

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
  printf("Error Handler - try power cycling the TMAG...\n");
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
