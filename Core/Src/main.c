/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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
#include "dma.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>
#include<stdio.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */



typedef struct
{
    
    int16_t rocker_l_; 
    int16_t rocker_l1; 
    int16_t rocker_r_; 
    int16_t rocker_r1; 

    int16_t SWA;      
    int16_t SWB;      
    int16_t SWC;      
    int16_t SWD;      

    int16_t VRA;      
    int16_t VRB;      

    uint8_t error_protect; 
    uint8_t byte_lose;     

} RC_ctrl_t;

volatile RC_ctrl_t rc_data;      

uint8_t buf[25];
uint8_t sbus_buf[25];
int16_t CH[16];
uint8_t start_byte;
uint8_t stop_byte;
uint8_t flag_byte;
uint32_t last_rx_time = 0;
volatile uint32_t test = 0;

volatile uint8_t  rc_online = 0;     
volatile uint32_t rx_event_cnt = 0; 
volatile uint16_t last_size = 0;    
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart,uint16_t size)
{
  if(huart == &huart3){
    rx_event_cnt++;
    last_size = size;

    if(size == 25 && buf[0] == 0x0F){
      memcpy(sbus_buf,buf,25);

      last_rx_time = HAL_GetTick();
      start_byte = sbus_buf[0];
      flag_byte = sbus_buf[23];
      stop_byte = sbus_buf[24];

      CH[0] = (((int16_t)sbus_buf[1] >> 0 | ((int16_t)sbus_buf[2] << 8 )) & 0x07FF) - 1024;
      CH[1] = (((int16_t)sbus_buf[2] >> 3 | ((int16_t)sbus_buf[3] << 5 )) & 0x07FF) - 1024;
      CH[2] = (((int16_t)sbus_buf[3] >> 6 | ((int16_t)sbus_buf[4] << 2 ) | ((int16_t)sbus_buf[5] << 10)) & 0x07FF) - 1024;
      CH[3] = (((int16_t)sbus_buf[5] >> 1 | ((int16_t)sbus_buf[6] << 7 )) & 0x07FF) - 1024;
      CH[4] = (((int16_t)sbus_buf[6] >> 4 | ((int16_t)sbus_buf[7] << 4 )) & 0x07FF) - 1024;
      CH[5] = (((int16_t)sbus_buf[7] >> 7 | ((int16_t)sbus_buf[8] << 1 ) | ((int16_t)sbus_buf[9] << 9)) & 0x07FF) - 1024;
      CH[6] = (((int16_t)sbus_buf[9] >> 2 | ((int16_t)sbus_buf[10] << 6)) & 0x07FF) - 1024;
      CH[7] = (((int16_t)sbus_buf[10] >> 5 | ((int16_t)sbus_buf[11] << 3 )) & 0x07FF) - 1024; 
      CH[8] = (((int16_t)sbus_buf[12] >> 0 | ((int16_t)sbus_buf[13] << 8 )) & 0x07FF) - 1024;
      CH[9] = (((int16_t)sbus_buf[13] >> 3 | ((int16_t)sbus_buf[14] << 5 )) & 0x07FF) - 1024;
        
        rc_data.rocker_r_ = CH[0];    
        rc_data.rocker_r1 = CH[1];    
        rc_data.rocker_l1 = CH[2];    
        rc_data.rocker_l_ = CH[3];    
        rc_data.SWA       = CH[4];    
        rc_data.SWB       = CH[5];    
        rc_data.SWC       = CH[6];    
        rc_data.SWD       = CH[7];    
        rc_data.VRA       = CH[8];    
        rc_data.VRB       = CH[9];    
        
        rc_data.byte_lose = (flag_byte >> 2) & 0x01;
        rc_data.error_protect = (flag_byte >> 3) & 0x01;

        rc_online = 1;     

        //char debug_buf[64];
        //sprintf(debug_buf,"CH0:%d,CH1:%d,CH2:%d,CH3:%d",CH[0],CH[1],CH[2],CH[3]);

        //HAL_UART_Transmit(&huart1,(uint8_t*)debug_buf,strlen(debug_buf),100);
  }
  HAL_UARTEx_ReceiveToIdle_DMA(&huart3,buf,25);
  __HAL_DMA_DISABLE_IT(huart3.hdmarx,DMA_IT_HT);
  }
}
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
  if(huart == &huart3){
    __HAL_UART_CLEAR_OREFLAG(huart);
    __HAL_UART_CLEAR_NEFLAG(huart);
    __HAL_UART_CLEAR_FEFLAG(huart);
    __HAL_UART_CLEAR_PEFLAG(huart);

    HAL_UARTEx_ReceiveToIdle_DMA(&huart3,buf,25);
    __HAL_DMA_DISABLE_IT(huart3.hdmarx,DMA_IT_HT);
  }
}
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

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_USART3_UART_Init();
  /* USER CODE BEGIN 2 */
  HAL_StatusTypeDef statues = HAL_UARTEx_ReceiveToIdle_DMA(&huart3, buf, 25);
  if(statues == HAL_OK)
  {
    if(huart3.RxState == HAL_UART_STATE_BUSY_RX){
      test = 1;
    }
    else test = 2;
  }

  __HAL_DMA_DISABLE_IT(huart3.hdmarx,DMA_IT_HT);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    if (rc_online && (HAL_GetTick() - last_rx_time > 100))
    {
      rc_online = 0;

      for(int i = 0;i < 10;i++){
        CH[i] = 0;
      }

      rc_data.rocker_l_ = 0;
      rc_data.rocker_l1 = 0;
      rc_data.rocker_r_ = 0;
      rc_data.rocker_r1 = 0;
      rc_data.SWA = 0;
      rc_data.SWB = 0;
      rc_data.SWC = 0;
      rc_data.SWD = 0;
      rc_data.VRA = 0;
      rc_data.VRB = 0;
      rc_data.byte_lose = 1;
      rc_data.error_protect = 1;
    }

    if (huart3.RxState != HAL_UART_STATE_BUSY_RX) 
    {
        __HAL_UART_CLEAR_OREFLAG(&huart3);
        __HAL_UART_CLEAR_NEFLAG(&huart3);
        __HAL_UART_CLEAR_FEFLAG(&huart3);
        __HAL_UART_CLEAR_PEFLAG(&huart3);
        
        // 重新启动
        HAL_UARTEx_ReceiveToIdle_DMA(&huart3, buf, 25);
        __HAL_DMA_DISABLE_IT(huart3.hdmarx, DMA_IT_HT);
    }
    HAL_Delay(10);
    
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
