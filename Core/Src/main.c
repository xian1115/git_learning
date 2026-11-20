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
#include "can.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "memory.h"
#include "stdlib.h"
#include "math.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct
{
  CAN_HandleTypeDef *can_handle; // can句柄
  CAN_TxHeaderTypeDef txconf;    //can报文发送配置
  uint32_t tx_id;                // 发送id
  uint32_t tx_mailbox;           // CAN消息填入的邮箱号
  uint8_t tx_buff[8];            // 发送缓存
  uint8_t rx_buff[8];            // 接收缓存
  uint32_t rx_id;                // 接收id
  uint8_t rx_len;                // 接收长度,可能为0-8
} CANInstance;

/* 2. DJI电机CAN反馈信息结构*/
typedef struct
{
  // 原始数据
  uint16_t ecd;                  // 0-8191,刻度总共有8192格
  int16_t speed_rpm;             // 转速,单位为rpm
  int16_t real_current;          // 实际电流
  float   current_amp;           //实际电流, 单位A (20A/16384 换算)
  uint8_t temperature;           // 温度 Celsius
  
  // 处理后的数据
  float   real_current_handle;   
  float speed_aps;               // 角速度（度/秒）
  uint16_t last_ecd;             // 上一次读取的编码器值
  float angle_single_round;      // 单圈角度
  float total_angle;             // 总角度,注意方向
  int32_t total_round;           // 总圈数,注意方向
} DJI_Motor_Measure_s;

#define PI 3.14159265358979f
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
float t = 0.0f; // 用于正弦波计算的时间变量
CANInstance motor1;
DJI_Motor_Measure_s motor1_measure;
int16_t current;
float current_filter = 0.9f;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void MotorDataRead(void)
{
    if (motor1.rx_len < 7) return;   //  固定 8 字节

    // 1. 提取转子机械角度 (0~8191)
    // 高字节在前，低字节在后，组合成16位无符号整数
    motor1_measure.ecd = (uint16_t)((motor1.rx_buff[0] << 8) | motor1.rx_buff[1]);

    // 2. 提取转子转速(RPM)，注意这里是有符号数，需要强制转换
    motor1_measure.speed_rpm = (int16_t)((motor1.rx_buff[2] << 8) | motor1.rx_buff[3]);

    // 3. 提取实际转矩电流: 原始值存 real_current, 安培值存 current_amp
    int16_t raw_current = (int16_t)((motor1.rx_buff[4] << 8) | motor1.rx_buff[5]);
    motor1_measure.real_current = raw_current;                     
    motor1_measure.current_amp  = raw_current * 20.0f / 16384.0f;   

    // 4. 提取电机温度 
    motor1_measure.temperature = motor1.rx_buff[6];

    // 计算单圈角度：ecd / 8192 * 360
    motor1_measure.angle_single_round = (float)motor1_measure.ecd * 360.0f / 8192.0f;
}
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
  MX_CAN1_Init();
  MX_CAN2_Init();
  /* USER CODE BEGIN 2 */
 /* 1. 初始化CAN服务 */
  HAL_CAN_Start(&hcan1);
  HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);
  HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO1_MSG_PENDING);

  HAL_CAN_Start(&hcan2);
  HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO0_MSG_PENDING);
  HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO1_MSG_PENDING);
  
  /* 2. 配置电机的基本参数 */
  motor1.txconf.StdId = 0x200;               // 发送id (控制报文)
  motor1.txconf.IDE = CAN_ID_STD;            // 使用标准id
  motor1.txconf.RTR = CAN_RTR_DATA;          // 发送数据帧
  motor1.txconf.DLC = 8;                     // 发送长度为8
  motor1.can_handle = &hcan1;
  motor1.tx_id = 0x200;                      
  motor1.rx_id = 0x201;                      // 接收id (反馈报文)
  
  /* 3. 配置这个电机的过滤器 */
  CAN_FilterTypeDef can_filter_conf;
  static uint8_t can1_filter_idx = 0, can2_filter_idx = 14; // 0-13给can1用,14-27给can2用
  can_filter_conf.FilterMode = CAN_FILTERMODE_IDLIST;
  can_filter_conf.FilterScale = CAN_FILTERSCALE_16BIT;
  can_filter_conf.FilterFIFOAssignment = (motor1.tx_id & 1) ? CAN_RX_FIFO0 : CAN_RX_FIFO1;
  can_filter_conf.SlaveStartFilterBank = 14;

  can_filter_conf.FilterIdLow      = motor1.rx_id << 5;
  can_filter_conf.FilterIdHigh     = motor1.rx_id << 5;
  can_filter_conf.FilterMaskIdLow  = motor1.rx_id << 5;
  can_filter_conf.FilterMaskIdHigh = motor1.rx_id << 5;
  can_filter_conf.FilterBank = motor1.can_handle == &hcan1 ? (can1_filter_idx++) : (can2_filter_idx++);
  can_filter_conf.FilterActivation = CAN_FILTER_ENABLE;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    current = (int16_t)(1000.0f * sin(t * PI));
    t += 0.02f;
    
    if (HAL_CAN_GetTxMailboxesFreeLevel(motor1.can_handle) == 0)
    {
        // 等待邮箱空闲
    }
    else
    {
      motor1.tx_buff[0] = (uint8_t)(current >> 8);      //高八位
      motor1.tx_buff[1] = (uint8_t)(current & 0x00ff);  //低八位
      motor1.tx_buff[2] = 0;
      motor1.tx_buff[3] = 0;
      motor1.tx_buff[4] = 0;
      motor1.tx_buff[5] = 0;
      motor1.tx_buff[6] = 0;
      motor1.tx_buff[7] = 0;

      HAL_CAN_AddTxMessage(motor1.can_handle, &motor1.txconf, motor1.tx_buff, &motor1.tx_mailbox);
      
    }
    HAL_Delay(10); 
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
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
/* 中断回调函数：处理接收到的数据并解析 */
static void CANFIFOxCallback(CAN_HandleTypeDef *hcan, uint32_t fifox)
{
  static CAN_RxHeaderTypeDef rxconf; 
  uint8_t can_rx_buff[8];
  
  // 只要 FIFO 不为空，就一直把数据读出
  while (HAL_CAN_GetRxFifoFillLevel(hcan, fifox)) 
  {
    HAL_CAN_GetRxMessage(hcan, fifox, &rxconf, can_rx_buff); // 从FIFO中获取数
    
    // 如果句柄匹配，且接收到的ID符合
    if (hcan == motor1.can_handle && rxconf.StdId == motor1.rx_id)
    {
      motor1.rx_len = rxconf.DLC;                      
      memcpy(motor1.rx_buff, can_rx_buff, rxconf.DLC); // 消息拷贝到对应实例
      motor1_measure.last_ecd = motor1_measure.ecd;
      
      // 解析电机数据(实现见文件开头 MotorDataRead)
      MotorDataRead(); 
      
      // 角速度一阶低通滤波
      motor1_measure.speed_aps = (1.0f - 0.85f) * motor1_measure.speed_aps + 0.85f * (360.0f / 60.0f) * (float)motor1_measure.speed_rpm;
      
      // 电流一阶低通滤波
      motor1_measure.real_current_handle = (1.0f - current_filter) * motor1_measure.real_current_handle + current_filter * (float)motor1_measure.real_current;  // [改动5] 用原始计数滤�??: �?? real_current / current 同量�??, 三条曲线才能在同�??坐标系对�??
      
      // 多圈角度计算 (利用跨度阈值判断是否越过零点)
      if (motor1_measure.ecd - motor1_measure.last_ecd > 4096)
      {
        motor1_measure.total_round--;
      }
      else if (motor1_measure.ecd - motor1_measure.last_ecd < -4096)
      {
        motor1_measure.total_round++;
      }
      
      motor1_measure.total_angle = motor1_measure.total_round * 360 + motor1_measure.angle_single_round;
    }
  }
  return;
}

// 这两个HAL 库的弱函数，其实就是用来调用上面那个封装好的 CANFIFOxCallback
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    CANFIFOxCallback(hcan, CAN_RX_FIFO0);
}

void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    CANFIFOxCallback(hcan, CAN_RX_FIFO1);
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug*/
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
