/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */
#define USART3_MAX_RECV_LEN		600					//最大接收缓存字节数
#define USART3_MAX_SEND_LEN		600					//最大发送缓存字节数
#define USART3_RX_EN 			1					//0,不接收;1,接收.

extern uint8_t  USART3_RX_BUF[USART3_MAX_RECV_LEN]; 		//接收缓冲,最大USART3_MAX_RECV_LEN字节
extern uint16_t USART3_RX_STA;   						//接收数据状态


/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define LED_Pin GPIO_PIN_13
#define LED_GPIO_Port GPIOC
#define RS485_control_Pin GPIO_PIN_4
#define RS485_control_GPIO_Port GPIOA
/* USER CODE BEGIN Private defines */

#define SENSOR_NUM 6								//传感器数量

//485发送接收控制引脚
//接收使能
#define RS485_RX_ENABLE  HAL_GPIO_WritePin(RS485_control_GPIO_Port,RS485_control_Pin,GPIO_PIN_RESET)
//发送使能
#define RS485_TX_ENABLE HAL_GPIO_WritePin(RS485_control_GPIO_Port,RS485_control_Pin,GPIO_PIN_SET)

void Modbus_Send_Byte(  uint8_t ch );//485发送字符函数

extern uint8_t RES;//485串口接收缓冲区
extern int sensor_data[SENSOR_NUM];		//传感器数据缓存

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
