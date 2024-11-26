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
#include "can.h"
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "esp8266.h"
#include "oled.h"
#include "modbus.h"
#include "cJSON.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/**
  * 函数功能: 使用单字节数据发送前要使能发送引脚，发送后要使能接收引脚
  * 输入参数: 无
  * 返 回 值: 无
  * 说    明：无
  */
void Modbus_Send_Byte(  uint8_t ch )
{
	/* 发送一个字节数据到USART2 */
    HAL_UART_Transmit(&huart2, (uint8_t *)&ch, 1, 0xff);	
}


void Send_Sensor_Data()
{
	
	Host_Read03_slave(0x01,0x0000,0x0001);//参数1从机地址，参数2起始地址，参数3寄存器个数
					if(modbus.Host_send_flag)
					{						
						HOST_ModbusRX();//接收数据进行处理			
						
						modbus.Host_time_flag=0;//发送数据标志位清零
						modbus.Host_send_flag=0;//清空发送结束数据标志位						
					}	
					
					Host_Read03_slave(0x02,0x0000,0x0001);
					if(modbus.Host_send_flag)
					{						
						HOST_ModbusRX();	
						
						modbus.Host_time_flag=0;
						modbus.Host_send_flag=0;					
					}
					
					Host_Read03_slave(0x03,0x0000,0x0001);
					if(modbus.Host_send_flag)
					{						
						HOST_ModbusRX();	
						
						modbus.Host_time_flag=0;
						modbus.Host_send_flag=0;					
					}
					
					Host_Read03_slave(0x04,0x0000,0x0002);
					if(modbus.Host_send_flag)
					{						
						HOST_ModbusRX();	
						
						modbus.Host_time_flag=0;
						modbus.Host_send_flag=0;					
					}
					
					Host_Read03_slave(0x05,0x0006,0x0001);
					if(modbus.Host_send_flag)
					{						
						HOST_ModbusRX();	
						
						modbus.Host_time_flag=0;
						modbus.Host_send_flag=0;					
					}
					
					modbus.Host_Sendtime=0;//发送完毕后计数清零（距离上次的时间）
}

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define RXBUFFERSIZE  4096     //最大接收字节数

//串口1配置-RS232缓存区
int8_t RxBuffer1[RXBUFFERSIZE]; 
uint8_t Uart1_Rx_Cnt = 0;
uint8_t aRxBuffer1;

//串口3配置-ESP8266缓存区
int8_t RxBuffer3[RXBUFFERSIZE];   //接收数据
uint8_t Uart3_Rx_Cnt = 0;
uint8_t aRxBuffer3;

uint8_t RES;	//串口2-RS485缓存区

//串口3配置
//串口发送缓存区
__align(8) uint8_t USART3_TX_BUF[USART3_MAX_SEND_LEN]; 	//发送缓冲,最大USART2_MAX_SEND_LEN字节 
//串口接收缓存区
uint8_t USART3_RX_BUF[USART3_MAX_RECV_LEN]; 				//接收缓冲,最大USART2_MAX_RECV_LEN个字节.

uint16_t USART3_RX_STA = 0;


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

uint16_t Wind,Water;			//控制器状态
uint16_t AliyunState = 0;	//云平台接收状态

/*
		0 - 风向传感器		1 - 风速传感器		2 - 空气质量传感器
		3 - 湿度传感器		4 - 温度传感器		5 - 光照传感器
*/
int sensor_data[SENSOR_NUM]={0};		//传感器状态缓存

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
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_USART3_UART_Init();
  MX_I2C1_Init();
  MX_TIM1_Init();
  MX_CAN_Init();
  /* USER CODE BEGIN 2 */
	//串口开启接收中断
  HAL_UART_Receive_IT(&huart1, &aRxBuffer1, 1);	
	HAL_UART_Receive_IT(&huart2, (uint8_t *)&RES, 1);
	HAL_UART_Receive_IT(&huart3, &aRxBuffer1, 1);
	HAL_TIM_Base_Start_IT(&htim1);
	
	//ESP8266_Init();			//ESP8266初始化
	
	RS485_RX_ENABLE;			//使能485控制端(启动发送)  
	Modbus_Init();			
	
	OLED_Init();                          //OLED初始化
  OLED_Clear();                         //先清屏
	HAL_Delay(1000);
	OLED_ShowCHinese(25,1,0);  						//在屏幕上方显示“此乃刘同学”             
	OLED_ShowCHinese(41,1,1);
	OLED_ShowCHinese(57,1,2);
	OLED_ShowCHinese(73,1,3);
	OLED_ShowCHinese(89,1,4);
	
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
			//每s执行操作		最大为60S
			
			if(modbus.Host_time_flag)//每1s发送一次数据
			{
				printf("hello\r\n");
					//Send_Sensor_Data();
			}
		
  }
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	
    if (huart->Instance == USART1)
    {
        if (Uart1_Rx_Cnt > RXBUFFERSIZE - 2)  // Overflow check
        {
            Uart1_Rx_Cnt = 0;
            memset(RxBuffer1, 0x00, sizeof(RxBuffer1));
        }
        else
        {
            RxBuffer1[Uart1_Rx_Cnt++] = aRxBuffer1;   // Store received data

             // 检查是否接收到结束符（例如换行符和回车符）
            if ((Uart1_Rx_Cnt > 1) && (RxBuffer1[Uart1_Rx_Cnt - 1] == 0x0A) && (RxBuffer1[Uart1_Rx_Cnt - 2] == 0x0D)) // End of data check
            {
                // 输出接收到的完整数据
                //HAL_UART_Transmit(&huart1, (uint8_t *)RxBuffer1, Uart1_Rx_Cnt, HAL_MAX_DELAY);
                Uart1_Rx_Cnt = 0;
                memset(RxBuffer1, 0x00, sizeof(RxBuffer1)); // Clear buffer
            }
        }
        HAL_UART_Receive_IT(&huart1, &aRxBuffer1, 1);
    }
    else if (huart->Instance == USART2)
    {
        if( modbus.reflag==1)  //有数据包正在处理
				{
						HAL_UART_Receive_IT(&huart2, (uint8_t *)&RES,1); 
					 return ;
				}		

				modbus.rcbuf[modbus.recount++] = RES;
				modbus.timout = 0;
				if(modbus.recount == 1)  //已经收到了第二个字符数据
				{
					modbus.timrun = 1;  //开启modbus定时器计时
				}
				HAL_UART_Receive_IT(&huart2, (uint8_t *)&RES,1); 
    }
		else  if (huart->Instance == USART3)
    {
        if (Uart3_Rx_Cnt < RXBUFFERSIZE - 1)  
        {
            RxBuffer3[Uart3_Rx_Cnt++] = aRxBuffer3;

            // Check for end of message (newline character)
            if (RxBuffer3[Uart3_Rx_Cnt - 1] == 0x0A) 
            {
								AliyunState = 1;	//设置云平台接收状态为1，485发送信息
								const char *start;

								// 提取 Wind 值
								start = strstr((const char *)RxBuffer3, "\"Wind\":"); // 转换为 const char *
								if (start) {
										start = strstr(start, "\"value\":");
										if (start) {
												start += strlen("\"value\":");
												Wind = strtol(start, (char **)&start, 10); 
												if(Wind == 1){
													Host_write06_slave(0x1F,0x05,0x0001,0xFF00);	//参数1从机地址，参数2功能码，参数3寄存器位，参数4FF00开0000关
												}else{
													Host_write06_slave(0x1F,0x05,0x0001,0x0000);
												}
										}
								}

								// 提取 Water 值
								start = strstr((const char *)RxBuffer3, "\"Water\":");
								if (start) {
										start = strstr(start, "\"value\":");
										if (start) {
												start += strlen("\"value\":");
												Water = strtol(start, (char **)&start, 10); 
												if(Water == 1){
													Host_write06_slave(0x1F,0x05,0x0002,0xFF00);
												}else{
													Host_write06_slave(0x1F,0x05,0x0002,0x0000);
												}
										}
								}
														
                // Reset the buffer after processing
                Uart3_Rx_Cnt = 0;
                memset(RxBuffer3, 0x00, sizeof(RxBuffer3));
            }
        }
        HAL_UART_Receive_IT(&huart3, &aRxBuffer3, 1);  // Re-enable interrupt for next byte
    }


}


void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){
 	if (htim->Instance == htim1.Instance) 
 	{
 		if(modbus.timrun != 0)//运行时间！=0表明
 		{
 		  modbus.timout++;
 		  if(modbus.timout >=100)
 		  {
				 modbus.timrun = 0;
				 modbus.reflag = 1;//接收数据完毕
 		  }
 		}
 		modbus.Host_Sendtime++;//发送完上一帧后的时间计数
		if(modbus.Host_Sendtime>1000)//距离发送上一帧数据1s了
		{
			//1s时间到
			modbus.Host_time_flag=1;//发送数据标志位置1
			
		}
		
 	}
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

