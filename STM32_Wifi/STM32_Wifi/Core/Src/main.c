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
  * ��������: ʹ�õ��ֽ����ݷ���ǰҪʹ�ܷ������ţ����ͺ�Ҫʹ�ܽ�������
  * �������: ��
  * �� �� ֵ: ��
  * ˵    ������
  */
void Modbus_Send_Byte(  uint8_t ch )
{
	/* ����һ���ֽ����ݵ�USART2 */
    HAL_UART_Transmit(&huart2, (uint8_t *)&ch, 1, 0xff);	
}


void Send_Sensor_Data()
{
	
	Host_Read03_slave(0x01,0x0000,0x0001);//����1�ӻ���ַ������2��ʼ��ַ������3�Ĵ�������
					if(modbus.Host_send_flag)
					{						
						HOST_ModbusRX();//�������ݽ��д���			
						
						modbus.Host_time_flag=0;//�������ݱ�־λ����
						modbus.Host_send_flag=0;//��շ��ͽ������ݱ�־λ						
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
					
					modbus.Host_Sendtime=0;//������Ϻ�������㣨�����ϴε�ʱ�䣩
}

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define RXBUFFERSIZE  4096     //�������ֽ���

//����1����-RS232������
int8_t RxBuffer1[RXBUFFERSIZE]; 
uint8_t Uart1_Rx_Cnt = 0;
uint8_t aRxBuffer1;

//����3����-ESP8266������
int8_t RxBuffer3[RXBUFFERSIZE];   //��������
uint8_t Uart3_Rx_Cnt = 0;
uint8_t aRxBuffer3;

uint8_t RES;	//����2-RS485������

//����3����
//���ڷ��ͻ�����
__align(8) uint8_t USART3_TX_BUF[USART3_MAX_SEND_LEN]; 	//���ͻ���,���USART2_MAX_SEND_LEN�ֽ� 
//���ڽ��ջ�����
uint8_t USART3_RX_BUF[USART3_MAX_RECV_LEN]; 				//���ջ���,���USART2_MAX_RECV_LEN���ֽ�.

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

uint16_t Wind,Water;			//������״̬
uint16_t AliyunState = 0;	//��ƽ̨����״̬

/*
		0 - ���򴫸���		1 - ���ٴ�����		2 - ��������������
		3 - ʪ�ȴ�����		4 - �¶ȴ�����		5 - ���մ�����
*/
int sensor_data[SENSOR_NUM]={0};		//������״̬����

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
	//���ڿ��������ж�
  HAL_UART_Receive_IT(&huart1, &aRxBuffer1, 1);	
	HAL_UART_Receive_IT(&huart2, (uint8_t *)&RES, 1);
	HAL_UART_Receive_IT(&huart3, &aRxBuffer1, 1);
	HAL_TIM_Base_Start_IT(&htim1);
	
	//ESP8266_Init();			//ESP8266��ʼ��
	
	RS485_RX_ENABLE;			//ʹ��485���ƶ�(��������)  
	Modbus_Init();			
	
	OLED_Init();                          //OLED��ʼ��
  OLED_Clear();                         //������
	HAL_Delay(1000);
	OLED_ShowCHinese(25,1,0);  						//����Ļ�Ϸ���ʾ��������ͬѧ��             
	OLED_ShowCHinese(41,1,1);
	OLED_ShowCHinese(57,1,2);
	OLED_ShowCHinese(73,1,3);
	OLED_ShowCHinese(89,1,4);
	
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
			//ÿsִ�в���		���Ϊ60S
			
			if(modbus.Host_time_flag)//ÿ1s����һ������
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

             // ����Ƿ���յ������������绻�з��ͻس�����
            if ((Uart1_Rx_Cnt > 1) && (RxBuffer1[Uart1_Rx_Cnt - 1] == 0x0A) && (RxBuffer1[Uart1_Rx_Cnt - 2] == 0x0D)) // End of data check
            {
                // ������յ�����������
                //HAL_UART_Transmit(&huart1, (uint8_t *)RxBuffer1, Uart1_Rx_Cnt, HAL_MAX_DELAY);
                Uart1_Rx_Cnt = 0;
                memset(RxBuffer1, 0x00, sizeof(RxBuffer1)); // Clear buffer
            }
        }
        HAL_UART_Receive_IT(&huart1, &aRxBuffer1, 1);
    }
    else if (huart->Instance == USART2)
    {
        if( modbus.reflag==1)  //�����ݰ����ڴ���
				{
						HAL_UART_Receive_IT(&huart2, (uint8_t *)&RES,1); 
					 return ;
				}		

				modbus.rcbuf[modbus.recount++] = RES;
				modbus.timout = 0;
				if(modbus.recount == 1)  //�Ѿ��յ��˵ڶ����ַ�����
				{
					modbus.timrun = 1;  //����modbus��ʱ����ʱ
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
								AliyunState = 1;	//������ƽ̨����״̬Ϊ1��485������Ϣ
								const char *start;

								// ��ȡ Wind ֵ
								start = strstr((const char *)RxBuffer3, "\"Wind\":"); // ת��Ϊ const char *
								if (start) {
										start = strstr(start, "\"value\":");
										if (start) {
												start += strlen("\"value\":");
												Wind = strtol(start, (char **)&start, 10); 
												if(Wind == 1){
													Host_write06_slave(0x1F,0x05,0x0001,0xFF00);	//����1�ӻ���ַ������2�����룬����3�Ĵ���λ������4FF00��0000��
												}else{
													Host_write06_slave(0x1F,0x05,0x0001,0x0000);
												}
										}
								}

								// ��ȡ Water ֵ
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
 		if(modbus.timrun != 0)//����ʱ�䣡=0����
 		{
 		  modbus.timout++;
 		  if(modbus.timout >=100)
 		  {
				 modbus.timrun = 0;
				 modbus.reflag = 1;//�����������
 		  }
 		}
 		modbus.Host_Sendtime++;//��������һ֡���ʱ�����
		if(modbus.Host_Sendtime>1000)//���뷢����һ֡����1s��
		{
			//1sʱ�䵽
			modbus.Host_time_flag=1;//�������ݱ�־λ��1
			
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

