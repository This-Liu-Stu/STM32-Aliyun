#ifndef _modbus_h_
#define _modbus_h_

//#include "stm32f10x.h"
#include "modbus_crc.h"
#include "main.h"

typedef struct 
{
	//作为从机时使用
  uint8_t  myadd;        //本设备从机地址
	uint8_t  rcbuf[100];   //modbus接受缓冲区
	uint8_t  timout;       //modbus数据持续时间
	uint8_t  recount;      //modbus端口接收到的数据个数
	uint8_t  timrun;       //modbus定时器是否计时标志
	uint8_t  reflag;       //modbus一帧数据接受完成标志位
	uint8_t  sendbuf[100]; //modbus接发送缓冲区
	
	//作为主机添加部分
	uint8_t Host_Txbuf[8];	//modbus发送数组	
	uint8_t slave_add;		//要匹配的从机设备地址（做主机实验时使用）
	uint8_t Host_send_flag;//主机设备发送数据完毕标志位
	uint16_t	Host_Sendtime;//发送完一帧数据后时间计数
	uint8_t Host_time_flag;//发送时间到标志位，=1表示到发送数据时间了
	uint8_t Host_End;//接收数据后处理完毕
}MODBUS;

extern MODBUS modbus;

void Modbus_Init(void);

void Host_Read03_slave(uint8_t slave,uint16_t StartAddr,uint16_t num);
//主机接收从机的消息进行处理
void HOST_ModbusRX(void);
void RS485_Usart_SendArray(USART_TypeDef* pUSARTx,uint8_t *array,uint8_t num);


//向一个寄存器中写数据的参数设置
void Host_write06_slave(uint8_t slave,uint8_t fun,uint16_t StartAddr,uint16_t num);

#endif

