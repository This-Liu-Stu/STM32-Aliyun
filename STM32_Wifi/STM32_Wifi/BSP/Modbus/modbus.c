#include "modbus.h"
#include "usart.h"

MODBUS modbus;//结构体变量
		 
// Modbus初始化函数
void Modbus_Init()
{
  modbus.myadd = 0x02; //从机设备地址为1
  modbus.timrun = 0;    //modbus定时器停止计算
	modbus.slave_add=0x04;//主机要匹配的从机地址
}

//主机选择从机
//参数1从机，参数2起始地址，参数3寄存器个数
void Host_Read03_slave(uint8_t slave,uint16_t StartAddr,uint16_t num)
{
	int j;
	uint16_t crc;//计算的CRC校验位
	modbus.slave_add=slave;
	modbus.Host_Txbuf[0]=slave;//这是要匹配的从机地址
	modbus.Host_Txbuf[1]=0x03;//功能码
	modbus.Host_Txbuf[2]=StartAddr/256;//起始地址高位
	modbus.Host_Txbuf[3]=StartAddr%256;//起始地址低位
	modbus.Host_Txbuf[4]=num/256;//寄存器个数高位
	modbus.Host_Txbuf[5]=num%256;//寄存器个数低位
	crc=Modbus_CRC16(&modbus.Host_Txbuf[0],6); //获取CRC校验位
	modbus.Host_Txbuf[6]=crc/256;//寄存器个数高位
	modbus.Host_Txbuf[7]=crc%256;//寄存器个数低位
	
	//发送数据包装完毕
	RS485_TX_ENABLE;//使能485控制端(启动发送)  
	HAL_Delay(100);
	for(j=0;j<8;j++)
	{
		Modbus_Send_Byte(modbus.Host_Txbuf[j]);
	}
	RS485_RX_ENABLE;//失能485控制端（改为接收）
	modbus.Host_send_flag=1;//表示发送数据完毕
}


//第三个是字节个数
//主机处理接受到的数据
void Host_Func3()
{
    int i;
    int count = (int)modbus.rcbuf[2]; //这是数据个数
		//根据不同的从机地址进行对应的数据解析
		switch(modbus.rcbuf[0]){	
			case 0x01:					//风向
				sensor_data[0] = (int)((modbus.rcbuf[4]) + ((modbus.rcbuf[3]) * 256));
				break;
			case 0x02:					//风速
				sensor_data[1] = (int)((modbus.rcbuf[4]) + ((modbus.rcbuf[3]) * 256));
				sensor_data[1]	= sensor_data[1]/10;
				break;
			case 0x03:					//空气质量
				sensor_data[2] = (int)((modbus.rcbuf[4]) + ((modbus.rcbuf[3]) * 256));
				break;
			case 0x04:					//湿度			温度
				sensor_data[3] = (int)((modbus.rcbuf[4]) + ((modbus.rcbuf[3]) * 256));
				sensor_data[4] = (int)((modbus.rcbuf[6]) + ((modbus.rcbuf[5]) * 256));
				sensor_data[3] /= 10;
				sensor_data[4] /= 10;
				break;
			case 0x05:					//光照强度
				sensor_data[5] = (int)((modbus.rcbuf[4]) + ((modbus.rcbuf[3]) * 256));
				break;
			default:
				printf("从机返回 %d 个寄存器数据：\r\n", count / 2);
				for (i = 0; i < count; i += 2)
				{
						printf("Temp_Hbit= %d Temp_Lbit= %d temp= %d\r\n", 
									 (int)modbus.rcbuf[3 + i], 
									 (int)modbus.rcbuf[4 + i], 
									 (int)((modbus.rcbuf[4 + i]) + ((modbus.rcbuf[3 + i]) * 256)));
				}
				break;
		}
    
    modbus.Host_End = 1; 		//接收的数据处理完毕
}


//主机接收从机的消息进行处理
void HOST_ModbusRX()
{
	uint16_t crc,rccrc;		//计算crc和接收到的crc

  if(modbus.reflag == 0)  	//如果接收未完成则返回空
	{
	   return;
	}
	//接收数据结束
	
	//（数组中除了最后两位CRC校验位其余全算）
	crc = Modbus_CRC16(&modbus.rcbuf[0],modbus.recount-2); 		//获取CRC校验位
	rccrc = modbus.rcbuf[modbus.recount-2]*256+modbus.rcbuf[modbus.recount-1];		//计算读取的CRC校验位
	
	if(crc == rccrc) //CRC检验成功 开始分析包
	{	
	   if(modbus.rcbuf[0] == modbus.slave_add)  	//检查地址是是对应从机发过来的
		 {
			 if(modbus.rcbuf[1]==3)	//功能码时03
		      Host_Func3();		//这是读取寄存器的有效数据位进行计算
		 }
		 
	}	
	 modbus.recount = 0;	//接收计数清零
	 modbus.reflag = 0; 	//接收标志清零
	 memset(modbus.rcbuf, 0, sizeof(modbus.rcbuf)); // 重置接收缓冲区
}


//向一个寄存器中写数据的参数设置
void Host_write06_slave(uint8_t slave,uint8_t fun,uint16_t StartAddr,uint16_t num)
{
	uint16_t crc,j;//计算的CRC校验位
	modbus.slave_add=slave;//从机地址赋值一下，后期有用
	modbus.Host_Txbuf[0]=slave;//这是要匹配的从机地址
	modbus.Host_Txbuf[1]=fun;//功能码
	modbus.Host_Txbuf[2]=StartAddr/256;//起始地址高位
	modbus.Host_Txbuf[3]=StartAddr%256;//起始地址低位
	modbus.Host_Txbuf[4]=num/256;
	modbus.Host_Txbuf[5]=num%256;
	crc=Modbus_CRC16(&modbus.Host_Txbuf[0],6); //获取CRC校验位
	modbus.Host_Txbuf[6]=crc/256;//寄存器个数高位
	modbus.Host_Txbuf[7]=crc%256;//寄存器个数低位
	
	//发送数据包装完毕
	RS485_TX_ENABLE;;//使能485控制端(启动发送)  
	HAL_Delay(100);
	for(j=0;j<8;j++)
	{
		Modbus_Send_Byte(modbus.Host_Txbuf[j]);
	}

}


