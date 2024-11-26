#include "modbus.h"
#include "usart.h"

MODBUS modbus;//�ṹ�����
		 
// Modbus��ʼ������
void Modbus_Init()
{
  modbus.myadd = 0x02; //�ӻ��豸��ַΪ1
  modbus.timrun = 0;    //modbus��ʱ��ֹͣ����
	modbus.slave_add=0x04;//����Ҫƥ��Ĵӻ���ַ
}

//����ѡ��ӻ�
//����1�ӻ�������2��ʼ��ַ������3�Ĵ�������
void Host_Read03_slave(uint8_t slave,uint16_t StartAddr,uint16_t num)
{
	int j;
	uint16_t crc;//�����CRCУ��λ
	modbus.slave_add=slave;
	modbus.Host_Txbuf[0]=slave;//����Ҫƥ��Ĵӻ���ַ
	modbus.Host_Txbuf[1]=0x03;//������
	modbus.Host_Txbuf[2]=StartAddr/256;//��ʼ��ַ��λ
	modbus.Host_Txbuf[3]=StartAddr%256;//��ʼ��ַ��λ
	modbus.Host_Txbuf[4]=num/256;//�Ĵ���������λ
	modbus.Host_Txbuf[5]=num%256;//�Ĵ���������λ
	crc=Modbus_CRC16(&modbus.Host_Txbuf[0],6); //��ȡCRCУ��λ
	modbus.Host_Txbuf[6]=crc/256;//�Ĵ���������λ
	modbus.Host_Txbuf[7]=crc%256;//�Ĵ���������λ
	
	//�������ݰ�װ���
	RS485_TX_ENABLE;//ʹ��485���ƶ�(��������)  
	HAL_Delay(100);
	for(j=0;j<8;j++)
	{
		Modbus_Send_Byte(modbus.Host_Txbuf[j]);
	}
	RS485_RX_ENABLE;//ʧ��485���ƶˣ���Ϊ���գ�
	modbus.Host_send_flag=1;//��ʾ�����������
}


//���������ֽڸ���
//����������ܵ�������
void Host_Func3()
{
    int i;
    int count = (int)modbus.rcbuf[2]; //�������ݸ���
		//���ݲ�ͬ�Ĵӻ���ַ���ж�Ӧ�����ݽ���
		switch(modbus.rcbuf[0]){	
			case 0x01:					//����
				sensor_data[0] = (int)((modbus.rcbuf[4]) + ((modbus.rcbuf[3]) * 256));
				break;
			case 0x02:					//����
				sensor_data[1] = (int)((modbus.rcbuf[4]) + ((modbus.rcbuf[3]) * 256));
				sensor_data[1]	= sensor_data[1]/10;
				break;
			case 0x03:					//��������
				sensor_data[2] = (int)((modbus.rcbuf[4]) + ((modbus.rcbuf[3]) * 256));
				break;
			case 0x04:					//ʪ��			�¶�
				sensor_data[3] = (int)((modbus.rcbuf[4]) + ((modbus.rcbuf[3]) * 256));
				sensor_data[4] = (int)((modbus.rcbuf[6]) + ((modbus.rcbuf[5]) * 256));
				sensor_data[3] /= 10;
				sensor_data[4] /= 10;
				break;
			case 0x05:					//����ǿ��
				sensor_data[5] = (int)((modbus.rcbuf[4]) + ((modbus.rcbuf[3]) * 256));
				break;
			default:
				printf("�ӻ����� %d ���Ĵ������ݣ�\r\n", count / 2);
				for (i = 0; i < count; i += 2)
				{
						printf("Temp_Hbit= %d Temp_Lbit= %d temp= %d\r\n", 
									 (int)modbus.rcbuf[3 + i], 
									 (int)modbus.rcbuf[4 + i], 
									 (int)((modbus.rcbuf[4 + i]) + ((modbus.rcbuf[3 + i]) * 256)));
				}
				break;
		}
    
    modbus.Host_End = 1; 		//���յ����ݴ������
}


//�������մӻ�����Ϣ���д���
void HOST_ModbusRX()
{
	uint16_t crc,rccrc;		//����crc�ͽ��յ���crc

  if(modbus.reflag == 0)  	//�������δ����򷵻ؿ�
	{
	   return;
	}
	//�������ݽ���
	
	//�������г��������λCRCУ��λ����ȫ�㣩
	crc = Modbus_CRC16(&modbus.rcbuf[0],modbus.recount-2); 		//��ȡCRCУ��λ
	rccrc = modbus.rcbuf[modbus.recount-2]*256+modbus.rcbuf[modbus.recount-1];		//�����ȡ��CRCУ��λ
	
	if(crc == rccrc) //CRC����ɹ� ��ʼ������
	{	
	   if(modbus.rcbuf[0] == modbus.slave_add)  	//����ַ���Ƕ�Ӧ�ӻ���������
		 {
			 if(modbus.rcbuf[1]==3)	//������ʱ03
		      Host_Func3();		//���Ƕ�ȡ�Ĵ�������Ч����λ���м���
		 }
		 
	}	
	 modbus.recount = 0;	//���ռ�������
	 modbus.reflag = 0; 	//���ձ�־����
	 memset(modbus.rcbuf, 0, sizeof(modbus.rcbuf)); // ���ý��ջ�����
}


//��һ���Ĵ�����д���ݵĲ�������
void Host_write06_slave(uint8_t slave,uint8_t fun,uint16_t StartAddr,uint16_t num)
{
	uint16_t crc,j;//�����CRCУ��λ
	modbus.slave_add=slave;//�ӻ���ַ��ֵһ�£���������
	modbus.Host_Txbuf[0]=slave;//����Ҫƥ��Ĵӻ���ַ
	modbus.Host_Txbuf[1]=fun;//������
	modbus.Host_Txbuf[2]=StartAddr/256;//��ʼ��ַ��λ
	modbus.Host_Txbuf[3]=StartAddr%256;//��ʼ��ַ��λ
	modbus.Host_Txbuf[4]=num/256;
	modbus.Host_Txbuf[5]=num%256;
	crc=Modbus_CRC16(&modbus.Host_Txbuf[0],6); //��ȡCRCУ��λ
	modbus.Host_Txbuf[6]=crc/256;//�Ĵ���������λ
	modbus.Host_Txbuf[7]=crc%256;//�Ĵ���������λ
	
	//�������ݰ�װ���
	RS485_TX_ENABLE;;//ʹ��485���ƶ�(��������)  
	HAL_Delay(100);
	for(j=0;j<8;j++)
	{
		Modbus_Send_Byte(modbus.Host_Txbuf[j]);
	}

}


