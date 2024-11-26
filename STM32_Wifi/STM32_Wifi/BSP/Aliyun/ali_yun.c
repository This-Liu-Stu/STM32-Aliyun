#include "ali_yun.h"
#include "esp8266.h"
#include "string.h"
#include "stdio.h"
#include "stdlib.h"

//======================wifi======================
extern uint8_t esp_buff[ESPBUFF_MAX_SIZE];
extern uint16_t esp_cnt;

#define post "/sys/k0796zJ6ms6/Esp01/thing/event/property/post"

#define MQTT_set	"AT+MQTTUSERCFG=0,1,\"NULL\",\"Esp01&k0796zJ6ms6\",\"4d2f41065d4e157be071612310e7fa2bae6fd1fe889de96c0335870864b1e08e\",0,0,\"\"\r\n"
#define MQTT_Client "AT+MQTTCLIENTID=0,\"k0796zJ6ms6.Esp01|securemode=2\\,signmethod=hmacsha256\\,timestamp=1726663442645|\"\r\n"
                                         
#define MQTT_Pass "AT+MQTTCONN=0,\"iot-06z00god4hf1dy1.mqtt.iothub.aliyuncs.com\",1883,1\r\n"

// ================================================

//=====================����=======================
uint16_t Wind;
uint16_t Water;
//================================================

// ���ð����˺�,����
void Ali_Yun_Init(void)
{
    // �����û���������
    ESP8266_SendCmd(MQTT_set, "OK");
    HAL_Delay(10);
    // ���ÿͻ���ID
    ESP8266_SendCmd(MQTT_Client, "OK");
    // ����MQTT
    ESP8266_SendCmd(MQTT_Pass, "OK");
    Ali_Yun_Topic();
}

void Ali_Yun_Topic(void)
{
	
    // ��������
    ESP8266_SendCmd("AT+MQTTSUB=0,\""ALI_TOPIC_SET"\",0\r\n", "OK");
		HAL_Delay(50);
    ESP8266_SendCmd("AT+MQTTSUB=0,\""ALI_TOPIC_POST"\",0\r\n", "OK");
		HAL_Delay(50);
}

//�����������ϴ�
void Ali_Yun_Send(void)
{
	uint8_t msg_buf[1024];
	uint8_t params_buf[1024];
	uint8_t data_value_buf[24];
	uint16_t move_num = 0;
	cJSON *send_cjson = NULL;

	char *str = NULL;
	int i=0;
	
	//printf("str = %p\r\n",&str);
	
	cJSON *params_cjson = NULL;
;
	memset(msg_buf,0,sizeof(msg_buf));
	memset(params_buf,0,sizeof(params_buf));
	memset(data_value_buf,0,sizeof(data_value_buf));

	send_cjson = cJSON_CreateObject();   // ����cjson
	
	// �������͵�json
	params_cjson = cJSON_CreateObject();
	
//============================================== ���͵�����================================================
//	cJSON_AddStringToObject(params_cjson, "temperature", "China");

	
//	memset(data_value_buf,0,sizeof(data_value_buf));
//	snprintf((char *)data_value_buf,sizeof(temp_value),"%1.3f",temp_value++);	
//	cJSON_AddItemToObject(params_cjson,"temperature",cJSON_CreateString((char *)data_value_buf));
//	
//	memset(data_value_buf,0,sizeof(data_value_buf));
//	snprintf((char *)data_value_buf,sizeof(humi_value),"%1.3f",humi_value++);	
//	cJSON_AddItemToObject(params_cjson,"Humidity",cJSON_CreateString((char *)data_value_buf));
	
	cJSON_AddNumberToObject(params_cjson,"Temp",11);
	cJSON_AddNumberToObject(params_cjson,"Humi",11);
	
//============================================== ���͵�����================================================
	// ��������json������
	cJSON_AddItemToObject(send_cjson, "params", params_cjson);
	cJSON_AddItemToObject(send_cjson,"version",cJSON_CreateString("1.0.0"));
	
	
	
	str = cJSON_PrintUnformatted(send_cjson);
	
	// ��ת���ַ�
	for(i=0;*str!='\0';i++)
	{
		params_buf[i] = *str;
		if(*(str+1)=='"'||*(str+1)==',')
		{
			params_buf[++i] = '\\';
		}
		str++;
		move_num++;
		
	}
	str = str - move_num;
	//printf("params_buf = %s\r\n",params_buf);
	
	// ������������
	sprintf((char *)msg_buf,"AT+MQTTPUB=0,\""ALI_TOPIC_POST"\",\"%s\",0,0\r\n",params_buf);
	
	ESP8266_SendCmd(msg_buf,"OK");

	
	ESP8266_Clear();
	cJSON_Delete(send_cjson);
	if(str!=NULL){
		free(str);
		str = NULL;
	}
}

uint8_t cjson_err_num = 0;  //cjson ��������Ĵ���
void Ali_Yun_GetRCV(void)
{
	cJSON *cjson = NULL;
	int num;
	char topic_buff[256];
	char recv_buffer[ESPBUFF_MAX_SIZE];
	char *ptr_recv = strstr((const char *)esp_buff,"+MQTTSUBRECV");
	
	if(ptr_recv!=NULL)  // ����
	{
		memset(topic_buff,0,sizeof(topic_buff));
		sscanf((char *)esp_buff,"+MQTTSUBRECV:0,%[^,],%d,%s",topic_buff,&num,recv_buffer);

		if(strstr(topic_buff,ALI_TOPIC_SET))      // �ж�����
		{
			printf("========================���ݽ�����ʼ===========================\r\n");
			
			//printf("�������ݳɹ�����ʼ����  %s\r\n",recv_buffer);
			cjson = cJSON_Parse(recv_buffer);
			
			
			if(cjson==NULL)
			{
				//printf("cjson ��������\r\n");
				cjson_err_num++;
				if(cjson_err_num>3){
					ESP8266_Clear();
					cjson_err_num = 0;
				}			
				//printf("========================���ݽ���ʧ��===========================\r\n");
			}
			else
			{
				cJSON *json_data = NULL;
				json_data = cJSON_GetObjectItem(cjson,"params");
				cjson_err_num = 0;
				if(json_data==NULL){
					//printf("cjson  û������\r\n");
					return;
				}else
				{
					//printf("cjson �ڴ��СΪ = %d\r\n",sizeof(cjson));
					// ====================================��������=========================================
					if(cJSON_GetObjectItem(json_data,"Wind")!=NULL)
					{
						Wind = cJSON_GetObjectItem(json_data,"Wind")->valueint;
						printf("csjon�����ɹ� Wind = %d\r\n",Wind);
					}
					
					if(cJSON_GetObjectItem(json_data,"Water")!=NULL)
					{
						Water = cJSON_GetObjectItem(json_data,"Water")->valueint;
						printf("csjon�����ɹ� Water = %d\r\n",Water);
					}
					
					//======================================================================================
				}
				
				ESP8266_Clear();
				cJSON_Delete(cjson);
				//printf("========================���ݽ����ɹ�===========================\r\n");
				
			}
			
			
			
			
			
		}
		
		
		
		

	}

	
	
}

