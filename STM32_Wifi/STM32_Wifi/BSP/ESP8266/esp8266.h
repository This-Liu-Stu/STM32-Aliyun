#ifndef _ESP8266_H_
#define _ESP8266_H_

#include "main.h"
#include "usart.h"

/*MQTT 连接参数*/
//#define clientId "a1Zw81YmcId.ESP8266|securemode=2\\,signmethod=hmacsha256\\,timestamp=1694524012615|"
//#define username "ESP8266&a1Zw81YmcId"
//#define passwd "a8bed5d35c6c689437c6ef2fa8d360044879460f6e6647fb4b8f985180ad42ca"
//#define mqttHostUrl "a1Zw81YmcId.iot-as-mqtt.cn-shanghai.aliyuncs.com"
//#define port "1883"
#define post "/sys/k0796zJ6ms6/Esp01/thing/event/property/post"

#define MQTT_set	"AT+MQTTUSERCFG=0,1,\"NULL\",\"Esp01&k0796zJ6ms6\",\"0126f01b2daec14e9a33c506b9ed393feafe2f89ad57fa0dc10a2bfaa8962997\",0,0,\"\""
#define MQTT_Client "AT+MQTTCLIENTID=0,\"k0796zJ6ms6.Esp01|securemode=2\\,signmethod=hmacsha256\\,timestamp=1726714706670|\""
                                         
#define MQTT_Pass "AT+MQTTCONN=0,\"iot-06z00god4hf1dy1.mqtt.iothub.aliyuncs.com\",1883,1"

#define WIFI_Name "111"
#define WIFI_Pass "20040816"

void ESP8266_Init(void);
unsigned char ESP8266_Re(unsigned char *ACK_AT);
void ESP8266_Send(char *property,int Data);
uint8_t esp8266_send_cmd(char *cmd,char *ack,uint16_t waittime);
uint8_t* esp8266_check_cmd(char *str);

#endif

