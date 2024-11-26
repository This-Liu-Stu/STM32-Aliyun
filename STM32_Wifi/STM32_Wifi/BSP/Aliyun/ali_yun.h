#ifndef _AL_YUN_H_
#define _AL_YUN_H_

#include "main.h"
#include "esp8266.h"
#include "cJSON.h"

#define		 ALI_USERNAME		"Esp01&k0796zJ6ms6"                                         // 用户名
#define		 ALICLIENTLD			"k0796zJ6ms6.Esp01|securemode=2\\,signmethod=hmacsha256\\,timestamp=1726661930000|"				// 客户id
#define		 ALI_PASSWD			"c1bf2e36924b49b161cbb260cd613b992a3e1113605c65b36c1fde50db18a5f1"           // MQTT 密码
#define		 ALI_MQTT_HOSTURL	"iot-06z00god4hf1dy1.mqtt.iothub.aliyuncs.com"			// mqtt连接的网址
#define		 ALI_PORT			"1883"				// 端口

#define      ALI_TOPIC_SET          "/sys/k0796zJ6ms6/Esp01/thing/service/property/set"
#define      ALI_TOPIC_POST         "/sys/k0796zJ6ms6/Esp01/thing/event/property/post"


// 数据上传 格式
#define JSON_FORMAT      "{\\\"params\\\":{\\\"Temp\\\":%f\\,\\\"Humi\\\":%f\\}\\,\\\"version\\\":\\\"1.0.0\\\"}"

void Ali_Yun_Init(void);

void Ali_Yun_Topic(void);

void Ali_Yun_Send(void);

void Ali_Yun_GetRCV(void);


#endif
