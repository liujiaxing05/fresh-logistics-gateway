#ifndef __WIFISTA_H
#define __WIFISTA_H

#include "sys.h"
void atk_8266_init(void);
u8* atk_8266_check_cmd(u8 *str);
void atk_8266_at_response(u8 mode);
u8 atk_8266_send_cmd(u8 *cmd,u8 *ack,u16 waittime);
u8 atk_8266_quit_trans(void);
u8 atk_8266_wifista_config(void);
//用户配置参数

 
extern const u8* wifista_ssid;		//WIFI STA SSID
extern const u8* wifista_encryption;//WIFI STA 加密方式
extern const u8* wifista_password; 	//WIFI STA 密码

extern const u8* ATK_ESP8266_ECN_TBL[5];


#endif

