#ifndef __CLOUD_SERVICE_H__
#define __CLOUD_SERVICE_H__

/**
 * @file cloud_service.h
 * @brief ESP8266 云端接收、命令路由和在线状态接口。
 */
#include "sys.h"

void cloud_service_init(void);
/** 由 USART3 ISR 调用，用信号量通知 cloudrx 线程处理新数据。 */
void cloud_service_notify_rx_isr(void);
/** 返回当前 Wi-Fi/MQTT 逻辑在线状态。 */
u8 cloud_service_is_online(void);
/** 供 ESP8266 配网流程和串口状态解析更新在线状态。 */
void cloud_service_set_online(u8 online);

#endif
