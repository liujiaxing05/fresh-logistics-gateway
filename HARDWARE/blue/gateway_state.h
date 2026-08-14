#ifndef __GATEWAY_STATE_H__
#define __GATEWAY_STATE_H__

/**
 * @file gateway_state.h
 * @brief 下层设备最新状态的线程安全共享快照。
 */
#include "sys.h"

/* 下层设备最近一次上报数据的一致性快照。 */
typedef struct
{
    float tin_c;             /* 内部温度，单位 ℃。 */
    float tout_c;            /* 外部温度，单位 ℃。 */
    float illuminance_lux;   /* 光照强度，单位 lx。 */
    u16 battery_mv;          /* 电池电压，单位 mV。 */
    u8 device_status;        /* 下层设备定义的状态位。 */
    u8 alarm_code;           /* 0 表示无告警，非 0 为故障码。 */
    u8 telemetry_valid;      /* 收到首帧遥测后置 1。 */
} gateway_state_snapshot_t;

void gateway_state_init(void);
/** 写入一帧下层遥测；输入值使用协议中的定点缩放单位。 */
void gateway_state_update_telemetry(s16 tin_x100, s16 tout_x100,
                                    u32 illuminance_x10, u16 battery_mv,
                                    u8 device_status);
void gateway_state_update_alarm(u8 alarm_code);
/** 原子复制当前全部字段到调用方缓冲。 */
void gateway_state_snapshot(gateway_state_snapshot_t *snapshot);

#endif
