/**
 * @file gateway_state.c
 * @brief 多线程共享状态的短临界区实现。
 */
#include "gateway_state.h"
#include <rthw.h>
#include <string.h>

static gateway_state_snapshot_t gateway_state;

void gateway_state_init(void)
{
    /* 清零过程与中断更新互斥，避免启动边界读到部分初始化状态。 */
    rt_base_t level = rt_hw_interrupt_disable();
    memset(&gateway_state, 0, sizeof(gateway_state));
    rt_hw_interrupt_enable(level);
}

void gateway_state_update_telemetry(s16 tin_x100, s16 tout_x100,
                                    u32 illuminance_x10, u16 battery_mv,
                                    u8 device_status)
{
    /* 所有字段在同一临界区更新，使读者获得属于同一遥测帧的数据。 */
    rt_base_t level = rt_hw_interrupt_disable();
    gateway_state.tin_c = (float)tin_x100 / 100.0f;
    gateway_state.tout_c = (float)tout_x100 / 100.0f;
    gateway_state.illuminance_lux = (float)illuminance_x10 / 10.0f;
    gateway_state.battery_mv = battery_mv;
    gateway_state.device_status = device_status;
    gateway_state.telemetry_valid = 1;
    rt_hw_interrupt_enable(level);
}

void gateway_state_update_alarm(u8 alarm_code)
{
    /* 告警可能由独立消息到达，因此与最近遥测分开更新。 */
    rt_base_t level = rt_hw_interrupt_disable();
    gateway_state.alarm_code = alarm_code;
    rt_hw_interrupt_enable(level);
}

void gateway_state_snapshot(gateway_state_snapshot_t *snapshot)
{
    rt_base_t level;
    if (snapshot == RT_NULL) return;
    /* 结构体整体复制期间禁止中断，保证快照内部字段一致。 */
    level = rt_hw_interrupt_disable();
    *snapshot = gateway_state;
    rt_hw_interrupt_enable(level);
}
