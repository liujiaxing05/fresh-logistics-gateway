/**
 * @file wifista.c
 * @brief 使用 ESP8266 AT 固件建立 Wi-Fi 和 MQTT 连接。
 *
 * 配网过程同步等待模块响应，只能由拥有 USART3 接收权的线程调用：启动阶段
 * 为主线程，运行期重连为 cloudrx 线程。mqtttx 在 cloud_online=0 时暂停。
 */
#include "wifista.h"
#include "wifi_config.h"
#include "usart3.h"
#include "cloud_service.h"
#include <rtthread.h>
#include <stdio.h>
#include <string.h>

/* 真实网络配置保存在被 Git 忽略的 wifi_config.h 中。 */
const u8 *wifista_ssid = WIFI_SSID;
const u8 *wifista_encryption = "WPA_WAP2_PSK";
const u8 *wifista_password = WIFI_PASSWORD;
const u8 *ATK_ESP8266_ECN_TBL[5] = {"OPEN", "WEP", "WPA_PSK", "WPA2_PSK", "WPA_WAP2_PSK"};

#define MQTT_HOST WIFI_MQTT_HOST
#define MQTT_PORT WIFI_MQTT_PORT
#define MQTT_CLIENT_ID WIFI_MQTT_CLIENT_ID
#define ESP8266_RESPONSE_MAX 256U

static char response[ESP8266_RESPONSE_MAX];
static u16 response_length;

static void esp8266_response_reset(void)
{
    /* 每条命令独立收集响应，避免上一条命令的 OK 造成误判。 */
    response_length = 0;
    response[0] = '\0';
}

static void esp8266_collect_response(void)
{
    u8 byte;
    /* 排空当前已到达的字节；外层等待循环每 10 ms 继续收集新数据。 */
    while (uart3_rx_pop(&byte))
    {
        if (response_length < sizeof(response) - 1U)
        {
            response[response_length++] = (char)byte;
            response[response_length] = '\0';
        }
    }
}

u8 *atk_8266_check_cmd(u8 *expected)
{
    /* 返回匹配位置仅用于判断是否命中，不允许调用方长期保存该指针。 */
    if (expected == RT_NULL) return RT_NULL;
    return (u8 *)strstr(response, (const char *)expected);
}

u8 atk_8266_send_cmd(u8 *command, u8 *expected, u16 waittime)
{
    u16 elapsed;
    if (command == RT_NULL) return 1;
    esp8266_response_reset();
    /* ESP8266 AT 命令要求以 CRLF 结尾。 */
    u3_printf("%s\r\n", command);
    if (expected == RT_NULL || waittime == 0) return 0;
    /* 历史接口中的 waittime 以 10 ms 为一个等待单位。 */
    for (elapsed = 0; elapsed < waittime; ++elapsed)
    {
        rt_thread_mdelay(10);
        esp8266_collect_response();
        if (atk_8266_check_cmd(expected) != RT_NULL) return 0;
    }
    return 1;
}

u8 atk_8266_quit_trans(void)
{
    /* “+++”后保留保护时间，再发送 AT 验证已退出透传模式。 */
    uart3_write((const u8 *)"+++", 3);
    rt_thread_mdelay(500);
    return atk_8266_send_cmd((u8 *)"AT", (u8 *)"OK", 100);
}

u8 atk_8266_wifista_config(void)
{
    char command[160];
    u8 attempt;
    /* 配置期间标记离线，阻止 mqtttx 与配网 AT 命令交叉写 USART3。 */
    cloud_service_set_online(0);

    /* 每轮任一步骤失败都从模块复位重新开始，最多尝试 3 轮。 */
    for (attempt = 0; attempt < 3; ++attempt)
    {
        if (atk_8266_send_cmd((u8 *)"AT+RST", (u8 *)"ready", 1000)) continue;
        if (atk_8266_send_cmd((u8 *)"ATE0", (u8 *)"OK", 100)) continue;
        if (atk_8266_send_cmd((u8 *)"AT+CWMODE=1", (u8 *)"OK", 100)) continue;
        snprintf(command, sizeof(command), "AT+CWJAP=\"%s\",\"%s\"", wifista_ssid, wifista_password);
        if (atk_8266_send_cmd((u8 *)command, (u8 *)"WIFI GOT IP", 3000)) continue;
        snprintf(command, sizeof(command), "AT+MQTTUSERCFG=0,1,\"%s\",\"\",\"\",0,0,\"\"", MQTT_CLIENT_ID);
        if (atk_8266_send_cmd((u8 *)command, (u8 *)"OK", 200)) continue;
        snprintf(command, sizeof(command), "AT+MQTTCONN=0,\"%s\",%u,1", MQTT_HOST, MQTT_PORT);
        if (atk_8266_send_cmd((u8 *)command, (u8 *)"OK", 500)) continue;
        if (atk_8266_send_cmd((u8 *)"AT+MQTTSUB=0,\"sent_AT\",0", (u8 *)"OK", 200)) continue;
        /* MQTT 连接和订阅均成功后，才恢复上行业务。 */
        cloud_service_set_online(1);
        return 1;
    }
    return 0;
}
