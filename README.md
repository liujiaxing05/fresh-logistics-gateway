# 生鲜品储运物联网网关

基于 STM32F407ZG 与 RT-Thread 3.1.5 的嵌入式边缘网关。下层设备通过 USART2/BLE 上报温度、光照、电池和告警；网关通过 USART3/ESP8266 接入 Wi-Fi 与 MQTT，实现遥测上报、远程控制、异常告警、断线重连和 RAM 缓存补传。

## 主要功能

- 自定义带序列号和 CRC16-Modbus 的 BLE 二进制协议；
- UART 中断、环形缓冲、信号量和线程化协议解析；
- 控制命令 ACK/NACK、500 ms 超时和最多 3 次重传；
- MQTT 关键/普通双优先级发送队列；
- ESP8266 断线自动重连与 60 条 RAM 遥测缓存；
- LCD 状态显示、蜂鸣器和 LED 本地告警；
- FinSH/MSH 串口调试控制台。

## 构建

1. 使用 Keil MDK 打开 `USER/Template.uvprojx`；
2. 将 `HARDWARE/esp8266/wifi_config.example.h` 复制为 `wifi_config.h`；
3. 在 `wifi_config.h` 中填写本地 Wi-Fi 与 MQTT 参数；
4. 编译并下载至 STM32F407ZG。

`wifi_config.h` 已加入 `.gitignore`，不会提交本地网络凭据。

## 文档

- [工程逻辑框架说明](工程逻辑框架说明.md)
- [下层 BLE 通信协议](HARDWARE/blue/DEVICE_BLE_PROTOCOL.md)
- [嵌入式物联网求职能力提升路线](嵌入式物联网求职能力提升路线.md)

## 说明

断线数据保存在静态 RAM 循环队列中，设备复位或掉电后不会保留。默认 60 条、2 秒一条，约覆盖 2 分钟断网时间。
