# 智能家居系统

基于STM32 + ESP8266 + Flutter的智能家居控制系统，支持手机APP控制灯光、传感器数据实时显示等功能。
后续会进一步完善，由于器件没买......
## 系统架构

```
┌─────────────┐         WiFi          ┌─────────────┐         UART         ┌─────────────┐
│  Flutter APP │ ◄──────────────────► │   ESP8266    │ ◄──────────────────► │    STM32     │
│   (手机端)   │    TCP 协议           │  (WiFi模块)  │    AT 指令           │  (主控制器)  │
└─────────────┘                       └─────────────┘                      └─────────────┘
                                                                                │
                                                                    ┌───────────┼───────────┐
                                                                    │           │           │
                                                                ┌───┴───┐  ┌────┴────┐  ┌───┴───┐
                                                                │ DHT11 │  │  LED x3 │  │ 继电器 │
                                                                │温湿度 │  │  灯光   │  │  门锁  │
                                                                └───────┘  └─────────┘  └───────┘

调试过程中：学习了以下：

1.ESP8266模块（WIFI STM32）】>>https://www.bilibili.com/video/BV1oSxteYEZy?vd_source=bf6d28135527fadd0cb75c4b2967e012
2.【HC-SR501人体红外传感器详解（STM32）】>>https://www.bilibili.com/video/BV1ySsEeUEdt?vd_source=bf6d28135527fadd0cb75c4b2967e012
3.这是我刚开始用St-linkV2烧不进去的看的，但是都没解决，结果是我用keil里面“debug-ST-link debugger-settings-port-SW”才识别到型号，但是这个文件还是有用的【金山文档 | WPS云文档】 3460782297STM32下载时常见报错及解决方法 >> https://www.kdocs.cn/l/cpwg3oMPMXfa
4.串口调试助手（正点原子）：官网下载：http://www.openedv.com/docs/old-products/ruanjian/ATK-XCOM.html；不过他这个也是提供百度网盘下载链接，（exe文件）1M不到，我这里挂一个夸克网盘的>>https://pan.quark.cn/s/283f46835e3f


如有问题，请提交 Issue 或联系作者：2023553292@qq.com
