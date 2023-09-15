# 基于YOLOv2的人脸识别门禁系统(K210+Python+RFID+Bluetooth)
>  If you find it useful, welcome to star⭐, and you are also welcome to submit an issue for further discussion or PR proofreading.

如果觉得有用，欢迎star⭐，同时也欢迎提issue进一步讨论或pr校对。

## 简介

该系统通过对人脸图像进行采集变换与分析，综合传感器、蓝牙和RFID进行验证逻辑设计与处理，数据采集模块采用YOLO算法特征提取并且与存储的特征值进行比对验证，人脸信息录入模块将人员信息与人脸特征值进行关联并实现断电存储，人脸蓝牙RFID综合验证模块接收蓝牙数据，根据收到的数据执行相应的注册、删除、临时开启以及清空等操作，结合RFID技术，增加了多种权限的门禁认证，此外检测到无人时会进入休眠模式节能。

## 演示视频



## 运行环境

- 开发环境：MaixPy IDE
- 烧录环境：kflash_gui
- 串口调试环境：XCOM
- Windows11

## 快速使用

>  若开发者不熟悉K210，建议先阅读一遍官方的文档指南：https://wiki.sipeed.com/news/MaixPy/K210_usage.html

1. 烧录K210固件 烧录完成后才可以连接IDE软件，固件版本可以根据实际需要，本项目的固件在目录 `/public/firmware` 下。
2. 将 `key_gen.bin` 这个固件通过 Kflash 烧录到开发板上，按照[官方文档](https://wiki.sipeed.com/news/MaixPy/K210_usage.html#%E5%A6%82%E4%BD%95%E8%8E%B7%E5%8F%96%E5%BC%80%E5%8F%91%E6%9D%BF%EF%BC%88%E6%9C%BA%E5%99%A8%E7%A0%81%EF%BC%89)的方法获取机器码。
3. [获取 AI 模型](https://maixhub.com/model/zoo/60)，本项目的示例模型在目录 `/public/model` 下。
4. 通过MaixPy IDE 将实际模型和初始的`/public/SD/faceinfo.txt`发送到SD卡中。(点击工具 -> 发送文件到开发板 选择初始的模型与文件)
5. 使用 MaixPy IDE 连接开发版并运行 `yolov2_face_recognition_access_control.py`。

## 详解文章

[基于YOLOv2和传感器的多功能门禁系统]()

## 文件说明

```
.
|-- LICENSE													# 项目LICENSE
|-- README.md												# README
|-- docs													# 设计文档与总结文档
|   |-- design.docx
|   |-- face-recognition-door.md
|   `-- summarize.docx
|-- pics													# 项目流程图、硬件图及调用图
|   |-- flowchart.jpg
|   |-- flowchart.vsdx
|   |-- hardware.png
|   `-- pycallgraph.png
|-- public
|   |-- SD													# SD卡保存内容
|   |   `-- faceinfo.txt
|   |-- firmware											# K210固件
|   |   |-- key_gen_v1.2.bin
|   |   `-- maixpy_v0.6.2_27_g4d8d4fbf0_minimum_with_ide_support.bin
|   `-- model												# 加密后的模型文件
|       |-- FaceDetection.smodel
|       |-- FaceLandmarkDetection.smodel
|       |-- FeatureExtraction.smodel
|       `-- main.py
|-- slides													# 设计slide与总结slide
|   |-- design.pptx
|   `-- summarize.pptx
|-- software												# 软件
|   |-- XCOM V2.3.exe
|   |-- dotNetFx40_Full_x86_x64.exe
|   |-- kflash_gu_v1.6.5_2.rar
|   `-- maixpy-ide-windows-0.2.5.exe
|-- utils													# 工具模块类
|   |-- STM32												# STM32主控模块
|   |   |-- CORE
|   |   |   |-- core_cm3.c
|   |   |   |-- core_cm3.h
|   |   |   |-- startup_stm32f10x_hd.s
|   |   |   `-- startup_stm32f10x_md.s
|   |   |-- HARDWARE
|   |   |   |-- JQ8X00
|   |   |   |   |-- JQ8X00.c
|   |   |   |   `-- JQ8X00.h
|   |   |   `-- LED
|   |   |       |-- led.c
|   |   |       `-- led.h
|   |   |-- OBJ
|   |   |   `-- USART.hex
|   |   |-- README.TXT
|   |   |-- STM32F10x_FWLib
|   |   |   |-- inc
|   |   |   |   |-- misc.h
|   |   |   |   |-- stm32f10x_adc.h
|   |   |   |   |-- stm32f10x_bkp.h
|   |   |   |   |-- stm32f10x_can.h
|   |   |   |   |-- stm32f10x_cec.h
|   |   |   |   |-- stm32f10x_crc.h
|   |   |   |   |-- stm32f10x_dac.h
|   |   |   |   |-- stm32f10x_dbgmcu.h
|   |   |   |   |-- stm32f10x_dma.h
|   |   |   |   |-- stm32f10x_exti.h
|   |   |   |   |-- stm32f10x_flash.h
|   |   |   |   |-- stm32f10x_fsmc.h
|   |   |   |   |-- stm32f10x_gpio.h
|   |   |   |   |-- stm32f10x_i2c.h
|   |   |   |   |-- stm32f10x_iwdg.h
|   |   |   |   |-- stm32f10x_pwr.h
|   |   |   |   |-- stm32f10x_rcc.h
|   |   |   |   |-- stm32f10x_rtc.h
|   |   |   |   |-- stm32f10x_sdio.h
|   |   |   |   |-- stm32f10x_spi.h
|   |   |   |   |-- stm32f10x_tim.h
|   |   |   |   |-- stm32f10x_usart.h
|   |   |   |   `-- stm32f10x_wwdg.h
|   |   |   `-- src
|   |   |       |-- misc.c
|   |   |       |-- stm32f10x_adc.c
|   |   |       |-- stm32f10x_bkp.c
|   |   |       |-- stm32f10x_can.c
|   |   |       |-- stm32f10x_cec.c
|   |   |       |-- stm32f10x_crc.c
|   |   |       |-- stm32f10x_dac.c
|   |   |       |-- stm32f10x_dbgmcu.c
|   |   |       |-- stm32f10x_dma.c
|   |   |       |-- stm32f10x_exti.c
|   |   |       |-- stm32f10x_flash.c
|   |   |       |-- stm32f10x_fsmc.c
|   |   |       |-- stm32f10x_gpio.c
|   |   |       |-- stm32f10x_i2c.c
|   |   |       |-- stm32f10x_iwdg.c
|   |   |       |-- stm32f10x_pwr.c
|   |   |       |-- stm32f10x_rcc.c
|   |   |       |-- stm32f10x_rtc.c
|   |   |       |-- stm32f10x_sdio.c
|   |   |       |-- stm32f10x_spi.c
|   |   |       |-- stm32f10x_tim.c
|   |   |       |-- stm32f10x_usart.c
|   |   |       `-- stm32f10x_wwdg.c
|   |   |-- SYSTEM
|   |   |   |-- delay
|   |   |   |   |-- delay.c
|   |   |   |   `-- delay.h
|   |   |   |-- sys
|   |   |   |   |-- sys.c
|   |   |   |   `-- sys.h
|   |   |   `-- usart
|   |   |       |-- usart.c
|   |   |       |-- usart.c.orig
|   |   |       |-- usart.h
|   |   |       |-- usart2.c
|   |   |       `-- usart2.h
|   |   |-- USER
|   |   |   |-- DebugConfig
|   |   |   |   `-- USART_STM32F103ZE_1.0.0.dbgconf
|   |   |   |-- JLinkSettings.ini
|   |   |   |-- USART.uvguix.Administrator
|   |   |   |-- USART.uvguix.JW
|   |   |   |-- USART.uvguix.hc
|   |   |   |-- USART.uvoptx
|   |   |   |-- USART.uvprojx
|   |   |   |-- main.c
|   |   |   |-- main.c.orig
|   |   |   |-- stm32f10x.h
|   |   |   |-- stm32f10x_conf.h
|   |   |   |-- stm32f10x_it.c
|   |   |   |-- stm32f10x_it.h
|   |   |   |-- system_stm32f10x.c
|   |   |   `-- system_stm32f10x.h
|   |   `-- keilkilll.bat
|   `-- temperature_measurement_and_voice.py					# 附加测温与语音播报模块
|-- video
|   `-- presentation.mp4										# 演示视频
`-- yolov2_face_recognition_access_control.py					# 系统代码
```

## Tips

1. 如果存在疑问或发现错误，欢迎提Issues交流订正。
2. 如果遇到图片无法加载的情况，可以考虑使用代理，或者访问[博客网站](https://blog.csdn.net/m0_52316372) 。
3. 如果发现Tex数学公式展示异常，可以安装插件[GitHub Math Display](https://chrome.google.com/webstore/detail/github-math-display/cgolaobglebjonjiblcjagnpmdmlgmda?hl=zh-CN)，安装后启用插件，刷新网页即可。也可以下载后本地软件打开。

## License

Provided under the [Apache-2.0 license](https://github.com/timerring/mmpretrain/blob/main/LICENSE).
