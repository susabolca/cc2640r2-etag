# cc2640r2-etag

cc2640r2 电子标签改电子时钟固件.
> 显示时钟日历，或静态图片。

目前支持,
* cc2640r2l_2in13_ssd1680_250x122
* cc2640r2l_2in9_ssd1680a_296x128 **主要支持**

![cc2640r2l_2in9_ssd1680a_296x128](doc/pic1.jpg)

单节或者双节 CR2450 电池供电.

### 8级灰度支持

通过控制 LUT，仅保留 Black 和 Red 的刷新，控制刷新电压及时间，在多次刷新下可获得灰度图片。

![8级灰度](doc/pic_gray8.jpg)

tools 下提供 bwr_gray8.act 调色板，便于在 Photoshop 里生成 BWR 色彩的8级灰度图像。

## 编译

IAR 9.40 或 CCS 12.5

SDK: simplelink_cc2640r2_sdk_1_40_00_45

直接 IDE 编译出固件刷入即可. 

## 烧写

电子标签为 cjtag 2 wire, 需要 xds 或者 jlink 烧写器.

> 可自制 XDS110 烧写器

#### cc2640r2l_2in9_ssd1680a_296x128

![cjtag](doc/pic2.jpg)

|PIN|FOR|
|-|-|
|1|GND|
|2|VCC|
|3|TCLK|
|4|TMS|
|7|NRST|

## 低功耗蓝牙 BLE5

电子标签可使用 BLE5 进行配置。

可使用 tools 下 cc2640r2_etag.html (chrome 蓝牙) 配置, 或手机端 nRF connect App 配置.

GATT 配置, 

* Main Service UUID: FFF0,
* Characteristic 配置,
  - UUID: FFF1, Unix Epoch 时间, uint32, 默认为 0 (1970-01-01 00:00)
  - UUID: FFF2, 时区偏移分钟, int32, 默认为北京时区 (+8*60)
  - UUID: FFF3, 电池电压 mV, uint16
  - UUID: FFF4, 温度 摄氏度, int8, (-127 ~ +128)
  - UUID: FFF5, RTC 微调, int8, (-5 ~ +5)
  - UUID: FFFE, RxTx 服务, 图片模式、灰度刷新、内置LUT更新等

### Advertising

本固件通过 BLE Adv 中的 Service Data 通告当前 etag 的数据，格式如下

```
UUID, MAC address, Display Mode, Unix Epoch Time, Temperature, Battery Level.
```

通告间隔为 1s。

可通过 ble5_ctrl.py 批量获得近距离 etag 的信息。

## webtools

当前配置能力由 cc2640r2_etag.html 完成，需使用支持 BLE5 蓝牙的 chrome 浏览器打开。

时钟模式功能，
1. 获取当前 etag 的信息
2. 一键设置当前时间、当前时区、RTC 微调等

图片模式，
1. 选择 BWR 三色图像 (默认刷新，15秒左右)
2. 选择 BWR 灰度图像 (8级灰度刷新，2分钟左右)

SNV 功能，
1. 保存当前设置 (时区、RTC 微调，模式等)
2. 保存和查看自定义 LUT (可定义 3 个LUT)
