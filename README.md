# 参考项目
1. HAL_血氧设计2022_3_27


2. 血氧仪
https://oshwhub.com/ilex_35/CW32L-xue-yang-yi

3. Oximeter
https://github.com/kmakise/Oximeter

    MAX30100
    一个基于STM32的MAX30100简易血氧心率图示仪，演示视频（B站）https://www.bilibili.com/video/BV1AJ411Y7yY。

    说明
    主控：STM32F103C8T6，屏幕ST7735 128*128 spi三线，传感器：MAX30100。

    IO连接
    传感器使用的是模拟IIC PB6，PB7，屏幕使用的是参考资料里面pdf中的最少连接。

    程序
    基于STM32标准库开发，数据处理使用的是FFT和DCfilter，由于内存问题就没有使用RTOS和显示缓冲区。

4. MAX30102_for_STM32_HAL
https://github.com/eepj/MAX30102_for_STM32_HAL
