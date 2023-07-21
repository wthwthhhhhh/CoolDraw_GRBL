#pragma once

//所有机器通用的Grbl设置
//这里应该没有必要做任何修改

#ifndef GRBL_SPI_FREQ
//您可以通过在一个板文件中定义它们来覆盖它们。
//若要覆盖，必须设置所有这些参数
//-1表示使用默认的单板引脚
#    define GRBL_SPI_SS -1
#    define GRBL_SPI_MOSI -1
#    define GRBL_SPI_MISO -1
#    define GRBL_SPI_SCK -1
#    define GRBL_SPI_FREQ 4000000
#endif

// ESP32 CPU Settings
const uint32_t fTimers = 80000000;// ESP32计时器速度的引用
//=============== 不改变或这些注释掉  ======================
//它们用于遗留目的，不会影响您的I/O

const int STEP_MASK = B111111;

const int PROBE_MASK = 1;
