#pragma once

//您可以在此文件中通过包含来选择机器类型
//如下所述的一个或多个机器定义文件。

#ifndef MACHINE_FILENAME

//! !对于初始测试，从禁用test_drive.h开始
//所有I/O引脚
//# include“机器/ atari_1020.h”
//#include "Machines/midTbot.h"
#include "Machines/Cool_Draw.h"
//#include "Machines/test_drive.h"
//! !实际使用时，请更改上面的线以选择一个板
//from Machines/，例如:
//# include“机器/ 3 axis_v4.h”
//=== OEM单文件配置选项
//希望发布配置为的源代码的oem
//特定的机器可以放入它们的所有配置定义
//直接在这个文件中，不包括上面的任何其他文件。

#else

//通过使用外部环境定义MACHINE_FILENAME，
//可以在不编辑此文件的情况下选择配置。
//这对于自动化测试脚本非常有用。
//
//例如，在使用平台编译环境时
//在Linux下，你可以发出以下命令行:
//PLATFORMIO_BUILD_FLAGS=-DMACHINE_FILENAME=3axis_v4.h平台运行
//
//在Windows下，使用PowerShell，命令是:
//$ env: PLATFORMIO_BUILD_FLAGS =“-DMACHINE_FILENAME = 3 axis_v4.h”;platformio运行
//
//在使用Arduino IDE时，没有简单的方法来传递变量
//到编译器，所以这个特性对Arduino没有用处。
//
//MACHINE_FILENAME不能包含Machines/路径前缀;它是
//自动提供。
//machine_pathname_quotation构造一个适合于#include的路径
#    define MACHINE_PATHNAME_QUOTED(name) <src/Machines/name>

#    include MACHINE_PATHNAME_QUOTED(MACHINE_FILENAME)

#endif  // MACHINE_FILENAME