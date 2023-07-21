/*
Motor.cpp
Grbl_ESP32的一部分
2020年，巴特·德林
Grbl是免费软件:您可以重新发布它和/或修改它
它遵循GNU通用公共许可证的条款，由
自由软件基金会，或许可版本3，或
(由你选择)任何以后的版本。
发布Grbl是希望它会有用，
但不作任何保证;甚至没有隐含的保证
适合某一特定目的的适销性或适用性。看到
有关GNU通用公共许可证的详细信息。
您应该已经收到GNU通用公共许可证的副本
还有Grbl。如果没有，请参见<http://www.gnu.org/licenses/>。
待办事项
确保public/private/protected被清理干净。
在init()中只设置了几个单极轴
摆脱Z_SERVO，只回复Z_SERVO_PIN
类已准备好处理非SPI引脚，但还不需要它们。
这将是很好的配置消息尽管
测试
(成功)
3轴(3标准步进)
MPCNC(与共享方向销连接)
TMC2130 Pen Laser(三极管，stallguard调谐)
单极
待办事项
4轴SPI(菊花链，带有独特的方向引脚)
参考
TMC2130 Datasheet https://www.trinamic.com/fileadmin/assets/Products/ICs_Documents/TMC2130_datasheet.pdf
*/

#include "Motor.h"

namespace Motors {
    Motor::Motor(uint8_t axis_index) : _axis_index(axis_index % MAX_AXES), _dual_axis_index(axis_index / MAX_AXES) {}

    void Motor::debug_message() {}

    bool Motor::test() { return true; };  // true = OK

}
