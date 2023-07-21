#pragma once

/*
Spindle.h
主轴类的头文件
有关更多细节，请参阅Spindle.cpp
Grbl_ESP32的一部分
2020 - bart Dring这个文件被修改为在ESP32上使用
CPU。不使用这与Grbl atMega328P
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
有关更多信息和参考，请参阅SpindleClass.cpp
*/

#include <cstdint>

enum class SpindleType : int8_t {
    NONE = 0,
    PWM,
    RELAY,
    LASER,
    DAC,
    HUANYANG,
    BESC,
    _10V,
    H2A,
    YL620,
    L510
};

#include "../Grbl.h"
#include <driver/dac.h>
#include <driver/uart.h>

// ===============  No floats! ===========================
// ================ NO FLOATS! ==========================

namespace Spindles {
    // This is the base class. Do not use this as your spindle
    class Spindle {
    public:
        Spindle() = default;

        Spindle(const Spindle&) = delete;
        Spindle(Spindle&&)      = delete;
        Spindle& operator=(const Spindle&) = delete;
        Spindle& operator=(Spindle&&) = delete;

        virtual void         init()                = 0;  // not in constructor because this also gets called when $$ settings change
        virtual uint32_t     set_rpm(uint32_t rpm) = 0;
        virtual void         set_state(SpindleState state, uint32_t rpm) = 0;
        virtual SpindleState get_state()                                 = 0;
        virtual void         stop()                                      = 0;
        virtual void         config_message()                            = 0;
        virtual bool         inLaserMode();
        virtual void         sync(SpindleState state, uint32_t rpm);
        virtual void         deinit();

        virtual ~Spindle() {}

        bool                  is_reversable;
        bool                  use_delays;  // will SpinUp and SpinDown delays be used.
        volatile SpindleState _current_state = SpindleState::Disable;
        uint32_t              _spinup_delay;
        uint32_t              _spindown_delay;

        static void select();
    };

}

extern Spindles::Spindle* spindle;
