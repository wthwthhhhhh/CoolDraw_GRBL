#pragma once

/*
  System.h - Header for system level commands and real-time processes
  Part of Grbl
  Copyright (c) 2014-2016 Sungeun K. Jeon for Gnea Research LLC

	2018 -	Bart Dring This file was modifed for use on the ESP32
		CPU. Do not use this with Grbl for atMega328P

  Grbl is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.
  Grbl is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.
  You should have received a copy of the GNU General Public License
  along with Grbl.  If not, see <http://www.gnu.org/licenses/>.
*/

// Execution states and alarm
#include "Exec.h"

// System states. The state variable primarily tracks the individual functions
// of Grbl to manage each without overlapping. It is also used as a messaging flag for
// critical events.
enum class State : uint8_t {
    Idle = 0,    // Must be zero.
    Alarm,       // In alarm state. Locks out all g-code processes. Allows settings access.
    CheckMode,   // G-code check mode. Locks out planner and motion only.
    Homing,      // Performing homing cycle
    Cycle,       // Cycle is running or motions are being executed.
    Hold,        // Active feed hold
    Jog,         // Jogging mode.
    SafetyDoor,  // Safety door is ajar. Feed holds and de-energizes system.
    Sleep,       // Sleep state.
};

// Step segment generator state flags.
struct StepControl {
    uint8_t endMotion : 1;
    uint8_t executeHold : 1;
    uint8_t executeSysMotion : 1;
    uint8_t updateSpindleRpm : 1;
};

// System suspend flags. Used in various ways to manage suspend states and procedures.
struct SuspendBits {
    uint8_t holdComplete : 1;     // Indicates initial feed hold is complete.
    uint8_t restartRetract : 1;   // Flag to indicate a retract from a restore parking motion.
    uint8_t retractComplete : 1;  // (Safety door only) Indicates retraction and de-energizing is complete.
    uint8_t initiateRestore : 1;  // (Safety door only) Flag to initiate resume procedures from a cycle start.
    uint8_t restoreComplete : 1;  // (Safety door only) Indicates ready to resume normal operation.
    uint8_t safetyDoorAjar : 1;   // Tracks safety door state for resuming.
    uint8_t motionCancel : 1;     // Indicates a canceled resume motion. Currently used by probing routine.
    uint8_t jogCancel : 1;        // Indicates a jog cancel in process and to reset buffers when complete.
};
union Suspend {
    uint8_t     value;
    SuspendBits bit;
};

typedef uint8_t AxisMask;  // Bits indexed by axis number
typedef uint8_t Percent;   // Integer percent
typedef uint8_t Counter;   // Report interval

enum class Override : uint8_t {
#ifdef DEACTIVATE_PARKING_UPON_INIT
    Disabled      = 0,  // (Default: Must be zero)
    ParkingMotion = 1,  // M56
#else
    ParkingMotion = 0,  // M56 (Default: Must be zero)
    Disabled      = 1,  // Parking disabled.
#endif
};

// Spindle stop override control states.
struct SpindleStopBits {
    uint8_t enabled : 1;
    uint8_t initiate : 1;
    uint8_t restore : 1;
    uint8_t restoreCycle : 1;
};
union SpindleStop {
    uint8_t         value;
    SpindleStopBits bit;
};

// Global system variables
typedef struct {
    volatile State state;               // 跟踪Grbl的当前系统状态。
    bool           abort;//系统中止标志。强制退出回主循环进行重置。
    Suspend        suspend;             // 系统暂停位标志变量，用于管理保持、取消和安全门。
    bool           soft_limit;          // 跟踪状态机的软限制错误。(布尔)
    StepControl    step_control;        // 根据系统状态控制步进段生成器。
    bool           probe_succeeded;//跟踪最后一个探测周期是否成功。
    AxisMask       homing_axis_lock;//当限制啮合时锁定轴。在步进ISR中用作轴运动掩码。
    Percent        f_override;//进给速率覆盖百分比值
    Percent        r_override;//急流覆盖百分比的值
    Percent        spindle_speed_ovr;//主轴转速以百分比表示
    SpindleStop    spindle_stop_ovr;//跟踪主轴停止覆盖状态
    Counter        report_ovr_counter;//跟踪何时向状态报告添加覆盖数据。
    Counter        report_wco_counter;//跟踪何时向状态报告添加工作坐标偏移数据。
#ifdef ENABLE_PARKING_OVERRIDE_CONTROL
    Override override_ctrl;//跟踪覆盖控制状态。
#endif
    uint32_t spindle_speed;
} system_t;
extern system_t sys;

// Control pin states
struct ControlPinBits {
    uint8_t safetyDoor : 1;
    uint8_t reset : 1;
    uint8_t feedHold : 1;
    uint8_t cycleStart : 1;
    uint8_t macro0 : 1;
    uint8_t macro1 : 1;
    uint8_t macro2 : 1;
    uint8_t macro3 : 1;
};
union ControlPins {
    uint8_t        value;
    ControlPinBits bit;
};

//注意:如果出现问题，这些位置变量可能需要声明为volatile。
extern int32_t sys_position[MAX_N_AXIS];//实时机器(又名home)的步长位置向量。
extern int32_t sys_probe_position[MAX_N_AXIS];//在机器坐标和步长的最后一个探测位置。

extern volatile Probe         sys_probe_state;    // Probing state value.  Used to coordinate the probing cycle with stepper ISR.
extern volatile ExecState     sys_rt_exec_state;  // Global realtime executor bitflag variable for state management. See EXEC bitmasks.
extern volatile ExecAlarm     sys_rt_exec_alarm;  // Global realtime executor bitflag variable for setting various alarms.
extern volatile ExecAccessory sys_rt_exec_accessory_override;  // Global realtime executor bitflag variable for spindle/coolant overrides.
extern volatile Percent       sys_rt_f_override;               // Feed override value in percent
extern volatile Percent       sys_rt_r_override;               // Rapid feed override value in percent
extern volatile Percent       sys_rt_s_override;               // Spindle override value in percent
extern volatile bool          cycle_stop;
extern volatile void* sys_pl_data_inflight;  // holds a plan_line_data_t while cartesian_to_motors has taken ownership of a line motion
#ifdef DEBUG
extern volatile bool sys_rt_exec_debug;
#endif

void system_ini();  // Renamed from system_init() due to conflict with esp32 files

// Returns bitfield of control pin states, organized by CONTROL_PIN_INDEX. (1=triggered, 0=not triggered).
ControlPins system_control_get_state();

// Returns if safety door is ajar(T) or closed(F), based on pin state.
uint8_t system_check_safety_door_ajar();

void isr_control_inputs();

// Execute the startup script lines stored in non-volatile storage upon initialization
void  system_execute_startup(char* line);
Error execute_line(char* line, uint8_t client, WebUI::AuthenticationLevel auth_level);
Error system_execute_line(char* line, WebUI::ESPResponseStream*, WebUI::AuthenticationLevel);
Error system_execute_line(char* line, uint8_t client, WebUI::AuthenticationLevel);
Error do_command_or_setting(const char* key, char* value, WebUI::AuthenticationLevel auth_level, WebUI::ESPResponseStream*);
void  system_flag_wco_change();

// Returns machine position of axis 'idx'. Must be sent a 'step' array.
float system_convert_axis_steps_to_mpos(int32_t* steps, uint8_t idx);

// Updates a machine 'position' array based on the 'step' array sent.
void   system_convert_array_steps_to_mpos(float* position, int32_t* steps);
float* system_get_mpos();

// A task that runs after a control switch interrupt for debouncing.
void controlCheckTask(void* pvParameters);
void system_exec_control_pin(ControlPins pins);

bool sys_set_digital(uint8_t io_num, bool turnOn);
void sys_digital_all_off();
bool sys_set_analog(uint8_t io_num, float percent);
void sys_analog_all_off();

int8_t sys_get_next_PWM_chan_num();

float* sys_get_rivetA();
float* sys_get_rivetB();