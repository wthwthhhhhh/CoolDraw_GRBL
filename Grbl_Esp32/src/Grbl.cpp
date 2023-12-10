/*
  Grbl.cpp - Initialization and main loop for Grbl
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

#include "Grbl.h"
#include <WiFi.h>
#include <esp_timer.h>
// 定时器ID
esp_timer_handle_t timer;
// 定时器回调函数
void timerCallback(void* arg)
{
    
                float* MPosition   = system_get_mpos();
 
                char strvalX[32];
                (void)sprintf(strvalX, "%.3f", MPosition[X_AXIS]);
                char strvalY[32];
                (void)sprintf(strvalY, "%.3f", MPosition[Y_AXIS]);
                //存储原点、当前点信息至eerom
                axis_settings[X_AXIS]->home_mpos->setStringValue(strvalX);
                axis_settings[X_AXIS]->run_current->setStringValue(strvalX);
                axis_settings[X_AXIS]->hold_current->setStringValue(strvalX);

                axis_settings[Y_AXIS]->home_mpos->setStringValue(strvalY);
                axis_settings[Y_AXIS]->run_current->setStringValue(strvalY);
                axis_settings[Y_AXIS]->hold_current->setStringValue(strvalY);
}

void grbl_init() {
    
#ifdef USE_I2S_OUT
    i2s_out_init();  // The I2S out must be initialized before it can access the expanded GPIO port
#endif
    WiFi.persistent(false);
    WiFi.disconnect(true);
    WiFi.enableSTA(false);
    WiFi.enableAP(false);
    WiFi.mode(WIFI_OFF);
    client_init();  // Setup serial baud rate and interrupts
    grbl_msg_sendf(CLIENT_SERIAL, MsgLevel::Info, "Grbl_ESP32 Ver %s Date %s", GRBL_VERSION, GRBL_VERSION_BUILD);  // print grbl_esp32 verion info
    grbl_msg_sendf(CLIENT_SERIAL, MsgLevel::Info, "Compiled with ESP32 SDK:%s", ESP.getSdkVersion());              // print the SDK version
// show the map name at startup
#ifdef MACHINE_NAME
    report_machine_type(CLIENT_SERIAL);
#endif
    settings_init();  // Load Grbl settings from non-volatile storage
    stepper_init();   // Configure stepper pins and interrupt timers
    system_ini();     // Configure pinout pins and pin-change interrupt (Renamed due to conflict with esp32 files)
    init_motors();
    //memset(sys_position, 0, sizeof(sys_position));  // Clear machine position.
    machine_init();       
    display_init();                        // weak definition in Grbl.cpp does nothing
    // Initialize system state.
#ifdef FORCE_INITIALIZATION_ALARM
    // Force Grbl into an ALARM state upon a power-cycle or hard reset.
    sys.state = State::Alarm;
#else
    sys.state = State::Idle;
#endif
//检查是否上电，如果homeing是强制homeing循环，则设置系统告警
//通过设置Grbl的报警状态。告警锁定所有g-code命令，包括
//启动脚本，但允许访问设置和内部命令。只有回家
//循环'$H'或杀死警报锁'$X'将使警报失效。
//注意:启动脚本将在homeing循环成功完成后运行，但是
//禁用告警锁后不生效。防止运动启动块碰撞
//事情无法控制。非常糟糕。
#ifdef HOMING_INIT_LOCK
    if (homing_enable->get()) {
        sys.state = State::Alarm;
    }
#endif
    Spindles::Spindle::select();
#ifdef ENABLE_WIFI
    WebUI::wifi_config.begin();
#endif
#ifdef ENABLE_BLUETOOTH
    WebUI::bt_config.begin();
#endif
    WebUI::inputBuffer.begin();

     esp_timer_create_args_t timerArgs;
    timerArgs.callback = &timerCallback;
    esp_timer_create(&timerArgs, &timer);
    
    // 启动定时器，设置定时器周期为1秒
    esp_timer_start_periodic(timer, 1000000); // 1秒 = 1,000,000 微秒
}

static void reset_variables() {
    // Reset system variables.
    State prior_state = sys.state;
    memset(&sys, 0, sizeof(system_t));  // Clear system struct variable.
    sys.state             = prior_state;
    sys.f_override        = FeedOverride::Default;              // Set to 100%
    sys.r_override        = RapidOverride::Default;             // Set to 100%
    sys.spindle_speed_ovr = SpindleSpeedOverride::Default;      // Set to 100%
    memset(sys_probe_position, 0, sizeof(sys_probe_position));  // Clear probe position.

    sys_probe_state                      = Probe::Off;
    sys_rt_exec_state.value              = 0;
    sys_rt_exec_accessory_override.value = 0;
    sys_rt_exec_alarm                    = ExecAlarm::None;
    cycle_stop                           = false;
    sys_rt_f_override                    = FeedOverride::Default;
    sys_rt_r_override                    = RapidOverride::Default;
    sys_rt_s_override                    = SpindleSpeedOverride::Default;

    // Reset Grbl primary systems.
    client_reset_read_buffer(CLIENT_ALL);
    gc_init();  // Set g-code parser to default state
    spindle->stop();
    coolant_init();
    limits_init();
    probe_init();
    plan_reset();  // Clear block buffer and planner variables
    st_reset();    // Clear stepper subsystem variables
    // Sync cleared gcode and planner positions to current system position.
    plan_sync_position();
    gc_sync_position();
    report_init_message(CLIENT_ALL);

    // used to keep track of a jog command sent to mc_line() so we can cancel it.
    // this is needed if a jogCancel comes along after we have already parsed a jog and it is in-flight.
    sys_pl_data_inflight = NULL;
}

void run_once() {
    reset_variables();
    // Start Grbl main loop. Processes program inputs and executes them.
    // This can exit on a system abort condition, in which case run_once()
    // is re-executed by an enclosing loop.
    protocol_main_loop();
}

void __attribute__((weak)) machine_init() {}

void __attribute__((weak)) display_init() {}

void __attribute__((weak)) user_m30() {}

void __attribute__((weak)) user_tool_change(uint8_t new_tool) {}
/*
  setup() and loop() in the Arduino .ino implements this control flow:

  void main() {
     init();          // setup()
     while (1) {      // loop()
         run_once();
     }
  }
*/
