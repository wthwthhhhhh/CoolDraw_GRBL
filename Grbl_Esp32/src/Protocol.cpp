/*
  Protocol.cpp - controls Grbl execution protocol and procedures
  Part of Grbl

  Copyright (c) 2011-2016 Sungeun K. Jeon for Gnea Research LLC
  Copyright (c) 2009-2011 Simen Svale Skogsrud

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

static void protocol_exec_rt_suspend();

static char    line[LINE_BUFFER_SIZE];     // Line to be executed. Zero-terminated.
static char    comment[LINE_BUFFER_SIZE];  // Line to be executed. Zero-terminated.
static uint8_t line_flags           = 0;
static uint8_t char_counter         = 0;
static uint8_t comment_char_counter = 0;

typedef struct {
    char buffer[LINE_BUFFER_SIZE];
    int  len;
    int  line_number;
} client_line_t;
client_line_t client_lines[CLIENT_COUNT];

static void empty_line(uint8_t client) {
    client_line_t* cl = &client_lines[client];
    cl->len           = 0;
    cl->buffer[0]     = '\0';
}
static void empty_lines() {
    for (uint8_t client = 0; client < CLIENT_COUNT; client++) {
        empty_line(client);
    }
}

Error add_char_to_line(char c, uint8_t client) {
    client_line_t* cl = &client_lines[client];
    // Simple editing for interactive input
    if (c == '\b') {
        // Backspace erases
        if (cl->len) {
            --cl->len;
            cl->buffer[cl->len] = '\0';
        }
        return Error::Ok;
    }
    if (cl->len == (LINE_BUFFER_SIZE - 1)) {
        return Error::Overflow;
    }
    if (c == '\r' || c == '\n') {
        cl->len = 0;
        cl->line_number++;
        return Error::Eol;
    }
    cl->buffer[cl->len++] = c;
    cl->buffer[cl->len]   = '\0';
    return Error::Ok;
}

Error execute_line(char* line, uint8_t client, WebUI::AuthenticationLevel auth_level) {
    Error result = Error::Ok;
    // Empty or comment line. For syncing purposes.
    if (line[0] == 0) {
        return Error::Ok;
    }
    // Grbl '$' or WebUI '[ESPxxx]' system command
    if (line[0] == '$' || line[0] == '[') {
        return system_execute_line(line, client, auth_level);
    }
    // Everything else is gcode. Block if in alarm or jog mode.
    if (sys.state == State::Alarm || sys.state == State::Jog) {
        return Error::SystemGcLock;
    }
    return gc_execute_line(line, client);
}

bool can_park() {
    return
#ifdef ENABLE_PARKING_OVERRIDE_CONTROL
        sys.override_ctrl == Override::ParkingMotion &&
#endif
        homing_enable->get() && !spindle->inLaserMode();
}

/*
  GRBL PRIMARY LOOP:
*/
void protocol_main_loop() {
    client_reset_read_buffer(CLIENT_ALL);
    empty_lines();
    //uint8_t client = CLIENT_SERIAL; // default client
    // Perform some machine checks to make sure everything is good to go.
#ifdef CHECK_LIMITS_AT_INIT
    if (hard_limits->get()) {
        if (limits_get_state()) {
            sys.state = State::Alarm;  // Ensure alarm state is active.
            report_feedback_message(Message::CheckLimits);
        }
    }
#endif
    // Check for and report alarm state after a reset, error, or an initial power up.
    // NOTE: Sleep mode disables the stepper drivers and position can't be guaranteed.
    // Re-initialize the sleep state as an ALARM mode to ensure user homes or acknowledges.
    if (sys.state == State::Alarm || sys.state == State::Sleep) {
        report_feedback_message(Message::AlarmLock);
        sys.state = State::Alarm;  // Ensure alarm state is set.
    } else {
        // Check if the safety door is open.
        sys.state = State::Idle;
        if (system_check_safety_door_ajar()) {
            sys_rt_exec_state.bit.safetyDoor = true;
            protocol_execute_realtime();  // Enter safety door mode. Should return as IDLE state.
        }
        // All systems go!
        system_execute_startup(line);  // Execute startup script.
    }
    // ---------------------------------------------------------------------------------
    // Primary loop! Upon a system abort, this exits back to main() to reset the system.
    // This is also where Grbl idles while waiting for something to do.
    // ---------------------------------------------------------------------------------
    int c;
    for (;;) {
#ifdef ENABLE_SD_CARD
        if (SD_ready_next) {
            char fileLine[255];
            if (readFileLine(fileLine, 255)) {
                SD_ready_next = false;
                report_status_message(execute_line(fileLine, SD_client, SD_auth_level), SD_client);
            } else {
                char temp[50];
                sd_get_current_filename(temp);
                grbl_notifyf("SD print done", "%s print is successful", temp);
                closeFile();  // close file and clear SD ready/running flags
            }
        }
#endif
        // Receive one line of incoming serial data, as the data becomes available.
        // Filtering, if necessary, is done later in gc_execute_line(), so the
        // filtering is the same with serial and file input.
        uint8_t client = CLIENT_SERIAL;
        char*   line;
        for (client = 0; client < CLIENT_COUNT; client++) {
            while ((c = client_read(client)) != -1) {
                Error res = add_char_to_line(c, client);
                switch (res) {
                    case Error::Ok:
                        break;
                    case Error::Eol:
                        protocol_execute_realtime();  // Runtime command check point.
                        if (sys.abort) {
                            return;  // Bail to calling function upon system abort
                        }
                        line = client_lines[client].buffer;
#ifdef REPORT_ECHO_RAW_LINE_RECEIVED
                        report_echo_line_received(line, client);
#endif
                        // auth_level can be upgraded by supplying a password on the command line
                        report_status_message(execute_line(line, client, WebUI::AuthenticationLevel::LEVEL_GUEST), client);
                        empty_line(client);
                        break;
                    case Error::Overflow:
                        report_status_message(Error::Overflow, client);
                        empty_line(client);
                        break;
                    default:
                        break;
                }
            }  // while serial read
        }      // for clients
        // If there are no more characters in the serial read buffer to be processed and executed,
        // this indicates that g-code streaming has either filled the planner buffer or has
        // completed. In either case, auto-cycle start, if enabled, any queued moves.
        protocol_auto_cycle_start();
        protocol_execute_realtime();  // Runtime command check point.
        if (sys.abort) {
            return;  // Bail to main() program loop to reset system.
        }
        // check to see if we should disable the stepper drivers ... esp32 work around for disable in main loop.
        if (stepper_idle && stepper_idle_lock_time->get() != 0xff) {
            if (esp_timer_get_time() > stepper_idle_counter) {
                motors_set_disable(true);
            }
        }
    }
    return; /* Never reached */
}

// Block until all buffered steps are executed or in a cycle state. Works with feed hold
// during a synchronize call, if it should happen. Also, waits for clean cycle end.
void protocol_buffer_synchronize() {
    // If system is queued, ensure cycle resumes if the auto start flag is present.
    protocol_auto_cycle_start();
    do {
        protocol_execute_realtime();  // Check and execute run-time commands
        if (sys.abort) {
            return;  // Check for system abort
        }
    } while (plan_get_current_block() || (sys.state == State::Cycle));
}

//当有一个动作准备执行时，如果主程序没有，自动循环启动就会触发
//主动解析命令。
//注意:此函数仅从主循环、缓冲区同步和mc_line()调用并执行
//当这些条件之一分别存在:没有更多的块发送(即流
//是完成，单命令)，一个命令，需要等待的运动在缓冲区
//执行调用缓冲区同步，或者计划缓冲区已满，可以执行。
void protocol_auto_cycle_start() {
    if (plan_get_current_block() != NULL) {//检查缓冲区中是否有任何块。
        sys_rt_exec_state.bit.cycleStart = true;//如果是，执行它们!
    }
}

// This function is the general interface to Grbl's real-time command execution system. It is called
// from various check points in the main program, primarily where there may be a while loop waiting
// for a buffer to clear space or any point where the execution time from the last check point may
// be more than a fraction of a second. This is a way to execute realtime commands asynchronously
// (aka multitasking) with grbl's g-code parsing and planning functions. This function also serves
// as an interface for the interrupts to set the system realtime flags, where only the main program
// handles them, removing the need to define more computationally-expensive volatile variables. This
// also provides a controlled way to execute certain tasks without having two or more instances of
// the same task, such as the planner recalculating the buffer upon a feedhold or overrides.
// NOTE: The sys_rt_exec_state.bit variable flags are set by any process, step or serial interrupts, pinouts,
// limit switches, or the main program.
void protocol_execute_realtime() {
    protocol_exec_rt_system();
    if (sys.suspend.value) {
        protocol_exec_rt_suspend();
    }
}

// Executes run-time commands, when required. This function primarily operates as Grbl's state
// machine and controls the various real-time features Grbl has to offer.
// NOTE: Do not alter this unless you know exactly what you are doing!
void protocol_exec_rt_system() {
    ExecAlarm alarm = sys_rt_exec_alarm;  // Temp variable to avoid calling volatile multiple times.
    if (alarm != ExecAlarm::None) {       // Enter only if an alarm is pending
        // System alarm. Everything has shutdown by something that has gone severely wrong. Report
        // the source of the error to the user. If critical, Grbl disables by entering an infinite
        // loop until system reset/abort.
        sys.state = State::Alarm;  // Set system alarm state
        report_alarm_message(alarm);
        // Halt everything upon a critical event flag. Currently hard and soft limits flag this.
        if ((alarm == ExecAlarm::HardLimit) || (alarm == ExecAlarm::SoftLimit)) {
            report_feedback_message(Message::CriticalEvent);
            sys_rt_exec_state.bit.reset = false;  // Disable any existing reset
            do {
                // Block everything, except reset and status reports, until user issues reset or power
                // cycles. Hard limits typically occur while unattended or not paying attention. Gives
                // the user and a GUI time to do what is needed before resetting, like killing the
                // incoming stream. The same could be said about soft limits. While the position is not
                // lost, continued streaming could cause a serious crash if by chance it gets executed.
            } while (!sys_rt_exec_state.bit.reset);
        }
        sys_rt_exec_alarm = ExecAlarm::None;
    }
    ExecState rt_exec_state;
    rt_exec_state.value = sys_rt_exec_state.value;  // Copy volatile sys_rt_exec_state.
    if (rt_exec_state.value != 0 || cycle_stop) {   // Test if any bits are on
        // Execute system abort.
        if (rt_exec_state.bit.reset) {
            sys.abort = true;  // Only place this is set true.
            return;            // Nothing else to do but exit.
        }
        // Execute and serial print status
        if (rt_exec_state.bit.statusReport) {
            report_realtime_status(CLIENT_ALL);
            sys_rt_exec_state.bit.statusReport = false;
        }
        // NOTE: Once hold is initiated, the system immediately enters a suspend state to block all
        // main program processes until either reset or resumed. This ensures a hold completes safely.
        if (rt_exec_state.bit.motionCancel || rt_exec_state.bit.feedHold || rt_exec_state.bit.safetyDoor || rt_exec_state.bit.sleep) {
            // State check for allowable states for hold methods.
            if (!(sys.state == State::Alarm || sys.state == State::CheckMode)) {
                // If in CYCLE or JOG states, immediately initiate a motion HOLD.
                if (sys.state == State::Cycle || sys.state == State::Jog) {
                    if (!(sys.suspend.bit.motionCancel || sys.suspend.bit.jogCancel)) {  // Block, if already holding.
                        st_update_plan_block_parameters();  // Notify stepper module to recompute for hold deceleration.
                        sys.step_control             = {};
                        sys.step_control.executeHold = true;  // Initiate suspend state with active flag.
                        if (sys.state == State::Jog) {        // Jog cancelled upon any hold event, except for sleeping.
                            if (!rt_exec_state.bit.sleep) {
                                sys.suspend.bit.jogCancel = true;
                            }
                        }
                    }
                }
                // If IDLE, Grbl is not in motion. Simply indicate suspend state and hold is complete.
                if (sys.state == State::Idle) {
                    sys.suspend.value            = 0;
                    sys.suspend.bit.holdComplete = true;
                }
                // Execute and flag a motion cancel with deceleration and return to idle. Used primarily by probing cycle
                // to halt and cancel the remainder of the motion.
                if (rt_exec_state.bit.motionCancel) {
                    // MOTION_CANCEL only occurs during a CYCLE, but a HOLD and SAFETY_DOOR may been initiated beforehand
                    // to hold the CYCLE. Motion cancel is valid for a single planner block motion only, while jog cancel
                    // will handle and clear multiple planner block motions.
                    if (sys.state != State::Jog) {
                        sys.suspend.bit.motionCancel = true;  // NOTE: State is State::Cycle.
                    }
                    sys_rt_exec_state.bit.motionCancel = false;
                }
                // Execute a feed hold with deceleration, if required. Then, suspend system.
                if (rt_exec_state.bit.feedHold) {
                    // Block SAFETY_DOOR, JOG, and SLEEP states from changing to HOLD state.
                    if (!(sys.state == State::SafetyDoor || sys.state == State::Jog || sys.state == State::Sleep)) {
                        sys.state = State::Hold;
                    }
                    sys_rt_exec_state.bit.feedHold = false;
                }
                // Execute a safety door stop with a feed hold and disable spindle/coolant.
                // NOTE: Safety door differs from feed holds by stopping everything no matter state, disables powered
                // devices (spindle/coolant), and blocks resuming until switch is re-engaged.
                if (rt_exec_state.bit.safetyDoor) {
                    report_feedback_message(Message::SafetyDoorAjar);
                    // If jogging, block safety door methods until jog cancel is complete. Just flag that it happened.
                    if (!(sys.suspend.bit.jogCancel)) {
                        // Check if the safety re-opened during a restore parking motion only. Ignore if
                        // already retracting, parked or in sleep state.
                        if (sys.state == State::SafetyDoor) {
                            if (sys.suspend.bit.initiateRestore) {  // Actively restoring
#ifdef PARKING_ENABLE
                                // Set hold and reset appropriate control flags to restart parking sequence.
                                if (sys.step_control.executeSysMotion) {
                                    st_update_plan_block_parameters();  // Notify stepper module to recompute for hold deceleration.
                                    sys.step_control                  = {};
                                    sys.step_control.executeHold      = true;
                                    sys.step_control.executeSysMotion = true;
                                    sys.suspend.bit.holdComplete      = false;
                                }  // else NO_MOTION is active.
#endif
                                sys.suspend.bit.retractComplete = false;
                                sys.suspend.bit.initiateRestore = false;
                                sys.suspend.bit.restoreComplete = false;
                                sys.suspend.bit.restartRetract  = true;
                            }
                        }
                        if (sys.state != State::Sleep) {
                            sys.state = State::SafetyDoor;
                        }
                        sys_rt_exec_state.bit.safetyDoor = false;
                    }
                    // NOTE: This flag doesn't change when the door closes, unlike sys.state. Ensures any parking motions
                    // are executed if the door switch closes and the state returns to HOLD.
                    sys.suspend.bit.safetyDoorAjar = true;
                }
            }
            if (rt_exec_state.bit.sleep) {
                if (sys.state == State::Alarm) {
                    sys.suspend.bit.retractComplete = true;
                    sys.suspend.bit.holdComplete    = true;
                }
                sys.state                   = State::Sleep;
                sys_rt_exec_state.bit.sleep = false;
            }
        }
//通过启动步进中断来开始执行队列中的块来执行循环开始。
        if (rt_exec_state.bit.cycleStart) {
//如果与hold命令同时调用，则阻塞:feed hold，运动取消和安全门。
//确保auto-cycle-start在没有明确的用户输入的情况下不会恢复暂停。
            if (!(rt_exec_state.bit.feedHold || rt_exec_state.bit.motionCancel || rt_exec_state.bit.safetyDoor)) {
//当停车运动已收回且车门已关闭时，恢复车门状态。
                if (sys.state == State::SafetyDoor && !(sys.suspend.bit.safetyDoorAjar)) {
                    if (sys.suspend.bit.restoreComplete) {
                        sys.state = State::Idle;//设置为IDLE立即恢复循环
                    } else if (sys.suspend.bit.retractComplete) {
//如果SAFETY_DOOR禁用，标记重新激活已供电的组件并恢复原始位置。
//注意:要使安全门恢复，开关必须关闭，如HOLD状态所示，和
//收回执行完成，这意味着初始提要保持不活动。来
//恢复正常操作，恢复过程必须由以下标志启动。有一次,
//当它们完成时，它会自动调用CYCLE_START来恢复并退出挂起。
                        sys.suspend.bit.initiateRestore = true;
                    }
                }
//只有在IDLE或等待完成并准备恢复时才开始循环。
                if (sys.state == State::Idle || (sys.state == State::Hold && sys.suspend.bit.holdComplete)) {
                    if (sys.state == State::Hold && sys.spindle_stop_ovr.value) {
                        sys.spindle_stop_ovr.bit.restoreCycle = true;//设置在暂停程序中恢复，然后循环启动。
                    } else {
//仅当计划器缓冲区中存在队列运动且运动未取消时才开始循环。
                        sys.step_control = {};//恢复步骤控制到正常操作
                        if (plan_get_current_block() && !sys.suspend.bit.motionCancel) {
                            sys.suspend.value = 0;//中断暂停状态。
                            sys.state         = State::Cycle;
                            st_prep_buffer();//在开始循环之前初始化步骤段缓冲区。
                            st_wake_up();
                        } else {//否则，什么都不做。设置并恢复IDLE状态。
                            sys.suspend.value = 0;//中断暂停状态。
                            sys.state         = State::Idle;
                        }
                    }
                }
            }
            sys_rt_exec_state.bit.cycleStart = false;
        }
        if (cycle_stop) {
//重新初始化循环计划和步进系统。被
//在主程序中实时执行命令，确保规划器安全重新规划。
//注意:Bresenham算法变量仍然通过规划器和步进器来维护
//循环重新初始化。步进路径应该继续，就像什么都没有发生过一样。
//注:cycle_stop由步进子系统设置，当一个循环或feed hold完成时。
            if ((sys.state == State::Hold || sys.state == State::SafetyDoor || sys.state == State::Sleep) && !(sys.soft_limit) &&
                !(sys.suspend.bit.jogCancel)) {
//保持完整。设置为表示准备恢复。保持HOLD或DOOR状态直到用户
//已经发出恢复命令或重置。
                plan_cycle_reinitialize();
                if (sys.step_control.executeHold) {
                    sys.suspend.bit.holdComplete = true;
                }
                sys.step_control.executeHold      = false;
                sys.step_control.executeSysMotion = false;
            } else {
//运动完成。包括CYCLE/JOG/HOMING状态和慢跑取消/运动取消/软限制事件。
//注意:运动和慢跑都在保持完成后立即返回空闲状态。
                if (sys.suspend.bit.jogCancel) {//对于jog取消，刷新缓冲区和同步位置。
                    sys.step_control = {};
                    plan_reset();
                    st_reset();
                    gc_sync_position();
                    plan_sync_position();
                }
                if (sys.suspend.bit.safetyDoorAjar) {//只在慢跑时安全门打开时发生。
                    sys.suspend.bit.jogCancel    = false;
                    sys.suspend.bit.holdComplete = true;
                    sys.state                    = State::SafetyDoor;
                } else {
                    sys.suspend.value = 0;
                    sys.state         = State::Idle;
                }
            }
            cycle_stop = false;
        }
    }
//执行重写。
    if ((sys_rt_f_override != sys.f_override) || (sys_rt_r_override != sys.r_override)) {
        sys.f_override         = sys_rt_f_override;
        sys.r_override         = sys_rt_r_override;
        sys.report_ovr_counter = 0;//设置立即报告更改
        plan_update_velocity_profile_parameters();
        plan_cycle_reinitialize();
    }

//注意:与运动覆盖不同，主轴覆盖不需要规划器重新初始化。
    if (sys_rt_s_override != sys.spindle_speed_ovr) {
        sys.step_control.updateSpindleRpm = true;
        sys.spindle_speed_ovr             = sys_rt_s_override;
        sys.report_ovr_counter            = 0;//设置立即报告更改
//如果spinlde是打开的，告诉它rpm已经被覆盖
        if (gc_state.modal.spindle != SpindleState::Disable) {
            spindle->set_rpm(gc_state.spindle_speed);
        }
    }

    if (sys_rt_exec_accessory_override.bit.spindleOvrStop) {
        sys_rt_exec_accessory_override.bit.spindleOvrStop = false;
//只有在HOLD状态下才允许主轴停止。
//注意:当执行主轴停止时，报表计数器在spindle_set_state()中设置。
        if (sys.state == State::Hold) {
            if (sys.spindle_stop_ovr.value == 0) {
                sys.spindle_stop_ovr.bit.initiate = true;
            } else if (sys.spindle_stop_ovr.bit.enabled) {
                sys.spindle_stop_ovr.bit.restore = true;
            }
        }
    }

//注意:由于冷却状态总是执行一个计划同步，每当它改变，当前
//运行状态可以通过检查解析器状态来确定。
    if (sys_rt_exec_accessory_override.bit.coolantFloodOvrToggle) {
        sys_rt_exec_accessory_override.bit.coolantFloodOvrToggle = false;
#ifdef COOLANT_FLOOD_PIN
        if (sys.state == State::Idle || sys.state == State::Cycle || sys.state == State::Hold) {
            gc_state.modal.coolant.Flood = !gc_state.modal.coolant.Flood;
            coolant_set_state(gc_state.modal.coolant);  // Report counter set in coolant_set_state().
        }
#endif
    }
    if (sys_rt_exec_accessory_override.bit.coolantMistOvrToggle) {
        sys_rt_exec_accessory_override.bit.coolantMistOvrToggle = false;
#ifdef COOLANT_MIST_PIN
        if (sys.state == State::Idle || sys.state == State::Cycle || sys.state == State::Hold) {
            gc_state.modal.coolant.Mist = !gc_state.modal.coolant.Mist;
            coolant_set_state(gc_state.modal.coolant);  // Report counter set in coolant_set_state().
        }
#endif
    }

#ifdef DEBUG
    if (sys_rt_exec_debug) {
        report_realtime_debug();
        sys_rt_exec_debug = false;
    }
#endif
//重新加载步骤段缓冲区
    switch (sys.state) {
        case State::Cycle:
        case State::Hold:
        case State::SafetyDoor:
        case State::Homing:
        case State::Sleep:
        case State::Jog:
            st_prep_buffer();
            break;
        default:
            break;
    }
}

//处理Grbl系统暂停程序，如送料舱，安全门，停车运动。
//系统将进入这个循环，为挂起任务创建局部变量，然后返回到
//调用挂起的函数，以便Grbl恢复正常操作。
//这个函数是用一种促进自定义停车运动的方式编写的。简单地使用这个作为
//模板
static void protocol_exec_rt_suspend() {
#ifdef PARKING_ENABLE
    // Declare and initialize parking local variables
    float             restore_target[MAX_N_AXIS];
    float             retract_waypoint = PARKING_PULLOUT_INCREMENT;
    plan_line_data_t  plan_data;
    plan_line_data_t* pl_data = &plan_data;
    memset(pl_data, 0, sizeof(plan_line_data_t));
    pl_data->motion                = {};
    pl_data->motion.systemMotion   = 1;
    pl_data->motion.noFeedOverride = 1;
#    ifdef USE_LINE_NUMBERS
    pl_data->line_number = PARKING_MOTION_LINE_NUMBER;
#    endif
#endif
    plan_block_t* block = plan_get_current_block();
    CoolantState  restore_coolant;
    SpindleState  restore_spindle;
    float         restore_spindle_speed;
    if (block == NULL) {
        restore_coolant       = gc_state.modal.coolant;
        restore_spindle       = gc_state.modal.spindle;
        restore_spindle_speed = gc_state.spindle_speed;
    } else {
        restore_coolant       = block->coolant;
        restore_spindle       = block->spindle;
        restore_spindle_speed = block->spindle_speed;
    }
#ifdef DISABLE_LASER_DURING_HOLD
    if (spindle->inLaserMode()) {
        sys_rt_exec_accessory_override.bit.spindleOvrStop = true;
    }
#endif

    while (sys.suspend.value) {
        if (sys.abort) {
            return;
        }
        // if a jogCancel comes in and we have a jog "in-flight" (parsed and handed over to mc_line()),
        //  then we need to cancel it before it reaches the planner.  otherwise we may try to move way out of
        //  normal bounds, especially with senders that issue a series of jog commands before sending a cancel.
        if (sys.suspend.bit.jogCancel && sys_pl_data_inflight != NULL && ((plan_line_data_t*)sys_pl_data_inflight)->is_jog) {
            sys_pl_data_inflight = NULL;
        }
        // Block until initial hold is complete and the machine has stopped motion.
        if (sys.suspend.bit.holdComplete) {
            // Parking manager. Handles de/re-energizing, switch state checks, and parking motions for
            // the safety door and sleep states.
            if (sys.state == State::SafetyDoor || sys.state == State::Sleep) {
                // Handles retraction motions and de-energizing.
#ifdef PARKING_ENABLE
                float* parking_target = system_get_mpos();
#endif
                if (!sys.suspend.bit.retractComplete) {
                    // Ensure any prior spindle stop override is disabled at start of safety door routine.
                    sys.spindle_stop_ovr.value = 0;  // Disable override
#ifndef PARKING_ENABLE
                    spindle->set_state(SpindleState::Disable, 0);  // De-energize
                    coolant_off();
#else
                    // Get current position and store restore location and spindle retract waypoint.
                    if (!sys.suspend.bit.restartRetract) {
                        memcpy(restore_target, parking_target, sizeof(restore_target[0]) * number_axis->get());
                        retract_waypoint += restore_target[PARKING_AXIS];
                        retract_waypoint = MIN(retract_waypoint, PARKING_TARGET);
                    }
                    // Execute slow pull-out parking retract motion. Parking requires homing enabled, the
                    // current location not exceeding the parking target location, and laser mode disabled.
                    // NOTE: State is will remain DOOR, until the de-energizing and retract is complete.
                    if (can_park() && parking_target[PARKING_AXIS] < PARKING_TARGET) {
                        // Retract spindle by pullout distance. Ensure retraction motion moves away from
                        // the workpiece and waypoint motion doesn't exceed the parking target location.
                        if (parking_target[PARKING_AXIS] < retract_waypoint) {
                            parking_target[PARKING_AXIS] = retract_waypoint;
                            pl_data->feed_rate           = PARKING_PULLOUT_RATE;
                            pl_data->coolant             = restore_coolant;
                            pl_data->spindle             = restore_spindle;
                            pl_data->spindle_speed       = restore_spindle_speed;
                            mc_parking_motion(parking_target, pl_data);
                        }
                        // NOTE: Clear accessory state after retract and after an aborted restore motion.
                        pl_data->spindle               = SpindleState::Disable;
                        pl_data->coolant               = {};
                        pl_data->motion                = {};
                        pl_data->motion.systemMotion   = 1;
                        pl_data->motion.noFeedOverride = 1;
                        pl_data->spindle_speed         = 0.0;
                        spindle->set_state(pl_data->spindle, 0);  // De-energize
                        coolant_set_state(pl_data->coolant);
                        // Execute fast parking retract motion to parking target location.
                        if (parking_target[PARKING_AXIS] < PARKING_TARGET) {
                            parking_target[PARKING_AXIS] = PARKING_TARGET;
                            pl_data->feed_rate           = PARKING_RATE;
                            mc_parking_motion(parking_target, pl_data);
                        }
                    } else {
                        // Parking motion not possible. Just disable the spindle and coolant.
                        // NOTE: Laser mode does not start a parking motion to ensure the laser stops immediately.
                        spindle->set_state(SpindleState::Disable, 0);  // De-energize
                        coolant_off();
                    }
#endif
                    sys.suspend.bit.restartRetract  = false;
                    sys.suspend.bit.retractComplete = true;
                } else {
                    if (sys.state == State::Sleep) {
                        report_feedback_message(Message::SleepMode);
                        // Spindle and coolant should already be stopped, but do it again just to be sure.
                        spindle->set_state(SpindleState::Disable, 0);  // De-energize
                        coolant_off();
                        st_go_idle();  // Disable steppers
                        while (!(sys.abort)) {
                            protocol_exec_rt_system();  // Do nothing until reset.
                        }
                        return;  // Abort received. Return to re-initialize.
                    }
                    // Allows resuming from parking/safety door. Actively checks if safety door is closed and ready to resume.
                    if (sys.state == State::SafetyDoor) {
                        if (!(system_check_safety_door_ajar())) {
                            sys.suspend.bit.safetyDoorAjar = false;  // Reset door ajar flag to denote ready to resume.
                        }
                    }
                    // Handles parking restore and safety door resume.
                    if (sys.suspend.bit.initiateRestore) {
#ifdef PARKING_ENABLE
                        // Execute fast restore motion to the pull-out position. Parking requires homing enabled.
                        // NOTE: State is will remain DOOR, until the de-energizing and retract is complete.
                        if (can_park()) {
                            // Check to ensure the motion doesn't move below pull-out position.
                            if (parking_target[PARKING_AXIS] <= PARKING_TARGET) {
                                parking_target[PARKING_AXIS] = retract_waypoint;
                                pl_data->feed_rate           = PARKING_RATE;
                                mc_parking_motion(parking_target, pl_data);
                            }
                        }
#endif
                        // Delayed Tasks: Restart spindle and coolant, delay to power-up, then resume cycle.
                        if (gc_state.modal.spindle != SpindleState::Disable) {
                            // Block if safety door re-opened during prior restore actions.
                            if (!sys.suspend.bit.restartRetract) {
                                if (spindle->inLaserMode()) {
                                    // When in laser mode, ignore spindle spin-up delay. Set to turn on laser when cycle starts.
                                    sys.step_control.updateSpindleRpm = true;
                                } else {
                                    spindle->set_state(restore_spindle, (uint32_t)restore_spindle_speed);
                                    // restore delay is done in the spindle class
                                    //delay_sec(int32_t(1000.0 * spindle_delay_spinup->get()), DwellMode::SysSuspend);
                                }
                            }
                        }
                        if (gc_state.modal.coolant.Flood || gc_state.modal.coolant.Mist) {
                            // Block if safety door re-opened during prior restore actions.
                            if (!sys.suspend.bit.restartRetract) {
                                // NOTE: Laser mode will honor this delay. An exhaust system is often controlled by this pin.
                                coolant_set_state(restore_coolant);
                                delay_msec(int32_t(1000.0 * coolant_start_delay->get()), DwellMode::SysSuspend);
                            }
                        }
#ifdef PARKING_ENABLE
                        // Execute slow plunge motion from pull-out position to resume position.
                        if (can_park()) {
                            // Block if safety door re-opened during prior restore actions.
                            if (!sys.suspend.bit.restartRetract) {
                                // Regardless if the retract parking motion was a valid/safe motion or not, the
                                // restore parking motion should logically be valid, either by returning to the
                                // original position through valid machine space or by not moving at all.
                                pl_data->feed_rate     = PARKING_PULLOUT_RATE;
                                pl_data->spindle       = restore_spindle;
                                pl_data->coolant       = restore_coolant;
                                pl_data->spindle_speed = restore_spindle_speed;
                                mc_parking_motion(restore_target, pl_data);
                            }
                        }
#endif
                        if (!sys.suspend.bit.restartRetract) {
                            sys.suspend.bit.restoreComplete  = true;
                            sys_rt_exec_state.bit.cycleStart = true;  // Set to resume program.
                        }
                    }
                }
            } else {
                // Feed hold manager. Controls spindle stop override states.
                // NOTE: Hold ensured as completed by condition check at the beginning of suspend routine.
                if (sys.spindle_stop_ovr.value) {
                    // Handles beginning of spindle stop
                    if (sys.spindle_stop_ovr.bit.initiate) {
                        if (gc_state.modal.spindle != SpindleState::Disable) {
                            spindle->set_state(SpindleState::Disable, 0);  // De-energize
                            sys.spindle_stop_ovr.value       = 0;
                            sys.spindle_stop_ovr.bit.enabled = true;  // Set stop override state to enabled, if de-energized.
                        } else {
                            sys.spindle_stop_ovr.value = 0;  // Clear stop override state
                        }
                        // Handles restoring of spindle state
                    } else if (sys.spindle_stop_ovr.bit.restore || sys.spindle_stop_ovr.bit.restoreCycle) {
                        if (gc_state.modal.spindle != SpindleState::Disable) {
                            report_feedback_message(Message::SpindleRestore);
                            if (spindle->inLaserMode()) {
                                // When in laser mode, ignore spindle spin-up delay. Set to turn on laser when cycle starts.
                                sys.step_control.updateSpindleRpm = true;
                            } else {
                                spindle->set_state(restore_spindle, (uint32_t)restore_spindle_speed);
                            }
                        }
                        if (sys.spindle_stop_ovr.bit.restoreCycle) {
                            sys_rt_exec_state.bit.cycleStart = true;  // Set to resume program.
                        }
                        sys.spindle_stop_ovr.value = 0;  // Clear stop override state
                    }
                } else {
                    // Handles spindle state during hold. NOTE: Spindle speed overrides may be altered during hold state.
                    // NOTE: sys.step_control.updateSpindleRpm is automatically reset upon resume in step generator.
                    if (sys.step_control.updateSpindleRpm) {
                        spindle->set_state(restore_spindle, (uint32_t)restore_spindle_speed);
                        sys.step_control.updateSpindleRpm = false;
                    }
                }
            }
        }
        protocol_exec_rt_system();
    }
}
