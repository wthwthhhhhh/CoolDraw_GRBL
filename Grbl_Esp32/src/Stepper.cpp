/*
  Stepper.cpp - stepper motor driver: executes motion plans using stepper motors
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

#include <atomic>

// Stores the planner block Bresenham algorithm execution data for the segments in the segment
// buffer. Normally, this buffer is partially in-use, but, for the worst case scenario, it will
// never exceed the number of accessible stepper buffer segments (SEGMENT_BUFFER_SIZE-1).
// NOTE: This data is copied from the prepped planner blocks so that the planner blocks may be
// discarded when entirely consumed and completed by the segment buffer. Also, AMASS alters this
// data for its own use.
typedef struct {
    uint32_t steps[MAX_N_AXIS];
    uint32_t step_event_count;
    uint8_t  direction_bits;
    uint8_t  is_pwm_rate_adjusted;//跟踪需要恒定激光功率/速率的运动
} st_block_t;
static st_block_t st_block_buffer[SEGMENT_BUFFER_SIZE - 1];

//主步进段环形缓冲器。包含步进器的小而短的线段
//的第一个块增量地“签出”
//规划师缓冲区。一旦“签出”，段缓冲区中的步骤不能被修改
//计划器，其中剩余的计划器块步骤仍然可以。
typedef struct {
    uint16_t n_step;//该段执行的步骤事件数
    uint16_t isrPeriod;//到下一次ISR滴答的时间，以计时器滴答为单位
    uint8_t  st_block_index;//步进块数据索引。使用此信息执行此段。
    uint8_t  amass_level;// ISR执行此段的AMASS级别
    uint16_t spindle_rpm;     // TODO 把这个丢掉。
} segment_t;
static segment_t segment_buffer[SEGMENT_BUFFER_SIZE];

//步进ISR数据结构。包含主步进ISR的运行数据。
typedef struct {
// bresenham线算法使用

    uint32_t counter[MAX_N_AXIS];// bresenham线示踪器的计数器变量

    uint8_t  step_bits;//存储out_bits输出以完成步骤脉冲延迟
    uint8_t  execute_step;//标记每个中断的步骤执行。
    uint8_t  step_pulse_time;//升阶后的脉冲复位时间
    uint8_t  step_outbits;//下一个要输出的步进位
    uint8_t  dir_outbits;
    uint32_t steps[MAX_N_AXIS];

    uint16_t    step_count;//保持线段运动的步骤
    uint8_t     exec_block_index;//跟踪当前st_block索引。Change表示新块。
    st_block_t* exec_block;//指向正在执行的段的块数据
    segment_t*  exec_segment;//正在执行的段的指针
} stepper_t;
static stepper_t st;

//步骤段环缓冲区索引
static volatile uint8_t segment_buffer_tail;
static uint8_t          segment_buffer_head;
static uint8_t          segment_next_head;

//用于避免ISR嵌套的“步进驱动中断”。但是不应该发生。
static std::atomic<bool> busy;

//从计划缓冲区准备的步骤段的指针。只能由
//主程序。指针可以在执行之前规划段或规划块。
static plan_block_t* pl_block;//指向正在准备的计划块的指针
static st_block_t*   st_prep_block;//正在准备的步进块数据的指针
//Esp32工作周围禁用在主循环
uint64_t stepper_idle_counter;//用于倒计时，直到禁用步进驱动器
bool     stepper_idle;

//段准备数据结构。包含计算新段所需的所有信息
//基于当前正在执行的计划程序块。
typedef struct {
    uint8_t  st_block_index;//正在准备的步进公共数据块索引
    PrepFlag recalculate_flag;

    float dt_remainder;
    float steps_remaining;
    float step_per_mm;
    float req_mm_increment;

#ifdef PARKING_ENABLE
    uint8_t last_st_block_index;
    float   last_steps_remaining;
    float   last_step_per_mm;
    float   last_dt_remainder;
#endif

    uint8_t ramp_type;//当前段匝道状态
    float   mm_complete;//速度剖面距离当前规划块末端的末端(mm)。
//注意:该值在转换时必须与step(没有尾数)一致。
    float current_speed;//分段缓冲区结束时的当前速度(mm/min)
    float maximum_speed;//执行块的最大速度。并不总是名义速度。(毫米/分钟)
    float exit_speed;//执行块退出速度(mm/min)
    float accelerate_until;//从块端开始测量加速度斜坡端(mm)
    float decelerate_after;//减速坡道从块端开始测量(mm)

    float inv_rate;//用于PWM激光模式，加速段计算。
    uint16_t current_spindle_pwm;//删除
    float current_spindle_rpm;

} st_prep_t;
static st_prep_t prep;

const char* stepper_names[] = {
    "Timed Steps",
    "RMT Steps",
    "I2S Steps, Stream",
    "I2S Steps, Static",
};

stepper_id_t current_stepper = DEFAULT_STEPPER;

/* “步进驱动中断”-这个定时器中断是Grbl的主力。Grbl雇佣了
古老的布列森汉线算法来管理和精确同步多轴移动。
与流行的DDA算法不同，Bresenham算法不受数值影响
舍入错误，只需要快速的整数计数器，这意味着低计算开销
最大化Arduino的功能。然而，布雷森纳姆算法的缺点
对于某些多轴运动，非主导轴是否会出现不光滑步进
脉冲序列或混叠，会导致奇怪的可听见的噪音或震动。这是
在低步进频率(0-5kHz)时特别明显或可能导致运动问题，但是
在更高的频率下通常不是物理问题，尽管可以听到。
     为了提高Bresenham多轴性能，Grbl使用了我们所说的自适应多轴
步骤平滑(AMASS)算法，顾名思义。在较低的阶跃频率下，
AMASS在不影响算法的情况下，人为地提高了布列森汉姆分辨率
天生的精确。AMASS根据步骤自动调整其分辨率级别
执行的频率，这意味着对于更低的步进频率，步进平滑
水平增加。在算法上，AMASS是通过简单的Bresenham移位来实现的
每个AMASS级别的步数。例如，对于1级步骤平滑，我们使用位移位
Bresenham步骤事件计数，有效地将其乘以2，而轴步骤计数
保持不变，然后将步进ISR频率加倍。实际上，我们允许
非优势布列南轴在中间ISR tick中步进，而优势轴在中间ISR tick中步进
每两个ISR滴答，而不是传统意义上的每一个ISR滴答。在积累
第2级，我们简单地再次位移，所以非主导布列森纳姆轴可以步进任何
在四个ISR节拍中，主轴每四个ISR节拍一步，并且四倍
步进ISR频率。等等......这实际上消除了多轴混叠
但这并不会显著改变Grbl的性能
事实上，在所有配置中更有效地利用未使用的CPU周期。
     AMASS retains the Bresenham algorithm exactness by requiring that it always executes a full
   Bresenham step, regardless of AMASS Level. Meaning that for an AMASS Level 2, all four
   intermediate steps must be completed such that baseline Bresenham (Level 0) count is always
   retained. Similarly, AMASS Level 3 means all eight intermediate steps must be executed.
   Although the AMASS Levels are in reality arbitrary, where the baseline Bresenham counts can
   be multiplied by any integer value, multiplication by powers of two are simply used to ease
   CPU overhead with bitshift integer operations.
     This interrupt is simple and dumb by design. All the computational heavy-lifting, as in
   determining accelerations, is performed elsewhere. This interrupt pops pre-computed segments,
   defined as constant velocity over n number of steps, from the step segment buffer and then
   executes them by pulsing the stepper pins appropriately via the Bresenham algorithm. This
   ISR is supported by The Stepper Port Reset Interrupt which it uses to reset the stepper port
   after each pulse. The bresenham line tracer algorithm controls all stepper outputs
   simultaneously with these two interrupts.

	 注意:这个中断必须尽可能高效，并且在下一个ISR tick之前完成，
对于ESP32 Grbl必须小于xx。xusec (TBD)。示波器测量时间
ISR是典型的5usec和最大的25usec，远远低于要求。
注意:这个ISR期望每个段至少执行一个步骤。
	 完整的步骤计时应该是这样的…
方向针设置
一个可选的延迟(direction_delay_microseconds)放在后面
步骤引脚已启动
脉冲长度确定(通过选项$0…pulse_microseconds)
脉冲结束了
方向将保持不变，直到发生另一个步骤，改变方向。

*/

 void stepper_clear_buff(){
 memset(&st_block_buffer, 0, sizeof(st_block_buffer));
 memset(&segment_buffer, 0, sizeof(segment_buffer));

//步骤段环缓冲区索引
 //segment_buffer_tail=0;
 //segment_buffer_head=0;
 segment_next_head=1;

 //pl_block=NULL;//指向正在准备的计划块的指针
//st_prep_block=NULL;//正在准备的步进块数据的指针
}
static void stepper_pulse_func();

//待办事项:以某种方式替换ISR中int32位置计数器的直接更新。也许用更小的
//Int8变量和仅在段结束时更新位置计数器。这很复杂
//探测和寻的周期需要真正的实时位置。
void IRAM_ATTR onStepperDriverTimer(void* para) {
    // Timer ISR, normally takes a step.
    //
    // When handling an interrupt within an interrupt serivce routine (ISR), the interrupt status bit
    // needs to be explicitly cleared.
    TIMERG0.int_clr_timers.t0 = 1;

    bool expected = false;
    if (busy.compare_exchange_strong(expected, true)) {
        stepper_pulse_func();

        TIMERG0.hw_timer[STEP_TIMER_INDEX].config.alarm_en = TIMER_ALARM_EN;

        busy.store(false);
    }
}

/**
*此阶段的ISR应仅为步进器产生脉冲。
*这可以防止抖动引起的间隔开始
*中断和脉冲的开始。不要在前面添加任何逻辑
*调用此方法，可能会导致时间变化。这样做的目的
*是保持脉冲时间尽可能规律。
*/
static void stepper_pulse_func() {
    auto n_axis = number_axis->get();

    if (motors_direction(st.dir_outbits)) {
        auto wait_direction = direction_delay_microseconds->get();
        if (wait_direction > 0) {
            // Stepper drivers need some time between changing direction and doing a pulse.
            switch (current_stepper) {
                case ST_I2S_STREAM:
                    i2s_out_push_sample(wait_direction);
                    break;
                case ST_I2S_STATIC:
                case ST_TIMED: {
                    // wait for step pulse time to complete...some time expired during code above
                    //
                    // If we are using GPIO stepping as opposed to RMT, record the
                    // time that we turned on the direction pins so we can delay a bit.
                    // If we are using RMT, we can't delay here.
                    auto direction_pulse_start_time = esp_timer_get_time() + wait_direction;
                    while ((esp_timer_get_time() - direction_pulse_start_time) < 0) {
                        NOP();  // spin here until time to turn off step
                    }
                    break;
                }
                case ST_RMT:
                    break;
            }
        }
    }

 //如果我们使用GPIO步进而不是RMT，记录
//当我们打开踏板时，我们可以把它们关掉
//在这个例程结束时，不会引起另一个中断。
//这对于RMT和I2S的步进是不必要的，因为两者都是
//这些方法自动计时关闭。
    //
    // NOTE: 我们可以使用direction_pulse_start_time + wait_direction, 但我们还是谨慎行事吧
    uint64_t step_pulse_start_time = esp_timer_get_time();
    motors_step(st.step_outbits);

//如果没有step segment，尝试从step buffer中弹出一个
    if (st.exec_segment == NULL) {
//缓冲区中有什么?如果是，加载并初始化下一步段。
        if (segment_buffer_head != segment_buffer_tail) {
//初始化新的步骤段并加载要执行的步骤数
            st.exec_segment = &segment_buffer[segment_buffer_tail];
//初始化每一步的步段计时，并加载要执行的步数。
            Stepper_Timer_WritePeriod(st.exec_segment->isrPeriod);
            st.step_count = st.exec_segment->n_step;//注意:当移动缓慢时，有时可以为零。
//如果新的段开始一个新的规划块，初始化步进变量和计数器。
//注意:当分段数据索引改变时，这表示一个新的规划块。
            if (st.exec_block_index != st.exec_segment->st_block_index) {
                st.exec_block_index = st.exec_segment->st_block_index;
                st.exec_block       = &st_block_buffer[st.exec_block_index];
//初始化Bresenham线和距离计数器
                for (int axis = 0; axis < n_axis; axis++) {
                    st.counter[axis] = (st.exec_block->step_event_count >> 1);
                }
            }
            st.dir_outbits = st.exec_block->direction_bits;
//根据AMASS等级调整Bresenham轴增量计数器。
            for (int axis = 0; axis < n_axis; axis++) {
                st.steps[axis] = st.exec_block->steps[axis] >> st.exec_segment->amass_level;
            }
//在第一步之前，在加载分段时设置实时主轴输出。
            spindle->set_rpm(st.exec_segment->spindle_rpm);
        } else {
//段缓冲区为空。关闭。
            st_go_idle();
            if (sys.state != State::Jog) {// add to prevent…探索碰撞后慢跑
//在完成速率控制运动时，确保pwm设置正确。
                if (st.exec_block != NULL && st.exec_block->is_pwm_rate_adjusted) {
                    spindle->set_rpm(0);
                }
            }
            cycle_stop = true;
            return;//除了退出什么都不做。
        }
    }
//检查探测状态。
    if (sys_probe_state == Probe::Active) {
        probe_state_monitor();
    }
//重置步进位。
    st.step_outbits = 0;

    for (int axis = 0; axis < n_axis; axis++) {
//采用Bresenham线算法执行步进位移剖面
        st.counter[axis] += st.steps[axis];
        if (st.counter[axis] > st.exec_block->step_event_count) {
            st.step_outbits |= bit(axis);
            st.counter[axis] -= st.exec_block->step_event_count;
            if (st.exec_block->direction_bits & bit(axis)) {
                sys_position[axis]--;
            } else {
                sys_position[axis]++;
            }
        }
    }

//在归巢周期中，锁定并阻止所需的轴移动。
    if (sys.state == State::Homing) {
        st.step_outbits &= sys.homing_axis_lock;
    }
    st.step_count--;//减少步骤事件计数
    if (st.step_count == 0) {
//段已完成。丢弃当前段并推进段索引。
        st.exec_segment = NULL;
        if (++segment_buffer_tail == SEGMENT_BUFFER_SIZE) {
            segment_buffer_tail = 0;
        }
    }

    switch (current_stepper) {
        case ST_I2S_STREAM:
//生成跨越pulse_microseconds所需的脉冲数
            i2s_out_push_sample(pulse_microseconds->get());
            motors_unstep();
            break;
        case ST_I2S_STATIC:
        case ST_TIMED:
//等待步骤脉冲时间完成…在上面的代码中，一些时间过期了
            while (esp_timer_get_time() - step_pulse_start_time < pulse_microseconds->get()) {
                NOP();//在这里旋转直到关闭step
            }
            motors_unstep();
            break;
        case ST_RMT:
            break;
    }
}

void stepper_init() {
    busy.store(false); 
    
    grbl_msg_sendf(CLIENT_SERIAL, MsgLevel::Info, "Axis count %d", number_axis->get());
    grbl_msg_sendf(CLIENT_SERIAL, MsgLevel::Info, "%s", stepper_names[current_stepper]);

#ifdef USE_I2S_STEPS
// I2S步进流模式使用回调但定时器中断
    i2s_out_set_pulse_callback(stepper_pulse_func);
#endif
//其他步进使用定时器中断
    Stepper_Timer_Init();
}

void stepper_switch(stepper_id_t new_stepper) {
    grbl_msg_sendf(CLIENT_SERIAL, MsgLevel::Debug, "Switch stepper: %s -> %s", stepper_names[current_stepper], stepper_names[new_stepper]);
    if (current_stepper == new_stepper) {
        // do not need to change
        return;
    }
#ifdef USE_I2S_STEPS
    if (current_stepper == ST_I2S_STREAM) {
        if (i2s_out_get_pulser_status() != PASSTHROUGH) {
            // Called during streaming. Stop streaming.
            grbl_msg_sendf(CLIENT_SERIAL, MsgLevel::Debug, "Stop the I2S streaming and switch to the passthrough mode.");
            i2s_out_set_passthrough();
            i2s_out_delay();  // Wait for a change in mode.
        }
    }
#endif
    current_stepper = new_stepper;
}

// enabled. Startup init and limits call this function but shouldn't start the cycle.
void st_wake_up() {
    //grbl_msg_sendf(CLIENT_SERIAL, MsgLevel::Info, "st_wake_up");
    // Enable stepper drivers.
    motors_set_disable(false);
    stepper_idle = false;

    // Initialize step pulse timing from settings. Here to ensure updating after re-writing.
#ifdef USE_RMT_STEPS
    // Step pulse delay handling is not require with ESP32...the RMT function does it.
    if (direction_delay_microseconds->get() < 1)
    {
        // Set step pulse time. Ad hoc computation from oscilloscope. Uses two's complement.
        st.step_pulse_time = -(((pulse_microseconds->get() - 2) * ticksPerMicrosecond) >> 3);
    }
#else  // Normal operation
    // Set step pulse time. Ad hoc computation from oscilloscope. Uses two's complement.
    st.step_pulse_time = -(((pulse_microseconds->get() - 2) * ticksPerMicrosecond) >> 3);
#endif

    // Enable Stepper Driver Interrupt
    Stepper_Timer_Start();
}

//重置和清除步进子系统变量
void st_reset() {
#ifdef ESP_DEBUG
    //Serial.println("st_reset()");
#endif
    // Initialize stepper driver idle state.
#ifdef USE_I2S_STEPS
    if (current_stepper == ST_I2S_STREAM) {
        i2s_out_reset();
    }
#endif
    st_go_idle();
    // Initialize stepper algorithm variables.
    memset(&prep, 0, sizeof(st_prep_t));
    memset(&st, 0, sizeof(stepper_t));
    st.exec_segment     = NULL;
    pl_block            = NULL;  // Planner block pointer used by segment buffer
    segment_buffer_tail = 0;
    segment_buffer_head = 0;  // empty = tail
    segment_next_head   = 1;
    st.step_outbits     = 0;
    st.dir_outbits      = 0;  // Initialize direction bits to default.
    // TODO do we need to turn step pins off?
}

//步进关闭
void st_go_idle() {
//禁用步进驱动中断。如果激活，允许步进端口复位中断完成。
    Stepper_Timer_Stop();

//根据设置和环境，设置步进驱动空闲状态，禁用或启用。
    if (((stepper_idle_lock_time->get() != 0xff) || sys_rt_exec_alarm != ExecAlarm::None || sys.state == State::Sleep) &&
        sys.state != State::Homing) {
//强制步进器在规定时间内锁定轴，以确保轴达到完整
//停止和不漂移的残余惯性力在最后一个运动结束。

        if (sys.state == State::Sleep || sys_rt_exec_alarm != ExecAlarm::None) {
            motors_set_disable(true);
        } else {
            stepper_idle         = true;  // esp32 work around for disable in main loop
            stepper_idle_counter = esp_timer_get_time() + (stepper_idle_lock_time->get() * 1000);  // * 1000 because the time is in uSecs
            // after idle countdown will be disabled in protocol loop
        }
    } else {
        motors_set_disable(false);
    }

    motors_unstep();
    st.step_outbits = 0;
}

//当执行块被新计划更新时，由planner_recalculate()调用。
void st_update_plan_block_parameters() {
    if (pl_block != NULL) {  // Ignore if at start of a new block.
        prep.recalculate_flag.recalculate = 1;
        pl_block->entry_speed_sqr         = prep.current_speed * prep.current_speed;  // Update entry speed.
        pl_block                          = NULL;  // Flag st_prep_segment() to load and check active velocity profile.
    }
}

#ifdef PARKING_ENABLE
//更改步骤段缓冲区的运行状态以执行特殊的停车运动。
void st_parking_setup_buffer() {
    // Store step execution data of partially completed block, if necessary.
    if (prep.recalculate_flag.holdPartialBlock) {
        prep.last_st_block_index  = prep.st_block_index;
        prep.last_steps_remaining = prep.steps_remaining;
        prep.last_dt_remainder    = prep.dt_remainder;
        prep.last_step_per_mm     = prep.step_per_mm;
    }
    // Set flags to execute a parking motion
    prep.recalculate_flag.parking     = 1;
    prep.recalculate_flag.recalculate = 0;
    pl_block                          = NULL;  // Always reset parking motion to reload new block.
}

//在停车运动后将步进段缓冲区恢复到正常运行状态。
void st_parking_restore_buffer() {
    // Restore step execution data and flags of partially completed block, if necessary.
    if (prep.recalculate_flag.holdPartialBlock) {
        st_prep_block                          = &st_block_buffer[prep.last_st_block_index];
        prep.st_block_index                    = prep.last_st_block_index;
        prep.steps_remaining                   = prep.last_steps_remaining;
        prep.dt_remainder                      = prep.last_dt_remainder;
        prep.step_per_mm                       = prep.last_step_per_mm;
        prep.recalculate_flag.holdPartialBlock = 1;
        prep.recalculate_flag.recalculate      = 1;
        prep.req_mm_increment                  = REQ_MM_INCREMENT_SCALAR / prep.step_per_mm;  // Recompute this value.
    } else {
        prep.recalculate_flag = {};
    }

    pl_block = NULL;  // Set to reload next block.
}
#endif

//增加步骤段缓冲区块数据环缓冲区。
static uint8_t st_next_block_index(uint8_t block_index) {
    block_index++;
    return block_index == (SEGMENT_BUFFER_SIZE - 1) ? 0 : block_index;
}

/*准备步骤段缓冲区。从主程序连续调用。
段缓冲区是步骤执行之间的中间缓冲区接口
通过步进算法和规划器生成的速度剖面。步进的
算法只执行段缓冲区内的步骤，由主程序填充
当步骤从计划缓冲区的第一个块中“签出”时。这样可以保持
步骤执行和计划优化过程是原子的，彼此相互保护。
从规划器缓冲区“签出”的步数和输入的段数
段缓冲区的大小和计算使主程序不需要任何操作
比步进算法在重新填充之前清空它所花费的时间长。
目前，段缓冲区保守地保存大约40-50 msec的步骤。
注意:计算单位为步、毫米和分钟。
*/
void st_prep_buffer() {
//当处于挂起状态且没有要执行的挂起运动时，阻塞步骤准备缓冲区。
    if (sys.step_control.endMotion) {
        return;
    }

    while (segment_buffer_tail != segment_next_head) {//检查是否需要填充缓冲区。
//确定我们是否需要加载一个新的规划块或块是否需要重新计算。
        if (pl_block == NULL) {
//排队块的查询计划器
            if (sys.step_control.executeSysMotion) {
                pl_block = plan_get_system_motion_block();
            } else {
                pl_block = plan_get_current_block();
            }

            if (pl_block == NULL) {
                return;//没有计划块。退出。
            }

//检查我们是否只需要重新计算速度剖面或加载一个新的块。
            if (prep.recalculate_flag.recalculate) {
#ifdef PARKING_ENABLE
                if (prep.recalculate_flag.parking) {
                    prep.recalculate_flag.recalculate = 0;
                } else {
                    prep.recalculate_flag = {};
                }
#else
                prep.recalculate_flag = {};
#endif
            } else {
//加载block的Bresenham步进数据。
                prep.st_block_index = st_next_block_index(prep.st_block_index);
//从新的规划块中准备并复制Bresenham算法段数据
//当段缓冲区完成规划块时，它可能会被丢弃
//段缓冲区完成了准备好的块，但是步进ISR仍然在执行它。
                st_prep_block                 = &st_block_buffer[prep.st_block_index];
                st_prep_block->direction_bits = pl_block->direction_bits;
                uint8_t idx;
                auto    n_axis = number_axis->get();

//将所有Bresenham数据乘以最大AMASS级，使
//我们不会在算法中除超出原始数据的任何地方。
//如果对原始数据进行除法，则从整数舍入开始将丢失一步。
                for (idx = 0; idx < n_axis; idx++) {
                    st_prep_block->steps[idx] = pl_block->steps[idx] << maxAmassLevel;
                }
                st_prep_block->step_event_count = pl_block->step_event_count << maxAmassLevel;

//初始化段缓冲区数据用于生成段
                prep.steps_remaining  = (float)pl_block->step_event_count;
                prep.step_per_mm      = prep.steps_remaining / pl_block->millimeters;
                prep.req_mm_increment = REQ_MM_INCREMENT_SCALAR / prep.step_per_mm;
                prep.dt_remainder     = 0.0;//重置新段块
                if ((sys.step_control.executeHold) || prep.recalculate_flag.decelOverride) {
//新块加载中hold。覆盖计划块进入速度强制减速。
                    prep.current_speed                  = prep.exit_speed;
                    pl_block->entry_speed_sqr           = prep.exit_speed * prep.exit_speed;
                    prep.recalculate_flag.decelOverride = 0;
                } else {
                    prep.current_speed = sqrt(pl_block->entry_speed_sqr);
                }

                st_prep_block->is_pwm_rate_adjusted = false;//设置默认值
// prep.inv_rate仅在is_pwm_rate_adjusted为真时使用
                if (spindle->inLaserMode()) {  //
                    if (pl_block->spindle == SpindleState::Ccw) {
//预先计算逆编程速率，加快每步段的PWM更新速度。
                        prep.inv_rate                       = 1.0 / pl_block->programmed_rate;
                        st_prep_block->is_pwm_rate_adjusted = true;
                    }
                }
            }
/* ---------------------------------------------------------------------------------
根据一个新的计划块的进入和退出计算它的速度剖面
加速，或重新计算部分完成的规划程序块的配置文件
规划师已经更新了。用于指令强制减速，如从进给
等待，超过计划速度，减速到目标出口速度。
*/
            prep.mm_complete  = 0.0;//默认速度轮廓在距离块端0毫米处完成。
            float inv_2_accel = 0.5 / pl_block->acceleration;
            if (sys.step_control.executeHold) {//[强制减速到零速度]
//计算进料保持过程中的速度剖面参数。此概要文件将覆盖
//规划块配置文件，强制减速到零速度。
                prep.ramp_type = RAMP_DECEL;
//计算相对于块末端的减速距离。
                float decel_dist = pl_block->millimeters - inv_2_accel * pl_block->entry_speed_sqr;
                if (decel_dist < 0.0) {
//减速通过整个计划块。进料保持的结束不在此块中。
                    prep.exit_speed = sqrt(pl_block->entry_speed_sqr - 2 * pl_block->acceleration * pl_block->millimeters);
                } else {
                    prep.mm_complete = decel_dist;// feed hold结束。
                    prep.exit_speed  = 0.0;
                }
            } else {//[正常运行]
//计算或重新计算预制规划块的速度剖面参数。
                prep.ramp_type        = RAMP_ACCEL;//初始化为加速斜坡
                prep.accelerate_until = pl_block->millimeters;
                float exit_speed_sqr;
                float nominal_speed;
                if (sys.step_control.executeSysMotion) {
                    prep.exit_speed = exit_speed_sqr = 0.0;//在系统运动结束时强制停止。
                } else {
                    exit_speed_sqr  = plan_get_exec_block_exit_speed_sqr();
                    prep.exit_speed = sqrt(exit_speed_sqr);
                }

                nominal_speed            = plan_compute_profile_nominal_speed(pl_block);
                float nominal_speed_sqr  = nominal_speed * nominal_speed;
                float intersect_distance = 0.5 * (pl_block->millimeters + inv_2_accel * (pl_block->entry_speed_sqr - exit_speed_sqr));
                if (pl_block->entry_speed_sqr > nominal_speed_sqr) {//只在重写还原时发生。
                    prep.accelerate_until = pl_block->millimeters - inv_2_accel * (pl_block->entry_speed_sqr - nominal_speed_sqr);
                    if (prep.accelerate_until <= 0.0) {  // 减速-only.
                        prep.ramp_type = RAMP_DECEL;
                        // prep.decelerate_after = pl_block->millimeters;
                        // prep.maximum_speed = prep.current_speed;
                        // 计算覆盖块退出速度，因为它不匹配计划退出速度d.
                        prep.exit_speed = sqrt(pl_block->entry_speed_sqr - 2 * pl_block->acceleration * pl_block->millimeters);
                        prep.recalculate_flag.decelOverride = 1;//标记加载下一个块作为减速覆盖。
// TODO:在减速中确定参数的正确处理。
//可能很棘手，因为入口速度将是当前速度，就像在进料点一样。
//同时，查看接近零速度的处理问题。
                    } else {
//减速到巡航或巡航-减速类型。保证与更新计划相交。
                        prep.decelerate_after = inv_2_accel * (nominal_speed_sqr - exit_speed_sqr);
                        prep.maximum_speed    = nominal_speed;
                        prep.ramp_type        = RAMP_DECEL_OVERRIDE;
                    }
                } else if (intersect_distance > 0.0) {
                    if (intersect_distance < pl_block->millimeters) {//三角形或梯形
//注意:对于加速巡航和仅巡航类型，下面的计算将是0.0。
                        prep.decelerate_after = inv_2_accel * (nominal_speed_sqr - exit_speed_sqr);
                        if (prep.decelerate_after < intersect_distance) {//梯形
                            prep.maximum_speed = nominal_speed;
                            if (pl_block->entry_speed_sqr == nominal_speed_sqr) {
//巡航减速或仅巡航类型。
                                prep.ramp_type = RAMP_CRUISE;
                            } else {
//全梯形或加速巡航型
                                prep.accelerate_until -= inv_2_accel * (nominal_speed_sqr - pl_block->entry_speed_sqr);
                            }
                        } else {//三角形类型
                            prep.accelerate_until = intersect_distance;
                            prep.decelerate_after = intersect_distance;
                            prep.maximum_speed    = sqrt(2.0 * pl_block->acceleration * intersect_distance + exit_speed_sqr);
                        }
                    } else {//仅减速类型
                        prep.ramp_type = RAMP_DECEL;
                        // prep.decelerate_after = pl_block->millimeters;
                        // prep.maximum_speed = prep.current_speed;
                    }
                } else {//仅加速类型
                    prep.accelerate_until = 0.0;
                    // prep.decelerate_after = 0.0;
                    prep.maximum_speed = prep.exit_speed;
                }
            }

            sys.step_control.updateSpindleRpm = true;//每当更新阻塞时强制更新。
        }

//初始化新段
        segment_t* prep_segment = &segment_buffer[segment_buffer_head];

//设置新段指向当前段数据块。
        prep_segment->st_block_index = prep.st_block_index;

        /*------------------------------------------------------------------------------------
            Compute the average velocity of this new segment by determining the total distance
          traveled over the segment time DT_SEGMENT. The following code first attempts to create
          a full segment based on the current ramp conditions. If the segment time is incomplete
          when terminating at a ramp state change, the code will continue to loop through the
          progressing ramp states to fill the remaining segment execution time. However, if
          an incomplete segment terminates at the end of the velocity profile, the segment is
          considered completed despite having a truncated execution time less than DT_SEGMENT.
            The velocity profile is always assumed to progress through the ramp sequence:
          acceleration ramp, cruising state, and deceleration ramp. Each ramp's travel distance
          may range from zero to the length of the block. Velocity profiles can end either at
          the end of planner block (typical) or mid-block at the end of a forced deceleration,
          such as from a feed hold.
        */
        float dt_max   = DT_SEGMENT;                                // Maximum segment time
        float dt       = 0.0;                                       // Initialize segment time
        float time_var = dt_max;                                    // Time worker variable
        float mm_var;                                               // mm-Distance worker variable
        float speed_var;                                            // Speed worker variable
        float mm_remaining = pl_block->millimeters;                 // New segment distance from end of block.
        float minimum_mm   = mm_remaining - prep.req_mm_increment;  // Guarantee at least one step.

        if (minimum_mm < 0.0) {
            minimum_mm = 0.0;
        }

        do {
            switch (prep.ramp_type) {
                case RAMP_DECEL_OVERRIDE:
                    speed_var = pl_block->acceleration * time_var;
                    mm_var    = time_var * (prep.current_speed - 0.5 * speed_var);
                    mm_remaining -= mm_var;
                    if ((mm_remaining < prep.accelerate_until) || (mm_var <= 0)) {
                        // Cruise or cruise-deceleration types only for deceleration override.
                        mm_remaining       = prep.accelerate_until;  // NOTE: 0.0 at EOB
                        time_var           = 2.0 * (pl_block->millimeters - mm_remaining) / (prep.current_speed + prep.maximum_speed);
                        prep.ramp_type     = RAMP_CRUISE;
                        prep.current_speed = prep.maximum_speed;
                    } else {  // Mid-deceleration override ramp.
                        prep.current_speed -= speed_var;
                    }
                    break;
                case RAMP_ACCEL:
                    // NOTE: Acceleration ramp only computes during first do-while loop.
                    speed_var = pl_block->acceleration * time_var;
                    mm_remaining -= time_var * (prep.current_speed + 0.5 * speed_var);
                    if (mm_remaining < prep.accelerate_until) {  // End of acceleration ramp.
                        // Acceleration-cruise, acceleration-deceleration ramp junction, or end of block.
                        mm_remaining = prep.accelerate_until;  // NOTE: 0.0 at EOB
                        time_var     = 2.0 * (pl_block->millimeters - mm_remaining) / (prep.current_speed + prep.maximum_speed);
                        if (mm_remaining == prep.decelerate_after) {
                            prep.ramp_type = RAMP_DECEL;
                        } else {
                            prep.ramp_type = RAMP_CRUISE;
                        }
                        prep.current_speed = prep.maximum_speed;
                    } else {  // Acceleration only.
                        prep.current_speed += speed_var;
                    }
                    break;
                case RAMP_CRUISE:
                    // NOTE: mm_var used to retain the last mm_remaining for incomplete segment time_var calculations.
                    // NOTE: If maximum_speed*time_var value is too low, round-off can cause mm_var to not change. To
                    //   prevent this, simply enforce a minimum speed threshold in the planner.
                    mm_var = mm_remaining - prep.maximum_speed * time_var;
                    if (mm_var < prep.decelerate_after) {  // End of cruise.
                        // Cruise-deceleration junction or end of block.
                        time_var       = (mm_remaining - prep.decelerate_after) / prep.maximum_speed;
                        mm_remaining   = prep.decelerate_after;  // NOTE: 0.0 at EOB
                        prep.ramp_type = RAMP_DECEL;
                    } else {  // Cruising only.
                        mm_remaining = mm_var;
                    }
                    break;
                default:  // case RAMP_DECEL:
                    // NOTE: mm_var used as a misc worker variable to prevent errors when near zero speed.
                    speed_var = pl_block->acceleration * time_var;  // Used as delta speed (mm/min)
                    if (prep.current_speed > speed_var) {           // Check if at or below zero speed.
                        // Compute distance from end of segment to end of block.
                        mm_var = mm_remaining - time_var * (prep.current_speed - 0.5 * speed_var);  // (mm)
                        if (mm_var > prep.mm_complete) {                                            // Typical case. In deceleration ramp.
                            mm_remaining = mm_var;
                            prep.current_speed -= speed_var;
                            break;  // Segment complete. Exit switch-case statement. Continue do-while loop.
                        }
                    }
                    // Otherwise, at end of block or end of forced-deceleration.
                    time_var           = 2.0 * (mm_remaining - prep.mm_complete) / (prep.current_speed + prep.exit_speed);
                    mm_remaining       = prep.mm_complete;
                    prep.current_speed = prep.exit_speed;
            }

            dt += time_var;  // Add computed ramp time to total segment time.
            if (dt < dt_max) {
                time_var = dt_max - dt;  // **Incomplete** At ramp junction.
            } else {
                if (mm_remaining > minimum_mm) {  // Check for very slow segments with zero steps.
                    // Increase segment time to ensure at least one step in segment. Override and loop
                    // through distance calculations until minimum_mm or mm_complete.
                    dt_max += DT_SEGMENT;
                    time_var = dt_max - dt;
                } else {
                    break;  // **Complete** Exit loop. Segment execution time maxed.
                }
            }
        } while (mm_remaining > prep.mm_complete);  // **Complete** Exit loop. Profile complete.

        /* -----------------------------------------------------------------------------------
          Compute spindle speed PWM output for step segment
        */
        if (st_prep_block->is_pwm_rate_adjusted || sys.step_control.updateSpindleRpm) {
            if (pl_block->spindle != SpindleState::Disable) {
                float rpm = pl_block->spindle_speed;
                // NOTE: Feed and rapid overrides are independent of PWM value and do not alter laser power/rate.
                if (st_prep_block->is_pwm_rate_adjusted) {
                    rpm *= (prep.current_speed * prep.inv_rate);
                    //grbl_msg_sendf(CLIENT_SERIAL, MsgLevel::Info, "RPM %.2f", rpm);
                    //grbl_msg_sendf(CLIENT_SERIAL, MsgLevel::Info, "Rates CV %.2f IV %.2f RPM %.2f", prep.current_speed, prep.inv_rate, rpm);
                }
                // If current_speed is zero, then may need to be rpm_min*(100/MAX_SPINDLE_SPEED_OVERRIDE)
                // but this would be instantaneous only and during a motion. May not matter at all.

                prep.current_spindle_rpm = rpm;
            } else {
                sys.spindle_speed        = 0.0;
                prep.current_spindle_rpm = 0.0;
            }
            sys.step_control.updateSpindleRpm = false;
        }
        prep_segment->spindle_rpm = prep.current_spindle_rpm;  // Reload segment PWM value

        /* -----------------------------------------------------------------------------------
           Compute segment step rate, steps to execute, and apply necessary rate corrections.
           NOTE: Steps are computed by direct scalar conversion of the millimeter distance
           remaining in the block, rather than incrementally tallying the steps executed per
           segment. This helps in removing floating point round-off issues of several additions.
           However, since floats have only 7.2 significant digits, long moves with extremely
           high step counts can exceed the precision of floats, which can lead to lost steps.
           Fortunately, this scenario is highly unlikely and unrealistic in CNC machines
           supported by Grbl (i.e. exceeding 10 meters axis travel at 200 step/mm).
        */
        float step_dist_remaining    = prep.step_per_mm * mm_remaining;             // Convert mm_remaining to steps
        float n_steps_remaining      = ceil(step_dist_remaining);                   // Round-up current steps remaining
        float last_n_steps_remaining = ceil(prep.steps_remaining);                  // Round-up last steps remaining
        prep_segment->n_step         = last_n_steps_remaining - n_steps_remaining;  // Compute number of steps to execute.

        // Bail if we are at the end of a feed hold and don't have a step to execute.
        if (prep_segment->n_step == 0) {
            if (sys.step_control.executeHold) {
                // Less than one step to decelerate to zero speed, but already very close. AMASS
                // requires full steps to execute. So, just bail.
                sys.step_control.endMotion = true;
#ifdef PARKING_ENABLE
                if (!(prep.recalculate_flag.parking)) {
                    prep.recalculate_flag.holdPartialBlock = 1;
                }
#endif
                return;  // Segment not generated, but current step data still retained.
            }
        }

        // Compute segment step rate. Since steps are integers and mm distances traveled are not,
        // the end of every segment can have a partial step of varying magnitudes that are not
        // executed, because the stepper ISR requires whole steps due to the AMASS algorithm. To
        // compensate, we track the time to execute the previous segment's partial step and simply
        // apply it with the partial step distance to the current segment, so that it minutely
        // adjusts the whole segment rate to keep step output exact. These rate adjustments are
        // typically very small and do not adversely effect performance, but ensures that Grbl
        // outputs the exact acceleration and velocity profiles as computed by the planner.

        dt += prep.dt_remainder;  // Apply previous segment partial step execute time
        // dt is in minutes so inv_rate is in minutes
        float inv_rate = dt / (last_n_steps_remaining - step_dist_remaining);  // Compute adjusted step rate inverse

        // Compute CPU cycles per step for the prepped segment.
        // fStepperTimer is in units of timerTicks/sec, so the dimensional analysis is
        // timerTicks/sec * 60 sec/minute * minutes = timerTicks
        uint32_t timerTicks = ceil((fStepperTimer * 60) * inv_rate);  // (timerTicks/step)
        int      level;

        // Compute step timing and multi-axis smoothing level.
        for (level = 0; level < maxAmassLevel; level++) {
            if (timerTicks < amassThreshold) {
                break;
            }
            timerTicks >>= 1;
        }
        prep_segment->amass_level = level;
        prep_segment->n_step <<= level;
        // isrPeriod is stored as 16 bits, so limit timerTicks to the
        // largest value that will fit in a uint16_t.
        prep_segment->isrPeriod = timerTicks > 0xffff ? 0xffff : timerTicks;

        // Segment complete! Increment segment buffer indices, so stepper ISR can immediately execute it.
        segment_buffer_head = segment_next_head;
        if (++segment_next_head == SEGMENT_BUFFER_SIZE) {
            segment_next_head = 0;
        }
        // Update the appropriate planner and segment data.
        pl_block->millimeters = mm_remaining;
        prep.steps_remaining  = n_steps_remaining;
        prep.dt_remainder     = (n_steps_remaining - step_dist_remaining) * inv_rate;
        // Check for exit conditions and flag to load next planner block.
        if (mm_remaining == prep.mm_complete) {
            // End of planner block or forced-termination. No more distance to be executed.
            if (mm_remaining > 0.0) {  // At end of forced-termination.
                // Reset prep parameters for resuming and then bail. Allow the stepper ISR to complete
                // the segment queue, where realtime protocol will set new state upon receiving the
                // cycle stop flag from the ISR. Prep_segment is blocked until then.
                sys.step_control.endMotion = true;
#ifdef PARKING_ENABLE
                if (!(prep.recalculate_flag.parking)) {
                    prep.recalculate_flag.holdPartialBlock = 1;
                }
#endif
                return;  // Bail!
            } else {     // End of planner block
                // The planner block is complete. All steps are set to be executed in the segment buffer.
                if (sys.step_control.executeSysMotion) {
                    sys.step_control.endMotion = true;
                    return;
                }
                pl_block = NULL;  // Set pointer to indicate check and load next planner block.
                plan_discard_current_block();
            }
        }
    }
}

//由实时状态报告调用，以获取正在执行的当前速度。这个值
//然而，并不完全是当前的速度，而是在最后一步段中计算的速度
//在段缓冲区中。它总是会落后最多段块的数量(-1)
//除以加速度滴答每秒(秒)。
float st_get_realtime_rate() {
    switch (sys.state) {
        case State::Cycle:
        case State::Homing:
        case State::Hold:
        case State::Jog:
        case State::SafetyDoor:
            return prep.current_speed;
        default:
            return 0.0f;
    }
}

//参数以生成isr的计时器的tick为单位
void IRAM_ATTR Stepper_Timer_WritePeriod(uint16_t timerTicks) {
    if (current_stepper == ST_I2S_STREAM) {
#ifdef USE_I2S_STEPS
        // 1 tick = fTimers / fStepperTimer
        // Pulse ISR is called for each tick of alarm_val.
        // The argument to i2s_out_set_pulse_period is in units of microseconds
        i2s_out_set_pulse_period(((uint32_t)timerTicks) / ticksPerMicrosecond);
#endif
    } else {
        timer_set_alarm_value(STEP_TIMER_GROUP, STEP_TIMER_INDEX, (uint64_t)timerTicks);
    }
}

void IRAM_ATTR Stepper_Timer_Init() {
    timer_config_t config;
    config.divider     = fTimers / fStepperTimer;
    config.counter_dir = TIMER_COUNT_UP;
    config.counter_en  = TIMER_PAUSE;
    config.alarm_en    = TIMER_ALARM_EN;
    config.intr_type   = TIMER_INTR_LEVEL;
    config.auto_reload = true;
    timer_init(STEP_TIMER_GROUP, STEP_TIMER_INDEX, &config);
    timer_set_counter_value(STEP_TIMER_GROUP, STEP_TIMER_INDEX, 0x00000000ULL);
    timer_enable_intr(STEP_TIMER_GROUP, STEP_TIMER_INDEX);
    timer_isr_register(STEP_TIMER_GROUP, STEP_TIMER_INDEX, onStepperDriverTimer, NULL, 0, NULL);
}

void IRAM_ATTR Stepper_Timer_Start() {
#ifdef ESP_DEBUG
    //Serial.println("ST Start");
#endif
    if (current_stepper == ST_I2S_STREAM) {
#ifdef USE_I2S_STEPS
        i2s_out_set_stepping();
#endif
    } else {
        timer_set_counter_value(STEP_TIMER_GROUP, STEP_TIMER_INDEX, 0x00000000ULL);
        timer_start(STEP_TIMER_GROUP, STEP_TIMER_INDEX);
        TIMERG0.hw_timer[STEP_TIMER_INDEX].config.alarm_en = TIMER_ALARM_EN;
    }
}

void IRAM_ATTR Stepper_Timer_Stop() {
#ifdef ESP_DEBUG
    //Serial.println("ST Stop");
#endif
    if (current_stepper == ST_I2S_STREAM) {
#ifdef USE_I2S_STEPS
        i2s_out_set_passthrough();
#endif
    } else {
        timer_pause(STEP_TIMER_GROUP, STEP_TIMER_INDEX);
    }
}

bool get_stepper_disable() {  // returns true if steppers are disabled
    bool disabled = false;
#ifdef X_STEPPERS_DISABLE_PIN
    disabled = digitalRead(X_STEPPERS_DISABLE_PIN);
#else
    return false;  // thery are never disabled if there is no pin defined
#endif
    if (step_enable_invert->get()) {
        disabled = !disabled;  // Apply pin invert.
    }
    return disabled;
}
