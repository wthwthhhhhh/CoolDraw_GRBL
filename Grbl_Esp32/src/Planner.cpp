/*
  Planner.cpp - buffers movement commands and manages the acceleration profile plan
  Part of Grbl

  Copyright (c) 2011-2016 Sungeun K. Jeon for Gnea Research LLC
  Copyright (c) 2009-2011 Simen Svale Skogsrud
  Copyright (c) 2011 Jens Geisler

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
#include <stdlib.h>  //PSoc实验室要求


static plan_block_t block_buffer[BLOCK_BUFFER_SIZE];//运动指令的环形缓冲区

static uint8_t      block_buffer_tail;//当前要处理的块的索引

static uint8_t      block_buffer_head;//下一个要压入的块的索引

static uint8_t      next_buffer_head;//下一个缓冲区头的索引

static uint8_t      block_buffer_planned;             //最优规划块的索引


//定义计划变量

typedef struct {
    int32_t position[MAX_N_AXIS];//工具在绝对步长的规划位置。分开
//从g代码的位置移动需要多线运动，
//即弧、封闭循环和回隙补偿。

    float previous_unit_vec[MAX_N_AXIS];//上一个路径线段的单位向量

    float previous_nominal_speed;//上一路径线段的标称速度

} planner_t;
static planner_t pl;

//返回循环缓冲区中下一个块的索引。也叫步进段缓冲器。

uint8_t plan_next_block_index(uint8_t block_index) {
    block_index++;
    if (block_index == BLOCK_BUFFER_SIZE) {
        block_index = 0;
    }
    return block_index;
}

//返回循环缓冲区中前一个块的索引
static uint8_t plan_prev_block_index(uint8_t block_index) {
    if (block_index == 0) {
        block_index = BLOCK_BUFFER_SIZE;
    }
    block_index--;
    return block_index;
}

/*                            PLANNER SPEED DEFINITION
                                     +--------+   <- current->nominal_speed
                                    /          \
         current->entry_speed ->   +            \
                                   |             + <- next->entry_speed (aka exit speed)
                                   +-------------+
                                       time -->

  Recalculates the motion plan according to the following basic guidelines:

    1. Go over every feasible block sequentially in reverse order and calculate the junction speeds
        (i.e. current->entry_speed) such that:
      a. No junction speed exceeds the pre-computed maximum junction speed limit or nominal speeds of
         neighboring blocks.
      b. A block entry speed cannot exceed one reverse-computed from its exit speed (next->entry_speed)
         with a maximum allowable deceleration over the block travel distance.
      c. The last (or newest appended) block is planned from a complete stop (an exit speed of zero).
    2. Go over every block in chronological (forward) order and dial down junction speed values if
      a. The exit speed exceeds the one forward-computed from its entry speed with the maximum allowable
         acceleration over the block travel distance.

  When these stages are complete, the planner will have maximized the velocity profiles throughout the all
  of the planner blocks, where every block is operating at its maximum allowable acceleration limits. In
  other words, for all of the blocks in the planner, the plan is optimal and no further speed improvements
  are possible. If a new block is added to the buffer, the plan is recomputed according to the said
  guidelines for a new optimal plan.

  To increase computational efficiency of these guidelines, a set of planner block pointers have been
  created to indicate stop-compute points for when the planner guidelines cannot logically make any further
  changes or improvements to the plan when in normal operation and new blocks are streamed and added to the
  planner buffer. For example, if a subset of sequential blocks in the planner have been planned and are
  bracketed by junction velocities at their maximums (or by the first planner block as well), no new block
  added to the planner buffer will alter the velocity profiles within them. So we no longer have to compute
  them. Or, if a set of sequential blocks from the first block in the planner (or a optimal stop-compute
  point) are all accelerating, they are all optimal and can not be altered by a new block added to the
  planner buffer, as this will only further increase the plan speed to chronological blocks until a maximum
  junction velocity is reached. However, if the operational conditions of the plan changes from infrequently
  used feed holds or feedrate overrides, the stop-compute pointers will be reset and the entire plan is
  recomputed as stated in the general guidelines.

  Planner buffer index mapping:
  - block_buffer_tail: Points to the beginning of the planner buffer. First to be executed or being executed.
  - block_buffer_head: Points to the buffer block after the last block in the buffer. Used to indicate whether
      the buffer is full or empty. As described for standard ring buffers, this block is always empty.
  - next_buffer_head: Points to next planner buffer block after the buffer head block. When equal to the
      buffer tail, this indicates the buffer is full.
  - block_buffer_planned: Points to the first buffer block after the last optimally planned block for normal
      streaming operating conditions. Use for planning optimizations by avoiding recomputing parts of the
      planner buffer that don't change with the addition of a new block, as describe above. In addition,
      this block can never be less than block_buffer_tail and will always be pushed forward and maintain
      this requirement when encountered by the plan_discard_current_block() routine during a cycle.

  NOTE: Since the planner only computes on what's in the planner buffer, some motions with lots of short
  line segments, like G2/3 arcs or complex curves, may seem to move slow. This is because there simply isn't
  enough combined distance traveled in the entire buffer to accelerate up to the nominal speed and then
  decelerate to a complete stop at the end of the buffer, as stated by the guidelines. If this happens and
  becomes an annoyance, there are a few simple solutions: (1) Maximize the machine acceleration. The planner
  will be able to compute higher velocity profiles within the same combined distance. (2) Maximize line
  motion(s) distance per block to a desired tolerance. The more combined distance the planner has to use,
  the faster it can go. (3) Maximize the planner buffer size. This also will increase the combined distance
  for the planner to compute over. It also increases the number of computations the planner has to perform
  to compute an optimal plan, so select carefully. The Arduino 328p memory is already maxed out, but future
  ARM versions should have enough memory and speed for look-ahead blocks numbering up to a hundred or more.

*/
 void planner_recalculate() {
    // Initialize block index to the last block in the planner buffer.
    uint8_t block_index = plan_prev_block_index(block_buffer_head);
    // Bail. Can't do anything with one only one plan-able block.
    if (block_index == block_buffer_planned) {
        return;
    }
    // Reverse Pass: Coarsely maximize all possible deceleration curves back-planning from the last
    // block in buffer. Cease planning when the last optimal planned or tail pointer is reached.
    // NOTE: Forward pass will later refine and correct the reverse pass to create an optimal plan.
    float         entry_speed_sqr;
    plan_block_t* next;
    plan_block_t* current = &block_buffer[block_index];
    // Calculate maximum entry speed for last block in buffer, where the exit speed is always zero.
    current->entry_speed_sqr = MIN(current->max_entry_speed_sqr, 2 * current->acceleration * current->millimeters);
    block_index              = plan_prev_block_index(block_index);
    if (block_index == block_buffer_planned) {  // Only two plannable blocks in buffer. Reverse pass complete.
        // Check if the first block is the tail. If so, notify stepper to update its current parameters.
        if (block_index == block_buffer_tail) {
            st_update_plan_block_parameters();
        }
    } else {  // Three or more plan-able blocks
        while (block_index != block_buffer_planned) {
            next        = current;
            current     = &block_buffer[block_index];
            block_index = plan_prev_block_index(block_index);
            // Check if next block is the tail block(=planned block). If so, update current stepper parameters.
            if (block_index == block_buffer_tail) {
                st_update_plan_block_parameters();
            }
            // Compute maximum entry speed decelerating over the current block from its exit speed.
            if (current->entry_speed_sqr != current->max_entry_speed_sqr) {
                entry_speed_sqr = next->entry_speed_sqr + 2 * current->acceleration * current->millimeters;
                if (entry_speed_sqr < current->max_entry_speed_sqr) {
                    current->entry_speed_sqr = entry_speed_sqr;
                } else {
                    current->entry_speed_sqr = current->max_entry_speed_sqr;
                }
            }
        }
    }
    // Forward Pass: Forward plan the acceleration curve from the planned pointer onward.
    // Also scans for optimal plan breakpoints and appropriately updates the planned pointer.
    next        = &block_buffer[block_buffer_planned];  // Begin at buffer planned pointer
    block_index = plan_next_block_index(block_buffer_planned);
    while (block_index != block_buffer_head) {
        current = next;
        next    = &block_buffer[block_index];
        // Any acceleration detected in the forward pass automatically moves the optimal planned
        // pointer forward, since everything before this is all optimal. In other words, nothing
        // can improve the plan from the buffer tail to the planned pointer by logic.
        if (current->entry_speed_sqr < next->entry_speed_sqr) {
            entry_speed_sqr = current->entry_speed_sqr + 2 * current->acceleration * current->millimeters;
            // If true, current block is full-acceleration and we can move the planned pointer forward.
            if (entry_speed_sqr < next->entry_speed_sqr) {
                next->entry_speed_sqr = entry_speed_sqr;  // Always <= max_entry_speed_sqr. Backward pass sets this.
                block_buffer_planned  = block_index;      // Set optimal plan pointer.
            }
        }
        // Any block set at its maximum entry speed also creates an optimal plan up to this
        // point in the buffer. When the plan is bracketed by either the beginning of the
        // buffer and a maximum entry speed or two maximum entry speeds, every block in between
        // cannot logically be further improved. Hence, we don't have to recompute them anymore.
        if (next->entry_speed_sqr == next->max_entry_speed_sqr) {
            block_buffer_planned = block_index;
        }
        block_index = plan_next_block_index(block_index);
    }
}


void plan_reset_buffer() {
    block_buffer_tail    = 0;
    block_buffer_head    = 0;  // Empty = tail
    next_buffer_head     = 1;  // plan_next_block_index(block_buffer_head)
    block_buffer_planned = 0;  // = block_buffer_tail;
}

void plan_reset() {
    memset(&pl, 0, sizeof(planner_t));  // Clear planner struct
    plan_reset_buffer();
}
void plan_clear() {
    memset(&block_buffer, 0, sizeof(block_buffer));  // Clear planner struct
    plan_reset();
}
void plan_discard_current_block() {
    if (block_buffer_head != block_buffer_tail) {  // Discard non-empty buffer.
        uint8_t block_index = plan_next_block_index(block_buffer_tail);
        // Push block_buffer_planned pointer, if encountered.
        if (block_buffer_tail == block_buffer_planned) {
            block_buffer_planned = block_index;
        }
        block_buffer_tail = block_index;
    }
}

// Returns address of planner buffer block used by system motions. Called by segment generator.
plan_block_t* plan_get_system_motion_block() {
    return &block_buffer[block_buffer_head];
}

// Returns address of first planner block, if available. Called by various main program functions.
plan_block_t* plan_get_current_block() {
    if (block_buffer_head == block_buffer_tail) {
        return NULL;  // Buffer empty
    }
    return &block_buffer[block_buffer_tail];
}

float plan_get_exec_block_exit_speed_sqr() {
    uint8_t block_index = plan_next_block_index(block_buffer_tail);
    if (block_index == block_buffer_head) {
        return 0.0f;
    }
    return block_buffer[block_index].entry_speed_sqr;
}

//返回块环缓冲区的可用状态。如果满了，是正确的。
uint8_t plan_check_full_buffer() {
    return block_buffer_tail == next_buffer_head;
}

// Computes and returns block nominal speed based on running condition and override values.
// NOTE: All system motion commands, such as homing/parking, are not subject to overrides.
float plan_compute_profile_nominal_speed(plan_block_t* block) {
    float nominal_speed = block->programmed_rate;
    if (block->motion.rapidMotion) {
        nominal_speed *= (0.01 * sys.r_override);
    } else {
        if (!(block->motion.noFeedOverride)) {
            nominal_speed *= (0.01 * sys.f_override);
        }
        if (nominal_speed > block->rapid_rate) {
            nominal_speed = block->rapid_rate;
        }
    }
    if (nominal_speed > MINIMUM_FEED_RATE) {
        return nominal_speed;
    }
    return MINIMUM_FEED_RATE;
}

// Computes and updates the max entry speed (sqr) of the block, based on the minimum of the junction's
// previous and current nominal speeds and max junction speed.
static void plan_compute_profile_parameters(plan_block_t* block, float nominal_speed, float prev_nominal_speed) {
    // Compute the junction maximum entry based on the minimum of the junction speed and neighboring nominal speeds.
    if (nominal_speed > prev_nominal_speed) {
        block->max_entry_speed_sqr = prev_nominal_speed * prev_nominal_speed;
    } else {
        block->max_entry_speed_sqr = nominal_speed * nominal_speed;
    }

    if (block->max_entry_speed_sqr > block->max_junction_speed_sqr) {
        block->max_entry_speed_sqr = block->max_junction_speed_sqr;
    }
}

// Re-calculates buffered motions profile parameters upon a motion-based override change.根据基于运动的覆盖更改重新计算缓冲运动配置文件参数。
void plan_update_velocity_profile_parameters() {
    uint8_t       block_index = block_buffer_tail;
    plan_block_t* block;
    float         nominal_speed;
    float         prev_nominal_speed = SOME_LARGE_VALUE;  // Set high for first block nominal speed calculation.
    while (block_index != block_buffer_head) {
        block         = &block_buffer[block_index];
        nominal_speed = plan_compute_profile_nominal_speed(block);
        plan_compute_profile_parameters(block, nominal_speed, prev_nominal_speed);
        prev_nominal_speed = nominal_speed;
        block_index        = plan_next_block_index(block_index);
    }
    pl.previous_nominal_speed = prev_nominal_speed;  // Update prev nominal speed for next incoming block.
}

uint8_t plan_buffer_line(float* target, plan_line_data_t* pl_data) {
    // Prepare and initialize new block. Copy relevant pl_data for block execution.
    plan_block_t* block = &block_buffer[block_buffer_head];
    memset(block, 0, sizeof(plan_block_t));  // Zero all block values.
    block->motion        = pl_data->motion;
    block->coolant       = pl_data->coolant;
    block->spindle       = pl_data->spindle;
    block->spindle_speed = pl_data->spindle_speed;

#ifdef USE_LINE_NUMBERS
    block->line_number = pl_data->line_number;
#endif
    // Compute and store initial move distance data.
    int32_t target_steps[MAX_N_AXIS], position_steps[MAX_N_AXIS];
    float   unit_vec[MAX_N_AXIS], delta_mm;
    uint8_t idx;
    // Copy position data based on type of motion being planned.
    if (block->motion.systemMotion) {
        memcpy(position_steps, sys_position, sizeof(sys_position));
    } else {
        memcpy(position_steps, pl.position, sizeof(pl.position));
    }
    auto n_axis = number_axis->get();
    for (idx = 0; idx < n_axis; idx++) {
        // Calculate target position in absolute steps, number of steps for each axis, and determine max step events.
        // Also, compute individual axes distance for move and prep unit vector calculations.
        // NOTE: Computes true distance from converted step values.
        target_steps[idx]       = lround(target[idx] * axis_settings[idx]->steps_per_mm->get());
        block->steps[idx]       = labs(target_steps[idx] - position_steps[idx]);
        block->step_event_count = MAX(block->step_event_count, block->steps[idx]);
        delta_mm                = (target_steps[idx] - position_steps[idx]) / axis_settings[idx]->steps_per_mm->get();
        unit_vec[idx]           = delta_mm;  // Store unit vector numerator
        // Set direction bits. Bit enabled always means direction is negative.
        if (delta_mm < 0.0) {
            block->direction_bits |= bit(idx);
        }
    }
    // Bail if this is a zero-length block. Highly unlikely to occur.
    if (block->step_event_count == 0) {
        return PLAN_EMPTY_BLOCK;
    }

    // Calculate the unit vector of the line move and the block maximum feed rate and acceleration scaled
    // down such that no individual axes maximum values are exceeded with respect to the line direction.
    // NOTE: This calculation assumes all axes are orthogonal (Cartesian) and works with ABC-axes,
    // if they are also orthogonal/independent. Operates on the absolute value of the unit vector.
    block->millimeters  = convert_delta_vector_to_unit_vector(unit_vec);
    block->acceleration = limit_acceleration_by_axis_maximum(unit_vec);
    block->rapid_rate   = limit_rate_by_axis_maximum(unit_vec);
    // Store programmed rate.
    if (block->motion.rapidMotion) {
        block->programmed_rate = block->rapid_rate;
    } else {
        block->programmed_rate = pl_data->feed_rate;
        if (block->motion.inverseTime) {
            block->programmed_rate *= block->millimeters;
        }
    }
    // TODO: Need to check this method handling zero junction speeds when starting from rest.
    if ((block_buffer_head == block_buffer_tail) || (block->motion.systemMotion)) {
        // Initialize block entry speed as zero. Assume it will be starting from rest. Planner will correct this later.
        // If system motion, the system motion block always is assumed to start from rest and end at a complete stop.
        block->entry_speed_sqr        = 0.0;
        block->max_junction_speed_sqr = 0.0;  // Starting from rest. Enforce start from zero velocity.
    } else {
        // Compute maximum allowable entry speed at junction by centripetal acceleration approximation.
        // Let a circle be tangent to both previous and current path line segments, where the junction
        // deviation is defined as the distance from the junction to the closest edge of the circle,
        // colinear with the circle center. The circular segment joining the two paths represents the
        // path of centripetal acceleration. Solve for max velocity based on max acceleration about the
        // radius of the circle, defined indirectly by junction deviation. This may be also viewed as
        // path width or max_jerk in the previous Grbl version. This approach does not actually deviate
        // from path, but used as a robust way to compute cornering speeds, as it takes into account the
        // nonlinearities of both the junction angle and junction velocity.
        //
        // NOTE: If the junction deviation value is finite, Grbl executes the motions in an exact path
        // mode (G61). If the junction deviation value is zero, Grbl will execute the motion in an exact
        // stop mode (G61.1) manner. In the future, if continuous mode (G64) is desired, the math here
        // is exactly the same. Instead of motioning all the way to junction point, the machine will
        // just follow the arc circle defined here. The Arduino doesn't have the CPU cycles to perform
        // a continuous mode path, but ARM-based microcontrollers most certainly do.
        //
        // NOTE: The max junction speed is a fixed value, since machine acceleration limits cannot be
        // changed dynamically during operation nor can the line move geometry. This must be kept in
        // memory in the event of a feedrate override changing the nominal speeds of blocks, which can
        // change the overall maximum entry speed conditions of all blocks.
        float junction_unit_vec[MAX_N_AXIS];
        float junction_cos_theta = 0.0;
        for (idx = 0; idx < n_axis; idx++) {
            junction_cos_theta -= pl.previous_unit_vec[idx] * unit_vec[idx];
            junction_unit_vec[idx] = unit_vec[idx] - pl.previous_unit_vec[idx];
        }
        // NOTE: Computed without any expensive trig, sin() or acos(), by trig half angle identity of cos(theta).
        if (junction_cos_theta > 0.999999) {
            //  For a 0 degree acute junction, just set minimum junction speed.
            block->max_junction_speed_sqr = MINIMUM_JUNCTION_SPEED * MINIMUM_JUNCTION_SPEED;
        } else {
            if (junction_cos_theta < -0.999999) {
                // Junction is a straight line or 180 degrees. Junction speed is infinite.
                block->max_junction_speed_sqr = SOME_LARGE_VALUE;
            } else {
                convert_delta_vector_to_unit_vector(junction_unit_vec);
                float junction_acceleration = limit_acceleration_by_axis_maximum(junction_unit_vec);
                float sin_theta_d2          = sqrt(0.5 * (1.0 - junction_cos_theta));  // Trig half angle identity. Always positive.
                block->max_junction_speed_sqr =
                    MAX(MINIMUM_JUNCTION_SPEED * MINIMUM_JUNCTION_SPEED,
                        (junction_acceleration * junction_deviation->get() * sin_theta_d2) / (1.0 - sin_theta_d2));
            }
        }
    }
    // Block system motion from updating this data to ensure next g-code motion is computed correctly.
    if (!(block->motion.systemMotion)) {
        float nominal_speed = plan_compute_profile_nominal_speed(block);
        plan_compute_profile_parameters(block, nominal_speed, pl.previous_nominal_speed);
        pl.previous_nominal_speed = nominal_speed;
        // Update previous path unit_vector and planner position.
        memcpy(pl.previous_unit_vec, unit_vec, sizeof(unit_vec));  // pl.previous_unit_vec[] = unit_vec[]
        memcpy(pl.position, target_steps, sizeof(target_steps));   // pl.position[] = target_steps[]
        // New block is all set. Update buffer head and next buffer head indices.
        block_buffer_head = next_buffer_head;
        next_buffer_head  = plan_next_block_index(block_buffer_head);
        // Finish up by recalculating the plan with the new block.
        planner_recalculate();
    }
    return PLAN_OK;
}

// 重置规划器位置向量. 由系统中止/初始化例程调用。
void plan_sync_position() {
// TODO:如果电机配置与机器位置不在同一个坐标系中，
//该函数需要更新以适应差异。
    uint8_t idx;
    auto    n_axis = number_axis->get();
    for (idx = 0; idx < n_axis; idx++) {
        pl.position[idx] = sys_position[idx];
    }
}

// Returns the number of available blocks are in the planner buffer.返回规划器缓冲区中可用块的数量。
uint8_t plan_get_block_buffer_available() {
    if (block_buffer_head >= block_buffer_tail) {
        return (BLOCK_BUFFER_SIZE - 1) - (block_buffer_head - block_buffer_tail);
    } else {
        return block_buffer_tail - block_buffer_head - 1;
    }
}

// Returns the number of active blocks are in the planner buffer.
// NOTE: Deprecated. Not used unless classic status reports are enabled in config.h
uint8_t plan_get_block_buffer_count() {
    if (block_buffer_head >= block_buffer_tail) {
        return block_buffer_head - block_buffer_tail;
    } else {
        return BLOCK_BUFFER_SIZE - (block_buffer_tail - block_buffer_head);
    }
}

// Re-initialize buffer plan with a partially completed block, assumed to exist at the buffer tail.用一个部分完成的块重新初始化缓冲区计划，假设存在于缓冲区尾部。
// Called after a steppers have come to a complete stop for a feed hold and the cycle is stopped.在步进器完全停止供料并停止循环后调用。
void plan_cycle_reinitialize() {
    // Re-plan from a complete stop. Reset planner entry speeds and buffer planned pointer.从头到尾重新计划。重置计划进入速度和缓冲区计划指针。
    st_update_plan_block_parameters();
    block_buffer_planned = block_buffer_tail;
    planner_recalculate();
}
