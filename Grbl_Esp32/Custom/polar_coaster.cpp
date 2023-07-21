/*
  polar_coaster.cpp - Implements simple inverse kinematics for Grbl_ESP32
  Part of Grbl_ESP32

  Copyright (c) 2019 Barton Dring @buildlog


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

	Inverse kinematics determine the joint parameters required to get to a position
	in 3D space. Grbl will still work as 3 axes of steps, but these steps could
	represent angles, etc instead of linear units.

	Unless forward kinematics are applied to the reporting, Grbl will report raw joint
	values instead of the normal Cartesian positions

	How it works...

	If you tell it to go to X10 Y10 Z10 in Cartesian space, for example, the equations
	will convert those values to the required joint values. In the case of a polar machine, X represents the radius,
	Y represents the polar degrees and Z would be unchanged.

	In most cases, a straight line in Cartesian space could cause a curve in the new system.
	To fix this, the line is broken into very small segments and each segment is converted
	to the new space. While each segment is also distorted, the amount is so small it cannot be seen.

	This segmentation is how normal Grbl draws arcs.

	Feed Rate

	Feed rate is given in steps/time. Due to the new coordinate units and non linearity issues, the
	feed rate may need to be adjusted. The ratio of the step distances in the original coordinate system
	determined and applied to the feed rate.

	TODO:
		Add y offset, for completeness
		Add ZERO_NON_HOMED_AXES option


*/

// This file is enabled by defining CUSTOM_CODE_FILENAME "polar_coaster.cpp"
// in Machines/polar_coaster.h, thus causing this file to be included
// from ../custom_code.cpp

void  calc_polar(float* target_xyz, float* polar, float last_angle);
float abs_angle(float ang);

static float last_angle  = 0;
static float last_radius = 0;

// this get called before homing
// return false to complete normal home
// return true to exit normal homing
bool kinematics_pre_homing(uint8_t cycle_mask) {
    return false;  // finish normal homing cycle
}

void kinematics_post_homing() {
    // sync the X axis (do not need sync but make it for the fail safe)
    last_radius = sys_position[X_AXIS];
    // reset the internal angle value
    last_angle = 0;
}

/*
 Apply inverse kinematics for a polar system

 float target: 					The desired target location in machine space
 plan_line_data_t *pl_data:		Plan information like feed rate, etc
 float *position:				The previous "from" location of the move

 Note: It is assumed only the radius axis (X) is homed and only X and Z have offsets

对极系应用逆运动学



浮动目标:期望的目标在机器空间中的位置

plan_line_data_t *pl_data:计划进料速率等信息

float *position:之前移动的“from”位置



注意:假设只有半径轴(X)是归属的，只有X和Z有偏移
*/

bool cartesian_to_motors(float* target, plan_line_data_t* pl_data, float* position) {
    float    dx, dy, dz;          // 每个笛卡尔轴上的距离
    float    p_dx, p_dy, p_dz;    // 每极轴上的距离
    float    dist, polar_dist;    // 两个系统的距离…用于确定进给速度
    uint32_t segment_count;       // 移动将被打断的段数。
    float    seg_target[N_AXIS];  // 当前细分市场的目标
    float    polar[N_AXIS];       // 极坐标下的目标位置
    float    x_offset = gc_state.coord_system[X_AXIS] + gc_state.coord_offset[X_AXIS];//从机器坐标系偏移
    float    z_offset = gc_state.coord_system[Z_AXIS] + gc_state.coord_offset[Z_AXIS];  // offset from machine coordinate system
    //grbl_sendf(CLIENT_SERIAL, "Position: %4.2f %4.2f %4.2f \r\n", position[X_AXIS] - x_offset, position[Y_AXIS], position[Z_AXIS]);
    //grbl_sendf(CLIENT_SERIAL, "Target: %4.2f %4.2f %4.2f \r\n", target[X_AXIS] - x_offset, target[Y_AXIS], target[Z_AXIS]);
    // 计算每个轴的笛卡尔移动距离
    dx = target[X_AXIS] - position[X_AXIS];
    dy = target[Y_AXIS] - position[Y_AXIS];
    dz = target[Z_AXIS] - position[Z_AXIS];
    //计算X、Y轴的总移动距离

// Z轴在两个坐标系中是相同的，因此被忽略
    dist = sqrt((dx * dx) + (dy * dy) + (dz * dz));
    if (pl_data->motion.rapidMotion) {
        segment_count = 1;  // 快速G0运动不用于绘制，所以跳过分割
    } else {
        segment_count = ceil(dist / SEGMENT_LENGTH);  // 确定我们需要的段数…四舍五入，至少有1个
    }
    dist /= segment_count;  // 直线距离
    for (uint32_t segment = 1; segment <= segment_count; segment++) {
        // 确定这部分的目标
        seg_target[X_AXIS] = position[X_AXIS] + (dx / float(segment_count) * segment) - x_offset;
        seg_target[Y_AXIS] = position[Y_AXIS] + (dy / float(segment_count) * segment);
        seg_target[Z_AXIS] = position[Z_AXIS] + (dz / float(segment_count) * segment) - z_offset;
        calc_polar(seg_target, polar, last_angle);
        //开始确定新的进料速率

//计算每个轴的移动距离
        p_dx                      = polar[RADIUS_AXIS] - last_radius;
        p_dy                      = polar[POLAR_AXIS] - last_angle;
        p_dz                      = dz;
        polar_dist                = sqrt((p_dx * p_dx) + (p_dy * p_dy) + (p_dz * p_dz));  // 计算总移动距离
        float polar_rate_multiply = 1.0;                                                  // fail safe rate  故障 安全 率
        if (polar_dist == 0 || dist == 0) {
            // prevent 0 feed rate and division by 0防止进料速率为0和除以0
            polar_rate_multiply = 1.0;  // default to same feed rate 默认为相同的进料速率
        } else {
            // calc a feed rate multiplier 计算一个进料倍增器
            polar_rate_multiply = polar_dist / dist;
            if (polar_rate_multiply < 0.5) {
                // prevent much slower speed 防止速度过慢
                polar_rate_multiply = 0.5;
            }
        }
        //进给速率
        pl_data->feed_rate *= polar_rate_multiply;  // apply the distance ratio between coord systems 应用坐标系统之间的距离比
        // end determining new feed rate 结束确定新的进料速率
        polar[RADIUS_AXIS] += x_offset;
        polar[Z_AXIS] += z_offset;

        // mc_line() returns false if a jog is cancelled.如果慢跑被取消，返回false。
        // In that case we stop sending segments to the planner.在这种情况下，我们停止向计划器发送段。
        if (!mc_line(polar, pl_data)) {
            return false;
        }

        //半径
        last_radius = polar[RADIUS_AXIS];
        //角度
        last_angle  = polar[POLAR_AXIS];
    }
    // TO DO don't need a feedrate for rapids TO DO在急流中不需要进给口
    return true;
}

/*
Forward kinematics converts position back to the original cartesian system. It is
typically used for reporting

For example, on a polar machine, you tell it to go to a place like X10Y10. It
converts to a radius and angle using inverse kinematics. The machine posiiton is now
in those units X14.14 (radius) and Y45 (degrees). If you want to report those units as
X10,Y10, you would use forward kinematics

position = the current machine position
converted = position with forward kinematics applied.

*/
void motors_to_cartesian(float* cartesian, float* motors, int n_axis) {
    cartesian[X_AXIS] = cos(radians(motors[Y_AXIS])) * motors[X_AXIS] * -1;
    cartesian[Y_AXIS] = sin(radians(motors[Y_AXIS])) * motors[X_AXIS];
    cartesian[Z_AXIS] = motors[Z_AXIS];  // unchanged
}

// helper functions

/*******************************************
*从笛卡尔值计算极坐标值

目标轴在笛卡尔(xyz)空间中的位置数组

float polar:返回极性值的数组

float last_angle:起始点的极坐标角度。

＊

*角度计算是0到360，但你不希望直线从350到10。这将

*向后排一长队。你想让它从350到370。反过来也是一样。

＊

*这意味着角度可以累积到非常高的正或负的粗值

一个很长的工作。

＊
*/
void calc_polar(float* target_xyz, float* polar, float last_angle) {
    float delta_ang;  // 上一个角度和下一个角度的差值
    polar[RADIUS_AXIS] = hypot_f(target_xyz[X_AXIS], target_xyz[Y_AXIS]);
    if (polar[RADIUS_AXIS] == 0) {
        polar[POLAR_AXIS] = last_angle;  // don't care about angle at center
    } else {
        polar[POLAR_AXIS] = atan2(target_xyz[Y_AXIS], target_xyz[X_AXIS]) * 180.0 / M_PI;
        // no negative angles...we want the absolute angle not -90, use 270
        polar[POLAR_AXIS] = abs_angle(polar[POLAR_AXIS]);
    }
    polar[Z_AXIS] = target_xyz[Z_AXIS];  // Z is unchanged
    delta_ang     = polar[POLAR_AXIS] - abs_angle(last_angle);
    // if the delta is above 180 degrees it means we are crossing the 0 degree line
    if (fabs(delta_ang) <= 180.0)
        polar[POLAR_AXIS] = last_angle + delta_ang;
    else {
        if (delta_ang > 0.0) {
            // crossing zero counter clockwise
            polar[POLAR_AXIS] = last_angle - (360.0 - delta_ang);
        } else
            polar[POLAR_AXIS] = last_angle + delta_ang + 360.0;
    }
}

// Return a 0-360 angle ... fix above 360 and below zero
float abs_angle(float ang) {
    ang = fmod(ang, 360.0);  // 0-360 or 0 to -360
    if (ang < 0.0)
        ang = 360.0 + ang;
    return ang;
}

// Polar coaster has macro buttons, this handles those button pushes.
void user_defined_macro(uint8_t index) {
    switch (index) {
        case 0:
            WebUI::inputBuffer.push("$H\r");  // home machine
            break;
        case 1:
            WebUI::inputBuffer.push("[ESP220]/1.nc\r");  // run SD card file 1.nc
            break;
        case 2:
            WebUI::inputBuffer.push("[ESP220]/2.nc\r");  // run SD card file 2.nc
            break;
        default:
            break;
    }
}

// handle the M30 command
void user_m30() {
    WebUI::inputBuffer.push("$H\r");
}
