/*
  CoreXY.cpp - 
  
  Copyright (c) 2020    Barton Dring @buildlog

  https://corexy.com/theory.html

  Limitations 
  - Must home via $H. $HX type homes not allowed
  - Must home one axis per cycle
  - limited to 3 axis systems...easy fix in increase (just donate)

  ============================================================================

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
    FYI: http://forums.trossenrobotics.com/tutorials/introduction-129/delta-robot-kinematics-3276/
    Better: http://hypertriangle.com/~alex/delta-robot-tutorial/
*/
#include "../src/Settings.h"

// working machine specification

// Homing axis search distance multiplier. Computed by this value times the cycle travel.寻的轴搜索距离倍增器。由这个值乘以周期行程计算。
#ifndef HOMING_AXIS_SEARCH_SCALAR
#    define HOMING_AXIS_SEARCH_SCALAR 1.1  // Must be > 1 to ensure limit switch will be engaged.必须为> 1，以确保限位开关接通。
#endif
#ifndef HOMING_AXIS_LOCATE_SCALAR
#    define HOMING_AXIS_LOCATE_SCALAR 2.0  // Must be > 1 to ensure limit switch is cleared.必须为> 1，以确保清除限位开关。
#endif

// The midTbot has a quirk where the x motor has to move twice as far as it would
// on a normal T-Bot or CoreXY
// midTbot有一个怪癖，x电机必须移动两倍于它应该移动的距离

//在正常的T-Bot或CoreXY上
#ifndef MIDTBOT
const float geometry_factor = 1.0;
#else
const float geometry_factor = 2.0;
#endif

static float last_cartesian[MAX_N_AXIS] = {};

// prototypes for helper functions辅助函数的原型
float three_axis_dist(float* point1, float* point2);

void machine_init() {
            // grbl_msg_sendf(CLIENT_SERIAL,
            //                MsgLevel::Warning,
            //                "init gc_state.position X:%f, Y:%f, Z:%f",
            //                gc_state.position[X_AXIS],
            //                gc_state.position[Y_AXIS],
            //                gc_state.position[Z_AXIS]);
    float* rivetA=sys_get_rivetA();
    float* rivetB=sys_get_rivetB();
    float MPosition[MAX_N_AXIS];
    //获取home位置，它记录的是笛卡尔坐标
    MPosition[X_AXIS] = axis_settings[X_AXIS]->home_mpos->get();//+ (rivetB[X_AXIS]  / 2);
    MPosition[Y_AXIS] = axis_settings[Y_AXIS]->home_mpos->get();//+ (rivetB[X_AXIS]  / 2);

    //memcpy(gc_state.position, MPosition, sizeof(MPosition));//2023.04.01 有可能导致启动漂移
    //memcpy(gc_state.coord_offset,MPosition, sizeof(MPosition));
    //memcpy(gc_state.coord_system,MPosition, sizeof(MPosition));
    auto n_axis = number_axis->get();
    // float rivetA[n_axis];
    // rivetA[X_AXIS] = 0;
    // rivetA[Y_AXIS] = axis_settings[Y_AXIS]->max_travel->get();
    // float rivetB[n_axis];
    // rivetB[X_AXIS] = axis_settings[X_AXIS]->max_travel->get();
    // rivetB[Y_AXIS] = axis_settings[Y_AXIS]->max_travel->get();
    //计算线缆A长度
    float A = sqrt((rivetA[X_AXIS] - MPosition[X_AXIS]) * (rivetA[X_AXIS] - MPosition[X_AXIS]) + (rivetA[Y_AXIS] - MPosition[Y_AXIS]) * (rivetA[Y_AXIS] - MPosition[Y_AXIS])) ;//- (rivetB[X_AXIS] / 2);
    //计算线缆B长度
    float B = sqrt((rivetB[X_AXIS] - MPosition[X_AXIS]) * (rivetB[X_AXIS] - MPosition[X_AXIS]) + (rivetB[Y_AXIS] - MPosition[Y_AXIS]) * (rivetB[Y_AXIS] - MPosition[Y_AXIS])) ;//- (rivetB[X_AXIS] / 2);
    
//长度*每毫米步数=机器位置（机器的步数）
    sys_position[X_AXIS] = (A)*axis_settings[X_AXIS]->steps_per_mm->get();
    sys_position[Y_AXIS] = (B)*axis_settings[Y_AXIS]->steps_per_mm->get();

    for (int idx = 0; idx < n_axis; idx++) {
        gc_state.coord_offset[idx] = 0;
        //gc_state.coord_system[idx] = 0;
    }

            // grbl_msg_sendf(CLIENT_SERIAL,
            //                MsgLevel::Warning,
            //                "init gc_state.position X:%f, Y:%f, Z:%f",
            //                gc_state.position[X_AXIS],
            //                gc_state.position[Y_AXIS],
            //                gc_state.position[Z_AXIS]);
    
    //     //machineSizeMm = {700,1000}
    //     // print a startup message to show the kinematics are enable打印一个启动消息显示运动学是启用的

    // #ifdef MIDTBOT
    //     grbl_msg_sendf(CLIENT_SERIAL, MsgLevel::Info, "CoreXY (midTbot) Kinematics Init");
    // #else
    //     grbl_msg_sendf(CLIENT_SERIAL, MsgLevel::Info, "CoreXY Kinematics Init");
    // #endif
}

// Converts Cartesian to motors with no motion control
// static void cartesian_to_motors(float* position) {
//     float motor[MAX_N_AXIS];
//     motor[X_AXIS] = getMachineA(position[X_AXIS], position[Y_AXIS]);
//     motor[Y_AXIS] = getMachineB(position[X_AXIS], position[Y_AXIS]);
//     position[X_AXIS] = motor[X_AXIS];
//     position[Y_AXIS] = motor[Y_AXIS];

//     // Z and higher just pass through unchangedZ和更高的值不变地通过
// }

// Cycle mask is 0 unless the user sends a single axis command like $HZ
// This will always return true to prevent the normal Grbl homing cycle
//周期掩码为0，除非用户发送一个单轴命令，如$HZ

//它将始终返回true，以防止正常的Grbl寻的循环
bool user_defined_homing(uint8_t cycle_mask) {
    machine_init();
    // grbl_msg_sendf(CLIENT_SERIAL,
    //                MsgLevel::Warning,
    //                "user_defined_homing sys_position X:%f,Y:%f,Z:%f",
    //                sys_position[X_AXIS],
    //                sys_position[Y_AXIS],
    //                sys_position[Z_AXIS]);
    return true;
}

//Inverse Kinematics calculates motor positions from real world cartesian positions
//position is the old machine position, target the new machine position
//Breaking into segments is not needed with CoreXY, because it is a linear system.
//逆运动学从真实世界的笛卡尔位置计算电机位置
//position为旧机器位置，target为新机器位置
//CoreXY是一个线性系统，不需要分割成多个部分。
//笛卡尔坐标转换为电机步数
//计算机器坐标
bool cartesian_to_motors(float* target, plan_line_data_t* pl_data, float* position) {
    float dx, dy, dz;  //每个笛卡尔轴上的距离
    auto n_axis = number_axis->get();
    // float rivetA[n_axis];
    // rivetA[X_AXIS] = 0;
    // rivetA[Y_AXIS] = axis_settings[Y_AXIS]->max_travel->get();
    // float rivetB[n_axis];
    // rivetB[X_AXIS] = axis_settings[X_AXIS]->max_travel->get();
    // rivetB[Y_AXIS] = axis_settings[Y_AXIS]->max_travel->get();
    float* rivetA=sys_get_rivetA();
    float* rivetB=sys_get_rivetB();

    isnan(target[X_AXIS]) ? target[X_AXIS] = 0 : target[X_AXIS];
    isnan(target[Y_AXIS]) ? target[Y_AXIS] = 0 : target[Y_AXIS];
    if(target[X_AXIS]<=0||target[Y_AXIS]<=0)
    return false;

    // grbl_msg_sendf(CLIENT_SERIAL,
    //    MsgLevel::Warning,
    //    "cartesian_to_motors targetX:%f(old:%f)  targetY:%f(old:%f) xsteps_per_mm:%f   ysteps_per_mm:%f",
    //    target[X_AXIS],
    //    position[X_AXIS],
    //    target[Y_AXIS],
    //    position[Y_AXIS],axis_settings[X_AXIS]->steps_per_mm->get(),axis_settings[Y_AXIS]->steps_per_mm->get());
    //targetX:34571.500000 to motor A:nan(old:34471.500000)  targetY:nan to motor B:nan(old:nan)
    float motors[n_axis];

    motors[X_AXIS] = sqrt((rivetA[X_AXIS] - target[X_AXIS]) * (rivetA[X_AXIS] - target[X_AXIS]) + (rivetA[Y_AXIS] - target[Y_AXIS]) * (rivetA[Y_AXIS] - target[Y_AXIS])) ;//- (rivetB[X_AXIS] / 2);
    motors[Y_AXIS] = sqrt((rivetB[X_AXIS] - target[X_AXIS]) * (rivetB[X_AXIS] - target[X_AXIS]) + (rivetB[Y_AXIS] - target[Y_AXIS]) * (rivetB[Y_AXIS] - target[Y_AXIS])) ;//- (rivetB[X_AXIS] / 2);

    // for (uint8_t axis = Z_AXIS; axis <= n_axis; axis++) {
    motors[Z_AXIS] = target[Z_AXIS];
    // }
    isnan(motors[X_AXIS]) ? motors[X_AXIS] = 0 : motors[X_AXIS];
    isnan(motors[Y_AXIS]) ? motors[Y_AXIS] = 0 : motors[Y_AXIS];
    //  transform_cartesian_to_motors(motors, target);
    #ifdef DEBUG
    
      grbl_msg_sendf(CLIENT_SERIAL,
                               MsgLevel::Warning,
                               "cartesian_to_motors cartesian X:%f,Y:%f,Z:%f to motor A:%f(old:%f),B:%f(old:%f),Z:%f(old:%f)",
                               target[X_AXIS],
                               target[Y_AXIS],
                               target[Z_AXIS],
                               motors[X_AXIS],
                               position[X_AXIS],
                               motors[Y_AXIS],
                               position[Y_AXIS],
                               motors[Z_AXIS],
                               position[Z_AXIS]);
    
    #endif  
    //         grbl_msg_sendf(CLIENT_SERIAL,
    //                        MsgLevel::Warning,
    //                        "gc_state.position X:%f, Y:%f, Z:%f",
    //                        gc_state.position[X_AXIS],
    //                        gc_state.position[Y_AXIS],
    //                        gc_state.position[Z_AXIS]);
    //         grbl_msg_sendf(CLIENT_SERIAL,
    //                        MsgLevel::Warning,
    //                        "gc_state.coord_system X:%f, Y:%f, Z:%f",
    //                        gc_state.coord_system[X_AXIS],
    //                        gc_state.coord_system[Y_AXIS],
    //                        gc_state.coord_system[Z_AXIS]);
    //         grbl_msg_sendf(CLIENT_SERIAL,
    //                        MsgLevel::Warning,
    //                        "gc_state.coord_offset X:%f, Y:%f, Z:%f",
    //                        gc_state.coord_offset[X_AXIS],
    //                        gc_state.coord_offset[Y_AXIS],
    //                        gc_state.coord_offset[Z_AXIS]);
    //         grbl_msg_sendf(CLIENT_SERIAL,
    //                        MsgLevel::Warning,
    //                        "sys_position X:%d, Y:%d, Z:%d",
    //                        sys_position[X_AXIS],
    //                        sys_position[Y_AXIS],
    //                        sys_position[Z_AXIS]);
    //         grbl_msg_sendf(CLIENT_SERIAL,
    //                        MsgLevel::Warning,
    //                        "sys_probe_position X:%d, Y:%d, Z:%d",
    //                        sys_probe_position[X_AXIS],
    //                        sys_probe_position[Y_AXIS],
    //                        sys_probe_position[Z_AXIS]);
    // if (!pl_data->motion.rapidMotion) {
    //     float last_motors[n_axis];
    //     transform_cartesian_to_motors(last_motors, position);
    //     pl_data->feed_rate *= (three_axis_dist(motors, last_motors) / dist);
    // }

    return mc_line(motors, pl_data);
}
        //获得物体中心和鼠标坐标连线，与y轴正半轴之间的夹角
        float getAngle(float* target1,float* target2) {
            float x = abs(target1[X_AXIS] - target2[X_AXIS]);
            float y = abs(target1[Y_AXIS] - target2[Y_AXIS]);
            float z = sqrt(pow(x, 2) + pow(y, 2));
            float cos = y / z;
            float radina = acos(cos);//用反三角函数求弧度
            float angle = floor(180 / (PI / radina));//将弧度转换成角度

            if (target2[X_AXIS] > target1[X_AXIS] && target2[Y_AXIS] > target1[Y_AXIS]) {//鼠标在第四象限
                angle = 180 - angle;
            }
            if (target2[X_AXIS] == target1[X_AXIS] && target2[Y_AXIS] > target1[Y_AXIS]) {//鼠标在y轴负方向上
                angle = 180;
            }
            if (target2[X_AXIS] > target1[X_AXIS] && target2[Y_AXIS] == target1[Y_AXIS]) {//鼠标在x轴正方向上
                angle = 90;
            }
            if (target2[X_AXIS] < target1[X_AXIS] && target2[Y_AXIS] > target1[Y_AXIS]) {//鼠标在第三象限
                angle = 180 + angle;
            }
            if (target2[X_AXIS] < target1[X_AXIS] && target2[Y_AXIS] == target1[Y_AXIS]) {//鼠标在x轴负方向
                angle = 270;
            }
            if (target2[X_AXIS] < target1[X_AXIS] && target2[Y_AXIS] < target1[Y_AXIS]) {//鼠标在第二象限
                angle = 360 - angle;
            }
            return angle;
        }

        // 两点间距离
        float getDistance(float* target1,float* target2) {
            float xdiff = abs(target2[X_AXIS] - target1[X_AXIS]);
            float ydiff = abs(target2[Y_AXIS] - target1[Y_AXIS]);
            return pow((xdiff * xdiff + ydiff * ydiff), 0.5);
        }
 //根据两点坐标和边长长度，计算第三点坐标，第三点有两个
 
        void Cal3rdPoint(float* outPoint,float* pointA,float* pointB,float bc,float ca) {
           
            //用两点坐标计算ab边的长度
            float ab = getDistance(pointA, pointB);

            // dy = pointB[Y_AXIS] - pointA[Y_AXIS];
            // dx = pointB[X_AXIS] - pointA[X_AXIS];
           float tmpValue = (ca * ca + ab * ab - bc * bc) / (2 * ca * ab);
            //AB的方位角
            // var angAB = getAngle(pointA, pointB);//原始坐标系(x轴左侧逆时针为正向角度)
            //铆钉AB
            float angAB = -getAngle(pointA, pointB) + 180;//转为屏幕坐标系角度(x轴右侧顺时针为正向角度)
            //console.log('对应角度',angAB,getAngle(pointA, pointB));
            angAB = angAB * (M_PI / 180);
            //float angAB = 1.5707963267948966;//转为屏幕坐标系角度(x轴右侧顺时针为正向角度)
            // angAB = atan(dy / dx);
            //A点对应BC边的角度
            float angBC = acos(tmpValue);
            //AC的方位角
          

           float angAC = angAB + angBC;
            outPoint[X_AXIS] = pointA[X_AXIS] + ca * sin(angAC);
            outPoint[Y_AXIS] = pointA[Y_AXIS] + ca * cos(angAC);
          
        }
        //电机步数转换为笛卡尔坐标
// motors -> cartesian
void motors_to_cartesian(float* cartesian, float* motor, int n_axis) {
    // apply the forward kinemetics to the machine coordinates
    // https://corexy.com/theory.html
    //calc_fwd[X_AXIS] = 0.5 / geometry_factor * (position[X_AXIS] + position[Y_AXIS]);
    // cartesian[X_AXIS] = 0.5 * (motors[X_AXIS] + motors[Y_AXIS]) / geometry_factor;//图形因子
    // cartesian[Y_AXIS] = 0.5 * (motors[X_AXIS] - motors[Y_AXIS]);
    // float rivetA[n_axis];
    // rivetA[X_AXIS] = 0;
    // rivetA[Y_AXIS] = axis_settings[Y_AXIS]->max_travel->get();
    // float rivetB[n_axis];
    // rivetB[X_AXIS] = axis_settings[X_AXIS]->max_travel->get();
    // rivetB[Y_AXIS] = axis_settings[Y_AXIS]->max_travel->get();
    float* rivetA=sys_get_rivetA();
    float* rivetB=sys_get_rivetB();

    float A = motor[X_AXIS] ;//+ (rivetB[X_AXIS] / 2);
    float B = motor[Y_AXIS] ;//+ (rivetB[X_AXIS] / 2);
    float MPosition[MAX_N_AXIS];
    //10000-
    Cal3rdPoint(MPosition,rivetA,rivetB,B,A);
    isnan(MPosition[X_AXIS]) ? MPosition[X_AXIS] = 0 : MPosition[X_AXIS];
    isnan(MPosition[Y_AXIS]) ? MPosition[Y_AXIS] = 0 : MPosition[Y_AXIS];
    
    if(MPosition[X_AXIS]<=0||MPosition[Y_AXIS]<=0)
    return ;
    cartesian[X_AXIS] = MPosition[X_AXIS];
    cartesian[Y_AXIS] = MPosition[Y_AXIS];

    //  grbl_msg_sendf(CLIENT_SERIAL,
    //                        MsgLevel::Warning,
    //                        "motors_to_cartesian   motors A:%f,B:%f cartesian:X%f,Y%f,Z%f",
    //                        A,
    //                        B,
    //                        cartesian[X_AXIS],
    //                        cartesian[Y_AXIS],
    //                        cartesian[Z_AXIS]);

    // grbl_msg_sendf(CLIENT_SERIAL,
    //                        MsgLevel::Warning,
    //                        "motors_to_cartesian  motor[X_AXIS]:%f to cartesian[X_AXIS]:%f  motor[Y_AXIS]:%f to cartesian[Y_AXIS]:%f  A:%f  B:%f",
    //                       motor[X_AXIS],
    //                        cartesian[X_AXIS],
    //                        motor[Y_AXIS],
    //                        cartesian[Y_AXIS],A,B);
    //其他轴不处理
    // for (int axis = Z_AXIS; axis < n_axis; axis++) {
    cartesian[Z_AXIS] = motor[Z_AXIS];
    // }
}
// //电机步数转换为笛卡尔坐标
// // motors -> cartesian
// void motors_to_cartesian(float* cartesian, float* motor, int n_axis) {
//     // apply the forward kinemetics to the machine coordinates
//     // https://corexy.com/theory.html
//     //calc_fwd[X_AXIS] = 0.5 / geometry_factor * (position[X_AXIS] + position[Y_AXIS]);
//     // cartesian[X_AXIS] = 0.5 * (motors[X_AXIS] + motors[Y_AXIS]) / geometry_factor;//图形因子
//     // cartesian[Y_AXIS] = 0.5 * (motors[X_AXIS] - motors[Y_AXIS]);
//     // float rivetA[n_axis];
//     // rivetA[X_AXIS] = 0;
//     // rivetA[Y_AXIS] = axis_settings[Y_AXIS]->max_travel->get();
//     // float rivetB[n_axis];
//     // rivetB[X_AXIS] = axis_settings[X_AXIS]->max_travel->get();
//     // rivetB[Y_AXIS] = axis_settings[Y_AXIS]->max_travel->get();
//     float* rivetA=sys_get_rivetA();
//     float* rivetB=sys_get_rivetB();

//     float A = motor[X_AXIS] ;//+ (rivetB[X_AXIS] / 2);
//     float B = motor[Y_AXIS] ;//+ (rivetB[X_AXIS] / 2);
//     float MPosition[MAX_N_AXIS];
//     //10000-
//     MPosition[X_AXIS] = (pow(rivetB[X_AXIS], 2.0f) - pow(B, 2.0f) + pow(A, 2.0f)) / (rivetB[X_AXIS] * 2.0f);
//     MPosition[Y_AXIS] = sqrt(pow(A, 2.0f) - pow(cartesian[X_AXIS], 2.0f));

//     isnan(MPosition[X_AXIS]) ? MPosition[X_AXIS] = 0 : MPosition[X_AXIS];
//     isnan(MPosition[Y_AXIS]) ? MPosition[Y_AXIS] = 0 : MPosition[Y_AXIS];
    
//     if(MPosition[X_AXIS]<=0||MPosition[Y_AXIS]<=0)
//     return ;
//     cartesian[X_AXIS] = MPosition[X_AXIS];
//     cartesian[Y_AXIS] = MPosition[Y_AXIS];

//     //  grbl_msg_sendf(CLIENT_SERIAL,
//     //                        MsgLevel::Warning,
//     //                        "motors_to_cartesian   motors A:%f,B:%f cartesian:X%f,Y%f,Z%f",
//     //                        A,
//     //                        B,
//     //                        motor[Z_AXIS],
//     //                        cartesian[X_AXIS],
//     //                        cartesian[Y_AXIS],
//     //                        cartesian[Z_AXIS]);

//     // grbl_msg_sendf(CLIENT_SERIAL,
//     //                        MsgLevel::Warning,
//     //                        "motors_to_cartesian  motor[X_AXIS]:%f to cartesian[X_AXIS]:%f  motor[Y_AXIS]:%f to cartesian[Y_AXIS]:%f  A:%f  B:%f",
//     //                       motor[X_AXIS],
//     //                        cartesian[X_AXIS],
//     //                        motor[Y_AXIS],
//     //                        cartesian[Y_AXIS],A,B);
//     //其他轴不处理
//     // for (int axis = Z_AXIS; axis < n_axis; axis++) {
//     cartesian[Z_AXIS] = motor[Z_AXIS];
//     // }
// }

bool kinematics_pre_homing(uint8_t cycle_mask) {
    return false;
}

void kinematics_post_homing() {
    // auto n_axis = number_axis->get();
    // memcpy(gc_state.position, last_cartesian, n_axis * sizeof(last_cartesian[0]));
}

void user_m30() {}

//================ 当地的辅助函数  =================
//确定(2)个3D点之间的单位距离
float three_axis_dist(float* point1, float* point2) {
    return sqrt(((point1[0] - point2[0]) * (point1[0] - point2[0])) + ((point1[1] - point2[1]) * (point1[1] - point2[1])) +
                ((point1[2] - point2[2]) * (point1[2] - point2[2])));
}
// /*
// 	custom_code_template.cpp (copy and use your machine name)
// 	Part of Grbl_ESP32

// 	copyright (c) 2020 -	Bart Dring. This file was intended for use on the ESP32

//   ...add your date and name here.

// 	CPU. Do not use this with Grbl for atMega328P

// 	Grbl_ESP32 is free software: you can redistribute it and/or modify
// 	it under the terms of the GNU General Public License as published by
// 	the Free Software Foundation, either version 3 of the License, or
// 	(at your option) any later version.

// 	Grbl_ESP32 is distributed in the hope that it will be useful,
// 	but WITHOUT ANY WARRANTY; without even the implied warranty of
// 	MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// 	GNU General Public License for more details.

// 	You should have received a copy of the GNU General Public License
// 	along with Grbl.  If not, see <http://www.gnu.org/licenses/>.

// 	=======================================================================

// This is a template for user-defined C++ code functions.  Grbl can be
// configured to call some optional functions. These functions have weak definitions
// in the main code. If you create your own version they will be used instead

// Put all of your functions in a .cpp file in the "Custom" folder.
// Add this to your machine definition file
// #define CUSTOM_CODE_FILENAME    "../Custom/YourFile.cpp"

// Be careful to return the correct values

// ===============================================================================

// Below are all the current weak function

// */

// /*
// This function is used as a one time setup for your machine.
// */
// void machine_init() {}

// /*
// This is used to initialize a display.
// */
// void display_init() {}

// /*
//   limitsCheckTravel() is called to check soft limits
//   It returns true if the motion is outside the limit values
// */
// bool limitsCheckTravel() {
//     return false;
// }

// /*
//   user_defined_homing(uint8_t cycle_mask) is called at the begining of the normal Grbl_ESP32 homing
//   sequence.  If user_defined_homing(uint8_t cycle_mask) returns false, the rest of normal Grbl_ESP32
//   homing is skipped if it returns false, other normal homing continues.  For
//   example, if you need to manually prep the machine for homing, you could implement
//   user_defined_homing(uint8_t cycle_mask) to wait for some button to be pressed, then return true.
// */
// bool user_defined_homing(uint8_t cycle_mask) {
//     // True = done with homing, false = continue with normal Grbl_ESP32 homing
//     return true;
// }

// /*
//   Inverse Kinematics converts X,Y,Z cartesian coordinate to the steps
//   on your "joint" motors.  It requires the following three functions:
// */

// /*
//   cartesian_to_motors() converts from cartesian coordinates to motor space.

//   Grbl_ESP32 processes arcs by converting them into tiny little line segments.
//   Kinematics in Grbl_ESP32 works the same way. Search for this function across
//   Grbl_ESP32 for examples. You are basically converting cartesian X,Y,Z... targets to

//     target = an N_AXIS array of target positions (where the move is supposed to go)
//     pl_data = planner data (see the definition of this type to see what it is)
//     position = an N_AXIS array of where the machine is starting from for this move
// */
// bool cartesian_to_motors(float* target, plan_line_data_t* pl_data, float* position) {
//     // this simply moves to the target. Replace with your kinematics.
//      grbl_msg_sendf(CLIENT_SERIAL,
//                            MsgLevel::Warning,
//                            "cartesian_to_motors   targetX:%f (old:%f)  targetY:%f (old:%f)",
//                            target[X_AXIS],
//                            position[X_AXIS],
//                            target[Y_AXIS],
//                            position[Y_AXIS]);
//     return mc_line(target, pl_data);
// }

// /*
//   kinematics_pre_homing() is called before normal homing
//   You can use it to do special homing or just to set stuff up

//   cycle_mask is a bit mask of the axes being homed this time.
// */
// bool kinematics_pre_homing(uint8_t cycle_mask) {
//     return false;  // finish normal homing cycle
// }

// /*
//   kinematics_post_homing() is called at the end of normal homing
// */
// void kinematics_post_homing() {}

// /*
//  status命令使用motors_to_笛卡儿()进行转换
// 你的运动位置与笛卡尔X,Y,Z…坐标。
// 在代码中将电机位置的N_AXIS数组转换为笛卡尔坐标。
// */
// void motors_to_cartesian(float* cartesian, float* motors, int n_axis) {
//         grbl_msg_sendf(CLIENT_SERIAL,
//                            MsgLevel::Warning,
//                            "motors_to_cartesian   motorsA:%f (cartesian:%f)  motorsB:%f (cartesian:%f)",
//                            motors[X_AXIS],
//                            cartesian[X_AXIS],
//                            motors[Y_AXIS],
//                            cartesian[Y_AXIS]);
//     // position[X_AXIS] =
//     // position[Y_AXIS] =
// }

// /*
//   user_tool_change() is called when tool change gcode is received,
//   to perform appropriate actions for your machine.当收到工具更改gcode时调用，

// 为您的机器执行适当的操作。
// */
// void user_tool_change(uint8_t new_tool) {}

// /*
//   options.  user_defined_macro() is called with the button number to
//   perform whatever actions you choose.调用的按钮号码为

// 执行你选择的任何行动。
// */
// void user_defined_macro(uint8_t index) {}

// /*
//   user_m30() is called when an M30 gcode signals the end of a gcode file.当M30 gcode标志着gcode文件的结束时调用。
// */
// void user_m30() {}

// // If you add any additional functions specific to your machine that
// // require calls from common code, guard their calls in the common code with
// // #ifdef USE_WHATEVER and add function prototypes (also guarded) to grbl.h
// //如果你为你的机器添加任何特定的附加功能

// //要求来自公共代码的调用，保护他们在公共代码中的调用

// // #ifdef USE_WHATEVER并添加函数原型(也被保护)到grbl.h