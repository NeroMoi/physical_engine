/*
*文件名称：globalVariable
*文件标识：
*摘要：全局变量和常量
*
*当前版本：v1.0
*作者：Nero
*完成日期：
*/
#ifndef __GLOBALVARIABLE_H__
#define __GLOBALVARIABLE_H__

#include <cstdio>
#include <chrono>
#include <unordered_map>
#include <random>
#include <cassert>
#include <algorithm>
#include <vector>
#include <cmath>
#include <memory>

#include "typealias.h" //类型别名
#include "bivector.h" //二维向量
#include "rigidBody.h"//刚体
#include "collision.h"//碰撞结构

#define FPS 30 //每秒帧数
#define GRAVITY -9.8 //重力
#define FRAME_SPAN (1.0 / FPS)
#define COLLISION_ITERATIONS 10 //碰撞迭代
#define EPSILON 1e-6     //ε介子，用于判断浮点数与0的比较
#define EPSILON_FORCE 1e-4  //ε力，极小的
#define EPSILON_V 1e-4      //速度
#define EPSILON_ANGLE_V 1e-4 //角速度
#define COLL_NORMAL_SCALE 1  //法线的
#define COLL_TANGENT_SCALE 1 //切线的
#define ENABLE_SLEEP 1       //静止
#define PI 3.1415926


using timePoint = std::chrono::time_point<std::chrono::high_resolution_clock>;

/*全局变量*/
extern  timePoint last_clock ;
extern  double dt ; //帧率，用于拖动积分
extern  double dt_inv;//一定时间步的帧数
extern  bool paused ; // 是否暂停

extern  v2 gravity;
extern uint16_t global_id;

extern bool mouse_drag;
extern v2 global_drag;//鼠标拖动
extern v2 global_drag_offset;//鼠标拖动位移

extern std::vector<c2d_body::ptr> bodies; // 寻常物体 ->使用基类的指针指向派生类的对象
extern std::vector<c2d_body::ptr> static_bodies; // 静态物体->边界

extern std::unordered_map<uint32_t, collision> collisions; //碰撞物体集合
/*全局常量*/
static const auto inf = std::numeric_limits<decimal>::infinity(); //该类型的极值


#endif