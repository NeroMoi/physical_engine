#include"globalVariable.h"

/*
* 全局变量的定义
*
*/

timePoint last_clock = std::chrono::high_resolution_clock::now();
double dt = FRAME_SPAN;
double dt_inv = 1.0 *FPS;
bool paused = false;

v2 gravity { 0, GRAVITY };

std::vector<c2d_body::ptr> bodies; // 寻常物体
std::vector<c2d_body::ptr> static_bodies; // 静态物体
std::unordered_map<uint32_t, collision> collisions;//碰撞结构体


uint16_t global_id = 1;
bool mouse_drag = false;
v2 global_drag;//鼠标拖动
v2 global_drag_offset;//鼠标拖动位移
