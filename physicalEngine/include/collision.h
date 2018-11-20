/*
*文件名称： collision
*文件标识：
*摘要：碰撞结构体以及碰撞检测
*
*当前版本：v1.0
*作者：Nero
*完成日期：
*/
#ifndef __COLLISION_H__
#define __COLLISION_H__

#include<array>
#include"rigidBody.h"
// 接触点
struct contact
{
	v2 pos; // 位置
	v2 ra, rb; // 物体接触点到重心的向量
	decimal sep{ 0 }; // 分离投影（重叠距离）
	decimal mass_normal{ 0 }; //法线
	decimal mass_tangent{ 0 };//切线
	decimal bias{ 0 }; //弹性碰撞系数
	decimal pn{ 0 }; // 法向冲量
	decimal pt{ 0 }; // 切向冲量
	int idxA{ 0 }, idxB{ 0 }; // （交点属于的）物体a和b的边索引+1，为正则属于B，为负属于A

	contact(v2 _pos, size_t index) : pos(_pos), idxA(index), idxB(index) {}

	bool operator==(const contact &other) const;
	bool operator!=(const contact &other) const;
	
};

// 碰撞结构
struct collision
{
	std::vector<contact> contacts; // 接触点集合
	c2d_body *bodyA{ nullptr },*bodyB{ nullptr }; // 碰撞的两个物体，避免野指针
	size_t idxA{ 0 }, idxB{0}; // 碰撞的两个物体对应的各自的轴
	decimal satA{ 0 }, satB{ 0 }; //碰撞物体投影的最大间隙 ->这里是负数，0代表最大即该边使得两个物体最接近或接触面最多
	v2 N;//法线

};

//用于创建碰撞物体的id
// 小的id在前面，大的id在后面
uint32_t make_id(uint16_t a, uint16_t b); //创建id,a在前16位,b在后16位


// 碰撞检测-SAT分离轴定理
// 检测两凸包是否相交
// 表述：如果两个凸多边形没有相交，那么存在这两个物体在一个轴上的投影不重叠。
// 轴：只需采用两个凸包的每个条做检测即可
// 只要最大间隙大于零，即为不相交
// separation：最大间隙
// idx：最大间隙对应的边
bool max_separating_axis(c2d_polygon *a, c2d_polygon *b, decimal &separation, size_t &idx);

//使用轴对齐包围盒快速判断碰撞
bool AABB_collide(c2d_polygon *a, c2d_polygon *b);

//找出相交的边
static size_t incident_edge(const v2 &N, c2d_polygon *body);

// Sutherland-Hodgman（多边形裁剪）
size_t clip(std::vector<contact> &out,
			const std::vector<contact> &in,
			size_t i,
			const v2 &p1, const v2 &p2);

//计算碰撞（返回是否碰撞）
bool solve_collision(collision &c);

//碰撞计算
void collision_update(collision &c, const collision &old_c);

/*
* 两动态物体碰撞检测
* 1.创建碰撞id供碰撞结构体使用
* 2. 通过AABB和SAT分离轴对两物体进行碰撞检测，并找出碰撞的轴
* 3.1 如果没有碰撞便在碰撞结构体里面寻找之前是否有过碰撞，有便删除，并标记为非碰撞物体
* 3.2 如果产生碰撞，构建碰撞结构体，并把碰撞物体标记为碰撞
*/
void collision_detection(const c2d_body::ptr &a, c2d_body::ptr &b);

/*
 * 碰撞检测
 * 1.对非静态物体进行遍历检测
 * 2.对于合外力为0的物体，忽略它碰撞其他物体的可能，这里合外力为零表示sleep
 * 3.物体与静态物体间的碰撞检测
*/

void collision_detection();



#endif;

