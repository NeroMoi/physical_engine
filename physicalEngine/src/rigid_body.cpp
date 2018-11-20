#include"rigidBody.h"
#include "globalVariable.h"
#include"myOpenGL.h"
#include<iostream>
/*
* 分数求导
*/

decimal_inv::decimal_inv(decimal v)
{
	set(v);
}

void decimal_inv::set(decimal v)
{
	value = v;
	if (std::isinf(value))//是否是无边界的数字即无穷大无穷小 1/无穷
		inv = 0;
	else if (std::abs(value) < EPSILON)// 1/0
		inv = inf;
	else
		inv = 1 / value;
}


/*
*  定义多边形刚体的方法实现
*/
decimal c2d_polygon::calc_polygon_area(const std::vector<v2>&vertices)
{
	decimal area = 0;
	auto size = vertices.size();

	//求所有三角形之和，原点在多边形内部
	for (size_t i =0; i <size; ++i)
	{
		auto j = (i + 1) % size;

		area += vertices[i].cross(vertices[j]);
	}

	return area / 2;

}

// 计算多边形重心
v2 c2d_polygon::calc_polygon_centroid(const std::vector<v2>&vertices)
{
	v2 centerpos;

	auto size = vertices.size();
	//重心 = （各三角形重心*面积）/总面积
	//三角形重心 = 两向量之和/3 ->有一端点是原点

	for (size_t i = 0; i < size; ++i)
	{
		auto j = (i + 1) % size;
		centerpos += (vertices[i] + vertices[j]) *vertices[i].cross(vertices[j]);
	}

	return centerpos / 6.0 / calc_polygon_area(vertices);

}

// 计算多边形转动惯量
decimal c2d_polygon::calc_polygon_inertia(decimal mass, const std::vector<v2>&vertices)
{
	if (std::isinf(mass))
		return mass;
	decimal acc0 = 0, acc1 = 0;
	auto size = vertices.size();

	//转动惯量 = m/6*(各三角形面积*其(a*a + b*b + a*b))/(总面积)

	for(size_t i =0; i <size; ++i)
	{
		auto a = vertices[i], b = vertices[(i + 1) % size];
		auto _area = std::abs(a.cross(b));

		acc0 += _area*(a.dot(a) + b.dot(b) + a.dot(b));
		acc1 += _area;
	}

	return (mass * acc0 / 6 / acc1);

}

// 计算边界（矩形包围）
void c2d_polygon::calc_bounds()
{
	boundMin = boundMax = vertex(0);

	for (size_t i = 1; i < verticesWorld.size(); ++i) 
	{
		boundMin.x = std::min(boundMin.x, vertex(i).x);
		boundMin.y = std::min(boundMin.y, vertex(i).y);
		boundMax.x = std::max(boundMax.x, vertex(i).x);
		boundMax.y = std::max(boundMax.y, vertex(i).y);

	}
}

// 判断点在边界内
bool c2d_polygon::contains_in_bound(const v2 &pt)
{
	return boundMin.x < pt.x &&
		boundMax.x > pt.x &&
		boundMin.y < pt.y &&
		boundMax.y > pt.y;
}

// 引射线法
// 不考虑点在边上
bool c2d_polygon::contains_in_polygon(const v2 &pt)
{
	const auto size = verticesWorld.size();
	auto ncross = false;//射线穿过多边形的次数是奇数还是偶数

	if (size < 3) return false;//不构成多边形，则不判断

	for (size_t i = 0, j = size - 1; i < size; j = i++)
	{
		//判断点引出的射线是否经过多边形的一条边
		//经过的次数判断

		if ((verticesWorld[i].y > pt.y) != (verticesWorld[j].y > pt.y) &&
			(pt.x < ((verticesWorld[j].x - verticesWorld[i].x) * (pt.y - verticesWorld[i].y) /
			(verticesWorld[j].y - verticesWorld[i].y) + verticesWorld[i].x))
			)

			ncross = !ncross;


	}

	return ncross;
}

//判断点是否在矩形框内
bool c2d_polygon::contains(const v2 &pt) 
{
	return contains_in_bound(pt) && contains_in_polygon(pt);
}


void c2d_polygon::init()
{
	inertia.set(calc_polygon_inertia(mass.value, vertices)); //转动惯量
	center = calc_polygon_centroid(vertices);//重心
	refresh();

}

//获得世界坐标
void c2d_polygon::refresh()
{
	R.rotata(angle);
	for (size_t i = 0; i < edges();++i)
	{
		auto v = R.rotate(vertices[i] - center) + center;
		vertex(i) = pos + v; //本地坐标转换为世界坐标
	}
	if(!statics)//不是静态物体
	calc_bounds();//计算边界
}

//冲量计算
void c2d_polygon::impulse(const v2 &p, const v2 &r)
{
	if (statics) return;

	auto _p = p * dt_inv;//力的累积

	F += _p; //物体的受力
	Fa += _p; //物体总的受力
	M += r.cross(_p); // 合外力矩=r x f = 转动惯量 *角加速度
}

void c2d_polygon::init_static()
{
	statics = true;

}

void c2d_polygon::update(int n)
{
	if (statics)
		return;

	if (sleep)
		return;

	switch (n)
	{
		case 0:
			pass0();break;
		case 1:
			pass1();break;
		case 2:
			pass2();break;
		case 3:
			pass3();break;
		case 4:
			pass4();break;
		case 5:
			pass5();break;
		default: break;
	}
}

void c2d_polygon::pass0()
{
	F.x = F.y = 0;
	M = 0;
}

void c2d_polygon::pass1()
{
	V += F * mass.inv *dt; // v = v0 +a0t;
	angleV += M * inertia.inv * dt; // anglev = angleV0 + a(angle)*dt;
}

void c2d_polygon::pass2()
{
	pos += V * dt;
	angle += angleV * dt;
	R.rotata(angle);
	for (size_t i = 0; i < edges(); ++i) 
	{
		auto v = R.rotate(vertices[i] - center) + center;
		vertex(i) = pos + v; // 本地坐标转换为世界坐标
	}
	calc_bounds();
}

//更新动量
void c2d_polygon::pass3() 
{
	F += gravity * mass.value * dt; // 重力的累积
	Fa += F;     //总外力
}

//初始化动量为0
void c2d_polygon::pass4() 
{
	Fa.x = Fa.y = 0;
}

//判断是否休眠
void c2d_polygon::pass5()
{

	// 当动量和速度、角速度为零时，判定休眠
	if (Fa.zero(EPSILON_FORCE) && V.zero(EPSILON_V) && std::abs(angleV) < EPSILON_ANGLE_V)
	{
		V.x = 0;
		V.y = 0;
		angleV = 0;
		pass0();//初始化动量与力矩
		pass4();
		collision = 0;//碰撞为0
		sleep = true; //静止态
	}
}

void c2d_polygon::drag(const v2 &pt, const v2 &offset)
{
	V += mass.inv *offset; //力大小offset，v=v0+at
	angleV +=  inertia.inv *(pt - pos - center).cross(offset); //角加速度=合外力矩/转动惯量
}

void c2d_polygon::draw()
{
	drawPolygon(*this);

}

//以idx为起点，下一顶点为终点的向量
v2 c2d_polygon::edge(size_t idx) const
{
	return verticesWorld[(idx + 1) % verticesWorld.size()] - verticesWorld[idx];
}

v2 & c2d_polygon::vertex(size_t idx) //返回顶点，世界坐标系下
{
	return verticesWorld[idx % verticesWorld.size()];
}

size_t c2d_polygon::index(size_t idx) const
{
	return idx % verticesWorld.size();
}

size_t c2d_polygon::edges() const //世界坐标系下，顶点的数量
{
	return verticesWorld.size();
}

c2d_polygon::~c2d_polygon()
{
	//std::cout << "release" <<std::endl;
}