/*
*文件名称：bivector
*文件标识：
*摘要：二维向量和二维矩阵类的创建
*
*当前版本：v1.0
*作者：Nero
*完成日期：
*/
#ifndef __BIVECTOR_H__
#define __BIVECTOR_H__

#include<cstdio>
#include<cmath>

#include"typealias.h"

typedef struct biVector //二维向量
{
	decimal x{ 0 }, y{ 0 }; //x,y方向标量

							//构造函数

	biVector() = default; //合成的默认构造函数
	biVector(decimal _x, decimal _y) : x(_x), y(_y) {}

	biVector(const biVector &v) = default;//合成的默认拷贝函数

	biVector& operator=(const biVector&v) = default;//合成的赋值运算符


	~biVector() = default;//合成的析构函数

	inline biVector operator*(decimal d) const
	{
		return biVector{ this->x*d ,this->y*d };
	}

	inline biVector operator/(decimal d) const
	{
		return biVector{ x / d ,y / d };
	}

	inline biVector operator+(const biVector &v) const
	{
		return biVector{ x + v.x ,y + v.y };
	}

	inline biVector operator-(const biVector &v) const
	{
		return biVector{ x - v.x ,y - v.y };
	}

	inline biVector& operator+=(const biVector &v)
	{
		x += v.x;
		y += v.y;
		return *this;
	}

	//友元函数，支持二维矢量的伸缩
	friend inline biVector operator*(decimal d, const biVector &v)
	{
		return biVector{ d*v.x, d*v.y };
	}

	//取反
	inline biVector operator-() const
	{
		return biVector{ -x, -y };
	}

	//矢量的点积，得到一个标量
	decimal dot(const biVector & v) const;

	//二维矢量的叉积，得到平行四边形的面积
	decimal cross(const biVector &v) const;

	//量度
	decimal magnitude() const;

	//归一化
	biVector normalize() const;

	//法线向量 顺时针旋转90度
	biVector normal() const;

	//求法向量
	biVector N() const;

	//与0的比较
	bool zero(decimal d) const;

}v2;

typedef struct martix2 //二维矩阵->旋转矩阵
{
	decimal x1{ 1 }, y1{ 0 }, x2{ 0 }, y2{ 1 }; //默认旋转0度
	martix2() = default;
	martix2(decimal _x1, decimal _y1, decimal _x2, decimal _y2) : x1(_x1), y1(_y1), x2(_x2), y2(_y2) {}
	martix2(const martix2& m) = default;
	martix2 &operator =(const martix2& m) = default;

	~martix2() = default;

	void rotata(decimal theta);
	v2 rotate(const v2 &v)const;

}m2;




#endif // ! __BIVECTOR_H__

