/*
*�ļ����ƣ�bivector
*�ļ���ʶ��
*ժҪ����ά�����Ͷ�ά������Ĵ���
*
*��ǰ�汾��v1.0
*���ߣ�Nero
*������ڣ�
*/
#ifndef __BIVECTOR_H__
#define __BIVECTOR_H__

#include<cstdio>
#include<cmath>

#include"typealias.h"

typedef struct biVector //��ά����
{
	decimal x{ 0 }, y{ 0 }; //x,y�������

							//���캯��

	biVector() = default; //�ϳɵ�Ĭ�Ϲ��캯��
	biVector(decimal _x, decimal _y) : x(_x), y(_y) {}

	biVector(const biVector &v) = default;//�ϳɵ�Ĭ�Ͽ�������

	biVector& operator=(const biVector&v) = default;//�ϳɵĸ�ֵ�����


	~biVector() = default;//�ϳɵ���������

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

	//��Ԫ������֧�ֶ�άʸ��������
	friend inline biVector operator*(decimal d, const biVector &v)
	{
		return biVector{ d*v.x, d*v.y };
	}

	//ȡ��
	inline biVector operator-() const
	{
		return biVector{ -x, -y };
	}

	//ʸ���ĵ�����õ�һ������
	decimal dot(const biVector & v) const;

	//��άʸ���Ĳ�����õ�ƽ���ı��ε����
	decimal cross(const biVector &v) const;

	//����
	decimal magnitude() const;

	//��һ��
	biVector normalize() const;

	//�������� ˳ʱ����ת90��
	biVector normal() const;

	//������
	biVector N() const;

	//��0�ıȽ�
	bool zero(decimal d) const;

}v2;

typedef struct martix2 //��ά����->��ת����
{
	decimal x1{ 1 }, y1{ 0 }, x2{ 0 }, y2{ 1 }; //Ĭ����ת0��
	martix2() = default;
	martix2(decimal _x1, decimal _y1, decimal _x2, decimal _y2) : x1(_x1), y1(_y1), x2(_x2), y2(_y2) {}
	martix2(const martix2& m) = default;
	martix2 &operator =(const martix2& m) = default;

	~martix2() = default;

	void rotata(decimal theta);
	v2 rotate(const v2 &v)const;

}m2;




#endif // ! __BIVECTOR_H__

