#include"bivector.h"

/*
* ��άʸ����ķ���ʵ�ֲ���
*/
decimal v2::dot(const biVector & v)const
{
	return x*v.x + y*v.y;
}

decimal v2::cross(const biVector &v) const
{
	return x*v.y - y*v.x;
}

decimal v2::magnitude() const
{
	return std::sqrt(x*x + y*y);
}

biVector v2::normalize() const
{
	return *this / magnitude();
}

biVector v2::normal() const //�˷���Ϊ˳ʱ����ת90�ȷ���
{
	return N().normalize();
}

biVector v2::N() const
{
	return biVector{ y,-x };
}

bool v2::zero(decimal d) const
{
	return std::abs(x) < d && std::abs(y) < d;
}

/*
* ��ά���󷽷�ʵ�ֲ���
*/


void m2::rotata(decimal theta)
{
	const auto _sin = std::sin(theta);
	const auto _cos = std::cos(theta);

	*this = m2{ _cos, -_sin, _sin, _cos };
}
v2 m2::rotate(const v2 &v)const
{
	return v2{ x1 * v.x + y1 * v.y, x2 * v.x + y2 * v.y };
}