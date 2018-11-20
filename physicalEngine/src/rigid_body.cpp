#include"rigidBody.h"
#include "globalVariable.h"
#include"myOpenGL.h"
#include<iostream>
/*
* ������
*/

decimal_inv::decimal_inv(decimal v)
{
	set(v);
}

void decimal_inv::set(decimal v)
{
	value = v;
	if (std::isinf(value))//�Ƿ����ޱ߽�����ּ����������С 1/����
		inv = 0;
	else if (std::abs(value) < EPSILON)// 1/0
		inv = inf;
	else
		inv = 1 / value;
}


/*
*  �������θ���ķ���ʵ��
*/
decimal c2d_polygon::calc_polygon_area(const std::vector<v2>&vertices)
{
	decimal area = 0;
	auto size = vertices.size();

	//������������֮�ͣ�ԭ���ڶ�����ڲ�
	for (size_t i =0; i <size; ++i)
	{
		auto j = (i + 1) % size;

		area += vertices[i].cross(vertices[j]);
	}

	return area / 2;

}

// ������������
v2 c2d_polygon::calc_polygon_centroid(const std::vector<v2>&vertices)
{
	v2 centerpos;

	auto size = vertices.size();
	//���� = ��������������*�����/�����
	//���������� = ������֮��/3 ->��һ�˵���ԭ��

	for (size_t i = 0; i < size; ++i)
	{
		auto j = (i + 1) % size;
		centerpos += (vertices[i] + vertices[j]) *vertices[i].cross(vertices[j]);
	}

	return centerpos / 6.0 / calc_polygon_area(vertices);

}

// ��������ת������
decimal c2d_polygon::calc_polygon_inertia(decimal mass, const std::vector<v2>&vertices)
{
	if (std::isinf(mass))
		return mass;
	decimal acc0 = 0, acc1 = 0;
	auto size = vertices.size();

	//ת������ = m/6*(�����������*��(a*a + b*b + a*b))/(�����)

	for(size_t i =0; i <size; ++i)
	{
		auto a = vertices[i], b = vertices[(i + 1) % size];
		auto _area = std::abs(a.cross(b));

		acc0 += _area*(a.dot(a) + b.dot(b) + a.dot(b));
		acc1 += _area;
	}

	return (mass * acc0 / 6 / acc1);

}

// ����߽磨���ΰ�Χ��
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

// �жϵ��ڱ߽���
bool c2d_polygon::contains_in_bound(const v2 &pt)
{
	return boundMin.x < pt.x &&
		boundMax.x > pt.x &&
		boundMin.y < pt.y &&
		boundMax.y > pt.y;
}

// �����߷�
// �����ǵ��ڱ���
bool c2d_polygon::contains_in_polygon(const v2 &pt)
{
	const auto size = verticesWorld.size();
	auto ncross = false;//���ߴ�������εĴ�������������ż��

	if (size < 3) return false;//�����ɶ���Σ����ж�

	for (size_t i = 0, j = size - 1; i < size; j = i++)
	{
		//�жϵ������������Ƿ񾭹�����ε�һ����
		//�����Ĵ����ж�

		if ((verticesWorld[i].y > pt.y) != (verticesWorld[j].y > pt.y) &&
			(pt.x < ((verticesWorld[j].x - verticesWorld[i].x) * (pt.y - verticesWorld[i].y) /
			(verticesWorld[j].y - verticesWorld[i].y) + verticesWorld[i].x))
			)

			ncross = !ncross;


	}

	return ncross;
}

//�жϵ��Ƿ��ھ��ο���
bool c2d_polygon::contains(const v2 &pt) 
{
	return contains_in_bound(pt) && contains_in_polygon(pt);
}


void c2d_polygon::init()
{
	inertia.set(calc_polygon_inertia(mass.value, vertices)); //ת������
	center = calc_polygon_centroid(vertices);//����
	refresh();

}

//�����������
void c2d_polygon::refresh()
{
	R.rotata(angle);
	for (size_t i = 0; i < edges();++i)
	{
		auto v = R.rotate(vertices[i] - center) + center;
		vertex(i) = pos + v; //��������ת��Ϊ��������
	}
	if(!statics)//���Ǿ�̬����
	calc_bounds();//����߽�
}

//��������
void c2d_polygon::impulse(const v2 &p, const v2 &r)
{
	if (statics) return;

	auto _p = p * dt_inv;//�����ۻ�

	F += _p; //���������
	Fa += _p; //�����ܵ�����
	M += r.cross(_p); // ��������=r x f = ת������ *�Ǽ��ٶ�
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
		vertex(i) = pos + v; // ��������ת��Ϊ��������
	}
	calc_bounds();
}

//���¶���
void c2d_polygon::pass3() 
{
	F += gravity * mass.value * dt; // �������ۻ�
	Fa += F;     //������
}

//��ʼ������Ϊ0
void c2d_polygon::pass4() 
{
	Fa.x = Fa.y = 0;
}

//�ж��Ƿ�����
void c2d_polygon::pass5()
{

	// ���������ٶȡ����ٶ�Ϊ��ʱ���ж�����
	if (Fa.zero(EPSILON_FORCE) && V.zero(EPSILON_V) && std::abs(angleV) < EPSILON_ANGLE_V)
	{
		V.x = 0;
		V.y = 0;
		angleV = 0;
		pass0();//��ʼ������������
		pass4();
		collision = 0;//��ײΪ0
		sleep = true; //��ֹ̬
	}
}

void c2d_polygon::drag(const v2 &pt, const v2 &offset)
{
	V += mass.inv *offset; //����Сoffset��v=v0+at
	angleV +=  inertia.inv *(pt - pos - center).cross(offset); //�Ǽ��ٶ�=��������/ת������
}

void c2d_polygon::draw()
{
	drawPolygon(*this);

}

//��idxΪ��㣬��һ����Ϊ�յ������
v2 c2d_polygon::edge(size_t idx) const
{
	return verticesWorld[(idx + 1) % verticesWorld.size()] - verticesWorld[idx];
}

v2 & c2d_polygon::vertex(size_t idx) //���ض��㣬��������ϵ��
{
	return verticesWorld[idx % verticesWorld.size()];
}

size_t c2d_polygon::index(size_t idx) const
{
	return idx % verticesWorld.size();
}

size_t c2d_polygon::edges() const //��������ϵ�£����������
{
	return verticesWorld.size();
}

c2d_polygon::~c2d_polygon()
{
	//std::cout << "release" <<std::endl;
}