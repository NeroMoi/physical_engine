/*
*�ļ����ƣ� collision
*�ļ���ʶ��
*ժҪ����ײ�ṹ���Լ���ײ���
*
*��ǰ�汾��v1.0
*���ߣ�Nero
*������ڣ�
*/
#ifndef __COLLISION_H__
#define __COLLISION_H__

#include<array>
#include"rigidBody.h"
// �Ӵ���
struct contact
{
	v2 pos; // λ��
	v2 ra, rb; // ����Ӵ��㵽���ĵ�����
	decimal sep{ 0 }; // ����ͶӰ���ص����룩
	decimal mass_normal{ 0 }; //����
	decimal mass_tangent{ 0 };//����
	decimal bias{ 0 }; //������ײϵ��
	decimal pn{ 0 }; // �������
	decimal pt{ 0 }; // �������
	int idxA{ 0 }, idxB{ 0 }; // ���������ڵģ�����a��b�ı�����+1��Ϊ��������B��Ϊ������A

	contact(v2 _pos, size_t index) : pos(_pos), idxA(index), idxB(index) {}

	bool operator==(const contact &other) const;
	bool operator!=(const contact &other) const;
	
};

// ��ײ�ṹ
struct collision
{
	std::vector<contact> contacts; // �Ӵ��㼯��
	c2d_body *bodyA{ nullptr },*bodyB{ nullptr }; // ��ײ���������壬����Ұָ��
	size_t idxA{ 0 }, idxB{0}; // ��ײ�����������Ӧ�ĸ��Ե���
	decimal satA{ 0 }, satB{ 0 }; //��ײ����ͶӰ������϶ ->�����Ǹ�����0������󼴸ñ�ʹ������������ӽ���Ӵ������
	v2 N;//����

};

//���ڴ�����ײ�����id
// С��id��ǰ�棬���id�ں���
uint32_t make_id(uint16_t a, uint16_t b); //����id,a��ǰ16λ,b�ں�16λ


// ��ײ���-SAT�����ᶨ��
// �����͹���Ƿ��ཻ
// �������������͹�����û���ཻ����ô����������������һ�����ϵ�ͶӰ���ص���
// �᣺ֻ���������͹����ÿ��������⼴��
// ֻҪ����϶�����㣬��Ϊ���ཻ
// separation������϶
// idx������϶��Ӧ�ı�
bool max_separating_axis(c2d_polygon *a, c2d_polygon *b, decimal &separation, size_t &idx);

//ʹ��������Χ�п����ж���ײ
bool AABB_collide(c2d_polygon *a, c2d_polygon *b);

//�ҳ��ཻ�ı�
static size_t incident_edge(const v2 &N, c2d_polygon *body);

// Sutherland-Hodgman������βü���
size_t clip(std::vector<contact> &out,
			const std::vector<contact> &in,
			size_t i,
			const v2 &p1, const v2 &p2);

//������ײ�������Ƿ���ײ��
bool solve_collision(collision &c);

//��ײ����
void collision_update(collision &c, const collision &old_c);

/*
* ����̬������ײ���
* 1.������ײid����ײ�ṹ��ʹ��
* 2. ͨ��AABB��SAT������������������ײ��⣬���ҳ���ײ����
* 3.1 ���û����ײ������ײ�ṹ������Ѱ��֮ǰ�Ƿ��й���ײ���б�ɾ���������Ϊ����ײ����
* 3.2 ���������ײ��������ײ�ṹ�壬������ײ������Ϊ��ײ
*/
void collision_detection(const c2d_body::ptr &a, c2d_body::ptr &b);

/*
 * ��ײ���
 * 1.�ԷǾ�̬������б������
 * 2.���ں�����Ϊ0�����壬��������ײ��������Ŀ��ܣ����������Ϊ���ʾsleep
 * 3.�����뾲̬��������ײ���
*/

void collision_detection();



#endif;

