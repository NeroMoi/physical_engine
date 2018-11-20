/*
*�ļ����ƣ�globalVariable
*�ļ���ʶ��
*ժҪ��ȫ�ֱ����ͳ���
*
*��ǰ�汾��v1.0
*���ߣ�Nero
*������ڣ�
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

#include "typealias.h" //���ͱ���
#include "bivector.h" //��ά����
#include "rigidBody.h"//����
#include "collision.h"//��ײ�ṹ

#define FPS 30 //ÿ��֡��
#define GRAVITY -9.8 //����
#define FRAME_SPAN (1.0 / FPS)
#define COLLISION_ITERATIONS 10 //��ײ����
#define EPSILON 1e-6     //�Ž��ӣ������жϸ�������0�ıȽ�
#define EPSILON_FORCE 1e-4  //��������С��
#define EPSILON_V 1e-4      //�ٶ�
#define EPSILON_ANGLE_V 1e-4 //���ٶ�
#define COLL_NORMAL_SCALE 1  //���ߵ�
#define COLL_TANGENT_SCALE 1 //���ߵ�
#define ENABLE_SLEEP 1       //��ֹ
#define PI 3.1415926


using timePoint = std::chrono::time_point<std::chrono::high_resolution_clock>;

/*ȫ�ֱ���*/
extern  timePoint last_clock ;
extern  double dt ; //֡�ʣ������϶�����
extern  double dt_inv;//һ��ʱ�䲽��֡��
extern  bool paused ; // �Ƿ���ͣ

extern  v2 gravity;
extern uint16_t global_id;

extern bool mouse_drag;
extern v2 global_drag;//����϶�
extern v2 global_drag_offset;//����϶�λ��

extern std::vector<c2d_body::ptr> bodies; // Ѱ������ ->ʹ�û����ָ��ָ��������Ķ���
extern std::vector<c2d_body::ptr> static_bodies; // ��̬����->�߽�

extern std::unordered_map<uint32_t, collision> collisions; //��ײ���弯��
/*ȫ�ֳ���*/
static const auto inf = std::numeric_limits<decimal>::infinity(); //�����͵ļ�ֵ


#endif