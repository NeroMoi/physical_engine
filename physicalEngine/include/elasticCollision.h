/*
*�ļ����ƣ�elasticcollision
*�ļ���ʶ��
*ժҪ��ʵ������ĵ�����ײ��Ħ����
*
*��ǰ�汾��v1.0
*���ߣ�Nero
*������ڣ�
*/
#ifndef __ELASTICCOLLSIION_H__
#define __ELASTICCOLLSIION_H__

#include"collision.h"


//��ײ����
void collision_prepare(collision &c);//ÿһ����ײ�ļ���

void collision_update(collision &c);//ÿһ����ײ

void collision_remove_sleep();//ȥ����̬

							  //ģ�庯��
template<typename ContainerT, typename PredicateT>
void erase_if(ContainerT &items, const PredicateT &predicate);




#endif



