/*
*文件名称：elasticcollision
*文件标识：
*摘要：实现物体的弹性碰撞与摩擦力
*
*当前版本：v1.0
*作者：Nero
*完成日期：
*/
#ifndef __ELASTICCOLLSIION_H__
#define __ELASTICCOLLSIION_H__

#include"collision.h"


//碰撞计算
void collision_prepare(collision &c);//每一次碰撞的集合

void collision_update(collision &c);//每一次碰撞

void collision_remove_sleep();//去休眠态

							  //模板函数
template<typename ContainerT, typename PredicateT>
void erase_if(ContainerT &items, const PredicateT &predicate);




#endif



