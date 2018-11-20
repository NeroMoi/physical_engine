/*
*项目名称：physical_engine
*文件标识：
*摘要：使用C++与OpenGL实现的一个简易2D物理引擎
* 目前实现功能：
*		  1.渲染一个凸多边形(逆时针)
*		  2.实现物体的三个自由度(两个平移一个旋转)
*         3.给物体添加重力
*		  4.实现物体的受力分析、速度、角速度
*		  5.物体的碰撞检测
*	      6.物体的弹性碰撞（摩擦力、反作用力）
*		  7.物体的休眠模式
* 待添加功能：
*		  1.连续碰撞实现(防止因物体速度过快导致的隧穿效应)
*		  2.joint(关节限制、马达)
*当前版本：v1.0
*作者：Nero
*完成日期：
*参考来源:https://github.com/erincatto/Box2D
*
*/

#include<cstdio>
#include<iostream>

#include"myOpenGL.h"

int main(int argc, char *argv[])
{
	myOpenGLInit(&argc, argv);

	return 0;
}