/*
*文件名称：myOpenGL
*文件标识：
*摘要：实现OpenGL的渲染配置
*
*当前版本：v1.0
*作者：Nero
*完成日期：
*/

//#pragma comment(linker, "/subsystem:\"windows\" /entry:\"mainCRTStartup\"") //去掉控制台

#ifndef __MYOPENGL_H__
#define __MYOPENGL_H__


#include<GL/freeglut.h> //OpenGL库
#include"rigidBody.h" //刚体
#include"collision.h"

c2d_body *find_body(const v2 &pos);//寻找被拖动的物体

static void c2d_step();//绘图操作步骤

void c2d_move(const v2 &v);//移动测试

void c2d_rotate(decimal d);//旋转测试

void c2d_offset(const v2 &pt, const v2 &offset);//拖动测试

void clear();//清除场景

void make_bound();//建立四周边界

void make_bound_special();//建立特定边界

void scene(int i);//选择场景

void init();//初始化场景

static void draw_text(int x, int y, const char*format, ...);//文字绘制

void display();//界面绘制

v2 screen2world(int x, int y); //屏幕坐标到世界坐标的转换

void keyboard(unsigned char key, int x, int y);//键盘输入

void motion(int x, int y);//鼠标拖动事件

void mouse(int button, int state, int x, int y);//鼠标点击事件

void reshape(int width, int height);//窗口大小改变事件

void idle();//绘制

//OpenGL初始化配置
void myOpenGLInit(int *argc, char** argv, //带命令行的输入
				int width = 800, int height = 600, //窗口大小
				int posX = 50, int posY = 50      //窗口位置
				);


//画多边形
void drawPolygon(const c2d_polygon & polygon);

//画碰撞情况
void draw_collision(collision &c);

//没有事件输入时候使用
void entry(int state);

#endif
