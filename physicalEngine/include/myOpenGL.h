/*
*�ļ����ƣ�myOpenGL
*�ļ���ʶ��
*ժҪ��ʵ��OpenGL����Ⱦ����
*
*��ǰ�汾��v1.0
*���ߣ�Nero
*������ڣ�
*/

//#pragma comment(linker, "/subsystem:\"windows\" /entry:\"mainCRTStartup\"") //ȥ������̨

#ifndef __MYOPENGL_H__
#define __MYOPENGL_H__


#include<GL/freeglut.h> //OpenGL��
#include"rigidBody.h" //����
#include"collision.h"

c2d_body *find_body(const v2 &pos);//Ѱ�ұ��϶�������

static void c2d_step();//��ͼ��������

void c2d_move(const v2 &v);//�ƶ�����

void c2d_rotate(decimal d);//��ת����

void c2d_offset(const v2 &pt, const v2 &offset);//�϶�����

void clear();//�������

void make_bound();//�������ܱ߽�

void make_bound_special();//�����ض��߽�

void scene(int i);//ѡ�񳡾�

void init();//��ʼ������

static void draw_text(int x, int y, const char*format, ...);//���ֻ���

void display();//�������

v2 screen2world(int x, int y); //��Ļ���굽���������ת��

void keyboard(unsigned char key, int x, int y);//��������

void motion(int x, int y);//����϶��¼�

void mouse(int button, int state, int x, int y);//������¼�

void reshape(int width, int height);//���ڴ�С�ı��¼�

void idle();//����

//OpenGL��ʼ������
void myOpenGLInit(int *argc, char** argv, //�������е�����
				int width = 800, int height = 600, //���ڴ�С
				int posX = 50, int posY = 50      //����λ��
				);


//�������
void drawPolygon(const c2d_polygon & polygon);

//����ײ���
void draw_collision(collision &c);

//û���¼�����ʱ��ʹ��
void entry(int state);

#endif
