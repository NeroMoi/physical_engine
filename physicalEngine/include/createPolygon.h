/*
*�ļ����ƣ�createPolygon
*�ļ���ʶ��
*ժҪ�����������
*
*��ǰ�汾��v1.0
*���ߣ�Nero
*������ڣ�
*/
#ifndef __CREATEPOLYGON_H__
#define __CREATEPOLYGON_H__

#include"globalVariable.h"

//���������
c2d_polygon *make_polygon(decimal mass, const std::vector<v2> &vertices, const v2 &pos, bool statics = false ,decimal angle = 0);

//�������ο�AABB
c2d_polygon *make_rect(decimal mass, decimal w, decimal h, const v2 &pos, bool statics = false,decimal angle = 0);





#endif
