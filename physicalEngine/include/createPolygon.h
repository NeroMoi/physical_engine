/*
*文件名称：createPolygon
*文件标识：
*摘要：创建多边形
*
*当前版本：v1.0
*作者：Nero
*完成日期：
*/
#ifndef __CREATEPOLYGON_H__
#define __CREATEPOLYGON_H__

#include"globalVariable.h"

//创建多边形
c2d_polygon *make_polygon(decimal mass, const std::vector<v2> &vertices, const v2 &pos, bool statics = false ,decimal angle = 0);

//创建矩形框AABB
c2d_polygon *make_rect(decimal mass, decimal w, decimal h, const v2 &pos, bool statics = false,decimal angle = 0);





#endif
