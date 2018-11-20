#include"createPolygon.h"

 c2d_polygon *make_polygon(decimal mass, const std::vector<v2> &vertices, const v2 &pos, bool statics,decimal angle ) 
 {

	auto polygon = std::make_unique<c2d_polygon>(global_id++, mass, vertices); //动态创建一个对象，返回指向该对象的指针
	polygon->pos = pos;
	polygon->angle = angle;
	polygon->refresh();

	auto obj = polygon.get();//返回保存的指针
	if (statics) //是否是静态物体
	{
		polygon->mass.set(inf);//物体质量无穷大
		polygon->init_static();
		static_bodies.push_back(std::move(polygon));
	}
	else 
	{
		bodies.push_back(std::move(polygon));
	}
	return obj;
}

 c2d_polygon *make_rect(decimal mass, decimal w, decimal h, const v2 &pos, bool statics ,decimal angle ) 
 {
	std::vector<v2> vertices = 
	{ 
		// 设置四个顶点，逆时针排序
		{ w / 2,  h / 2 },
		{ -w / 2, h / 2 },
		{ -w / 2, -h / 2 },
		{ w / 2,  -h / 2 }
	};
	return make_polygon(mass, vertices, pos, statics,angle);//创建矩形
}


