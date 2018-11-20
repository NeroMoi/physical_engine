#include"createPolygon.h"

 c2d_polygon *make_polygon(decimal mass, const std::vector<v2> &vertices, const v2 &pos, bool statics,decimal angle ) 
 {

	auto polygon = std::make_unique<c2d_polygon>(global_id++, mass, vertices); //��̬����һ�����󣬷���ָ��ö����ָ��
	polygon->pos = pos;
	polygon->angle = angle;
	polygon->refresh();

	auto obj = polygon.get();//���ر����ָ��
	if (statics) //�Ƿ��Ǿ�̬����
	{
		polygon->mass.set(inf);//�������������
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
		// �����ĸ����㣬��ʱ������
		{ w / 2,  h / 2 },
		{ -w / 2, h / 2 },
		{ -w / 2, -h / 2 },
		{ w / 2,  -h / 2 }
	};
	return make_polygon(mass, vertices, pos, statics,angle);//��������
}


