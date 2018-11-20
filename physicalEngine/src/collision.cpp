#include"collision.h"
#include"globalVariable.h"
#include<utility>
#include<cmath>


bool contact:: operator==(const contact &other) const
{
	if (idxA == other.idxA && idxB == other.idxB)
	{
		return true;
	}
	return idxA == other.idxB && idxB == other.idxA; // 是否反了
}

bool contact::operator!=(const contact &other) const
{
	return !(*this == other);
}


//创建碰撞id
uint32_t make_id(uint16_t a, uint16_t b)
{
	return (std::min(a,b)<<16)|(std::max(a,b));
}

/* 碰撞检测 SAT分离轴
*   注意点理解：
*   1.两个多边形的点排列都是有序的
*   2.分离轴选择为边对应的法线
*   3. 法线有方向，方向决定了投影为正值或负值的关系
*   4. 当法线朝向物体内部，若存在当B中所有点与A中某个点的连线，在其法线上的投影都小于0时，说明不想交
*	4.1当法线朝向物体外部，若存在当B中所有点与A中某个点的连线，在其法线上的投影都大于0时，说明不想交
*/
bool max_separating_axis(c2d_polygon *a, c2d_polygon *b, decimal &separation, size_t &idx)
{
	separation = -inf;//默认最大间隙为一个极值，以供比较

	//遍历集合物体A的所有顶点
	for (size_t i = 0; i < a->edges(); ++i)
	{
		//获得A各顶点的世界坐标
		auto va = a->verticesWorld[i];
		//获得当前顶点到下一顶点的边单位法向量
		auto N = a->edge(i).normal();
		//最小分离向量
		auto min_sep = inf;

		//遍历几何物体B
		for (size_t j = 0; j < b->edges(); ++j) //找出AB构成的向量在AA'上的投影最小的间隙
		{
			//获得B各顶点的世界坐标
			auto vb = b->vertex(j);
			// vb - va = 从顶点A到顶点B的向量
			// normal  = 从顶点A到顶点A'的单位法向量
			// dot(vb - va, normal) = 若点B到边AA'投影为P，结果为AP的长度
			// 由于这里取最小值，因此
			// min_sep = 以AA'边的法向量N为轴，将B物体各顶点所做投影的最小长度
			// 如果

			min_sep = std::min(min_sep,(vb - va).dot(N) ); 
		
		}
		if (min_sep > separation)//找出B与A构成的向量，在A中各边对应法线上最大的间隙，若大于0则说明不相交
		{
			separation = min_sep; //所有不同方向投影中最大的间隙

			idx = i;//最大间隙对应的边，如果产生了碰撞，则为最小的负数，即为产生碰撞的那条边接触最大
		}


	}
	return separation > 0; //在A中存在一个分离轴，使得B在轴上的投影不再A投影的区间，是则不相交

}

//AABB快速判断碰撞
// 返回true 为相交
bool AABB_collide(c2d_polygon * a, c2d_polygon *b)
{
	auto centerA = (a->boundMin + a->boundMax) / 2;//矩形包围圈中心点
	auto centerB = (b->boundMin + b->boundMax) / 2;

	auto sizeA = (a->boundMax - a->boundMin) / 2;//原矩形包围圈1/4的大小，长宽为原来的一半
	auto sizeB = (b->boundMax - b->boundMin) / 2;

	return (std::abs(centerB.x - centerA.x) <= (sizeA.x + sizeB.x)) && // 两者中心点的距离小于矩形间的距离
			(std::abs(centerB.y - centerA.y) <= (sizeA.y + sizeB.y));
}

/*找到B中离A物体最近的边
* 两法线的点积最小，即为所求
* 适合，碰撞的物体不能穿透（真实情况）
* 若考虑穿透效果，则不能单靠最小
*/
static size_t incident_edge(const v2 &N, c2d_polygon *body)
{
	size_t idx = SIZE_MAX;
	auto min_dot = inf;

	//遍历B物体的边
	for (size_t i = 0; i < body->edges(); ++i)
	{
		//获得边上的法向量
		auto edge_normal = body->edge(i).normal();

		auto dot = edge_normal.dot(N);
		  
		//找出最小投影，即最小间隙
		if (dot < min_dot)
		{
			min_dot = dot;//最小间隙
			idx = i; //返回索引
		}
	}

	return idx;

}

/*Sutherland-Hodgman（多边形裁剪）(逐边裁剪法)
 *https://blog.csdn.net/damotiansheng/article/details/43274183
 *
 */
size_t clip(std::vector<contact> &out,//输出在A内的点
	const std::vector<contact> &in, //输入点集，B作为交点的两个点
	size_t i,                   //A中用来裁剪的那条边
	const v2 &p1, const v2 &p2) //作为裁剪线 的那条边的两个端点
{

	size_t num_out = 0;
	auto N = (p2 - p1).normal();//作为裁剪线的这条边上的法线

	//计算投影
	auto dist0 = N.dot(in[0].pos - p1); //判别B被裁剪边的两个端点是否在A内
	auto dist1 = N.dot(in[1].pos - p1);

	//1.如果投影都小于0,则B中两点都在A内，输出两个点
	//2. 投影都大于0，即都在A外无输出,s
	//3. 计算交点，输出在A内的点
	//
	if (dist0 <= 0)//这条边的起点在A内，
	{
		out[num_out++] = in[0];
	}

	if (dist1 <= 0)//这条边的终点在A内
	{
		out[num_out++] = in[1];
	}

	//否则两点一个在A内，一个在A外
	//这时候需要计算交点
	if (dist0*dist1 < 0)
	{
		//计算比率，B线段在A内外的比例
		auto interp = dist0 / (dist0 - dist1); //这里因为是一正一负所以用减法，也可写成 abs(dist0)/(abs(dist0) +abs(dist1))
		//计算p1,p2与in1,in2交点

		out[num_out].pos = in[0].pos + interp*(in[1].pos - in[0].pos);//第二点为边与边交点的坐标
		out[num_out].idxA = -(int)i -1;//（交点属于的）物体a和b的边索引+1，为正则属于B，为负属于A
		++num_out;
	}

	return num_out;//返回点的数目，若小于2这说明，两个点都在A外
}

bool solve_collision(collision &c)
{
	if (c.satA < c.satB) //目的在于说明，谁才是主动撞击的物体(sat值大的物体)
	{// 排列：A比B的SAT更大，更接近零
		std::swap(c.bodyA, c.bodyB); /*交换元素*/
		std::swap(c.idxA, c.idxB);
		std::swap(c.satA, c.satB);
	}  

	auto bodyA = dynamic_cast<c2d_polygon *>(c.bodyA); //这里是较大sat对应的物体 皆为负数
	auto bodyB = dynamic_cast<c2d_polygon *>(c.bodyB);

	// 计算SAT的轴法线
	// edge = A物体离B物体最近的边
	// N = edge的法线，指向B物体
	c.N = bodyA->edge(c.idxA).normal();

	// 此时要找到B物体中离A物体最近的边，A的法线，b的点
	c.idxB = incident_edge(c.N, bodyB); //这里计算出来B物体参与碰撞的那条边

	//接触点集合
	decltype(c.contacts) contacts;


	// 假定两个接触点（即idxB两端点）

	contacts.emplace_back(bodyB->vertex(c.idxB), bodyB->index(c.idxB) + 1);
	contacts.emplace_back(bodyB->vertex(c.idxB + 1), bodyB->index(c.idxB + 1) + 1);

	auto tmp = contacts;//对接触点集合的拷贝

	// 将idxB线段按bodyA进行多边形裁剪
	for (size_t i = 0; i < bodyA->edges(); ++i) 
	{
		if (i == c.idxA)//接触的那条边最后处理
		{
			continue;
		}
		if (clip(tmp, contacts, i, bodyA->vertex(i), bodyA->vertex(i + 1)) < 2)//作为交点的两个点都在A外，不合理
		{
			return false;
		}
		contacts = tmp; //把刚才的输出点作为输入点，进行判断
	}
	// 最后才裁剪idxA边
	auto va = bodyA->vertex(c.idxA);

	// 筛选交点
	for (auto &contact : contacts) 
	{
		// 交点：contact.pos
		// 参考点：接触边端点va
		// 接触边法向量（指向物体B）
		auto sep = (contact.pos - va).dot(c.N);
		if (sep <= 0)//撞击点在A内或边上

		{ // 找在idxA向bodyA一侧的（bodyA内的接触点）
			contact.sep = sep; // sep越小，端点va到交点pos所成线段的斜率越接近法线N (穿过的距离)
			contact.ra = contact.pos - c.bodyA->pos - c.bodyA->center;//该接触点到AB两物体重心的向量，用来计算旋转力矩
			contact.rb = contact.pos - c.bodyB->pos - c.bodyB->center;//r
			c.contacts.push_back(contact);
		}
	}

	return true;

}

//两物体间的碰撞
void collision_detection(const c2d_body::ptr &a, c2d_body::ptr &b)
{
	auto bodyA = dynamic_cast<c2d_polygon *>(a.get()); //基类指针向派生类指针动态转换，因为原来基类指针所指
													  //对象是派生类对象，所以转型是成功的

	auto bodyB = dynamic_cast<c2d_polygon *>(b.get());

	decimal satA, satB;//投影后对应的最大的间隙  ->碰撞后的投影

	size_t idxA, idxB;//对应最大间隙对应的那条边 ->也可能是产生碰撞的那条边

	auto id = make_id(bodyA->id, bodyB->id);//创建组合id 

											//线判断是否碰撞，没有碰撞的话，检测之前有无碰撞，之前有碰撞，便删除碰撞（用于实现两物体碰撞后的分离过程）
	if(!AABB_collide(bodyA,bodyB)|| //在已经判断轴对齐包围盒相交的情况下，进行下面的判断
		max_separating_axis(bodyA,bodyB,satA,idxA)|| //在A中碰撞的边，以及投影最大距离(这里如果相交，为负数，-1>-2,所以表达最近的意思)
		max_separating_axis(bodyB, bodyA, satB, idxB) //在b中碰撞的边，以及投影最大距离
		) //满足不碰撞条件
	{
		auto prev = collisions.find(id); //查找下先前是否有过碰撞
		if(prev != collisions.end())//之前产生过碰撞,现标记不碰撞
		{
			collisions.erase(prev); //删除之前的碰撞记录
			bodyA->collision--;//与周围物体间的碰撞次数减去1
			bodyB->collision--;
		}

		return;//最大分离轴投影>0 不相交

	}

	//相交，产生碰撞
	//利用前面的sat ,idx继续求撞击点
	else
	{

		//发生了碰撞，用于实现模拟两物体碰撞的过程，动量的转移
		// 新建碰撞结构

		collision c; //初始化一个碰撞单元
		c.bodyA = bodyA;
		c.bodyB = bodyB;
		c.idxA = idxA;
		c.idxB = idxB;
		c.satA = satA;
		c.satB = satB;


		auto prev = collisions.find(id);//查看先前是否碰撞过

		if(prev == collisions.end())//之前无碰撞
		{
			
			//碰撞接触点这里开始计算
			if (solve_collision(c))//计算碰撞点，以及碰撞点是否合法 
			{
				collisions.insert(std::make_pair(id, c));//返回一个用id和c进行值初始化的pair,类型由id和c推断出来

				//A和B拥有的碰撞次数加1
				bodyA->collision++;
				bodyB->collision++;

				bodyA->sleep = false;//相互碰撞的物体不让他们处于休眠态
				bodyB->sleep = false;

			}


		}

		else //之前产生过碰撞
		{

			if (solve_collision(c)) 
			{ // 计算碰撞点
				collision_update(c, collisions[id]);//新的碰撞与旧的碰撞比较
				collisions[id] = c;//重新定义碰撞点
			}
			else 
			{ // 没有碰撞
				collisions.erase(prev);
				bodyA->collision--; // 碰撞次数减一
				bodyB->collision--;
			}


		}


	}
}

//碰撞检测
void collision_detection()
{
	auto size = bodies.size();

	for (size_t i = 0; i < size; i++)
	{
		if(bodies[i]->sleep)//合外力为零，则为sleep
		{
			continue;//跳过
		}

		for (size_t j = 0; j < size; j++)//避免两两间重复检测
		{
			if(bodies[j]->sleep || i<j)//休眠态与休眠态物体间的接触检测，以及动态对静态物体的撞击检测
			collision_detection(bodies[i], bodies[j]);
		}

		for(auto &body: static_bodies)//物体与静态物体间的碰撞
		{
			collision_detection(bodies[i], body);
		}
	}

}

//碰撞计算
void collision_update(collision &c, const collision &old_c)
{
	auto &a = *c.bodyA;//新的碰撞的物体
	auto &b = *c.bodyB;

	const auto &old_contacts = old_c.contacts;//旧的碰撞产生的碰撞点集合
	for (auto &new_contact : c.contacts) //对新的碰撞中的每一个碰撞点，检测之前是否有过该碰撞点
	{
		auto old_contact = std::find(old_contacts.begin(), old_contacts.end(), new_contact);//查找旧的碰撞是否包含该新的碰撞

		if (old_contact != old_contacts.end())
		{ // 同一个碰撞点的更新
			new_contact.pn = old_contact->pn;//继承上一碰撞点的横向与切线的力
			new_contact.pt = old_contact->pt;

			auto tangent = c.N.normal(); // 新的切线，碰撞平面 （A物体离B物体最近的边）

			auto p = new_contact.pn * c.N + new_contact.pt * tangent; // 新的矢量力

			a.impulse(-p, new_contact.ra); // 施加力矩
			b.impulse(p, new_contact.rb);
		}


	}
}







