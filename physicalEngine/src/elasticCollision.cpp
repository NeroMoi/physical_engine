#include"elasticCollision.h"
#include"globalVariable.h"


//模板函数
template<typename ContainerT, typename PredicateT>
void erase_if(ContainerT &items, const PredicateT &predicate) {
	for (auto it = items.begin(); it != items.end();)
	{
		if (predicate(*it))
		{
			it = items.erase(it);
		}
		else
		{
			++it;
		}
	}
}

// 碰撞计算准备
void collision_prepare(collision &c) //每一次碰撞的集合
{
	static const decimal kBiasFactor = 0.2;
	const auto &a = *c.bodyA;
	const auto &b = *c.bodyB;

	auto tangent = c.N.normal(); // 接触面

								 // 先计算好碰撞系数相关的量
								 // 法向力防止物体穿透
								 // 切向力模拟摩擦力
	for (auto &contact : c.contacts)
	{
		auto nA = contact.ra.cross(c.N);//法线方向，撞击点到重心的矢量 与 撞击点所在直线上的(单位法线）的叉积
		auto nB = contact.rb.cross(c.N);

		auto kn = a.mass.inv + b.mass.inv +   // >=0 ，=0的情况是a,b两个物体都是静态物体
			std::abs(a.inertia.inv) * nA * nA +
			std::abs(b.inertia.inv) * nB * nB;

		contact.mass_normal = kn > 0 ? COLL_NORMAL_SCALE / kn : 0.0;

		auto tA = contact.ra.cross(tangent);//切线方向
		auto tB = contact.rb.cross(tangent);

		auto kt = a.mass.inv + b.mass.inv +
			std::abs(a.inertia.inv) * tA * tA +
			std::abs(b.inertia.inv) * tB * tB;

		contact.mass_tangent = kt > 0 ? COLL_TANGENT_SCALE / kt : 0.0;

		contact.bias = -kBiasFactor * dt_inv * std::min(0.0, contact.sep);
	}
}

// 碰撞计算
void collision_update(collision &c) //每一次碰撞
{
	auto &a = *c.bodyA;
	auto &b = *c.bodyB;
	auto tangent = c.N.normal(); // 接触面

	for (auto &contact : c.contacts) //每一个碰撞点
	{
		/* ab两物体的速度差，由线速度和撞击点切面方向上的角速度组成*/
		auto dv = (b.V + (-b.angleV * contact.rb.N())) -   //contact.rb.N()->重心到切点方向的矢量 顺时针旋转90度的单位矢量
			(a.V + (-a.angleV * contact.ra.N()));

		// 法向力
		auto vn = dv.dot(c.N); //速度差矢量，在撞击方向上的投影

		auto dpn = (-vn + contact.bias) * contact.mass_normal;//靠近与离开的比率

		if (contact.pn + dpn < 0) //撞击后，反向离开
		{
			dpn = -contact.pn;
		}

		// 切向力
		auto vt = dv.dot(tangent); //摩擦力
		auto dpt = -vt * contact.mass_tangent;

		auto friction = sqrt(a.f * b.f) * contact.pn;// f= μN

		dpt = std::max(-friction, std::min(friction, contact.pt + dpt)) - contact.pt;


		a.update(0);
		b.update(0); // 初始化力和力矩

		auto p = dpn * c.N + dpt * tangent;

		a.impulse(-p, contact.ra);
		b.impulse(p, contact.rb);

		contact.pn += dpn;
		contact.pt += dpt;

		a.update(1);
		b.update(1); // 计算力和力矩，得出速度和角速度
	}
}

// 去除休眠物体的碰撞
void collision_remove_sleep() {
	erase_if(collisions, [&](auto &c) {
		if (c.second.bodyA->statics)
			return c.second.bodyB->sleep;
		if (c.second.bodyB->statics)
			return c.second.bodyA->sleep;
		return c.second.bodyA->sleep && c.second.bodyB->sleep;
	});
}


