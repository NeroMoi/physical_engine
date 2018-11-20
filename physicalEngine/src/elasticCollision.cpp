#include"elasticCollision.h"
#include"globalVariable.h"


//ģ�庯��
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

// ��ײ����׼��
void collision_prepare(collision &c) //ÿһ����ײ�ļ���
{
	static const decimal kBiasFactor = 0.2;
	const auto &a = *c.bodyA;
	const auto &b = *c.bodyB;

	auto tangent = c.N.normal(); // �Ӵ���

								 // �ȼ������ײϵ����ص���
								 // ��������ֹ���崩͸
								 // ������ģ��Ħ����
	for (auto &contact : c.contacts)
	{
		auto nA = contact.ra.cross(c.N);//���߷���ײ���㵽���ĵ�ʸ�� �� ײ��������ֱ���ϵ�(��λ���ߣ��Ĳ��
		auto nB = contact.rb.cross(c.N);

		auto kn = a.mass.inv + b.mass.inv +   // >=0 ��=0�������a,b�������嶼�Ǿ�̬����
			std::abs(a.inertia.inv) * nA * nA +
			std::abs(b.inertia.inv) * nB * nB;

		contact.mass_normal = kn > 0 ? COLL_NORMAL_SCALE / kn : 0.0;

		auto tA = contact.ra.cross(tangent);//���߷���
		auto tB = contact.rb.cross(tangent);

		auto kt = a.mass.inv + b.mass.inv +
			std::abs(a.inertia.inv) * tA * tA +
			std::abs(b.inertia.inv) * tB * tB;

		contact.mass_tangent = kt > 0 ? COLL_TANGENT_SCALE / kt : 0.0;

		contact.bias = -kBiasFactor * dt_inv * std::min(0.0, contact.sep);
	}
}

// ��ײ����
void collision_update(collision &c) //ÿһ����ײ
{
	auto &a = *c.bodyA;
	auto &b = *c.bodyB;
	auto tangent = c.N.normal(); // �Ӵ���

	for (auto &contact : c.contacts) //ÿһ����ײ��
	{
		/* ab��������ٶȲ�����ٶȺ�ײ�������淽���ϵĽ��ٶ����*/
		auto dv = (b.V + (-b.angleV * contact.rb.N())) -   //contact.rb.N()->���ĵ��е㷽���ʸ�� ˳ʱ����ת90�ȵĵ�λʸ��
			(a.V + (-a.angleV * contact.ra.N()));

		// ������
		auto vn = dv.dot(c.N); //�ٶȲ�ʸ������ײ�������ϵ�ͶӰ

		auto dpn = (-vn + contact.bias) * contact.mass_normal;//�������뿪�ı���

		if (contact.pn + dpn < 0) //ײ���󣬷����뿪
		{
			dpn = -contact.pn;
		}

		// ������
		auto vt = dv.dot(tangent); //Ħ����
		auto dpt = -vt * contact.mass_tangent;

		auto friction = sqrt(a.f * b.f) * contact.pn;// f= ��N

		dpt = std::max(-friction, std::min(friction, contact.pt + dpt)) - contact.pt;


		a.update(0);
		b.update(0); // ��ʼ����������

		auto p = dpn * c.N + dpt * tangent;

		a.impulse(-p, contact.ra);
		b.impulse(p, contact.rb);

		contact.pn += dpn;
		contact.pt += dpt;

		a.update(1);
		b.update(1); // �����������أ��ó��ٶȺͽ��ٶ�
	}
}

// ȥ�������������ײ
void collision_remove_sleep() {
	erase_if(collisions, [&](auto &c) {
		if (c.second.bodyA->statics)
			return c.second.bodyB->sleep;
		if (c.second.bodyB->statics)
			return c.second.bodyA->sleep;
		return c.second.bodyA->sleep && c.second.bodyB->sleep;
	});
}


