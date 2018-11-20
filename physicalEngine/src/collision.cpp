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
	return idxA == other.idxB && idxB == other.idxA; // �Ƿ���
}

bool contact::operator!=(const contact &other) const
{
	return !(*this == other);
}


//������ײid
uint32_t make_id(uint16_t a, uint16_t b)
{
	return (std::min(a,b)<<16)|(std::max(a,b));
}

/* ��ײ��� SAT������
*   ע�����⣺
*   1.��������εĵ����ж��������
*   2.������ѡ��Ϊ�߶�Ӧ�ķ���
*   3. �����з��򣬷��������ͶӰΪ��ֵ��ֵ�Ĺ�ϵ
*   4. �����߳��������ڲ��������ڵ�B�����е���A��ĳ��������ߣ����䷨���ϵ�ͶӰ��С��0ʱ��˵�����뽻
*	4.1�����߳��������ⲿ�������ڵ�B�����е���A��ĳ��������ߣ����䷨���ϵ�ͶӰ������0ʱ��˵�����뽻
*/
bool max_separating_axis(c2d_polygon *a, c2d_polygon *b, decimal &separation, size_t &idx)
{
	separation = -inf;//Ĭ������϶Ϊһ����ֵ���Թ��Ƚ�

	//������������A�����ж���
	for (size_t i = 0; i < a->edges(); ++i)
	{
		//���A���������������
		auto va = a->verticesWorld[i];
		//��õ�ǰ���㵽��һ����ıߵ�λ������
		auto N = a->edge(i).normal();
		//��С��������
		auto min_sep = inf;

		//������������B
		for (size_t j = 0; j < b->edges(); ++j) //�ҳ�AB���ɵ�������AA'�ϵ�ͶӰ��С�ļ�϶
		{
			//���B���������������
			auto vb = b->vertex(j);
			// vb - va = �Ӷ���A������B������
			// normal  = �Ӷ���A������A'�ĵ�λ������
			// dot(vb - va, normal) = ����B����AA'ͶӰΪP�����ΪAP�ĳ���
			// ��������ȡ��Сֵ�����
			// min_sep = ��AA'�ߵķ�����NΪ�ᣬ��B�������������ͶӰ����С����
			// ���

			min_sep = std::min(min_sep,(vb - va).dot(N) ); 
		
		}
		if (min_sep > separation)//�ҳ�B��A���ɵ���������A�и��߶�Ӧ���������ļ�϶��������0��˵�����ཻ
		{
			separation = min_sep; //���в�ͬ����ͶӰ�����ļ�϶

			idx = i;//����϶��Ӧ�ıߣ������������ײ����Ϊ��С�ĸ�������Ϊ������ײ�������߽Ӵ����
		}


	}
	return separation > 0; //��A�д���һ�������ᣬʹ��B�����ϵ�ͶӰ����AͶӰ�����䣬�����ཻ

}

//AABB�����ж���ײ
// ����true Ϊ�ཻ
bool AABB_collide(c2d_polygon * a, c2d_polygon *b)
{
	auto centerA = (a->boundMin + a->boundMax) / 2;//���ΰ�ΧȦ���ĵ�
	auto centerB = (b->boundMin + b->boundMax) / 2;

	auto sizeA = (a->boundMax - a->boundMin) / 2;//ԭ���ΰ�ΧȦ1/4�Ĵ�С������Ϊԭ����һ��
	auto sizeB = (b->boundMax - b->boundMin) / 2;

	return (std::abs(centerB.x - centerA.x) <= (sizeA.x + sizeB.x)) && // �������ĵ�ľ���С�ھ��μ�ľ���
			(std::abs(centerB.y - centerA.y) <= (sizeA.y + sizeB.y));
}

/*�ҵ�B����A��������ı�
* �����ߵĵ����С����Ϊ����
* �ʺϣ���ײ�����岻�ܴ�͸����ʵ�����
* �����Ǵ�͸Ч�������ܵ�����С
*/
static size_t incident_edge(const v2 &N, c2d_polygon *body)
{
	size_t idx = SIZE_MAX;
	auto min_dot = inf;

	//����B����ı�
	for (size_t i = 0; i < body->edges(); ++i)
	{
		//��ñ��ϵķ�����
		auto edge_normal = body->edge(i).normal();

		auto dot = edge_normal.dot(N);
		  
		//�ҳ���СͶӰ������С��϶
		if (dot < min_dot)
		{
			min_dot = dot;//��С��϶
			idx = i; //��������
		}
	}

	return idx;

}

/*Sutherland-Hodgman������βü���(��߲ü���)
 *https://blog.csdn.net/damotiansheng/article/details/43274183
 *
 */
size_t clip(std::vector<contact> &out,//�����A�ڵĵ�
	const std::vector<contact> &in, //����㼯��B��Ϊ�����������
	size_t i,                   //A�������ü���������
	const v2 &p1, const v2 &p2) //��Ϊ�ü��� �������ߵ������˵�
{

	size_t num_out = 0;
	auto N = (p2 - p1).normal();//��Ϊ�ü��ߵ��������ϵķ���

	//����ͶӰ
	auto dist0 = N.dot(in[0].pos - p1); //�б�B���ü��ߵ������˵��Ƿ���A��
	auto dist1 = N.dot(in[1].pos - p1);

	//1.���ͶӰ��С��0,��B�����㶼��A�ڣ����������
	//2. ͶӰ������0��������A�������,s
	//3. ���㽻�㣬�����A�ڵĵ�
	//
	if (dist0 <= 0)//�����ߵ������A�ڣ�
	{
		out[num_out++] = in[0];
	}

	if (dist1 <= 0)//�����ߵ��յ���A��
	{
		out[num_out++] = in[1];
	}

	//��������һ����A�ڣ�һ����A��
	//��ʱ����Ҫ���㽻��
	if (dist0*dist1 < 0)
	{
		//������ʣ�B�߶���A����ı���
		auto interp = dist0 / (dist0 - dist1); //������Ϊ��һ��һ�������ü�����Ҳ��д�� abs(dist0)/(abs(dist0) +abs(dist1))
		//����p1,p2��in1,in2����

		out[num_out].pos = in[0].pos + interp*(in[1].pos - in[0].pos);//�ڶ���Ϊ����߽��������
		out[num_out].idxA = -(int)i -1;//���������ڵģ�����a��b�ı�����+1��Ϊ��������B��Ϊ������A
		++num_out;
	}

	return num_out;//���ص����Ŀ����С��2��˵���������㶼��A��
}

bool solve_collision(collision &c)
{
	if (c.satA < c.satB) //Ŀ������˵����˭��������ײ��������(satֵ�������)
	{// ���У�A��B��SAT���󣬸��ӽ���
		std::swap(c.bodyA, c.bodyB); /*����Ԫ��*/
		std::swap(c.idxA, c.idxB);
		std::swap(c.satA, c.satB);
	}  

	auto bodyA = dynamic_cast<c2d_polygon *>(c.bodyA); //�����ǽϴ�sat��Ӧ������ ��Ϊ����
	auto bodyB = dynamic_cast<c2d_polygon *>(c.bodyB);

	// ����SAT���ᷨ��
	// edge = A������B��������ı�
	// N = edge�ķ��ߣ�ָ��B����
	c.N = bodyA->edge(c.idxA).normal();

	// ��ʱҪ�ҵ�B��������A��������ıߣ�A�ķ��ߣ�b�ĵ�
	c.idxB = incident_edge(c.N, bodyB); //����������B���������ײ��������

	//�Ӵ��㼯��
	decltype(c.contacts) contacts;


	// �ٶ������Ӵ��㣨��idxB���˵㣩

	contacts.emplace_back(bodyB->vertex(c.idxB), bodyB->index(c.idxB) + 1);
	contacts.emplace_back(bodyB->vertex(c.idxB + 1), bodyB->index(c.idxB + 1) + 1);

	auto tmp = contacts;//�ԽӴ��㼯�ϵĿ���

	// ��idxB�߶ΰ�bodyA���ж���βü�
	for (size_t i = 0; i < bodyA->edges(); ++i) 
	{
		if (i == c.idxA)//�Ӵ��������������
		{
			continue;
		}
		if (clip(tmp, contacts, i, bodyA->vertex(i), bodyA->vertex(i + 1)) < 2)//��Ϊ����������㶼��A�⣬������
		{
			return false;
		}
		contacts = tmp; //�Ѹղŵ��������Ϊ����㣬�����ж�
	}
	// ���Ųü�idxA��
	auto va = bodyA->vertex(c.idxA);

	// ɸѡ����
	for (auto &contact : contacts) 
	{
		// ���㣺contact.pos
		// �ο��㣺�Ӵ��߶˵�va
		// �Ӵ��߷�������ָ������B��
		auto sep = (contact.pos - va).dot(c.N);
		if (sep <= 0)//ײ������A�ڻ����

		{ // ����idxA��bodyAһ��ģ�bodyA�ڵĽӴ��㣩
			contact.sep = sep; // sepԽС���˵�va������pos�����߶ε�б��Խ�ӽ�����N (�����ľ���)
			contact.ra = contact.pos - c.bodyA->pos - c.bodyA->center;//�ýӴ��㵽AB���������ĵ�����������������ת����
			contact.rb = contact.pos - c.bodyB->pos - c.bodyB->center;//r
			c.contacts.push_back(contact);
		}
	}

	return true;

}

//����������ײ
void collision_detection(const c2d_body::ptr &a, c2d_body::ptr &b)
{
	auto bodyA = dynamic_cast<c2d_polygon *>(a.get()); //����ָ����������ָ�붯̬ת������Ϊԭ������ָ����ָ
													  //�������������������ת���ǳɹ���

	auto bodyB = dynamic_cast<c2d_polygon *>(b.get());

	decimal satA, satB;//ͶӰ���Ӧ�����ļ�϶  ->��ײ���ͶӰ

	size_t idxA, idxB;//��Ӧ����϶��Ӧ�������� ->Ҳ�����ǲ�����ײ��������

	auto id = make_id(bodyA->id, bodyB->id);//�������id 

											//���ж��Ƿ���ײ��û����ײ�Ļ������֮ǰ������ײ��֮ǰ����ײ����ɾ����ײ������ʵ����������ײ��ķ�����̣�
	if(!AABB_collide(bodyA,bodyB)|| //���Ѿ��ж�������Χ���ཻ������£�����������ж�
		max_separating_axis(bodyA,bodyB,satA,idxA)|| //��A����ײ�ıߣ��Լ�ͶӰ������(��������ཻ��Ϊ������-1>-2,���Ա���������˼)
		max_separating_axis(bodyB, bodyA, satB, idxB) //��b����ײ�ıߣ��Լ�ͶӰ������
		) //���㲻��ײ����
	{
		auto prev = collisions.find(id); //��������ǰ�Ƿ��й���ײ
		if(prev != collisions.end())//֮ǰ��������ײ,�ֱ�ǲ���ײ
		{
			collisions.erase(prev); //ɾ��֮ǰ����ײ��¼
			bodyA->collision--;//����Χ��������ײ������ȥ1
			bodyB->collision--;
		}

		return;//��������ͶӰ>0 ���ཻ

	}

	//�ཻ��������ײ
	//����ǰ���sat ,idx������ײ����
	else
	{

		//��������ײ������ʵ��ģ����������ײ�Ĺ��̣�������ת��
		// �½���ײ�ṹ

		collision c; //��ʼ��һ����ײ��Ԫ
		c.bodyA = bodyA;
		c.bodyB = bodyB;
		c.idxA = idxA;
		c.idxB = idxB;
		c.satA = satA;
		c.satB = satB;


		auto prev = collisions.find(id);//�鿴��ǰ�Ƿ���ײ��

		if(prev == collisions.end())//֮ǰ����ײ
		{
			
			//��ײ�Ӵ������￪ʼ����
			if (solve_collision(c))//������ײ�㣬�Լ���ײ���Ƿ�Ϸ� 
			{
				collisions.insert(std::make_pair(id, c));//����һ����id��c����ֵ��ʼ����pair,������id��c�ƶϳ���

				//A��Bӵ�е���ײ������1
				bodyA->collision++;
				bodyB->collision++;

				bodyA->sleep = false;//�໥��ײ�����岻�����Ǵ�������̬
				bodyB->sleep = false;

			}


		}

		else //֮ǰ��������ײ
		{

			if (solve_collision(c)) 
			{ // ������ײ��
				collision_update(c, collisions[id]);//�µ���ײ��ɵ���ײ�Ƚ�
				collisions[id] = c;//���¶�����ײ��
			}
			else 
			{ // û����ײ
				collisions.erase(prev);
				bodyA->collision--; // ��ײ������һ
				bodyB->collision--;
			}


		}


	}
}

//��ײ���
void collision_detection()
{
	auto size = bodies.size();

	for (size_t i = 0; i < size; i++)
	{
		if(bodies[i]->sleep)//������Ϊ�㣬��Ϊsleep
		{
			continue;//����
		}

		for (size_t j = 0; j < size; j++)//�����������ظ����
		{
			if(bodies[j]->sleep || i<j)//����̬������̬�����ĽӴ���⣬�Լ���̬�Ծ�̬�����ײ�����
			collision_detection(bodies[i], bodies[j]);
		}

		for(auto &body: static_bodies)//�����뾲̬��������ײ
		{
			collision_detection(bodies[i], body);
		}
	}

}

//��ײ����
void collision_update(collision &c, const collision &old_c)
{
	auto &a = *c.bodyA;//�µ���ײ������
	auto &b = *c.bodyB;

	const auto &old_contacts = old_c.contacts;//�ɵ���ײ��������ײ�㼯��
	for (auto &new_contact : c.contacts) //���µ���ײ�е�ÿһ����ײ�㣬���֮ǰ�Ƿ��й�����ײ��
	{
		auto old_contact = std::find(old_contacts.begin(), old_contacts.end(), new_contact);//���Ҿɵ���ײ�Ƿ�������µ���ײ

		if (old_contact != old_contacts.end())
		{ // ͬһ����ײ��ĸ���
			new_contact.pn = old_contact->pn;//�̳���һ��ײ��ĺ��������ߵ���
			new_contact.pt = old_contact->pt;

			auto tangent = c.N.normal(); // �µ����ߣ���ײƽ�� ��A������B��������ıߣ�

			auto p = new_contact.pn * c.N + new_contact.pt * tangent; // �µ�ʸ����

			a.impulse(-p, new_contact.ra); // ʩ������
			b.impulse(p, new_contact.rb);
		}


	}
}







