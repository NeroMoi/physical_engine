/*
*�ļ����ƣ�rigidbody
*�ļ���ʶ��
*ժҪ�����������Ĵ���
*
*��ǰ�汾��v1.0
*���ߣ�Nero
*������ڣ�
*/


#ifndef __RIGIDBODY_H__
#define __RIGIDBODY_H__

#include<memory>
#include<vector>
#include<algorithm> 
#include"bivector.h"
#include<iostream>

// �����󸡵����ĵ���
// �����ĸΪ0 �����
// �����ĸΪ��������
//
struct decimal_inv
{
	decimal value{ 0 }, inv{ 0 }; //inv��Ϊ��õ�����value��Ϊ��ĸ

								  //��ʾ����
	explicit decimal_inv(decimal v);


	//���õ�����������ĸ�Ŀɿ���
	void set(decimal v);

};



//������࣬����Ϊ��̬����������õ��ڴ����
class c2d_body {

	public:
		using ptr = std::unique_ptr<c2d_body>;

		c2d_body(uint16_t _id, decimal _mass) :id(_id), mass(_mass) {}
		c2d_body(const c2d_body&) = delete;//��ֹ����
		c2d_body &operator=(const c2d_body &) = delete;//��ֹ��ֵ
		virtual ~c2d_body() = default;

		virtual void drag(const v2&pt, const v2 &offset) = 0;//�϶���ʩ������
		virtual bool contains(const v2&pt) = 0;//�Ƿ��������������
		virtual void impulse(const v2 &p, const v2 &r) = 0; //�������
		/*
		*�ֽ׶θ��¸�������
		*i=0,��������
		*i=1����һ�׶Σ������ٶȡ����ٶ�
		*i=2,�ڶ��׶Σ�����λ�õ������
		*/
		virtual void update(int) = 0;//״̬����
		virtual void draw() = 0;//����

		bool sleep{ false };//�Ƿ�����

		bool statics {false};//�Ƿ�Ϊ��̬����
		int collision{ 0 };//������ײ�Ĵ���
		uint16_t id{ 0 };//����ID
		decimal_inv mass{ 1 };//��������
		v2 pos;//λ��
		v2 center;//����
		v2 V;//�ٶ�
		decimal angle{ 0 };//�Ƕ�
		decimal angleV{ 0 };//������
		decimal_inv inertia{ 0 };//ת������
		decimal f{ 1 };//����/��Ħ��ϵ��
		m2 R;//�����Ӧ����ת����
		v2 F;//�������ܵ���
		v2 Fa; // �������ۼƺ�����
		decimal M{ 0 }; // ����

};

//����θ��壨����ֻ����͹����Σ��㼯λ��ʱ������

class c2d_polygon:public c2d_body
{
	public:

		/*
		*��������������id,��������������
		*/
		c2d_polygon(uint16_t _id, decimal _mass, const std::vector<v2>& _vertices) :
			c2d_body(_id, _mass), vertices(_vertices), verticesWorld(_vertices)
		{
			//�������ġ�����ת������������߽�
			init();
		}

		

		~c2d_polygon();

		//�����������
		static decimal calc_polygon_area(const std::vector<v2>&vertices);
		
		//������������
		static v2 calc_polygon_centroid(const std::vector<v2>&vertices);

		//��������ת������
		static decimal calc_polygon_inertia(decimal mass, const std::vector<v2>&vertices);

		//����߽磨���ΰ�Χ��
		void calc_bounds();
		
		//�ж��Ƿ��ڱ߽���->�϶�ʹ��
		bool contains_in_bound(const v2 &pt);

		//�ж��Ƿ��ڶ�����ڲ�
		bool contains_in_polygon(const v2 &pt);

		//�жϵ��Ƿ��ڰ�Χ�򡢶�����ڲ�
		bool contains(const v2 &pt) override;
	
		/*
		 * ��������
		 * ����ת������
		 * ����߽�AABB
		 */
		void init();

		void refresh(); //����������ת��Ϊ��������
		
		void init_static();//�Ƿ��Ǿ�̬����

		//����
		void impulse(const v2 &p, const v2 &r) override;

		void update(int n)override;

		//��ʼ����������Ϊ0
		void pass0();

		//���㵱ǰʱ�̵��ٶȺͽ��ٶ�
		void pass1();
		
		//������������ϵ�µģ�λ�ã��ͽǶ�
		void pass2();

		//���¶���
		void pass3();

		//��ʼ������
		void pass4();

		// ���������ٶȡ����ٶ�Ϊ��ʱ���ж�����
		void pass5();

		//��ҷ����
		void drag(const v2 &pt, const v2 &offset) override;

		//��ͼ
		void draw() override;

		//��idxΪ��㣬��һ����Ϊ�յ������
		v2 edge(size_t idx)const;

		v2 &vertex(size_t idx);//���ض��㣬��������ϵ��

		size_t index(size_t idx) const;

		size_t edges()const;//��������ϵ�£����������
		

		std::vector<v2> vertices;//����εĶ��㣨�������꣩
		std::vector<v2> verticesWorld;//��������
		v2 boundMin, boundMax;//�������
};


#endif
