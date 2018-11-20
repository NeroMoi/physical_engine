/*
*文件名称：rigidbody
*文件标识：
*摘要：刚体相关类的创建
*
*当前版本：v1.0
*作者：Nero
*完成日期：
*/


#ifndef __RIGIDBODY_H__
#define __RIGIDBODY_H__

#include<memory>
#include<vector>
#include<algorithm> 
#include"bivector.h"
#include<iostream>

// 用于求浮点数的倒数
// 解决分母为0 的情况
// 解决分母为无穷的情况
//
struct decimal_inv
{
	decimal value{ 0 }, inv{ 0 }; //inv作为求得的数，value作为分母

								  //显示构造
	explicit decimal_inv(decimal v);


	//设置倒数，并检查分母的可靠性
	void set(decimal v);

};



//刚体基类，该类为动态创建，因此用到内存管理
class c2d_body {

	public:
		using ptr = std::unique_ptr<c2d_body>;

		c2d_body(uint16_t _id, decimal _mass) :id(_id), mass(_mass) {}
		c2d_body(const c2d_body&) = delete;//禁止拷贝
		c2d_body &operator=(const c2d_body &) = delete;//禁止赋值
		virtual ~c2d_body() = default;

		virtual void drag(const v2&pt, const v2 &offset) = 0;//拖动，施加力矩
		virtual bool contains(const v2&pt) = 0;//是否包含该世界坐标
		virtual void impulse(const v2 &p, const v2 &r) = 0; //计算冲量
		/*
		*分阶段更新刚体数据
		*i=0,重置外力
		*i=1，第一阶段，计算速度、角速度
		*i=2,第二阶段，计算位置等相关量
		*/
		virtual void update(int) = 0;//状态更新
		virtual void draw() = 0;//绘制

		bool sleep{ false };//是否休眠

		bool statics {false};//是否为静态物体
		int collision{ 0 };//参与碰撞的次数
		uint16_t id{ 0 };//物体ID
		decimal_inv mass{ 1 };//物体质量
		v2 pos;//位置
		v2 center;//重心
		v2 V;//速度
		decimal angle{ 0 };//角度
		decimal angleV{ 0 };//角速率
		decimal_inv inertia{ 0 };//转动惯量
		decimal f{ 1 };//滑动/静摩擦系数
		m2 R;//物体对应的旋转矩阵
		v2 F;//物体所受的力
		v2 Fa; // 受力（累计合力）
		decimal M{ 0 }; // 力矩

};

//多边形刚体（这里只是用凸多边形，点集位逆时针排序）

class c2d_polygon:public c2d_body
{
	public:

		/*
		*输入参数：刚体的id,质量，顶点坐标
		*/
		c2d_polygon(uint16_t _id, decimal _mass, const std::vector<v2>& _vertices) :
			c2d_body(_id, _mass), vertices(_vertices), verticesWorld(_vertices)
		{
			//计算重心、计算转动惯量、计算边界
			init();
		}

		

		~c2d_polygon();

		//计算多边形面积
		static decimal calc_polygon_area(const std::vector<v2>&vertices);
		
		//计算多边形重心
		static v2 calc_polygon_centroid(const std::vector<v2>&vertices);

		//计算多边形转动惯量
		static decimal calc_polygon_inertia(decimal mass, const std::vector<v2>&vertices);

		//计算边界（矩形包围）
		void calc_bounds();
		
		//判断是否在边界内->拖动使用
		bool contains_in_bound(const v2 &pt);

		//判断是否在多边形内部
		bool contains_in_polygon(const v2 &pt);

		//判断点是否在包围框、多边形内部
		bool contains(const v2 &pt) override;
	
		/*
		 * 计算重心
		 * 计算转动惯量
		 * 计算边界AABB
		 */
		void init();

		void refresh(); //将本地坐标转换为世界坐标
		
		void init_static();//是否是静态物体

		//冲量
		void impulse(const v2 &p, const v2 &r) override;

		void update(int n)override;

		//初始化力与力矩为0
		void pass0();

		//计算当前时刻的速度和角速度
		void pass1();
		
		//更新世界坐标系下的，位置，和角度
		void pass2();

		//更新动量
		void pass3();

		//初始化动量
		void pass4();

		// 当动量和速度、角速度为零时，判定休眠
		void pass5();

		//拖曳物体
		void drag(const v2 &pt, const v2 &offset) override;

		//绘图
		void draw() override;

		//以idx为起点，下一顶点为终点的向量
		v2 edge(size_t idx)const;

		v2 &vertex(size_t idx);//返回顶点，世界坐标系下

		size_t index(size_t idx) const;

		size_t edges()const;//世界坐标系下，顶点的数量
		

		std::vector<v2> vertices;//多边形的顶点（本地坐标）
		std::vector<v2> verticesWorld;//世界坐标
		v2 boundMin, boundMax;//外包矩形
};


#endif
