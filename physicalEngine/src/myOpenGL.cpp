#include"myOpenGL.h"
#include"globalVariable.h"
#include"createPolygon.h"
#include"elasticCollision.h"

decltype(auto) sleep_bodies() //使用返回值替换auto,在用decltype推断类型
{
	return std::count_if(bodies.begin(), bodies.end(), [&](auto &b) {
		return b->sleep;
	});
}

void drawPolygon(const c2d_polygon & polygon)
{
	if (polygon.statics)//画静态物体
	{
		glColor3f(0.9f, 0.9f, 0.9f);//设置要放在glbegin之前
		glBegin(GL_LINE_LOOP);

		for (auto &v : polygon.verticesWorld)
		{
			glVertex2d(v.x, v.y);
		}
		glEnd();
		return ;
	}

	if (polygon.sleep)
	{ // 画休眠物体
		glColor3f(0.3f, 0.3f, 0.3f);
		glBegin(GL_LINE_LOOP);
		for (auto &v : polygon.verticesWorld) {
			glVertex2d(v.x, v.y);
		}
		glEnd();
		glColor3f(0.0f, 1.0f, 0.0f);
		glPointSize(1.0f);
		glBegin(GL_POINTS);
		auto p = polygon.pos + polygon.center;
		glVertex2d(p.x, p.y); // 中心
		glEnd();
		return;
	}

	//开启走反样->非静态物体

	glEnable(GL_BLEND);//开启混合功能，将引入的值与颜色缓冲的值混合
	glEnable(GL_LINE_SMOOTH);//抗锯齿
	glHint(GL_LINE_SMOOTH_HINT, GL_FASTEST);//线，给出最有效的选择
	glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);//使用源颜色的alpha值作为源因子，使用1-源颜色的alpha值作为目标因子，线性插值
	glColor3f(0.12f, 0.12f, 0.12f);

	glBegin(GL_LINE_LOOP); /*画出物体的边框*/
	glVertex2d(polygon.boundMin.x, polygon.boundMin.y);
	glVertex2d(polygon.boundMin.x, polygon.boundMax.y);
	glVertex2d(polygon.boundMax.x, polygon.boundMax.y);
	glVertex2d(polygon.boundMax.x, polygon.boundMin.y);
	glEnd();

	if (polygon.collision > 0)//碰撞次数大于0
	{
		glColor3f(0.8f, 0.2f, 0.4f);

	}
	else
	{
		glColor3f(0.8f, 0.8f, 0.0f);
	}

	glBegin(GL_LINE_LOOP);
	for (auto &v : polygon.verticesWorld)
	{
		glVertex2d(v.x, v.y);
	}
	glEnd();


	//默认物体是中心对称的，重心就是重心，后面会计算重心

	auto p = polygon.pos + polygon.center;
	auto v = p + polygon.V*0.2;

	glLineWidth(0.6f);
	glColor3f(0.8f, 0.2f, 0.2f);
	glBegin(GL_LINES);
	glVertex2d(p.x, p.y);
	glVertex2d(p.x + (polygon.Fa.x >= 0 ? 0.2 : -0.2) * std::log10(1 + std::abs(polygon.Fa.x) * 5),
		p.y + (polygon.Fa.y >= 0 ? 0.2 : -0.2) * std::log10(1 + std::abs(polygon.Fa.y) * 5)); // 力向量
	glEnd();

	glColor3f(0.0f, 1.0f, 0.0f);
	glBegin(GL_LINES);
	glVertex2d(p.x, p.y);
	glVertex2d(v.x, v.y); // 速度向量
	glEnd();

	glColor3f(0.2f, 0.2f, 0.2f);
	glBegin(GL_LINES);
	glVertex2d(p.x, p.y);
	glVertex2d(p.x + polygon.R.x1 * 0.2, p.y + polygon.R.x2 * 0.2); // 角度方向向量
	glEnd();

	glColor3f(0.0f, 1.0f, 0.0f);
	glPointSize(3.0f);
	glBegin(GL_POINTS);
	glVertex2d(p.x, p.y); // 重心描绘
	glEnd();

	glDisable(GL_BLEND);
	glDisable(GL_LINE_SMOOTH);
	glLineWidth(1.0f);

}

//绘制碰撞情况
void draw_collision(collision &c)
{
	auto bodyA = dynamic_cast<c2d_polygon *>(c.bodyA);
	auto bodyB = dynamic_cast<c2d_polygon *>(c.bodyB);

	glColor3f(0.2f, 0.5f, 0.4f);
	// 绘制A、B经过SAT计算出来的边

	glBegin(GL_LINES);
	v2 ptA1, ptA2;

	//碰撞物体A参与碰撞线段描绘
	if (!c.bodyA->statics) 
	{
		ptA1 = bodyA->vertex(c.idxA);
		ptA2 = bodyA->vertex(c.idxA + 1);
		glVertex2d(ptA1.x, ptA1.y);
		glVertex2d(ptA2.x, ptA2.y);
	}
	//B描绘
	if (!c.bodyB->statics) 
	{
		auto ptB1 = bodyB->vertex(c.idxB);
		auto ptB2 = bodyB->vertex(c.idxB + 1);
		glVertex2d(ptB1.x, ptB1.y);
		glVertex2d(ptB2.x, ptB2.y);
	}
	glEnd();
	

/*	if (!c.bodyA->statics)
	{
		// 绘制A的SAT边法线 ->分割线
		glColor3f(0.1f, 0.4f, 0.2f);
		glBegin(GL_LINES);
		auto pt3 = (ptA1 + ptA2) / 2;
		auto pt4 = pt3 + c.N * 0.3;
		glVertex2d(pt3.x, pt3.y);
		glVertex2d(pt4.x, pt4.y);
		glEnd();
	}*/

	// 绘制接触点
	glColor3f(1.0f, 0.2f, 0.2f);
	glPointSize(2.0f);
	glBegin(GL_POINTS);
	for (auto &contact : c.contacts) 
	{
		glVertex2d(contact.pos.x, contact.pos.y);
	}
	glEnd();
}

// 根据位置找到物体
c2d_body *find_body(const v2 &pos)
{
	auto body = std::find_if(bodies.begin(), bodies.end(), [&](auto &b) //lambda表达式，使用引用的方式 
	{
		return b->contains(pos); //返回true/false，作为判断条件
	});
	if (body != bodies.end())
	{
		return (*body).get();//迭代器操作，返回指向符合判断的对象的指针
	}

	return nullptr;
}


// 每步操作
static void c2d_step()
{
	glMatrixMode(GL_MODELVIEW); // 转换视图开始绘制
	glLoadIdentity();
	glTranslatef(0.0f, 0.0f, -10.0f); //摄像机向里面平移

	if (!paused) //没有暂停
	{
		collision_detection();//进行碰撞检测

		for (auto &col : collisions) //对发生碰撞的物体进行处理
		{
			collision_prepare(col.second);//碰撞计算准备
		}

		for (auto &body : bodies)
			body->update(4); // 合外力累计清零

							 // 迭代十次
		for (auto i = 0; i < COLLISION_ITERATIONS; ++i)
		{

			for (auto &col : collisions)
			{
				collision_update(col.second);
			}
		}

		for (auto &body : bodies)
		{
			body->update(0); // 初始化力和力矩
			body->update(3); // 添加重力
			body->update(1); // 计算力和力矩，得出速度和角速度
			body->update(2); // 计算位移和角度
			body->update(5); // 判定休眠
		}

	}


	collision_remove_sleep();

	for (auto &body : static_bodies)
	{
		 //角度变换，模拟特殊场景
		body->draw();/*边界绘制*/
	}
	for (auto &body : bodies)
	{
		body->draw();/*物体绘制*/
	}

	for (auto &col : collisions)
	{
		draw_collision(col.second);/*碰撞物体绘制*/
	}

	if (mouse_drag) /*鼠标拖动绘制*/
	{
		glLineWidth(1.0f);
		glColor3f(0.6f, 0.6f, 0.6f);

		glBegin(GL_LINES);
		glVertex2d(global_drag.x, global_drag.y);
		glVertex2d(global_drag.x + global_drag_offset.x, global_drag.y + global_drag_offset.y);
		glEnd();

		glColor3f(0.9f, 0.7f, 0.4f);
		glPointSize(4.0f);
		glBegin(GL_POINTS);
		glVertex2d(global_drag.x, global_drag.y);
		glVertex2d(global_drag.x + global_drag_offset.x, global_drag.y + global_drag_offset.y);
		glEnd();
	} 
}


// 移动（调试）
void c2d_move(const v2 &v) 
{
	for (auto &body : bodies) 
	{
		body->sleep = false;
		body->V += v;
	}
}

// 旋转（调试）
void c2d_rotate(decimal d) 
{
	for (auto &body : bodies) 
	{
		body->sleep = false;
		body->angleV += d;
	}
}

// 拖动（调试）
void c2d_offset(const v2 &pt, const v2 &offset) 
{
	auto body = find_body(pt);
	if (body) 
	{
		body->sleep = false;
		body->drag(pt, offset*body->mass.value);
	}
}



// 清除所有物体
void clear() 
{
	global_id = 1;
	bodies.clear();
	static_bodies.clear();
	collisions.clear();
}


// 建立四周边界
void make_bound() 
{
	make_rect(inf, 10, 0.1, { 0, 3 }, true)->f = 0.8;
	make_rect(inf, 10, 0.1, { 0, -3 }, true)->f = 0.8;
	make_rect(inf, 0.1, 6, { 5, 0 }, true)->f = 0.8;
	make_rect(inf, 0.1, 6, { -5, 0 }, true)->f = 0.8;

}

void make_bound_special()
{

	auto body = make_rect(inf, 2, 0.1, { -4, 0 }, true);
	body->f = 0.3;

	auto body1 = make_rect(inf, 2, 0.1, { -3 + 0.5*sqrt(2), -0.5*sqrt(2) }, true, -(PI/4.0));
	body1->f = 0.3;
	
	auto body2 = make_rect(inf, 2, 0.1, { 4, 0 }, true);
	body2->f = 0.3;

	auto body3 = make_rect(inf, 4/sqrt(3), 0.1, {2,1/sqrt(3) }, true, -(PI / 6.0));
	body3->f = 0.3;


	
}

void scene(int i)//场景选择
{
	clear();
	make_bound();//边界制作
	switch (i) 
	{
		case 1: 
		{ 
			// 多米诺骨牌
			make_bound_special();
			std::vector<v2> vertices = 
			{
				{ -0.5, 0 },
				{ 0.5,  0 },
				{ 0,   0.5}
			};
			make_polygon(5, vertices, v2(4.0, 0.06))->f = 0.2;
			make_polygon(5, vertices, v2(-2.0, 2))->f = 0.2;

			for (auto i = 0; i < 8; i++)
			{
				make_rect(2, 0.3, 2, v2( (-1 + i*0.5), -1.8))->f = 0.2;
			}
			

		}
				break;
		case 2: 
		{ // 堆叠的方块
			static std::default_random_engine e;//使用默认种子
			static std::normal_distribution<decimal> dist(-0.1, 0.1);//非均匀分布的随机数
			for (auto i = 0; i < 10; ++i) 
			{
				auto x = dist(e);
				auto body = make_rect(1, 0.5, 0.4, v2(x,  -2.6+0.4 * i));
				body->f = 0.2;
			}
		}
				break;
		case 3: 
		{ // 金字塔

			v2 posOrigin{ -1.2, -2.4 };
			v2 pos;
			int n = 6;
			for (auto i = 0; i < n; ++i) 
			{
				pos = posOrigin;
				for (auto j = i; j < n; ++j) 
				{
					if (i != 1 || j != 3)
					{
						auto body = make_rect(1, 0.4, 0.4, pos);
						body->f = 0.2;
					}
					pos += v2{ 0.41, 0.0 };
				}
				posOrigin += v2{ 0.205, 0.41 };
			}
		}
			break;
		default:
		{	//模拟杠杆

			make_rect(5, 0.5, 0.5, v2(0, -2.6))->f = 0.2;
			auto lever = make_rect(5, 8, 0.2, v2(0, -2.2));
			lever->f = 0.2;
			lever->angle = -1 * (std::atan(0.06));

			make_rect(5, 0.5, 0.5, v2(-2, -1.5))->f = 0.2;
			make_rect(5, 0.5, 0.5, v2(-2.5, 2.5))->f = 0.2;

			make_rect(5, 0.5, 0.5, v2(2, 0))->f = 0.2;
			

			
		}
			break;
	}
}

void init()
{
	scene(1); //默认场景1
}
static void draw_text(int x, int y, const char*format, ...)//绘制文字
{
	glMatrixMode(GL_PROJECTION);
	glPushMatrix();
	glLoadIdentity();

	int h = glutGet(GLUT_WINDOW_HEIGHT);
	int w = glutGet(GLUT_WINDOW_WIDTH);
	gluOrtho2D(0, w, h, 0);//正射投影，无3D透视效果

	glMatrixMode(GL_MODELVIEW);//绘制物体时，修改了视图
	glPushMatrix();
	glLoadIdentity();

	glColor3f(0.9f, 0.9f, 0.9f);
	glRasterPos2i(x, y);//设置文字的起始位置

	char buffer[256];//这里暂时不做缓冲区溢出判断
	va_list args;
	va_start(args, format);
	int len = vsprintf_s(buffer, format, args);//格式化字符串
	va_end(args);

	for (int i = 0; i < len; ++i)
	{
		glutBitmapCharacter(GLUT_BITMAP_TIMES_ROMAN_24, buffer[i]);//第一个参数为字体大小，第二个参数为字符
	}
	glPopMatrix();
	glMatrixMode(GL_PROJECTION);
	glPopMatrix();
}



//刷新界面
void display()
{
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	int h = glutGet(GLUT_WINDOW_HEIGHT);
	int w = glutGet(GLUT_WINDOW_WIDTH);

	c2d_step(); //坐标轴同直角坐标系

	//绘制文字

	draw_text(w - 110, 40, "FPS:%.1f", dt_inv);
	draw_text(110, 40, "1-4  Change Scene");
	draw_text(w - 290, h - 20, "Collisions: %d, Sleep: %d", collisions.size(), sleep_bodies());

	if (paused)
		draw_text(w / 2 - 30, 20, "PAUSED");
	glutSwapBuffers();//切换双缓冲

}
void reshape(int width, int height)//窗口大小改变事件
{
	glViewport(0, 0, width, height);//改变视口大小
	glMatrixMode(GL_PROJECTION);//透视投影
	glLoadIdentity();//重置单位矩阵
	gluPerspective(45.0, width / (float)height, 0.1, 100.0);//透视投影
}

void keyboard(unsigned char key, int x, int y)//键盘输入事件
{
	if (key >= '0' && key <= '9')
	{
		scene(key - '0');
	}
	else
	{
		switch (key)
		{
			case 27:
				glutLeaveMainLoop(); // 按ESC退出
				break;
			case ' ':
				paused = !paused;
				break;
			case 'w':
				c2d_move(v2(0, 0.1));
				break;
			case 'a':
				c2d_move(v2(-0.1, 0));
				break;
			case 's':
				c2d_move(v2(0, -0.1));
				break;
			case 'd':
				c2d_move(v2(0.1, 0));
				break;
			case 'q':
				c2d_rotate(0.1);
				break;
			case 'e':
				c2d_rotate(-0.1);
				break;
			case 'g':
				gravity.y = gravity.y < 0 ? 0 : GRAVITY;
				for (auto &body : bodies) 
				{

					body->sleep = false;

				}
				break;
			default:
				break;

		}
	}
}

v2 screen2world(int x, int y)//屏幕坐标到世界坐标的转换
{
	GLint viewport[4];
	GLdouble modelview[16];
	GLdouble projection[16];

	GLfloat winX, winY, winZ;
	GLdouble posX, posY, posZ;

	glGetIntegerv(GL_VIEWPORT, viewport); //得到GL_VIEWPORT状态的值，输出到viewport中
	glGetDoublev(GL_MODELVIEW_MATRIX, modelview);//视图矩阵
	glGetDoublev(GL_PROJECTION_MATRIX, projection);//投影矩阵

	winX = x;
	winY = viewport[3] - y;
	winZ = 0.9; //0.1=近 =>0 100=远=>1
	
	gluUnProject(winX, winY, winZ, modelview, projection, viewport, &posX, &posY, &posZ);

	return v2(posX, posY) * 10; //当前投影视距
}
void mouse(int button, int state, int x, int y)
{
	if (button == GLUT_LEFT_BUTTON)
	{
		if (state == GLUT_DOWN)
		{
			mouse_drag = true;
			global_drag = screen2world(x, y);
			global_drag_offset.x = 0;
			global_drag_offset.y = 0;

		}
		else
		{
			mouse_drag = false;
			auto pt = screen2world(x, y);
			global_drag_offset.x = (pt.x - global_drag.x);
			global_drag_offset.y = (pt.y - global_drag.y);

			c2d_offset(global_drag, global_drag_offset);

			global_drag.x = pt.x;
			global_drag.y = pt.y;
		}
	}
}

//鼠标拖动事件
void motion(int x, int y)
{
	if (mouse_drag)
	{
		auto pt = screen2world(x, y); //屏幕坐标至世界坐标

		global_drag_offset.x = (pt.x - global_drag.x);
		global_drag_offset.y = (pt.y - global_drag.y);
	}
}

//计算帧用时，已决定是否刷新资源
void idle()
{
	auto now = std::chrono::high_resolution_clock::now();
	//计算当前帧到上一帧时间间隔
	dt = std::chrono::duration_cast<std::chrono::duration<double>>(now - last_clock).count();

	//锁帧
	if (dt > FRAME_SPAN)//用时超过每秒30帧的时间,便刷新
	{
		dt_inv = 1.0 / dt; //刷新了多少帧
		last_clock = now;
		display();
	}
}

void entry(int state) 
{
	paused = state == GLUT_LEFT;
}

//OpenGL初始化配置
void myOpenGLInit(int *argc,  char** argv, //带命令行的输入
	int width, int height, //窗口大小
	int posX, int posY       //窗口位置
	)
{
	glutInit(argc, argv);
	glutInitWindowSize(800, 600);
	glutInitWindowPosition(50, 50);

	glutInitDisplayMode(GLUT_RGBA | GLUT_DOUBLE);//采用双缓冲，避免闪存
	glutCreateWindow("二维物理引擎");
	init();//初始化场景
	glutDisplayFunc(&idle); //绘制
	glutReshapeFunc(reshape); //窗口大小改变事件
	glutMouseFunc(&mouse); //鼠标点击事件
	glutMotionFunc(&motion);//鼠标拖动事件
	glutKeyboardFunc(&keyboard);//键盘输入
	glutIdleFunc(&idle);//没有事件输入时调用
	glutEntryFunc(&entry); // 没有事件输入时调用，这里不用它
	glutSetOption(GLUT_ACTION_ON_WINDOW_CLOSE, GLUT_ACTION_CONTINUE_EXECUTION);
	glutMainLoop();//主事件循环


}