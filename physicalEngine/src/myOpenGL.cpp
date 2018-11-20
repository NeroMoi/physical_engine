#include"myOpenGL.h"
#include"globalVariable.h"
#include"createPolygon.h"
#include"elasticCollision.h"

decltype(auto) sleep_bodies() //ʹ�÷���ֵ�滻auto,����decltype�ƶ�����
{
	return std::count_if(bodies.begin(), bodies.end(), [&](auto &b) {
		return b->sleep;
	});
}

void drawPolygon(const c2d_polygon & polygon)
{
	if (polygon.statics)//����̬����
	{
		glColor3f(0.9f, 0.9f, 0.9f);//����Ҫ����glbegin֮ǰ
		glBegin(GL_LINE_LOOP);

		for (auto &v : polygon.verticesWorld)
		{
			glVertex2d(v.x, v.y);
		}
		glEnd();
		return ;
	}

	if (polygon.sleep)
	{ // ����������
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
		glVertex2d(p.x, p.y); // ����
		glEnd();
		return;
	}

	//�����߷���->�Ǿ�̬����

	glEnable(GL_BLEND);//������Ϲ��ܣ��������ֵ����ɫ�����ֵ���
	glEnable(GL_LINE_SMOOTH);//�����
	glHint(GL_LINE_SMOOTH_HINT, GL_FASTEST);//�ߣ���������Ч��ѡ��
	glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);//ʹ��Դ��ɫ��alphaֵ��ΪԴ���ӣ�ʹ��1-Դ��ɫ��alphaֵ��ΪĿ�����ӣ����Բ�ֵ
	glColor3f(0.12f, 0.12f, 0.12f);

	glBegin(GL_LINE_LOOP); /*��������ı߿�*/
	glVertex2d(polygon.boundMin.x, polygon.boundMin.y);
	glVertex2d(polygon.boundMin.x, polygon.boundMax.y);
	glVertex2d(polygon.boundMax.x, polygon.boundMax.y);
	glVertex2d(polygon.boundMax.x, polygon.boundMin.y);
	glEnd();

	if (polygon.collision > 0)//��ײ��������0
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


	//Ĭ�����������ĶԳƵģ����ľ������ģ�������������

	auto p = polygon.pos + polygon.center;
	auto v = p + polygon.V*0.2;

	glLineWidth(0.6f);
	glColor3f(0.8f, 0.2f, 0.2f);
	glBegin(GL_LINES);
	glVertex2d(p.x, p.y);
	glVertex2d(p.x + (polygon.Fa.x >= 0 ? 0.2 : -0.2) * std::log10(1 + std::abs(polygon.Fa.x) * 5),
		p.y + (polygon.Fa.y >= 0 ? 0.2 : -0.2) * std::log10(1 + std::abs(polygon.Fa.y) * 5)); // ������
	glEnd();

	glColor3f(0.0f, 1.0f, 0.0f);
	glBegin(GL_LINES);
	glVertex2d(p.x, p.y);
	glVertex2d(v.x, v.y); // �ٶ�����
	glEnd();

	glColor3f(0.2f, 0.2f, 0.2f);
	glBegin(GL_LINES);
	glVertex2d(p.x, p.y);
	glVertex2d(p.x + polygon.R.x1 * 0.2, p.y + polygon.R.x2 * 0.2); // �Ƕȷ�������
	glEnd();

	glColor3f(0.0f, 1.0f, 0.0f);
	glPointSize(3.0f);
	glBegin(GL_POINTS);
	glVertex2d(p.x, p.y); // �������
	glEnd();

	glDisable(GL_BLEND);
	glDisable(GL_LINE_SMOOTH);
	glLineWidth(1.0f);

}

//������ײ���
void draw_collision(collision &c)
{
	auto bodyA = dynamic_cast<c2d_polygon *>(c.bodyA);
	auto bodyB = dynamic_cast<c2d_polygon *>(c.bodyB);

	glColor3f(0.2f, 0.5f, 0.4f);
	// ����A��B����SAT��������ı�

	glBegin(GL_LINES);
	v2 ptA1, ptA2;

	//��ײ����A������ײ�߶����
	if (!c.bodyA->statics) 
	{
		ptA1 = bodyA->vertex(c.idxA);
		ptA2 = bodyA->vertex(c.idxA + 1);
		glVertex2d(ptA1.x, ptA1.y);
		glVertex2d(ptA2.x, ptA2.y);
	}
	//B���
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
		// ����A��SAT�߷��� ->�ָ���
		glColor3f(0.1f, 0.4f, 0.2f);
		glBegin(GL_LINES);
		auto pt3 = (ptA1 + ptA2) / 2;
		auto pt4 = pt3 + c.N * 0.3;
		glVertex2d(pt3.x, pt3.y);
		glVertex2d(pt4.x, pt4.y);
		glEnd();
	}*/

	// ���ƽӴ���
	glColor3f(1.0f, 0.2f, 0.2f);
	glPointSize(2.0f);
	glBegin(GL_POINTS);
	for (auto &contact : c.contacts) 
	{
		glVertex2d(contact.pos.x, contact.pos.y);
	}
	glEnd();
}

// ����λ���ҵ�����
c2d_body *find_body(const v2 &pos)
{
	auto body = std::find_if(bodies.begin(), bodies.end(), [&](auto &b) //lambda���ʽ��ʹ�����õķ�ʽ 
	{
		return b->contains(pos); //����true/false����Ϊ�ж�����
	});
	if (body != bodies.end())
	{
		return (*body).get();//����������������ָ������жϵĶ����ָ��
	}

	return nullptr;
}


// ÿ������
static void c2d_step()
{
	glMatrixMode(GL_MODELVIEW); // ת����ͼ��ʼ����
	glLoadIdentity();
	glTranslatef(0.0f, 0.0f, -10.0f); //�����������ƽ��

	if (!paused) //û����ͣ
	{
		collision_detection();//������ײ���

		for (auto &col : collisions) //�Է�����ײ��������д���
		{
			collision_prepare(col.second);//��ײ����׼��
		}

		for (auto &body : bodies)
			body->update(4); // �������ۼ�����

							 // ����ʮ��
		for (auto i = 0; i < COLLISION_ITERATIONS; ++i)
		{

			for (auto &col : collisions)
			{
				collision_update(col.second);
			}
		}

		for (auto &body : bodies)
		{
			body->update(0); // ��ʼ����������
			body->update(3); // �������
			body->update(1); // �����������أ��ó��ٶȺͽ��ٶ�
			body->update(2); // ����λ�ƺͽǶ�
			body->update(5); // �ж�����
		}

	}


	collision_remove_sleep();

	for (auto &body : static_bodies)
	{
		 //�Ƕȱ任��ģ�����ⳡ��
		body->draw();/*�߽����*/
	}
	for (auto &body : bodies)
	{
		body->draw();/*�������*/
	}

	for (auto &col : collisions)
	{
		draw_collision(col.second);/*��ײ�������*/
	}

	if (mouse_drag) /*����϶�����*/
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


// �ƶ������ԣ�
void c2d_move(const v2 &v) 
{
	for (auto &body : bodies) 
	{
		body->sleep = false;
		body->V += v;
	}
}

// ��ת�����ԣ�
void c2d_rotate(decimal d) 
{
	for (auto &body : bodies) 
	{
		body->sleep = false;
		body->angleV += d;
	}
}

// �϶������ԣ�
void c2d_offset(const v2 &pt, const v2 &offset) 
{
	auto body = find_body(pt);
	if (body) 
	{
		body->sleep = false;
		body->drag(pt, offset*body->mass.value);
	}
}



// �����������
void clear() 
{
	global_id = 1;
	bodies.clear();
	static_bodies.clear();
	collisions.clear();
}


// �������ܱ߽�
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

void scene(int i)//����ѡ��
{
	clear();
	make_bound();//�߽�����
	switch (i) 
	{
		case 1: 
		{ 
			// ����ŵ����
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
		{ // �ѵ��ķ���
			static std::default_random_engine e;//ʹ��Ĭ������
			static std::normal_distribution<decimal> dist(-0.1, 0.1);//�Ǿ��ȷֲ��������
			for (auto i = 0; i < 10; ++i) 
			{
				auto x = dist(e);
				auto body = make_rect(1, 0.5, 0.4, v2(x,  -2.6+0.4 * i));
				body->f = 0.2;
			}
		}
				break;
		case 3: 
		{ // ������

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
		{	//ģ��ܸ�

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
	scene(1); //Ĭ�ϳ���1
}
static void draw_text(int x, int y, const char*format, ...)//��������
{
	glMatrixMode(GL_PROJECTION);
	glPushMatrix();
	glLoadIdentity();

	int h = glutGet(GLUT_WINDOW_HEIGHT);
	int w = glutGet(GLUT_WINDOW_WIDTH);
	gluOrtho2D(0, w, h, 0);//����ͶӰ����3D͸��Ч��

	glMatrixMode(GL_MODELVIEW);//��������ʱ���޸�����ͼ
	glPushMatrix();
	glLoadIdentity();

	glColor3f(0.9f, 0.9f, 0.9f);
	glRasterPos2i(x, y);//�������ֵ���ʼλ��

	char buffer[256];//������ʱ��������������ж�
	va_list args;
	va_start(args, format);
	int len = vsprintf_s(buffer, format, args);//��ʽ���ַ���
	va_end(args);

	for (int i = 0; i < len; ++i)
	{
		glutBitmapCharacter(GLUT_BITMAP_TIMES_ROMAN_24, buffer[i]);//��һ������Ϊ�����С���ڶ�������Ϊ�ַ�
	}
	glPopMatrix();
	glMatrixMode(GL_PROJECTION);
	glPopMatrix();
}



//ˢ�½���
void display()
{
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	int h = glutGet(GLUT_WINDOW_HEIGHT);
	int w = glutGet(GLUT_WINDOW_WIDTH);

	c2d_step(); //������ֱͬ������ϵ

	//��������

	draw_text(w - 110, 40, "FPS:%.1f", dt_inv);
	draw_text(110, 40, "1-4  Change Scene");
	draw_text(w - 290, h - 20, "Collisions: %d, Sleep: %d", collisions.size(), sleep_bodies());

	if (paused)
		draw_text(w / 2 - 30, 20, "PAUSED");
	glutSwapBuffers();//�л�˫����

}
void reshape(int width, int height)//���ڴ�С�ı��¼�
{
	glViewport(0, 0, width, height);//�ı��ӿڴ�С
	glMatrixMode(GL_PROJECTION);//͸��ͶӰ
	glLoadIdentity();//���õ�λ����
	gluPerspective(45.0, width / (float)height, 0.1, 100.0);//͸��ͶӰ
}

void keyboard(unsigned char key, int x, int y)//���������¼�
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
				glutLeaveMainLoop(); // ��ESC�˳�
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

v2 screen2world(int x, int y)//��Ļ���굽���������ת��
{
	GLint viewport[4];
	GLdouble modelview[16];
	GLdouble projection[16];

	GLfloat winX, winY, winZ;
	GLdouble posX, posY, posZ;

	glGetIntegerv(GL_VIEWPORT, viewport); //�õ�GL_VIEWPORT״̬��ֵ�������viewport��
	glGetDoublev(GL_MODELVIEW_MATRIX, modelview);//��ͼ����
	glGetDoublev(GL_PROJECTION_MATRIX, projection);//ͶӰ����

	winX = x;
	winY = viewport[3] - y;
	winZ = 0.9; //0.1=�� =>0 100=Զ=>1
	
	gluUnProject(winX, winY, winZ, modelview, projection, viewport, &posX, &posY, &posZ);

	return v2(posX, posY) * 10; //��ǰͶӰ�Ӿ�
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

//����϶��¼�
void motion(int x, int y)
{
	if (mouse_drag)
	{
		auto pt = screen2world(x, y); //��Ļ��������������

		global_drag_offset.x = (pt.x - global_drag.x);
		global_drag_offset.y = (pt.y - global_drag.y);
	}
}

//����֡��ʱ���Ѿ����Ƿ�ˢ����Դ
void idle()
{
	auto now = std::chrono::high_resolution_clock::now();
	//���㵱ǰ֡����һ֡ʱ����
	dt = std::chrono::duration_cast<std::chrono::duration<double>>(now - last_clock).count();

	//��֡
	if (dt > FRAME_SPAN)//��ʱ����ÿ��30֡��ʱ��,��ˢ��
	{
		dt_inv = 1.0 / dt; //ˢ���˶���֡
		last_clock = now;
		display();
	}
}

void entry(int state) 
{
	paused = state == GLUT_LEFT;
}

//OpenGL��ʼ������
void myOpenGLInit(int *argc,  char** argv, //�������е�����
	int width, int height, //���ڴ�С
	int posX, int posY       //����λ��
	)
{
	glutInit(argc, argv);
	glutInitWindowSize(800, 600);
	glutInitWindowPosition(50, 50);

	glutInitDisplayMode(GLUT_RGBA | GLUT_DOUBLE);//����˫���壬��������
	glutCreateWindow("��ά��������");
	init();//��ʼ������
	glutDisplayFunc(&idle); //����
	glutReshapeFunc(reshape); //���ڴ�С�ı��¼�
	glutMouseFunc(&mouse); //������¼�
	glutMotionFunc(&motion);//����϶��¼�
	glutKeyboardFunc(&keyboard);//��������
	glutIdleFunc(&idle);//û���¼�����ʱ����
	glutEntryFunc(&entry); // û���¼�����ʱ���ã����ﲻ����
	glutSetOption(GLUT_ACTION_ON_WINDOW_CLOSE, GLUT_ACTION_CONTINUE_EXECUTION);
	glutMainLoop();//���¼�ѭ��


}