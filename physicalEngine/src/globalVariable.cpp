#include"globalVariable.h"

/*
* ȫ�ֱ����Ķ���
*
*/

timePoint last_clock = std::chrono::high_resolution_clock::now();
double dt = FRAME_SPAN;
double dt_inv = 1.0 *FPS;
bool paused = false;

v2 gravity { 0, GRAVITY };

std::vector<c2d_body::ptr> bodies; // Ѱ������
std::vector<c2d_body::ptr> static_bodies; // ��̬����
std::unordered_map<uint32_t, collision> collisions;//��ײ�ṹ��


uint16_t global_id = 1;
bool mouse_drag = false;
v2 global_drag;//����϶�
v2 global_drag_offset;//����϶�λ��
