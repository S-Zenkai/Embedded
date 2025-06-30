#include "circle.h"


/*���ð뾶*/
void Circle::set_r(int fr)
{
	r = fr;
}

void Circle::set_center(int rx,int ry)
{
	center.set_x(rx);
	center.set_y(ry);
}

/*�жϵ��Բ�Ĺ�ϵ*/
int Circle::point_circle(Point& p)
{
	int px = p.get_x(); // ���x����
	int py = p.get_y(); // ���y����
	/*�㵽Բ�ĵľ����ƽ��*/
	int d = (px - center.get_x()) * (px - center.get_x()) + (py - center.get_y()) * (py - center.get_y());
	if (d < r * r)
	{
		return 1; // ����Բ��
	}
	else if (d == r * r)
	{
		return 0; // ����Բ��
	}
	else
	{
		return -1; // ����Բ��
	}
}
