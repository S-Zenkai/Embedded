#include <iostream>
#include "circle.h"
#include "point.h"
using namespace std;

//class Point
//{
//private:
//	int x, y;/*������*/
//public:
//	void set_point(int fx, int fy)
//	{
//		x = fx;
//		y = fy;
//	}
//	/*��ȡ�������*/
//	int get_x() 
//	{ 
//		return x; 
//	}
//	int get_y() 
//	{ 
//		return y;
//	}
//};

//class Circle
//{
//private:
//	int x, y; // Բ������
//	int r; // �뾶
//public:
//	void set_circle(int fx, int fy, int fr)
//	{
//		x = fx;
//		y = fy;
//		r = fr;
//	}
//	/*�жϵ��Բ�Ĺ�ϵ*/
//	int point_circle(Point& p)
//	{
//		int px = p.get_x(); // ���x����
//		int py = p.get_y(); // ���y����
//		/*�㵽Բ�ĵľ����ƽ��*/
//		int d = (px - x) * (px - x) + (py - y) * (py - y);
//		if(d < r * r)
//		{
//			return 1; // ����Բ��
//		}
//		else if (d == r * r)
//		{
//			return 0; // ����Բ��
//		}
//		else
//		{
//			return -1; // ����Բ��
//		}
//	}
//};

int main()
{
	Point p;
	Circle c;
	p.set_x(5);
	p.set_y(6);
	c.set_center(5, 0);
	c.set_r(5);
	int ret = c.point_circle(p);
	if (ret == 1)
	{
		cout << "����Բ��" << endl;
	}
	else if (ret == 0)
	{
		cout << "����Բ��" << endl;
	}
	else
	{
		cout << "����Բ��" << endl;
	}
	system("pause");
	return 0;
}

