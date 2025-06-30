#include <iostream>
#include "circle.h"
#include "point.h"
using namespace std;

//class Point
//{
//private:
//	int x, y;/*点坐标*/
//public:
//	void set_point(int fx, int fy)
//	{
//		x = fx;
//		y = fy;
//	}
//	/*获取点的坐标*/
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
//	int x, y; // 圆心坐标
//	int r; // 半径
//public:
//	void set_circle(int fx, int fy, int fr)
//	{
//		x = fx;
//		y = fy;
//		r = fr;
//	}
//	/*判断点和圆的关系*/
//	int point_circle(Point& p)
//	{
//		int px = p.get_x(); // 点的x坐标
//		int py = p.get_y(); // 点的y坐标
//		/*点到圆心的距离的平方*/
//		int d = (px - x) * (px - x) + (py - y) * (py - y);
//		if(d < r * r)
//		{
//			return 1; // 点在圆内
//		}
//		else if (d == r * r)
//		{
//			return 0; // 点在圆上
//		}
//		else
//		{
//			return -1; // 点在圆外
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
		cout << "点在圆内" << endl;
	}
	else if (ret == 0)
	{
		cout << "点在圆上" << endl;
	}
	else
	{
		cout << "点在圆外" << endl;
	}
	system("pause");
	return 0;
}

