#include "circle.h"


/*设置半径*/
void Circle::set_r(int fr)
{
	r = fr;
}

void Circle::set_center(int rx,int ry)
{
	center.set_x(rx);
	center.set_y(ry);
}

/*判断点和圆的关系*/
int Circle::point_circle(Point& p)
{
	int px = p.get_x(); // 点的x坐标
	int py = p.get_y(); // 点的y坐标
	/*点到圆心的距离的平方*/
	int d = (px - center.get_x()) * (px - center.get_x()) + (py - center.get_y()) * (py - center.get_y());
	if (d < r * r)
	{
		return 1; // 点在圆内
	}
	else if (d == r * r)
	{
		return 0; // 点在圆上
	}
	else
	{
		return -1; // 点在圆外
	}
}
