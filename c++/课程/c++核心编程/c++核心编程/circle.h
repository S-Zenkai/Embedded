#pragma once
#include <iostream>
#include "point.h"
using namespace std;

/*圆类*/
class Circle
{
private:
	Point center; // 圆心
	int r; // 半径
public:
	/*设置圆心*/
	void set_center(int rx, int ry);
	/*设置半径*/
	void set_r(int fr);
		/*判断点和圆的关系*/
	int point_circle(Point& p);
};
