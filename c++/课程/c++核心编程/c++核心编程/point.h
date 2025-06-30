#pragma once
#include <iostream>
using namespace std;

/*点类*/
class Point
{
private:
	int x, y;/*点坐标*/
public:
	/*设置点的x坐标*/
	void set_x(int fx);
	/*设置点的y坐标*/
	void set_y(int fy);
	/*获取点的坐标*/
	int get_x();
	int get_y();
};