#pragma once
#include <iostream>
using namespace std;

/*����*/
class Point
{
private:
	int x, y;/*������*/
public:
	/*���õ��x����*/
	void set_x(int fx);
	/*���õ��y����*/
	void set_y(int fy);
	/*��ȡ�������*/
	int get_x();
	int get_y();
};