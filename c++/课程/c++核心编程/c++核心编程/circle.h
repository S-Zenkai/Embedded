#pragma once
#include <iostream>
#include "point.h"
using namespace std;

/*Բ��*/
class Circle
{
private:
	Point center; // Բ��
	int r; // �뾶
public:
	/*����Բ��*/
	void set_center(int rx, int ry);
	/*���ð뾶*/
	void set_r(int fr);
		/*�жϵ��Բ�Ĺ�ϵ*/
	int point_circle(Point& p);
};
