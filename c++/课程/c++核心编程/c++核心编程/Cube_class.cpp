//#include <iostream>
//using namespace std;
//
//
//class Cube 
//{
//private:
//	/*����*/
//	int length;/*��*/
//	int width;/*��*/
//	int height;/*��*/
//	int area;/*���*/
//	int volume;/*���*/
//public:
//	/*��Ϊ*/
//	void set_cube(int l, int w, int h)
//	{
//		length = l;
//		width = w;
//		height = h;
//	}
//	int get_area()
//	{
//		area = 2 * (length * width + width * height + height * length);
//		return area;
//	}
//	int get_volume()
//	{
//		volume = length * width * height;
//		return volume;
//	}
//	/*�ж������������Ƿ���ͬ*/
//	bool is_same(Cube &c)
//	{
//		if (length == c.length && width == c.width && height == c.height)
//			return true;
//		else
//			return false;
//	}
//};
//
////bool is_same(Cube &c1, Cube &c2)
////{
////	
////}
//
//int main()
//{
//	Cube c1, c2;
//	c1.set_cube(3, 4, 5);
//	c2.set_cube(3, 4, 5);
//	cout << "c1�����Ϊ��" << c1.get_area() << endl;
//	cout << "c1�����Ϊ��" << c1.get_volume() << endl;
//	cout << "c2�����Ϊ��" << c2.get_area() << endl;
//	cout << "c2�����Ϊ��" << c2.get_volume() << endl;
//	if (c1.is_same(c2))
//		cout << "c1��c2����ͬ��������" << endl;
//	else
//		cout << "c1��c2������ͬ��������" << endl;
//	system("pause");
//	return 0;
//}
