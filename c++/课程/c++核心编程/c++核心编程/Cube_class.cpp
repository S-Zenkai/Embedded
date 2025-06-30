//#include <iostream>
//using namespace std;
//
//
//class Cube 
//{
//private:
//	/*属性*/
//	int length;/*长*/
//	int width;/*宽*/
//	int height;/*高*/
//	int area;/*面积*/
//	int volume;/*体积*/
//public:
//	/*行为*/
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
//	/*判断两个立方体是否相同*/
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
//	cout << "c1的面积为：" << c1.get_area() << endl;
//	cout << "c1的体积为：" << c1.get_volume() << endl;
//	cout << "c2的面积为：" << c2.get_area() << endl;
//	cout << "c2的体积为：" << c2.get_volume() << endl;
//	if (c1.is_same(c2))
//		cout << "c1和c2是相同的立方体" << endl;
//	else
//		cout << "c1和c2不是相同的立方体" << endl;
//	system("pause");
//	return 0;
//}
