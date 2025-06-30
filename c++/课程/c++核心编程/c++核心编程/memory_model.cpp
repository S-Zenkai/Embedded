#include "iostream"
using namespace std;

/*全局区*/
//int g = 10; //全局变量
//static int s = 20; //静态变量
//const int c = 30; //全局常量
//const char* st = "hello"; //全局字符串常量
//int main()
//{
//	/*栈区*/
//	const int c1 = 0; //局部常量
//	cout << "全局变量g = 0x" << &g << endl;
//	cout << "静态变量s = 0x" << &s << endl;
//	cout << "全局常量c = 0x" << &c << endl;
//	cout << "全局字符串常量st = 0x" << &st << endl;
//	cout << "局部常量c1 = 0x" << &c1 << endl;
//	cout << "局部字符串常量 = 0x" << &"hello" << endl;
//	system("pause");
//	return 0;
//}

/*栈区*/
/*函数返回值不可以使用局部变量地址*/

//int* fun(void)
//{
//	int a = 10; //局部变量
//	return &a; //返回局部变量地址
//}
//int main()
//{
//	int* p = fun(); //获取函数返回值
//	cout << "局部变量a的值 = " << *p << endl; //输出局部变量值
//	system("pause");
//	return 0;
//}
