//#include "iostream"
//using namespace std;
//
////int main()
////{
////	int a = 0;
////	int& b = a;
////	cout << "a = " << a << endl; // 输出 a 的值
////	b = 20;
////	cout << "a = " << a << endl; // 输出修改后的 a 的值
////	system("pause");
////	return 0;
////}
//
///*引用作为函数返回值*/
//int& fun1(void)
//{
//	int a = 10;
//	return a;
//}
//int& fun2(void)
//{
//	static int b = 10; // 静态变量
//	return b;
//}
//int main()
//{
//	//int& c = fun1(); // 错误：fun1 返回的是局部变量的引用
//	//c = 20;
//	//cout << "c = " << c << endl; // 未定义行为，可能导致程序崩溃或输出垃圾值
//	int& d = fun2();
//	cout << "d = " << d << endl;
//	d = 20;
//	cout << "d = " << d << endl;
//		
//	fun2() = 50;
//	cout << "d = " << d << endl;
//	system("pause");
//	return 0;
//}
