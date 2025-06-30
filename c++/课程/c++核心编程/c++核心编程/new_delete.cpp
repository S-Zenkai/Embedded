//#include "iostream"
//using namespace std;
/*c++利用new操作符在堆区分配内存，通过delete释放内存*/
//int* fun(void)
//{
//	int* p = new int(10);
//	return p;
//}
//int main()
//{
//	int* p1 = fun();
//	/*在堆中的数据由用户释放，所以可以作为函数的返回值*/
//	cout << "p指向的值为：" << *p1 << endl;
//	cout << "p指向的值为：" << *p1 << endl;
//	delete p1; // 释放堆区内存
//	//cout << "p指向的值为：" << *p1 << endl;
//	system("pause");
//	return 0;
//}

/*通过new创建数组*/
//int main()
//{
//	int* p = new int[10]; // 在堆区分配10个int类型的空间
//	for (int i = 0; i < 10; i++)
//	{
//		p[i] = i; // 给数组元素赋值
//	}
//	for(int i= 0; i < 10; i++)
//	{
//		cout << "p[" << i << "] = " << p[i] << endl; // 输出数组元素
//	}
//	delete[] p; // 释放堆区内存
//	//cout << "p指向的值为：" << *p1 << endl;
//	system("pause");
//	return 0;
//}

