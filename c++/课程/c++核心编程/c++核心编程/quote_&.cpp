//#include "iostream"
//using namespace std;
//
////int main()
////{
////	int a = 0;
////	int& b = a;
////	cout << "a = " << a << endl; // ��� a ��ֵ
////	b = 20;
////	cout << "a = " << a << endl; // ����޸ĺ�� a ��ֵ
////	system("pause");
////	return 0;
////}
//
///*������Ϊ��������ֵ*/
//int& fun1(void)
//{
//	int a = 10;
//	return a;
//}
//int& fun2(void)
//{
//	static int b = 10; // ��̬����
//	return b;
//}
//int main()
//{
//	//int& c = fun1(); // ����fun1 ���ص��Ǿֲ�����������
//	//c = 20;
//	//cout << "c = " << c << endl; // δ������Ϊ�����ܵ��³���������������ֵ
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
