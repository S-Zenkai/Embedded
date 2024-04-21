#define _CRT_SECURE_NO_WARNINGS
#include <stdio.h>
//int main()
//{
//	int a = 0;//a在内存中存储是00000000000000000000000000000000
//	int b = ~a;//~指将a对应的二进制数按位取反，即11111111111111111111111111111111，首位是1代表负数，在内存中是以补码的形式存储
//	           //原码到反码：符号位不变，其余位取反。反码到补码：补码加1
//	printf("%d", ~b);
//}
//int main()
//{
//	int a = 10;
//	//int b = a++;//后置++，先使用，后++，a=11,b=10
//	//int b = ++a;//前置++，先++，后使用，a=11,b=11
//	int b = a--;//后置--，先使用，后--，a=9,b=10
//	printf("a=%d\nb=%d",a,b);
//	return 0;
//}
//int main()
//{
//	int a = (int)3.14;
//	printf("%d", a);
//	return 0;
//}
//int MAX(int x, int y)
//{
//	if (x > y)
//		return x;
//	else
//		return y;
//}
//int main()
//{
//	int a = 0;
//	int b = 0;
//	int max = 0;
//	scanf("%d%d", &a ,& b);
//	max = MAX(a, b);
//	printf("%d", max);
//	/*if (a > b)
		//printf("max=%d", a);
//	else
//		printf("max=%d", b);*/
//	return 0;
//}
//
//int main()
//{
//	int a = 0;
//	int b = 0;
//	scanf("%d%d", &a, &b);
//	int max = a > b ? a : b;//exp1？exp2：exp3的意思是如果表达式1为真，则输出表达式2，反之，则输出表达式3.即如果a>b成立，则输出a，反之输出二
//	printf("max=%d", max);
//	return 0;
//}
//int main()
//{
//	register int a = 0;//建议将a的值放入寄存器中去，注意是建议，最终放不放还是由编译器决定。（因为寄存器内存有限，当其内存不足时不放）
//	return 0;
//}
//
//int main()
//{
//	//int float = 0;//err,定义的变量名不能和关键字相同
//	unsigned int num1 = 10;
//	typedef unsigned int u_int;//typedef(类型重定义，num1与num2定义的变量是相同的，即unsigned int=u_int，注意使用时直接写定义的变量即可，不需写等号
//	u_int num2 = 10;
//	return 0;
//}

//void test()
//{
//	static int a = 0;//statac修饰局部变量时称为静态局部变量，在不加时，运行结果是5个1，加了static，则是1到5
//	a++;
//	printf("a=%d\n", a);
//}
//int main()
//{
//	int i = 0;
//	while (i < 5)
//	{
//		test();
//		i++;
//	}
//	return 0;
//}

//int main()
//{
//	extern int g_val;//用于引入外部变量
//	printf("%d", g_val);
//	return 0;
//}

//int main()
//{
//	extern ADD(int, int);
//	int a = 10;
//	int b = 65;
//	int sum = ADD(a, b);
//	printf("%d", sum);
//	return 0;
//}

//#define c 100//define定义的标识符常量，c本质上是常量，注意格式，不是c(100)
//#define MAX(x,y)(x>y?x:y)//一个错误：宏的x，y不必加类型
//int main()
//{
//	int a = 51;
//	int b = 25;
//	int max = MAX(a, b);
//	printf("c=%d", c);
//	printf("max=%d", max);
//	return 0;
//}

//int main()
//{
//	int a = 10;
//	printf("%p\n", &a);//%p代表以地址的形式打印
//	int* p= &a;//p为一个整形指针变量（注意定义方式：int*)，p中放的是a的地址
//	printf("%p\n", p);
//	*p = 20;//*--解引用操作符，*p=20相当于a=20
//	printf("a=%d", a);
//	return 0;
//}

//int main()
//{
//	char a = 'x';
//	printf("%c\n", a);
//	printf("%p\n", &a);
//	char* p = &a;
//	printf("%p\n", p);
//	printf("%c\n", (*p));
//	*p = 'y';
//	printf("%c\n", a);
//	printf("%d\n", sizeof(&a));
//	printf("%d\n", sizeof(*p));
//	return 0;
//}
//
//int main()
//{
//	printf("    ***     \n     *     \n*************\n    ***     \n    ***     \n     *     ");
//	return 0;
//}

//int main()
//{
//	int M = 0;
//	scanf("%d", &M);
//	int a = M % 5;
//	if (a == 0)//注意这里要用==
//		printf("yes");
//	else
//		printf("NO");
//	return 0;
//}

struct Book
{
	char name[20];
	int price;
	int page;
};
int main()
{
	struct Book b1={"c语言程序设计",55,528 };//注意形式，直接写结构体，不必：name=“c语言程序设计”（err）
	/*struct Book* pb = &b1;*/
	//printf("书名为：%s\n", pb->name); //注意.和->的区别，.:结构体变量.成员   ->:结构体指针->成员
	//printf("价格为：%d\n", pb->price);
	//printf("页数为：%d\n", pb->page);
	//printf("书名为：%s\n", (*pb).name);//注意，*pb相当于b1，但在打印时要括起来
	//printf("价格为：%d\n", (*pb).price);
	//printf("页数为：%d\n", (*pb).page);
	printf("书名为：%s\n", b1.name);
	printf("价格为：%d\n", b1.price);
	printf("页数为：%d\n", b1.page);
	b1.price = 15;
	printf("修改后的价格为：%d\n", b1.price);
	b1.name = "c语言程序设计基础";
	return 0;
}

//struct People
//{
//	char Name[20];
//	int Age;
//	char Sex[20];
//	char interest[20];
//	char power[20];
//	char evaluate[20];
//};
//int main()
//{
//	struct People b1={ "孙振凯", 26, "男", "沉迷手机", "装疯卖傻", "是个废物" };
//	printf("姓名：%s\n", b1.Name);
//	printf("年龄：%d\n", b1.Age);
//	printf("性别：%s\n", b1.Sex);
//	printf("兴趣：%s\n", b1.interest);
//	printf("能力：%s\n", b1.power);
//	printf("评价：%s\n", b1.evaluate);
//	return 0;
//}