#define _CRT_SECURE_NO_WARNINGS
#include <stdio.h>
#include <string.h>
//int main()
//{
//	int a, b, c;
//	a = 5;
//	c = ++a;//a=6,c=6
//	b = ++c, c++, ++a, a++;//c=8,a=8,b=7
//	b += a++ + c;//a=9,b=23,c=8
//	printf("a = %d b = %d c = %d\n:", a, b, c);
//	return 0;
//}

//struct Stu
//{
//	char name[10];
//	int age;
//	char sex[10];
//	int score;
//};
//struct Stu1
//{
//	struct Stu s1;
//	char id[20];
//	struct Stu1* p;//可以在结构体内部定义该结构体类型的指针
//};
//int main()
//{
//	struct Stu stu = { "小红",18,"保密",60 };
//	struct Stu1 S1 = { {"小张",25,"男",60},"308148",NULL };
//	printf("%d\n", S1.s1.age);
//	//strcpy(stu.name, "小蓝");
//	//struct Stu* p1 = &stu;
//	//*(p1 + 1) = 10;//err
//	//(*p1).age = 10;
//	//printf("%s\n", p1->name);
//	//printf("%d\n", stu.age);
//	//printf("%s\n", p1->sex);
//	//printf("%s", S1.id);
//	return 0;
//}

//struct Stu
//{
//	char name[10];
//	int age;
//	char sex[10];
//	int score;
//};
//
//void print(struct Stu* p)
//{
//	printf("%s\n%d\n%s\n%d\n", p->name, p->age, (*p).sex, (*p).score);
//}
//
//int main()
//{
//	struct Stu s1= { "小红",18,"保密",60 };
//	print(&s1);
//	return 0;
//}

//计算二进制中1的个数
int Judge_1_num (int a)
{
		int count = 0;
		while (a != 0)
		{
			a = a & a - 1;//每与一次，a中减少一个1
			count++;
		}
		return count;

}

//int main()
//{
//	unsigned int n = 0;
//	int count = 0;
//	scanf("%d", &n);
//	while (n != 0)
//	{
//		if (n % 2 == 1)
//		{
//			count++;
//		}
//		n = n / 2;
//	}
//	printf("%d\n", count);
//	return 0;
//}
//判断一个是是不是2的n次方
//int main()
//{
//	int n = 0;
//	scanf("%d", &n);
//	if (Judge_1_num(n) == 1)
//	{
//		printf("yes");
//	}
//	else
//		printf( "no" );
//	return 0;
//}
//两个整数的二进制为有多少位相同
int main()
{
	int m = 3;
	int n = 5;
	//scanf("%d%d", &m, &n);
	int count = Judge_1_num(m ^ n);
	printf("%d\n", count);
	return 0;
}

//011
//101
//int main()
//{
//	char PS[20] = { 0 };
//	printf("请输入密码：");
//	scanf("%s", PS);
//	printf("请确认");
//	getchar();
//	int ch = getchar();
//	return 0;
//}
