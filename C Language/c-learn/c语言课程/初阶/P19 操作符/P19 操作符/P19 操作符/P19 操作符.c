#define _CRT_SECURE_NO_WARNINGS
#include <stdio.h>
//int main()
//{
//	int a = 5;
//	//00000000000000000000000000001010
//	//00000000000000000000000000010100  
//	int b = a << 1;
//	int c = a >> 1;
//	printf("%d", b);
//	//10000000000000000000000000001010
//	//11111111111111111111111111110101反码
//	//11111111111111111111111111110110补码
//	//11111111111111111111111111111011
//	//11111111111111111111111111111010
//	//10000000000000000000000000000101  -5（算数右移）
//	//01111111111111111111111111111011逻辑右移
//	//01111111111111111111111111111100
//	//00000000000000000000000000000011  3
//	printf("%d", c);
//}


//int main()
//{
//	int a = 1;            //00000000000000000000000000000001
//	int b = 2;            //00000000000000000000000000000010
//	printf("%d\n", a & b);//00000000000000000000000000000000=0
//	printf("%d\n", a | b);//00000000000000000000000000000011=3
//	printf("%d\n", a ^ b);//00000000000000000000000000000011=3
//	return 0;
//}

//int main()
//{
//	int a = 800000000;//0001  0101
//	int b = 500000000;//0101  0100
//	                  
//	/*a = a + b;
//	b = a - b;
//	a = a - b;*/
//	a = a ^ b;
//	b = b ^ a;
//	a = b ^ a;
//	printf("a=%d\n", a);
//	printf("b=%d\n", b);
//	return 0;
//}

//编写代码实现：求一个整数存储在内存中的二进制中1的个数。
////int main()
////{
////	int a = 0;
////	int i = 0;
////	int count = 0;
////	scanf("%d", &a);
////	for(i = 0;i < 64;i++)
////	{
////		if (a & 1 == 1)
////			count++;
////		a = a >> 1;
////	}
////		printf("%d\n", count++);
////	return 0;
////}

//void test1(int arr[])
//{
//	printf("%d\n", sizeof(arr));//(2)
//}
//void test2(char ch[])
//{
//	printf("%d\n", sizeof(ch));//(4)
//}
//int main()
//{
//	int arr[10] = { 0 };
//	char ch[10] = { 0 };
//	printf("%d\n", sizeof(arr));//(1)
//	printf("%d\n", sizeof(ch));//(3)
//	test1(arr);
//	test2(ch);
//	return 0;
//}

//int main()
//{
//    int i = 0, a = 0, b = 2, c = 3, d = 4;
//    i = a++ && ++b && d++;
//    //i = a++||++b||d++;
//    printf("a = %d\n b = %d\n c = %d\nd = %d\n", a, b, c, d);
//    return 0;
//}

//int main()
//{
//	int a = 0;
//	int b = 0;
//	scanf("%d", &a);
//	b = a < 5 ? 3 : -3;
//	printf("%d\n", b);
//	return 0;
//}
//int main()
//{
//	int a = 1;
//	int b = 2;
//	int c = (a > b, a = b + 10, a, b = a + 1);
//	printf("%d", c);
//}

//int main()
//{
//	int num = -1;
//	int i = 0;
//	int count = 0;//计数
//	while (num)
//	{
//		count++;
//		num = num & (num - 1);
//	}
//	printf("二进制中1的个数 = %d\n", count);
//	return 0;
//}

//int main()
//{
//	printf("%d\n", sizeof(int*));
//	return 0;
//}

//不创建临时变量，实现两个数的互换
//int main()
//{
//	int a = -5;
//	int b = -10;
//	a = a + b;
//	b = a - b;
//	a = a - b;
//	printf("a=%d\n", a);
//	printf("b=%d\n", b);
//	return 0;
//}

//int main()
//{
//	int a = 5;
//	int b = 10;
//	a = a ^ b;
//	b = b ^ a;
//	a = a ^ b;
//	printf("a=%d\n", a);
//	printf("b=%d\n", b);
//	return 0;
//}

//求一个整数存储在内存中的二进制中1的个数
//int main()
//{
//	int a = 0;
//	scanf("%d", &a);
//	int i = 0;
//	int count = 0;
//	for (i = 0; i < 32; i++)
//	{
//		if (a & 1)
//			count++;
//		a = a >> 1;
//	}
//	printf("%d", count);
//	return 0;
//}
//-1
//10000000000000000000000000000001
//11111111111111111111111111111110
//11111111111111111111111111111111

/// <summary>
/// //////////////////////
/// </summary>
/// <returns></returns>
//int main()
//{
//	int a = -10;
//	int* p = NULL;
//	printf("%d\n", !2);
//	printf("%d\n", !0);
//	a = -a;
//	p = &a;
//	printf("%d\n", sizeof(a));//10
//	printf("%d\n", sizeof(int));//4
//	printf("%d\n", sizeof a);//这样写行不行？
//	return 0;
//}

//////////////////////////////////
//void test1(int arr[])
//{
//	printf("%d\n", sizeof(arr));//(2)
//}
//void test2(char ch[])
//{
//	printf("%d\n", sizeof(ch));//(4)
//}
//int main()
//{
//	int arr[10] = { 0 };
//	char ch[10] = { 0 };
//	printf("%d\n", sizeof(arr));//(1)
//	printf("%d\n", sizeof(ch));//(3)
//	test1(arr);
//	test2(ch);
//	return 0;
//}

/////////////////////////////////
//int main()
//{
//    int i = 0, a = 0, b = 2, c = 3, d = 4;
//    //i = a++ && ++b && d++;
//    i = a++||++b||d++;
//    printf("a = %d\n b = %d\n c = %d\nd = %d\n", a, b, c, d);
//    return 0;
//}

/////////////////////////////
//void test1()
//{
//	printf("hehe\n");
//}
//void test2(const char* str)
//{
//	printf("%s\n", str);
//}
//int main()
//{
//	test1();            //实用（）作为函数调用操作符。
//	test2("hello bit.");//实用（）作为函数调用操作符。
//	return 0;
//}

////////////////////////////////
//void test1()
//{
//	printf("hehe\n");
//}
//void test2(char str)
//{
//	printf("%s\n", str);
//}
//int main()
//{
//	test1();            //实用（）作为函数调用操作符。
//	test2("hello bit.");//实用（）作为函数调用操作符。
//	return 0;
//}

///////////////////////
//void test2(char* b)
//{
//	printf("%c\n", *b);
//	//printf("%c\n", *b);//	将字符串传给函数和将数字数组传给函数是不一样的,虽然字符串传给函数的仍是首字母地址,但用%s是可以把整个字符串打印出来的
//
//}
//int main()
//{
//	char b[10] = { "hello " };
//	test2(b);
//	return 0;
//}

//struct Stu
//{
//	int age;
//	int name[10];
//	char sex[10];
//	int scare;
//};
//int main()
//{
//	struct Stu stu;
//	stu.age = 15;
//	printf("%d", stu.age);
//	struct Stu* pstu = &stu;
//	pstu -> name = 10;
//	printf("%d", stu.age);
//	return 0;
//}

//struct Stu 
//{
//	int age;
//	char name[10];
//	char sex[10];
//	int scare;
//};
//int main()
//{
//	struct Stu stu;
//	stu.age = 15;
//	struct Stu* pstu = &stu;
//	pstu -> age = 20;
//
//	return 0;
//}

//int main()
//{
//	char a = 0xb6;
//	short b = 0xb600;
//	int c = 0xb6000000;
//	if (a == 0xb6)
//		printf("a");
//	if (b == 0xb600)
//		printf("b");
//	if (c == 0xb6000000)
//		printf("c");
//	return 0;
//}

//int main()
//{
//	char c = 1;
//	printf("%u\n", sizeof(c));
//	printf("%u\n", sizeof(+c));
//	printf("%u\n", sizeof(-c));
//	return 0;
//}

//int main()
//{
//	unsigned int a = -1;//10000000000000000000000000000001
//	printf("%u", a);    //11111111111111111111111111111111
//	return 0;           //01111111111111111111111111111111
//	                    //00000000000000000000000000000001
//

int fun()
{
    static int count = 1;
    return ++count;
}
int main()
{
    int answer;
    answer = fun() - fun() * fun();
    printf("%d\n", answer);//输出多少？
    return 0;
}
