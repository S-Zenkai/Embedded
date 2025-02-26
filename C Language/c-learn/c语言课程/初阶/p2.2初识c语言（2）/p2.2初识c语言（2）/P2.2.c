#define _CRT_SECURE_NO_WARNINGS

//#define max 10//#define定义的标识符常量，本质上是常量
#include <stdio.h>

//enum Sex   //枚举关键字enum
//{
//	MALE,   //0
//	FEMALE,//1
//	SECARE//2
//};
//int main()
//{
//	printf("%d\n", FEMALE);
	/*printf("孙胜凯是傻瓜\n");*/
	/*printf("%d\n", max);*/
	/*return 0;*/
	/*int num1 = 0;
	int num2 = 0;
	scanf("%d%d", &num1, &num2);
	int sum = 0;
	sum = num1 + num2;
	printf("sum=%d\n",sum);*/
	/*const int num = 4;*/
	//const-常属性
	//num是const修饰的常变量，num本质上仍是变量，只是具有常属性，不可更改
	/*printf("%d\n", num);
	num = 8;
	printf("%d\n", num);*/
	/*return 0;*/
//}
//enum colour
//{
//	red,
//		yellow,
//		bluw
//};
//int main()
//{
//	printf("%d\n", bluw);
//	return 0;
//}
//
//int main()
//{
//	char arr[] = "abc";//arr中储存的值分别是a、b、c、\0;其中，\0是字符串结束标志，
//	char arr1[] = { 'a','b','c' };//arr1中储存的是a、b、c；故二者是有区别的。将其改为char arr1{'a','b','c','0'}则运行的结果一致
//	printf("%s\n", arr);
//	printf("%s\n", arr1);
//	return 0;
//}
//
//
//int main()
//{
//	//printf("c\test\32\test.c");//\t:转义字符   \32:转义字符：即\ddd,ddd表示1~3个八进制的数字。32（八进制）-26（十进制）-Z（ASC11)
//	printf("%c\n",'\132');
//	return 0;
//int main()
//{
//	int input = 0;
//	printf("马上研一了\n");
//	printf("你要好好学习吗？\n 随机按下一个键决定你的命运<:");
//	scanf("%d", &input);
//	if (input == 0,6,2)
//		if (input == 2)
//			printf("变个傻子");
//		if (input == 0)
//			printf("回家种地");
//		if (input == 6)
//			printf("当个废物");
//	else
//		printf("在家喂猪");
//	return 0;
//}
//int main()
//{
//	int rank = 90;
//	printf("大傻子沉迷看电视\n");
//	while (rank < 2)
//	{
//		printf("每次看完电视后的智商：%d", rank);
//		rank--;
//	}
//	if (rank < 0)
//		printf("被卖到山里");
//	return 0;
//}
//int Add(int x, int y)    //注意此处结尾无分号
//{                        //注意要带大括号
//	int z = x + y;
//	return z;}
//
//int main()
//{
//	int num1 = 10;
//	int num2 = 2;
//	int sum = 0;
//	sum = Add(num1, num2);
//	printf("%d", sum);
//	return 0;
//}
//	int main()

//int main()
//{
//	int arr[9] = { 1,2,3,4,5,6,7,8,9 };
//	printf("%d\n", arr[7]);
//	int i = 0;
//	while (i <= 8)
//	{
//		printf("%d\n", arr[i]);
//		i++;
//	}
//	return 0;
//}
//int main()
//{
//	int a = 10;
//	printf("%d\n", sizeof(a));
//	printf("%d\n", sizeof a);//变量时两者皆可
//	printf("%d\n", sizeof(int));//注意类时必须要带括号
//	return 0;
//}
int main()
{
	int input = 0;
	printf("马上高三了\n");
	printf("你要好好学习吗？\n 随机按下一个键决定你的命运<:");
	scanf("%d", &input);
	if (input == 0, 6, 2)
		if (input == 2)
			printf("变个傻子");
	if (input == 0)
		printf("回家种地");
	if (input == 6)
		printf("当个废物");
	else
		printf("在家喂猪");
	return 0;
}
