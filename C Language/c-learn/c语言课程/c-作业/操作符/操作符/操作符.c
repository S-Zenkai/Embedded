#define _CRT_SECURE_NO_WARNINGS
#include <stdio.h>

//void print_parity(int a)
//{
//	int i = 0;
//	while (i < 16)
//	{
//		printf("%d ",a & 1);
//		a = a>>2;
//		i++;
//	}
//	printf("\n");
//}
//int main()
//{
//	int n = 55;
//	print_parity(n);
//	print_parity(n>>1);
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
//	int n = 0;
//	int i = 0;
//	int j = 0;
//	printf("输入：");
//	while (scanf("%d", &n) == 1)
//	{
//		for (i = 0; i < n; i++)
//		{
//			for (j = 0; j < n; j++)
//			{
//				if (i == j || (i + j) == (n - 1))
//				{
//					printf("%c", '*');
//				}
//				else
//				{
//					printf("%c", ' ');
//				}
//			}
//			printf("\n");
//		}
//	}
	/*for (i = 0; i < n; i++)
	{
		for (j = 0; j < n; j++)
		{
			if (i == j || (i + j) == (n - 1))
			{
				printf("%c", '*');
			}
			else
			{
				printf("%c", ' ');
			}
		}
		printf("\n");
	}*/
//	return 0;
//}

//int main()
//{
//	int i = 0;
//	int j = 0;
//	int z = 0;
//	int ret = scanf("%d", &i);
//	int rret = scanf("%d%d", &i, &j);
//	int rrret = scanf("%d%d%d", &i, &j,&z);
//	printf("%d %d %d", ret, rret,rrret);
//	printf("\n%d%d%d", i, j,z);
//	return 0;
//}
//打印整数二进制的奇数位和偶数位
//int main()
//{
//	int n = 0;
//	int i = 0;
//	scanf("%d", &n);
//	printf("偶数位：");
//	for (i = 31; i >= 0; i = i - 2)
//	{
//		printf("%d ", (n >> i) & 1);
//	}
//	printf("\n");
//	printf("奇数位：");
//	for (i = 30; i >= 0; i = i - 2)
//	{
//		printf("%d ", (n >> i) & 1);
//	}
//	return 0;
//}

//int i;//i=0;存放在静态区
//int main()
//{
//	i--;
//	if (i > sizeof(i))//sizeof操作符返回sizeof_t,是无符号整型，与有符号整形的i发生算术运算会发生算术转换，将（i=-1)转化为无符号整型
//	{
//		printf(">\n");
//	}
//	else
//	{
//		printf("<\n");
//	}
//	return 0;
//}

//获取某年某月有多少天
//int Judge_year(int n)
//{
//	if ((n % 4 == 0 && n % 100 != 0) || n % 400 == 0)
//		return 1;
//}
//int main()
//{
//	int year = 0;
//	int monch = 0;
//	int days[] = { 31,28,31,30,31,30,31,31,30,31,30,31 };
//	while (scanf("%d%d", &year, &monch) == 2)
//	{
//		int ret = Judge_year(year);
//		if (ret == 1 && monch == 2)
//			printf("%d", days[monch - 1] + 1);
//		else
//		    printf("%d\n", days[monch - 1]);
//
//	}
//	return 0;
//}