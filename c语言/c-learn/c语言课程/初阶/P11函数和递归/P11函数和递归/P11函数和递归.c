#define _CRT_SECURE_NO_WARNINGS
#include <stdio.h>

//void print(int x)
//{
//	if(x>10)
//	{
//		print(x / 10);
//	}
//	printf("%d ",x % 10);
//}
//int main()
//{
//	int a = 1234;
//	print(a);
//}

#include <string.h>
//int str_len(char* str)
//{
//	if (*str != '\0')
//		return (1 + str_len(str+1));//主要思路是采用递归的方法，每次剔除一个字符后+1；到\0处+0
//	else
//		return 0;
//}
//int main()
//{
//	char arr[] = { "welcome" };
//	int ret=str_len(arr);
//	printf("%d", ret);
//	return 0;
//}

//int Factorial(int x)
//{
//	int i = 1;
//	if (x != 1)
//	{
//		return x * Factorial(x - 1);
//	}
//	else
//		return 1;
//
//}
//int main()
//{
//	int n = 0;
//	int ret = 0;
//	scanf("%d", &n);
//    ret = Factorial(n);
//	printf("%d", ret);
//	return 0;
//}

int Fib(int n)
{
	int i = 0;
	int F1 = 1;
	int F2 = 1;
	int F = 1;
	if (n > 2)
	{
		for (i = 3; i <= n; i++)
		{
			F1 = F2;
			F2 = F;
			F = F1 + F2;
		}
		return F;
	}
	else
		return 1;
}
int main()
{
	int i = 0;
	scanf("%d",&i);
	int ret = Fib(i);
	printf("%d", ret);
	return 0;
}