#define _CRT_SECURE_NO_WARNINGS
#define _CRT_SECURE_NO_WARNINGS
#include <stdio.h>
#include <math.h>
//int main()
//{
//	int a = 0;
//	int b = 0;
//	int c = 0;//存放余数
//	int d = 0;//存放初始a
//	int e = 0;//存放初始b
//	printf("a=");
//	scanf("%d", &a);
//	printf("b=");
//	scanf("%d", &b);
//	d = a;//暂存a、b，为下面的显示a和b的最大公约数提供条件
//	e = b;
//	while(b!=0)//通过辗转相除法实现求最大公因数
//	{
//		c = a % b;
//		if (c == 0)
//		{
//			printf("%d和%d的最大公因数为：%d",d,e,b);
//			break;
//		}
//		a = b;//相处后将b（除数）赋值给a（被除数），c（余数）赋值给b
//		b = c;
//	}
//
//	return 0;
//}

//int main()
//{
//	int a = 0;
//	int b = 0;
//	int c = 0;
//	int d = 0;
//	scanf("%d%d%d", &a ,&b,&c);
//	if (a <= b)
//	{
//		d = b;
//		b = a;
//		a = d;
//		if (b <= c)
//		{
//			d = c;
//			c = b;
//			b = d;
//			if (a <= b)
//			{
//				d = b;
//				b = a;
//				a = d;
//			}
//		}
//	}
//	else
//	{
//		if (b <= c)
//		{
//			d = c;
//			c = b;
//			b = d;
//			if (a <= b)
//			{
//				d = b;
//				b = a;
//				a = d;
//			}
//		}
//	}
//	printf("%d %d %d", a, b, c);
//	return 0;
//}

//int main()//通过a与b、a与c、b与c的比较和调换位置实现abc从大到小排序
//{
//	int a = 0;//a、b、cb比较大小
//	int b = 0;
//	int c = 0;
//	scanf("%d%d%d", &a, &b, &c);
//	int tmp = 0;//中间变量，用以储存abc的值（tmp代表临时文件）
//	if (a < b)//a与b比较，a<b时两者交换数值
//	{
//		tmp = b;
//		b = a;
//		a = tmp;
//	}
//	if (a < c)//a与c比较
//	{
//		tmp = c;
//		c = a;
//		a = tmp;
//	}
//	if (b < c)//b于c比较
//	{
//		tmp = c;
//		c = b;
//		b = tmp;
//	}
//	printf("%d %d %d", a, b, c);
//	return 0;
//}

//int main()
//{
//	int a = 1;
//	while(a<=100)//限定1<=a<=100
//	{
//		if (a % 3 == 0)
//			printf("%d\n", a);
//		a++;
//	}
//	return 0;
//}

//int main()//打印闰年
//{
//	int year = 1000;
// int count=0;
//	while(1000<=year&&year<=2000)//注意不能写成1000<=year<=2000
//	{
//		if (year % 100 != 0&& year % 4 == 0)//判断闰年的两种方法之一，不能被100整除能被4整除是闰年
//		{
//			printf("%d\n", year);
//          count++;
//		}
//		if(year%100==0&& year % 400 == 0)//能被400整除是闰年
//		{
//			printf("%d\n", year);
//          count++;
//		}
//		year++;
//	}
//	printf("\n%d", count);
//	return 0;
//}

//int main()
//{
//	int a = 100;
//	int i = 2;
//	while(a<=200)
//	{
//		for (i = 2; i <= a; i++)
//		{
//			if (a % i == 0)
//				break;
//		}
//		if (i == a)
//			printf("%d\n", a);
//		a++;
//	}
//	return 0;
//}

//int main()
//{
//	int a = 100;
//	int i = 1;
//	int k = 0;
//	int b = 0;
//	while(100<=a&&a<=200)
//	{
//		while (i * i <= a)
//		{
//			i++;
//		}
//		b = --i;//a的根号向下取整
//		for (k = 2; 2 <= k && k <= b; k++)
//		{
//			if (a % k == 0)
//				break;
//		}
//		if (k == b + 1)
//			printf("%d\n", a);
//		a++;
//	}
//	return 0;
//}

//int main()
//{
//	int a = 0;
//	int b = 0;
//	int c = 0;
//	scanf("%d%d", &a, &b);
//	while (a % b)
//	{
//		c = a % b;
//		a = b;
//		b = c;
//	}
//	printf("%d", b);
//	return 0;
//}

//int main()
//{
//	int year = 0;
//	int count = 0;
//	for (year = 1000; year <= 2000; year++)
//	{
//		if (year % 100 != 0 && year % 4 == 0 || year % 400 == 0)
//		{
//			printf("%d ", year);
//			count++;
//		}
//
//	}
//	printf("\n%d", count);
//	return 0;
//}

//int main()
//{
//	int a = 100;
//	int i = 0;
//	int j = 0;
//	for(a=101;a<=200;a+=2)
//	{
//		for (i = 2; i <= sqrt(a); i++)//若对于2到根下a之间的所有整数，a均不能整除，则a为素数
//		{
//			if (a % i == 0)
//				break;
//		}//不论能否除到根号a均跳出循环，当i>根号a时为素数
//		if (i > sqrt(a))
//		{
//			printf("%d\n", a);
//			j++;
//		}
//	}
//	printf("\n%d", j);
//	return 0;
//}

