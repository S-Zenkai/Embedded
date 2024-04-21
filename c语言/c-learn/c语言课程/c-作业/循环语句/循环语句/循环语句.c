#define _CRT_SECURE_NO_WARNINGS
#include <stdio.h>
//int main()//数9的个数
//{
//	int n = 1;
//	int i = 0;
//	for (n = 1; n <= 100; n++)
//	{
//		if (n % 10 == 9)//计算个位上的9
//			i++;
//		if (n / 10 == 9)//计算十位上的9
//			i++;//注意，这里如果用else if（,,,),最后i=19，原因是在99时，if计算完个位上的9，elseif便不执行了，二者并不是并列关系
//	}
//	printf("%d", i);
//	return 0;
//}

//int main()
//{
//	int i = 0;
//	double count = 0.0;
//	for (i = 1; i <= 100; i++)
//	{
//		if (i % 2 == 0)
//			i = -i;
//		count = count + 1.0 / i;
//		if (i % 2 == 0)
//			i = -i;
//	}
//	printf("%lf", count);
//	return 0;
//}

int main()
{
	int i = 0;
	double media = 0;
	double n = 0;
	for (i = 1; i <= 10; i++)
	{
		if ( i % 2 == 0)
			media = -1 / i;
		else
			media = 1 / i;
		n = n + media;
	}
	printf("%lf", n);
		return 0;
}

//int main()//求10个整数的最大值
//{
//	int arr[] = { 15,65,87,152,45,65,48,78,12,48 };
//	int i = 0;
//	int j = 0;
//	int media = 0;
//	int a = 0;
//	for (i = 0; i <= 9; i++)
//	{
//		a++;
//		for ( j=a ; j <= 9; j++)
//		{
//			if (arr[i] < arr[j])
//			{
//				media = arr[j];
//				arr[j] = arr[i];
//				arr[i] = media;
//			}
//		}
//	}
//	printf("%d\n", arr[0]);
//	for (int k = 0; k <= 9; k++)
//		printf("%d ", arr[k]);
//	return 0;
//}

//int main()//求10个数的最大值，主要思路是假设max为最大值，和arr中的元素相比，当有比max大的时将max替代
//{
//	int arr[] = { -15,-48,-51,-65,-5,-47,-14,-451,-11,-215,-23 };
//	int max = arr[0];//注意，这里如果使max=0，那么当arr中的元素都为负数的话会导致最后显示的结果为0
//	int i = 0;
//	for (i = 0; i <= sizeof(arr) / sizeof(arr[0]) - 2; i++)
//	{
//		if (max <= arr[i])
//		{
//			max = arr[i];
//		}
//	}
//	printf("%d", max);
//	return 0;
//}

//int main()
//{
//	int a = 0;
//	int b = 0;
//	int c = 0;
//	for (b = 1; b <= 9; b++)
//	{
//		for (a = 1; a <= b; a++)
//		{
//			c = a * b;
//			printf("%d*%d=%-2d ",a, b, c);//2d:两位数，不足两位以空格代替（可实现右对齐的效果）。-2d：同上，可实现左对齐的效果
//		}
//		printf("\n");
//	}
//	return 0;
//}

//int main()
//{
//	abc://注意，标志后面是：，使用goto语句后会跳到abc处执行下面的语句
//	printf("hello 11");
//	goto abc;
//	return 0;
//}
//#include <stdlib.h>
//#include <string.h>
//int main()
//{
//	char arr[100] = { 0 };
//	while(1)
//	{
//		system("shutdown -s -t 60");
//		printf("请输入“我是猪”取消关机");
//		scanf("%s", &arr);
//		if (strcmp(arr, "我是猪") == 0)
//		{
//			system("shutdown -a");
//			break;
//		}
//	}
//	return 0;
//}

//#include <stdio.h>
//int main()
//{
//    char input[10] = { 0 };
//    system("shutdown -s -t 60");
//again:
//    printf("电脑将在1分钟内关机，如果输入：我是猪，就取消关机!\n请输入:>");
//    scanf("%s", input);
//    if (0 == strcmp(input, "我是猪"))
//    {
//        system("shutdown -a");
//    }
//    else
//    {
//        goto again;
//    }
//    return 0;
//}

//int main()
//{
//	char arr[100] = { 0 };
//	while(1)
//	{
//		system("shutdown -s -t 60");
//		printf("请输入“我是猪”取消关机");
//		scanf("%s", &arr);
//		if (strcmp(arr, "我是猪") == 0)
//		{
//			system("shutdown -a");
//			break;
//		}
//	}
//	return 0;
//}