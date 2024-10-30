#define _CRT_SECURE_NO_WARNINGS
#include <stdio.h>
//int Mul(int x, int y)
//{
//	int z = x * y;
//	return z;
//}
//int main()
//{
//	int a = 2;
//	int b = 4;
//	int z = Mul(a, b);
//	printf("%d", z);
//	return 0;
//}

//int main()
//{
//	int a = 0;//a在内存中存储是00000000000000000000000000000000
//	int b = ~a;//~指将a对应的二进制数按位取反，即11111111111111111111111111111111，首位是1代表负数，在内存中是以补码的形式存储
//	//原码到反码：符号位不变，其余位取反。反码到补码：补码加1
//	printf("%d", ~b);
//}

//#define Max(x,y) x>y?x:y
//int main()
//{
//	int a = 10;
//	int b = 2;
//	int c = 0;
//	c = Max(a, b);
//	printf("%d", c);
//	return 0;
//}

//int main()
//{
//	int m = 2;
//	int n = 1;
//	switch (n)
//	{
//	case 1:
//	case 2:
//		m++;
//		n++;
//	case 3:
//		switch (n)
//		{
//		case 1:
//			n++;
//		case 2:
//			m++;
//			n++;
//			break;//此处break结束的是第二个switch,故case4仍会执行,你直接把其跳到default了
//		}
//	case 4:
//		m++;
//		break;
//	default:
//		break;
//	}
//	printf("m=%d\nn=%d", m, n);
//	return 0;
//}

//int main()
//{
//	int arr[] = { 1,2,3,4,5,6,7,8,9 };
//	int l = 0;
//	int r = sizeof(arr)/4-1;
//	int n = 0;
//	scanf("%d", &n);
//	while(arr[(l+r)/2]!=n)
//	{
//		if (arr[(l + r) / 2] < n)
//		{
//			l = (l + r) / 2+1;
//		}
//		else if (arr[(l + r) / 2] > n)
//		{
//			r = (l + r) / 2-1;
//		}
//	}
//	printf("n=arr[%d]", (l + r) / 2);
//	return 0;
//}

//int main()
//{
//	int arr[] = { 1,2,3,4,5,7,8,9 };//有序数组
//	int l = 0;//左下标
//	int r = sizeof(arr) / sizeof(arr[0]) - 1;//右下标
//	int n = 0;//要查找的数字
//	printf("n=");
//	scanf("%d", &n);
//	while (l < r)//只有当左下标小于右下标时才可能查找到n
//	{
//		if (n < arr[(l + r) / 2])//n小于左右下标平均值对应的数字，这取右下标为左右下标平均值减一
//		{
//			r = (l + r) / 2 - 1;
//		}
//		else if (n > arr[(l + r) / 2])//与上同理
//		{
//			l = (l + r) / 2+1;
//		}
//		else
//		{
//			printf("arr中存在%d，位于下标%d", n, (l + r) / 2);
//		}
//	}
//	if (l >= r)
//		printf("arr中不存在%d", n);
//	return 0;
//}

//int main()
//{
//	char arr1[] = { "welcome to Hezhe" };
//	char arr2[] = { "****************" };
//	int i = 0;
//	int j = sizeof(arr1) / sizeof(arr1[0])-2;
//	for ( ; i <= j; i++, j--)
//	{
//		arr2[i] = arr1[i];
//		arr2[j] = arr1[j];
//		printf("%s\n", arr2);
//		Sleep(1000);
//		system("cls");
//	}
//	printf("%s", arr2);
//	return 0;
//}

int B(int a)
{
	a = 3;
	return a;
	a = 4;
	return a;
}
int main()
{
	int a = 0;
	int ret = 0;
	ret = B(a);
	printf("%d", ret);
	return 0;
}
