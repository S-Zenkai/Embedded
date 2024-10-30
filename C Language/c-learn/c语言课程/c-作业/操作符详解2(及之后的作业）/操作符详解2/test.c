#define _CRT_SECURE_NO_WARNINGS
#include <stdio.h>

//int main()
//{
//    int arr[] = { 1,2,3,4,5 };//在内存中：0100000002000000030000000400000005000000
//    short* p = (short*)arr;//short*只能访问两个字节
//    int i = 0;
//    for (i = 0; i < 4; i++)
//    {
//        *(p + i) = 0;
//    }
//
//    for (i = 0; i < 5; i++)
//    {
//        printf("%d ", arr[i]);
//    }
//    return 0;
//}

//int i;//i=0;
//int main()
//{
//    i--;//i=-1
//    if (i > sizeof(i))
//    {
//        printf(">\n");
//    }
//    else
//    {
//        printf("<\n");
//    }
//    return 0;
//}

//int Sn(int a,int n)
//{
//	int Sn = 0;
//	int i = 0;
//	for (i = 0; i <  n; i++)
//	{
//		Sn = Sn + a;
//		a = a * 10 + 2;
//
//	}
//	return Sn;
//}
//int main()
//{
//	int a = 0;
//	int n = 0;
//	scanf("%d%d", &a, &n);
//	int ret = Sn(a,n);
//	printf("%d\n", ret);
//	return 0;
//}

//求出0～100000之间的所有“水仙花数”并输出。
//int Digit(int n)
//{
//	int count = 1;
//	while (n = n / 10)
//	{
//		count++;
//	}
//	return count;
//}
//int Narcissistic_number(int n)
//{
//	int tmp1 = 0;
//	int tmp2 = 0;
//	int tmp3 = 0;
//	int sum = 0;
//	int ret = 0;
//	int mul = 1;
//	ret = Digit(n);
//	tmp2 = n;
//	while (n)
//	{
//		mul = 1;
//		tmp3 = ret;
//		tmp1 = n % 10;
//		while (tmp3)
//		{
//			mul = mul * tmp1;
//			tmp3--;
//		}
//		sum = sum + mul;
//		n = n / 10;
//	}
//	if (sum == tmp2)
//	{
//		return tmp2;
//	}
//	else
//		return -1;
//}
//int main()
//{
//	int n = 0;
//	int ret = 0;
//	for (n = 0; n <= 100000; n++)
//	{
//		ret = Narcissistic_number(n);
//		if(ret != -1)
//		{
//			printf("%d\n", ret);
//		}
//	}
//	return 0;
//}
#include <math.h>
//int main()
//{
//	int i = 0;
//	for (i = 0; i <= 999999; i++)
//	{
//		int count = 1;
//		int tmp = i;
//		int sum = 0;
//		//判断i是否为水仙花数
//		//1. 求判断数字的位数
//		while (tmp / 10)
//		{
//			count++;
//			tmp = tmp / 10;
//		}
//
//		//2. 计算每一位的次方和
//		tmp = i;
//		while (tmp)
//		{
//			sum += pow(tmp % 10, count);
//			tmp = tmp / 10;
//		}
//
//		//3. 判断
//		if (sum == i)
//			printf("%d ", i);
//	}
//	return 0;
//}

//int main()
//{
//	int n = 0;
//	int tmp = 0;
//	int sum = 0;
//	for (n = 0; n <= 100000; n++)
//	{
//		int count = 1;
//		sum = 0;
//		tmp = n;
//		while (tmp = tmp / 10)
//		{
//			count++;
//		}
//		tmp = n;
//		while(tmp)
//		{
//			sum = sum + pow(tmp % 10, count); 
//			tmp = tmp / 10;
//		}
//		if (n == sum)
//			printf("%d ", n);
//	}
//	return 0;
//}

//////使用指针打印数组内容
//int main()
//{
//	int arr[] = { 1,2,3,4,5,6,7,8,9 };
//	int i = 0;
//	int sz = sizeof(arr) / sizeof(arr[1]);
//	int* p = arr;
//	for (i = 0; i < sz; i++)
//	{
//		printf("%d ", *p);
//		p++;
//	}
//	return 0;
//}








///////VS环境调试技巧和演示

//int main()
//{
//	int i = 0;
//	int arr[] = { 1,2,3,4,5,6,7,8,9,10 };//栈区存储：由高地址到低地址；数组存储由低地址到高地址
//	for (i = 0; i <= 12; i++)
//	{
//		arr[i] = 0;
//		printf("hello bit\n");
//	}
//	return 0;
//}

//调整数组使奇数全部都位于偶数前面。

//int main()
//{
//	int arr[10] = { 0 };
//	int i = 0;
//	int count = 0;
//	int tmp = 0;
//	for (i = 0; i < 10; i++)
//	{
//		scanf("%d", &arr[i]);
//	}
//	for (i = 0; i < 10; i++)
//	{
//		if (arr[i] % 2 != 0)
//		{
//			count++;
//			tmp = arr[i];
//			arr[i] = arr[count - 1];
//			arr[count - 1] = tmp;
//		}
//	}
//	for (i = 0; i < 10; i++)
//	{
//		printf("%d ", arr[i]);
//	}
//	return 0;
//}

//三角形判断
//int main()
//{
//	int arr[ 6 ] = { 0 };
//	int i = 0;
//	int tmp = 0;
//	int j = 2;
//	for(i=0;i<6;i++)
//	{
//		scanf("%d", &arr[i]);
//	}
//	//判断长短边
//	for (i = 0; i <= 3; i += 3)
//	{
//		if (arr[i] > arr[i + 1])
//		{
//			tmp = arr[i];
//			arr[i] = arr[i + 1];
//			arr[i + 1] = tmp;
//		}
//		if (arr[i + 1] > arr[i + 2])
//		{
//			tmp = arr[i + 1];
//			arr[i + 1] = arr[i + 2];
//			arr[i + 2] = tmp;
//		}
//		if (arr[i] > arr[i + 1])
//		{
//			tmp = arr[i];
//			arr[i] = arr[i + 1];
//			arr[i + 1] = tmp;
//		}
//	}
//	for (i = 0; i <= 3; i += 3)
//	{
//		if (arr[i] + arr[i + 1] > arr[i + 2])
//		{
//			if (arr[i] == arr[i + 1] && arr[i] != arr[i + 2])
//			{
//				printf("等腰三角形\n");
//			}
//			else if (arr[i] == arr[i + 1] == arr[i + 2])//arr[i] == arr[i + 1] == arr[i + 2]的实现应该用a==b&&b==c;
//			{
//				printf("等边三角形\n");
//			}
//			else
//				printf("普通三角形\n");
//		}
//		else
//			printf("不是三角形\n");
//	}
//}

//int main()
//{
//	int a, b, c;
//	while (scanf("%d %d %d", &a, &b, &c) == 3)
//	{
//		if (a + b > c && a + c > b && b + c > a)
//		{
//			if (a == b && b == c)
//			{
//				printf("等边三角形");
//			}
//			else if (a == b && b != c || b == c && a != b || c == a && a != b)
//			{
//				printf("等腰三角形");
//			}
//			else
//				printf("普通三角形");
//		}
//		else
//		{
//			printf("不是三角形");
//		}
//	}
//	return 0;
//}



//int main()
//{
//	char arr[20] = { 0 };
//	//scanf("%s", &arr);//这里scanf不能在输入的字符串中包含空格，可以用gets代替
//	gets(arr);
//	int sz = strlen(arr);
//	char tmp = 0;
//	int i = 0;
//	while(i < sz - 1 - i)
//	{
//		tmp = arr[i];
//		arr[i] = arr[sz - 1 - i];
//		arr[sz - 1 - i] = tmp;
//		i++;
//	}
//	printf("%s", arr);
//=======
//int main()
//{
//	int arr[10] = { 0 };
//	int i = 0;
//	int count = 0;
//	int tmp = 0;
//	for (i = 0; i < 10; i++)
//	{
//		scanf("%d", &arr[i]);
//	}
//	for (i = 0; i < 10; i++)
//	{
//		if (arr[i] % 2 != 0)
//		{
//			count++;
//			tmp = arr[i];
//			arr[i] = arr[count - 1];
//			arr[count - 1] = tmp;
//		}
//	}
//	for (i = 0; i < 10; i++)
//	{
//		printf("%d ", arr[i]);
//	}
//>>>>>>> cdf16b330d4fd60800a7b21a50ed00aea1df512f
//	return 0;
//}

//#include <windows.h>
//int main()
//{
//	while (1)
//	{
//		printf("hehe");
//		Sleep(1000);
//	}
//	return 0;
//}

//int main()
//{
//	char ch1[] = { "welcome to doming province" };//总体思想是用ch1逐个替代ch2，以实现多个字符从两端移动，向中间汇集
//	int sz = sizeof(ch1) / sizeof(ch1[0]) - 2;//因定义的ch1中结尾有\n,在计算sizeof时把其也计算其中，故需减2
//	//int sz = strlen(ch1)-1;//实现的效果与上一段代码一致，strlen计算时并不把\n算入其中
//	char ch2[] = { "##########################" };
//	int r = 0;//左下标
//	int l = 0;//右下标
//	for (r = 0, l = sz; r <= l; r++, l--)
//	{
//		ch2[r] = ch1[r];
//		ch2[l] = ch1[l];
//		printf("%s", ch2);
//		Sleep(1000);//暂停1s（1000ms），注意S大写
//		system("cls");//清屏
//	}
//	printf("%s", ch2);
//	return 0;
//}

//int main()
//{
//	int arr[] = { 2,4,6,8,10,12,14,16,18,20 };
//	int count = 0;
//	int i = 0;
//	int tmp = 0;
//	for (i = 0; i < 10; i++)
//	{
//			do
//			{
//				if (arr[i] % 2 == 0)
//				{
//					tmp = arr[i];
//					arr[i] = arr[9 - count];
//					arr[9 - count] = tmp;
//					count++;
//				}
//			} while (count<9 && arr[i] % 2 == 0);
//		if(count>=9)
//		{
//			break;
//		}
//	}
//	for (i = 0; i < 9; i++)
//	{
//		printf("%d ", arr[i]);
//	}
//	return 0;
//}

//Fun(int* arr, int sz)
//{
//	int r = 0;
//	int l = sz - 1;
//	int tmp = 0;
//	while(r<l)
//	{
//		while (r<sz&&arr[r] % 2 == 1)
//		{
//			r++;
//		}
//		while (l>0&& arr[l] % 2 == 0)
//		{
//			l--;
//		}
//		tmp = arr[l];
//		arr[l] = arr[r];
//		arr[r] = tmp;
//	}
//}
//int main()
//{
//	int arr[] = { 2,4,6,8,10,12,14,16,18,20 };
//	int sz = sizeof(arr) / sizeof(arr[0]);
//	int i = 0;
//	Fun(arr, sz);
//	for (i = 0; i < 9; i++)
//	{
//		printf("%d ", arr[i]);
//	}
//	return 0;
//}

//void Merge(int* arr1, int* arr2, int* arr3, int m, int n)
//{
//	int i = 0;
//	int j = 0;
//	int k = 0;
//	while (i < m && j < n)
//	{
//		if (arr1[i] <= arr2[j])
//		{
//			arr3[k] = arr1[i];
//			i++;
//			k++;
//			if (i == m)
//			{
//				while(j<n)
//				{
//					arr3[k] = arr2[j];
//					j++;
//					k++;
//				}
//				break;
//			}
//		}
//		if (arr1[i] > arr2[j])
//		{
//			arr3[k] = arr2[j];
//			j++;
//			k++;
//			if (j == n)
//			{
//				while (i < m)
//				{
//					arr3[k] = arr1[i];
//					i++;
//					k++;
//				}
//				break;
//
//			}
//		}
//
//	}
//}
//int main()
//{
//	int m = 0;
//	int n = 0;
//	scanf("%d%d", &m, &n);
//	int arr1[100] = { 0 };
//	int arr2[100] = { 0 };
//	int arr3[100] = { 0 };
//	int i = 0;
//	for(i=0;i<m;i++)
//	{
//		scanf("%d", &arr1[i]);
//	}
//	for (i = 0; i < n; i++)
//	{
//		scanf("%d", &arr2[i]);
//	}
//	Merge(arr1, arr2, arr3, m, n);
//	for (i = 0; i < m + n; i++)
//	{
//		printf("%d ", arr3[i]);
//	}
//	return 0;
//}

typedef enum
{
	GPIO_Speed_10MHz = 1,
	GPIO_Speed_2MHz,
	GPIO_Speed_50MHz
}GPIOSpeed_TypeDef;
int main()
{
	GPIOSpeed_TypeDef A;
	A = 5;
	return 0;
}
