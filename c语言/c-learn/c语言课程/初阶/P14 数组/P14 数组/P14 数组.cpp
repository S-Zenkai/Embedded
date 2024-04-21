#define _CRT_SECURE_NO_WARNINGS
#include <stdio.h>
//void Bubble_sort(int arr[], int sz)
//{
//	int i = 0;
//	int j = 0;
//	int tmp = 0;
//	for (i = 0; i < sz-1; i++)
//	{
//		int flag = 1;//改进后的冒泡算法，如果一次比较时arr顺序并没有改变，则已完成排序
//		for (j = 0; j < sz-i-1; j++)
//		{
//			if (arr[j] >= arr[j+1])
//			{
//				tmp = arr[j+1];
//				arr[j+1] = arr[j];
//				arr[j] = tmp;
//				flag = 0;
//			}
//		}
//		if (flag == 1)
//		{
//			break;
//		}
//	}
//}
//int main()
//{
//	int i = 0;
//	int arr[] = { 2,1,3,4,5,6,7,8,9 };
//	int sz = sizeof(arr) / sizeof(arr[1]);
//	Bubble_sort(arr, sz);
//	for (i = 0; i < sz ; i++)
//	{
//		printf("%d ", arr[i]);
//	}
//	return 0;
//}

int main()
{
	int arr[] = { 2,1,3,4,5,6,7,8,9 };
	printf("%p\n", arr);
	printf("%p\n", arr + 1);
	printf("%p\n", &arr[0]);
	printf("%p\n", &arr[0]+1);
	printf("%p\n", &arr);
	printf("%p\n", &arr+1);
	return 0;
}