#define _CRT_SECURE_NO_WARNINGS
#include <stdio.h>
//int main()
//{
//	char* p = "hello bit.";
//	printf("%s\n", p);
//	return 0;
//}

//int main()
//{
//    int i = 0;
//    int arr[10] = { 0 };
//    for (i = 0; i < 10; i++)
//    {
//        arr[i] = 1;
//        printf("hehe\n");
//        int a = 0;
//    }
//    for (i = 0; i < 10; i++)
//    {
//        printf("haha\n");
//    }
//    return 0;
//}

//int main()
//{
//	int a[10] = { 1,2,3,4,5,6,7,8,9,10 };
//	return 0;
//}

//void test2()
//{
//	printf("haha");
//}
//void test1()
//{
//	test2();
//}
//void test()
//{
//	test1();
//}
//void test3()
//{
//	printf("hehe");
//}
//int main()
//{
//	test();
//	test3();
//	return 0;
//}



int main()
{
    int i = 0;
    int arr[10] = { 1,2,3,4,5,6,7,8,9,10 };
    for (i = 0; i <= 12; i++)
    {
        arr[i] = 0;
        printf("hehe\n");
    }
    return 0;
}

//int Add(int x, int y)
//{
//	return x + y;
//}
//int main()
//{
//	int a = 10;
//	int b = 20;
//	int c = Add(a, b);
//	return 0;
//}

//void Fun(int* x)
//{
//	//
//}
//int main()
//{
//	int arr[] = { 0,1,2,3,4,5,6,7,8,9 };
//	Fun(arr);
//	return 0;
//}
//自己写一个stycpy函数
//void my_strcpy(char* dest, char* src)
//{
//	*dest = *src;
//	while ( *src++ )
//	{
//		dest++;
//		*dest = *src;
//	}
//}

#include <assert.h>
//const防止src被改变，char*可以实现链式访问
//char* my_strcpy(char* dest, const char* src)
//{
//	char* ret = dest;
//	//防止空指针
//	assert(dest != NULL);
//	assert(src != NULL);
//	//对于*dest++ = *src++；计算顺序：先*src=*dest(返回*dest的ASC码值），再++；
//	while (*dest++ = *src++)
//	{
//
//	}
//	return ret;
//}
//int main()
//{
//	char arr1[20] = { "hellow world"};
//	char arr2[20] = { "111111111111111"};
//	my_strcpy(arr2, arr1);
//	//printf("%s\n", arr2);
//	printf("%s\n", my_strcpy(arr2, arr1));
//	return 0;
//}

//const的用法
//int main()
//{
//	const int num = 1;
//	int num1 = 0;
	//err,变量前加const变量不能通过直接赋值更改
	//num = 10;
	// 通过解引用操作改变num的值
	//int* pn = &num;
	//*pn = 10;
	//printf("%d\n", num);
	//err，const加在*左边，不可以通过解引用改变num的值,但可以改变pn
	//const int* pn = &num;
	//*pn = 10;
	//pn = &num1;
	//const放在*的右边,可以通过解引用改变num的值，但不能改变pn的值
//	int* const pn = &num;
//	*pn = 10;
//	printf("%d\n", num);
//	pn = &num1;
//	return 0;
//}

//int main()
//{
//
//}


