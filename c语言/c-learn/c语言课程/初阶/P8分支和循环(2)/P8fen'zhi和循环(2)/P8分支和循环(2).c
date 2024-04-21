#define _CRT_SECURE_NO_WARNINGS
#include <stdio.h>
//int main()
//{
//	char pw[20] = { 0 };
//	int a = 0;
//	printf("请输入密码:<");
//	scanf("%s", &pw);//输入123456
//	printf("请确认(X/Y):<");
//	if((a=getchar())=='Y')
//		printf("审核通过");
//	else
//		printf("密码错误");//代码直接输出密码错误,原因是存在缓冲区,输入123456后要打回车键,这样缓冲区中还有字符'\n',a=getchar()获取\n直接输出密码错误
//	return 0;
//}

//int main()
//{
//	int a = 0;
//	char pw[20] = { 0 };
//	printf("请输入密码:<");
//	scanf("%s", &pw);
//	printf("请确认(Y/N):<");
//	while (getchar() != '\n')
//	{
//		;
//	}
//	if ((a = getchar()) == 'Y')
//		printf("确认成功");
//	else
//		printf("取消确认");
//	return 0;
//}

//int main()
//{
//	int i = 1;
//	while (i <= 10)
//	{
//		if (i == 5)
//			continue;//i=5时程序跳转到判断语句,程序陷入死循环
//		printf("%d ", i);
//		i++;
//	}
//	return 0;
//}

//int main()
//{
//	int i = 0;
//	for (i = 1; i <= 10; i++)
//	{
//		if (i == 5)
//			continue;//i=5后,跳过之后的打印语句,执行调整语句i++,故结果不打印5
//		printf("%d ", i);
//	}
//	return 0;
//}

//int main()
//{
//	int arr[10] = { 1,2,3,4,5,6,7,8,9,10 };
//	int i = 0;
//	for (i = 0; i < 10; i++)//0<=i<10,前闭后开
//		printf("%d ", arr[i]);
//	return 0;
//}

//int main()
//{
//	int i = 0;
//	int j = 0;
//	int z = 0;
//	for (; i < 10; i++)
//	{
//		for (; j < 10; j++)
//		{
//			z++;
//			printf("hehe%d ", z);//打印结果为10个hehe,原因是:在i=0时进入j循环,j=10时跳出j循环,i=1后又进入j循环,但因j循环缺少初始化条件,故j仍=10,跳出j循环,之后类似情况
//		}
//	}
//	return 0;
//}

//int main()
//{
//	int x, y;
//	for (x = 0, y = 0; y = 0; x++, ++y)
//		y++;//循环一次都不进行，原因是经过初始化后，进入判断条件，y=0为假，不进入循环。
//	return 0;
//}

//int main()
//{
//	int x, y;
//	for (x = 0, y = 0; x < 2 && y < 5; y++, x++)
//		printf("hehe ");
//	return 0;
//}

//int main()
//{
//	int i = 1;
//	do
//	{
//		if (i == 5)
//			continue;//程序在i=5时陷入死循环，原因是i=5时，跳过循环之后的语句，直接跳转到判断语句后，i=5，判断为真，继续循环，i仍等于5，进入死循环
//		printf("%d ", i);
//		i++;
//	}
//	while (i<11);
//	return 0;
//}

//int main()
//{
//	int n = 0;
//	int i = 1;
//	int ret1 = 1;
//	int ret2 = 0;
//	printf("请输入n=");
//	scanf("%d", &n);
//	for (i = 1; i <= n; i++)
//	{
//		ret1 = ret1 * i;
//		ret2 = ret2 + ret1;
//	}
//	printf("n!=%d\n", ret1);
//	printf("n!+(n-1)!+...+1!=%d", ret2);
//	return 0;
//}

//int main()
//{
//	int n = 1;
//	int i = 1;
//	int ret = 0;
//	for (i = 1; i <= 10; i++)
//	{
//		n = i * n;
//		ret = n + ret;
//	}
//	printf("%d", ret);
//	return 0;
//}

//int main()
//{
//	int ret = 1;
//	int sum = 0;
//	int i = 1;
//	int n = 0;
//	for (sum = 0; i <= 10; i++)
//	{
//		for (n = 1; n <= i; n++)
//		{
//			ret = n * ret;
//		}
//		sum = sum + ret;
//		ret = 1;//注意要加上ret=1以使每次内部循环完将ret初始化，否则内部循环产生的效果是n！*n！（因每次内部循环完ret都得到保留，那么下次循环n*ret便是n！*n！
//	}
//	printf("sum=%d", sum);
//	return 0;
//}

//int main()
//{
//
//	int arr[] = {1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18};
//	int n = 0;
//	int a = sizeof(arr)/4;
//	int i = 0;
//	int j = 0;
//	printf("n=");
//	scanf("%d", &n);
//	for (i = 0; i <= a; i++)
//	{
//		if (n == arr[i])
//		{
//			break;
//		}
//	}
//	printf("arr中%d的个数是;%d", n, j);
//	return 0;
//}

//int main()
//{
//	int arr[] = { 1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18 };
//	int sz = sizeof(arr) / sizeof(arr[0]);
//	int a = sz / 2;
//	int b = sz / 2;
//	int n = 0;
//	int i = 0;
//	printf("n=");
//	scanf("%d", &n);
//	for (i = 0; i < sz; i ++)
//	{
//		if (arr[a] < n)
//		{
//			b = b / 2;
//			a = a + b;
//		}
//		else if (arr[a] > n)
//		{
//			b = b / 2;
//			a = a - b;
//		}
//		else
//		{
//			printf("在arr中找到n，位于下表%d", a);
//			break;
//		}
//	}
//	return 0;
//}

//int main()
//{
//	int arr[] = { 1,2,3,4,5,7,8,9 };//有序数组
//	int l = 0;//左下标
//	int r = sizeof(arr)/sizeof(arr[0])-1;//右下标
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
//			l = (1 + r) / 2 + 1;
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

int main()
{
	char ch1[] = {"welcome to doming province"};//总体思想是用ch1逐个替代ch2，以实现多个字符从两端移动，向中间汇集
	int sz = sizeof(ch1) / sizeof(ch1[0])-2;//因定义的ch1中结尾有\n,在计算sizeof时把其也计算其中，故需减2
	//int sz = strlen(ch1)-1;//实现的效果与上一段代码一致，strlen计算时并不把\n算入其中
	char ch2[] = {"##########################"};
	int r = 0;//左下标
	int l = 0;//右下标
	for (r = 0, l = sz; r<=l; r++, l--)
	{
		ch2[r] = ch1[r];
		ch2[l] = ch1[l];
		printf("%s", ch2);
		Sleep(1000);//暂停1s（1000ms），注意S大写
		system("cls");//清屏
	}
	printf("%s", ch2);
	return 0;
}

//int main()
//{
//	char password[20] = {"abc123456"};//创建密码
//   	char ch[20] = "0";
//	int i = 0;
//	printf("请输入密码:");
//	scanf("%s", &ch);//输入密码
//	if ( strcmp (ch,password) != 0)//比较两个字符串的大小要用strcmp，如果两个字符相等，则返回0
//	{
//		for (i = 0; i < 2; i++)
//		{
//			printf("密码错误\n");
//			printf("请输入密码：");
//			scanf("%s", &ch);
//			if (strcmp(ch, password) == 0)
//			{
//				printf("登录成功");
//				break;
//			}
//		}
//	}
//	else
//		printf("登录成功\n");
//	return 0;
//}

//int main()
//{
//	int i = 0;
//	char password[20] = "abc123456";//原密码
//	char psme[20] = "0";//输入的密码
//	for (i = 0; i < 3; i++)//实现输入3次出错退出程序，输入正确亦推出程序
//	{
//		printf("请输入密码:");
//		scanf("%s", &psme);//输入密码
//		if (strcmp(password, psme) == 0)//密码输入正确时
//		{
//			printf("密码正确");
//			break;
//		}
//		else//输入错误时
//		{
//			if (i == 2)
//				break;
//			printf("密码错误\n");
//		}
//	}
//	return 0;
//}


//int main()
//{
//	enum Sex   //枚举关键字enum
//	{
//		MALE,   //0
//		FEMALE,//1
//		SECARE//2  MALE,FEMALE,SECURE便是枚举常量
//	};
//	printf("%d\n", FEMALE);
//	return 0;
//}