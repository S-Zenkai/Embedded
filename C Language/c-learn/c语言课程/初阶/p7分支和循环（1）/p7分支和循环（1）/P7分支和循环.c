#define _CRT_SECURE_NO_WARNINGS
#include <stdio.h>
//int main()
//{
//	int age = 0;
//	scanf("%d", &age);
//	if (age < 18)
//		printf("未成年");
//	else
//	{
//		if (age >= 18 && age < 30)
//			printf("青年");
//		else if (age >= 30 && age < 50)
//			printf("壮年");
//		else if (age >= 50 && age < 80)
//			printf("老年");
//		else
//			printf("得道升仙");
//	}
	/*else if (age >= 18 && age < 30)
		printf("青年");
	else if (age >= 30 && age < 50)
		printf("壮年");
	else if (age >= 50 && age < 80)
		printf("老年");
	else
		printf("得道升仙");
	return 0;*/
//}

//int main()
//{
//	int height = 0;
//	scanf("%d", &height);
//	if (height >= 0 && height < 150)
//	{
//		printf("未成年");
//		printf("小低个");
//	}
//	else
//		printf("大高个");
//	return 0;
//}

//int main()   //应是什么都没有输出，原因是else语句是于最近的if语句配对的，故else与上一个if语句为一个代码块，同属第一个if语句之下，因a=1为假，故该if语句不执行,程序无输出
//{
//	int a = 0;
//	int b = 2;
//	if (a == 1)
//		if (b == 2)
//			printf("hehe");
//		else
//			printf("haha");
//	return 0;
//}

//int main()
//{
//	int a = 0;
//	scanf("%d", &a);
//	if (a % 2 == 0)
//		printf("NO");
//	else
//		printf("YES");
//	return 0;
//}

//int main()
//{
//	int a = 1;
//	int i = 0;
//	while (i < 100)
//	{
//		if (a % 2 == 1)
//			printf("%d\n", a);
//		a++;
//		i++;
//	}
//	return 0;
//}

//int main()
//{
//	int grade = 0;
//	scanf("%d", &grade);
//	switch (grade)//()内需为整形表达式
//	{
//	case 1://case后需为整形常量表达式 
//		printf("一年级");
//		break;
//	case 2:
//		printf("二年级");
//		break;
//	case 3:
//		printf("三年级");
//		break;
//	case 4:
//		printf("四年级");
//		break;
//	case 5:
//		printf("五年级");
//		break;
//	case 6:
//		printf("六年级");
//		break;
//	}
//	return 0;
//}

//int main()
//{
//	int grade = 0;
//	scanf("%d", &grade);
//	switch (grade)
//	{
//	case 1:
//	case 2:
//	case 3://输入1\2\3会显示低年级
//		printf("低年级");
//		break;
//	case 4:
//	case 5:
//	case 6:
//		printf("高年级");
//		break;
//	default://输入1-6之外的其他数字会显示输入错误
//		printf("输入错误");
//	}
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
//	printf("m=%d\nn=%d",m, n);
//	return 0;
//}

//int main()
//{
//	int a= 1;
//	while (a <= 10)
//	{
//		printf("%d", a);
//		a++;
//	}
//	return 0;
//}

//int main()
//{
//	int i = 1;
//	while (i <= 10)
//	{
//		if (i == 5)
//			break;
//		printf("%d ", i);
//		i++;
//	}
//	return 0;
//}

//int main()
//{
//	int i = 1;
//	while (i <= 10)
//	{
//		i++;
//		if (i == 5)
//			continue;
//		printf("%d ", i);
//	}
//	return 0;
//}

//int main()
//{
//	int a = 0;
//	while ((a =getchar())!= EOF)//注意,这里的a=getchar()要用括号括起来,否者编程出错,按ctrl+z即代表EOF
//	{
//		putchar (a);
//	}
//	return 0;
//}