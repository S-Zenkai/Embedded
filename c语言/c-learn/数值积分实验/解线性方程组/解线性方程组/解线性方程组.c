#define _CRT_SECURE_NO_WARNINGS
#include <stdio.h>
//#define ROW 6
//#define COL 6
/*loat* Doolittle(float x[ROW][COL])
{*/
	//float U[ROW][COL] = { 0 };
	//float L[ROW][COL] = { 0 };
	//printf("%l", x[6][6]);
	//int i = 0;//ÐÐ
	//int i2 = 0;
	//int j = 0;//ÁÐ
	//int k = 0;
	//int j2 = 0;
	//float sum = 0;
	//float itm = 0;
	//for (i = 0; i++; i < 6)
	//{
	//	for (j = 0; j++; j < 6)
	//	{
	//		for (k = 0; k++; k < i - 1)
	//		{
	//			itm = L[i][k] * U[k][j];
	//			sum = sum + itm;
	//		}
	//		U[i][j] = x[i][j] - sum;
	//	}
	//}
//}
#include <stdio.h>
int main()
{
	float arr[6][6] = {
		{2.4492,0,0.0142,-0.2125,-0.1799,0.0706},
		{0,8,0,0,0,0},
		{0.0142,0,0.1292,0.0125,0.0106,-0.0042},
		{-0.2125,0,0.0125,11.0625,-0.1588,0.0623},
		{-0.1799,0,0.0106,-0.1588,1.0861,0.0511},
		{0.0706,0,-0.0042,0.0623,-0.0551,0.2082}
	};
	float U[6][6] = { 0 };
	float L[6][6] = { {1,0,0,0,0},{0,1,0,0,0,0},{0,0,1,0,0,0},{0,0,0,1,0,0},{0,0,0,0,1,0},{0,0,0,0,0,1} };
	float b[] = { 0.8371,2.5200,0.1537,6.8912,0.2273,-1.9413 };
	float y[6] = { 0 };
	float x[6] = { 0 };
	int i = 0;
	int j = 0;
	int k = 0;
	float sum = 0;
	float itm = 0;
	for(i=0; i<6; i++)
	{
		for (j = 0; j < 6; j++)
		{
			sum = 0;
			itm = 0;
			if(i<=j)
			{
				for (k = 0; k <= i - 1; k++)
				{
					itm = L[i][k] * U[k][j];
					sum = sum + itm;
				}
				U[i][j] = arr[i][j] - sum;
			}
			else
			{
				for (k = 0; k <= j - 1; k++)
				{
					itm = L[i][k] * U[k][j];
					sum = sum + itm;
				}
				L[i][j] = (arr[i][j] - sum) / U[j][j];
			}
		}
	}
	for (i = 0; i < 6; i++)
	{
		sum = 0;
		itm = 0;
		for (k = 0; k <= i - 1; k++)
		{
			itm = L[i][k] * y[k];
			sum = sum + itm;
		}
		y[i] = b[i] - sum;
	}
	for (i = 5; i >= 0; i--)
	{
		sum = 0;
		itm = 0;
		for (k = i+1; k <= 5; k++)
		{
			itm = U[i][k] * x[k];
			sum = sum + itm;
		}
		x[i] = (y[i] - sum) / U[i][i];
	}
	for (i=0;i<6;i++)
	{
		printf("%f\n", x[i]);
	}
	return 0;
}