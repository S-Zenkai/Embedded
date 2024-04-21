#define _CRT_SECURE_NO_WARNINGS
#include <stdio.h>
float Compound_Trapezoidal(float x[], float y[], int z)
{
	int i = 0;
	float tmp = 0;
	float sum = 0;
	for (i = 0; i < z-1; i++)
	{
		tmp = (y[i] + y[i + 1])*(x[i+1]-x[i])/2;
		sum = sum + tmp;
	}
	return sum;
}
int main()
{
	float X[] = { 7,10.5,13,17.5,34,40.5,44.5,48,56,61,68.5,76.5,80.5,91,96,101,104,106.5,111.5,118,123.5,136.5,142,146,150,157,158 };
	float Y1[] = {44,45,47,50,50,38,30,30,34,36,34,41,45,46,43,37,33,28,32,65,55,54,52,50,66,66,68};
	float Y2[] = { 44,59,70,72,93,100,110,110,110,117,118,116,118,118,121,124,121,121,121,122,116,83,81,82,86,85,68 };
	int sz = sizeof(X) / sizeof(X[0]);
	float ret1 = 0;
	float ret2 = 0;
	float S = 0;
	ret1 = Compound_Trapezoidal(X, Y1, sz);
	ret2 = Compound_Trapezoidal(X, Y2, sz); 
	S = ret2 - ret1;
	printf("S=%f\n", S);
	printf("按比例放大后：%f", S * 40 * 40 / 18 / 18);
	return 0;
}