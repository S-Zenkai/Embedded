#define _CRT_SECURE_NO_WARNINGS
#include <stdio.h>
#include <math.h>
int main()
{
	
	double x = 0;
	int i = 0;
	printf("ÊäÈëµü´ú³õÖµ£º");
	scanf("%lf", &x);
	for (i = 0; i <= 10; i++)
	{
		x = x - (((1 / 3) * log(x) + (64 / 63) * log(1 / (1 - x)) + log(0.95 - x) - log(0.9)) / ((1 / 3) * (1 / x) + (64 / 63) * (1 / (1 - x)) - (1 / (0.95 - x))));
	}
	printf("x=%lf", x);
	return 0;
}