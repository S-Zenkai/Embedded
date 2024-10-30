#define _CRT_SECURE_NO_WARNINGS
#include <stdio.h>
#include <math.h>

double fact(int n)
{
    if (n <= 1) 
        return 1;
    else 
        return n * fact(n - 1);
}

#define _CRT_SECURE_NO_WARNINGS
#include <stdio.h>
#include <math.h>

double calculate_S(int k)
{
    double sum = 0;
    double base = (28.0 + 100.0) / 400.0;
    for (int n = 0; n <= k; n++)
    {
        double num = (pow(-1, n) * (n + 2) * pow(base, n + 2));
        double den = fact(n) + fact(n + 1) + fact(n + 2);
        sum = sum + num / den;
    }
    return sum;
}

int main()
{
    printf(" k         S(k)\n");
    for (int k = 0; k <= 11; k++)
    {
        double ret = calculate_S(k);
        printf("%2d         %.12f\n", k, ret);
    }
    return 0;
}


//#include <stdio.h>
//
//int RES[8] = { 0, 0, 0, 0, 0, 0, 0, 0 }; // 存放8个整数的数组
//
//int calc(int mm) 
//{
//    int* R7 = RES;
//    int R1 = 1000000;
//    int R0 = mm + 100;
//    R0 *= R1;
//    int R2 = 1000;
//    int R3, R4, R5, R6;
//
//    for (int R1 = 8; R1 > 0; R1--)
//    {
//        R3 = R2 * R2;
//        R4 = R3 + R3;
//        R5 = R4 + R3;
//        R6 = R4 * R2 + R0;
//        R6 /= R5;
//        R3 = R6 - R2;
//        R2 = R6;
//        *R7++ = R2;
//    }
//
//    return R3;
//}
//
//int main() 
//{
//    int mm = 28;
//    int result = calc(mm);
//
//    printf("Result array: [");
//    for (int i = 0; i < 8; i++)
//    {
//        printf("%d", RES[i]);
//        if (i < 7) printf(", ");
//    }
//    printf("]\n");
//
//    printf("Final result in R0: %d\n", result);
//
//    return 0;
//}

//#include <stdio.h>
//
//int RES[8] = { 0, 0, 0, 0, 0, 0, 0, 0 }; // 存放8个整数的数组
//
//int calc(int mm) {
//    int* R7 = RES;
//    int R1 = 1000000;
//    int R0 = mm + 100;
//    R0 *= R1;
//    int R2 = 1000;
//    int R3, R4, R5, R6;
//
//    for (int R1 = 8; R1 > 0; R1--) {
//        R3 = R2 * R2;
//        R4 = R3 + R3;
//        R5 = R4 + R3;
//        R6 = R4 * R2 + R0;
//        R6 = (int)((unsigned int)R6 / R5); // 强制类型转换来模拟UDIV
//        R3 = R6 - R2;
//        R2 = R6;
//        *R7++ = R2;
//    }
//
//    return R3;
//}
//
//int main() {
//    int mm = 28;
//    int result = calc(mm);
//
//    printf("Result array: [");
//    for (int i = 0; i < 8; i++) {
//        printf("%d", RES[i]);
//        if (i < 7) printf(", ");
//    }
//    printf("]\n");
//
//    printf("Final result in R0: %d\n", result);
//
//    return 0;
//}

//#include <stdio.h>
//
//int RES[8] = { 0, 0, 0, 0, 0, 0, 0, 0 }; // 存放8个整数的数组
//
//int calc(int mm) {
//    int* R7 = RES;
//    int R1 = 1000000;
//    int R0 = mm + 100;
//    R0 *= R1;
//    int R2 = 1000;
//    int R3, R4, R5, R6;
//
//    for (int R1 = 8; R1 > 0; R1--) {
//        R3 = R2 * R2;
//        R4 = R3 + R3;
//        R5 = R4 + R3;
//        R6 = R4 * R2 + R0;
//        R6 /= R5;
//        R3 = R6 - R2;
//        R2 = R6;
//        *R7++ = R2;
//    }
//
//    return R3;
//}
//
//int main() {
//    int mm = 12;
//    int result = calc(mm);
//
//    printf("Result array: [");
//    for (int i = 0; i < 8; i++) {
//        printf("%d", RES[i]);
//        if (i < 7) printf(", ");
//    }
//    printf("]\n");
//
//    printf("Final result: %d\n", result);
//
//    return 0;
//}
