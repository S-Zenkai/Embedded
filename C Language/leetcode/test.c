/**
 * *****************************************************************************
 * @file        test.c
 * @brief       
 * @author      S-Zenkai (1747098083@qq.com)
 * @date        2024-10-30
 * @copyright   xxxx技有限公司
 * *****************************************************************************
 */

// #include <stdio.h>
// #include <stdlib.h>

#if 0
void BubbleSort(int* arr,int sz)
{
    int i = 0;
    int j = 0;
    int tmp = 0;
    for (j = 0; j < sz - 1;j++)
    {
        for (i = 0; i + 1 + j < sz; i++)
        {
            if(arr[i]>arr[i+1])
            {
                tmp = arr[i + 1];
                arr[i + 1] = arr[i];
                arr[i] = tmp;
            }
        }
    }
}
void merge(int *nums1, int nums1Size, int m, int *nums2, int nums2Size, int n)
{
    int i = 0;
    for (i = 0; i < n; i++)
    {
        nums1[m + i] = nums2[i];
    }
    BubbleSort(nums1, nums1Size);
}
void main(void)
{
    int nums1[10]={0};
    int nums2[5]={0};
    int m = 6;
    int n = 4;
    int i = 0;
    int sz = sizeof(nums1) / sizeof(nums1[0]);
    printf("nums1=");
    for (i = 0; i < m;i++)
    {
        scanf("%d", &nums1[i]);
    }
    printf("nums2=");
    for (i = 0; i < n; i++)
    {
        scanf("%d", &nums2[i]);
    }
    for (i = 0; i < n;i++)
    {
        nums1[m + i] = nums2[i];
    }
    BubbleSort(nums1, sz);
    for (i = 0; i < m + n; i++)
    {
        printf("%d ", nums1[i]);
    }
}
#elif 0

    int compae(const void *a, const void *b)
{
    return *(int *)a - *(int *)b;
}
void merge(int *nums1, int nums1Size, int m, int *nums2, int nums2Size, int n)
{
    int i = 0;
    int sz = sizeof(nums1[0]);
    for (i = 0; i < n; i++)
    {
        nums1[m + i] = nums2[i];
    }
    qsort(nums1, nums1Size, sz, compae);
}

int main(void)
{
    int nums1[] = {1,2,3,0,0,0};
    int nums2[] = {2,5,6};
    int nums1Size = sizeof(nums1) / sizeof(nums1[0]);
    int nums2Size = sizeof(nums2) / sizeof(nums2[0]);
    int m = 3;
    int n = 3;
    int i = 0;
    merge(nums1, nums1Size, m, nums2, nums2Size, n);
    printf("nums1=");
    for (i = 0; i < m + n; i++)
    {
        printf("%d ", nums1[i]);
    }
}

#elif 1



// int main(void)
// {
//     float a = 10;
//     float b = 10;
//     float c = 3;
//     a = b / c;
//     printf("%f", a);
// }

// #include <stdio.h>
// #include <stdlib.h>
// #include <string.h>

// #define MAX_LENGTH 100
// #define MAX_ELEMENTS 1000 // 假设最多有这么多个元素，可根据实际情况调整

// int main()
// {
//     FILE *fp;
//     char line[MAX_LENGTH];
//     char *token;
//     char data[MAX_ELEMENTS][MAX_LENGTH]; // 一维数组用于存储数据
//     int index = 0;                       // 用于记录一维数组的存储位置索引

//     // 打开CSV文件
//     fp = fopen("C:\\Users\\Administrator\\Desktop\\111.csv", "r");
//     if (fp == NULL)
//     {
//         perror("Error opening file");
//         return 1;
//     }

//     // 逐行读取文件内容并存储到一维数组
//     while (fgets(line, MAX_LENGTH, fp) != NULL && index < MAX_ELEMENTS)
//     {
//         token = strtok(line, ",");
//         while (token != NULL && index < MAX_ELEMENTS)
//         {
//             strcpy(data[index], token);
//             index++;
//             token = strtok(NULL, ",");
//         }
//     }

//     // 可以在这里对存储在一维数组中的数据进行操作，比如打印出来查看
//     // for (int i = 0; i < index; i++)
//     // {
//     //     printf("%s ", data[i]);
//     // }
//     // printf("\n");
//     printf("index=%d", index);
//     int j = 1;
//     for (int i = 0; i < index-1;i++)
//     {
//         if (data[i] != data[i+1])
//         {
//             j++;
//         }
//     }
//     printf("j=%d", j++);

//     // 关闭文件
//     fclose(fp);
//     return 0;
// }

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define MAX_LENGTH 100
#define MAX_ELEMENTS 1000  // 假设最多有这么多个元素，可根据实际情况调整

int main() {
    FILE *fp;
    char line[MAX_LENGTH];
    char *token;
    int data[MAX_ELEMENTS];  // int类型的一维数组用于存储数据
    int index = 0;  // 用于记录一维数组的存储位置索引

    // 打开CSV文件
    fp = fopen("C:\\Users\\Administrator\\Desktop\\111.csv", "r");
    if (fp == NULL) {
        perror("Error opening file");
        return 1;
    }

    // 逐行读取文件内容并转换存储到int类型的一维数组
    while (fgets(line, MAX_LENGTH, fp)!= NULL && index < MAX_ELEMENTS) {
        token = strtok(line, ",");
        while (token!= NULL && index < MAX_ELEMENTS) {
            data[index] = atoi(token);  // 使用atoi函数将字符串转换为整数并存储
            index++;
            token = strtok(NULL, ",");
        }
    }

    // 可以在这里对存储在int类型一维数组中的数据进行操作，比如打印出来查看
    for (int i = 0; i < index; i++) {
        printf("data[%d]: %d\n", i, data[i]);
    }

    // 关闭文件
    fclose(fp);
    return 0;
}
#endif



