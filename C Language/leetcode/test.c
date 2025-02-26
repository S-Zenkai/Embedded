/**
 * *****************************************************************************
 * @file        test.c
 * @brief       
 * @author      S-Zenkai (1747098083@qq.com)
 * @date        2024-10-30
 * @copyright   xxxx�����޹�˾
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
// #define MAX_ELEMENTS 1000 // �����������ô���Ԫ�أ��ɸ���ʵ���������

// int main()
// {
//     FILE *fp;
//     char line[MAX_LENGTH];
//     char *token;
//     char data[MAX_ELEMENTS][MAX_LENGTH]; // һά�������ڴ洢����
//     int index = 0;                       // ���ڼ�¼һά����Ĵ洢λ������

//     // ��CSV�ļ�
//     fp = fopen("C:\\Users\\Administrator\\Desktop\\111.csv", "r");
//     if (fp == NULL)
//     {
//         perror("Error opening file");
//         return 1;
//     }

//     // ���ж�ȡ�ļ����ݲ��洢��һά����
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

//     // ����������Դ洢��һά�����е����ݽ��в����������ӡ�����鿴
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

//     // �ر��ļ�
//     fclose(fp);
//     return 0;
// }

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define MAX_LENGTH 100
#define MAX_ELEMENTS 1000  // �����������ô���Ԫ�أ��ɸ���ʵ���������

int main() {
    FILE *fp;
    char line[MAX_LENGTH];
    char *token;
    int data[MAX_ELEMENTS];  // int���͵�һά�������ڴ洢����
    int index = 0;  // ���ڼ�¼һά����Ĵ洢λ������

    // ��CSV�ļ�
    fp = fopen("C:\\Users\\Administrator\\Desktop\\111.csv", "r");
    if (fp == NULL) {
        perror("Error opening file");
        return 1;
    }

    // ���ж�ȡ�ļ����ݲ�ת���洢��int���͵�һά����
    while (fgets(line, MAX_LENGTH, fp)!= NULL && index < MAX_ELEMENTS) {
        token = strtok(line, ",");
        while (token!= NULL && index < MAX_ELEMENTS) {
            data[index] = atoi(token);  // ʹ��atoi�������ַ���ת��Ϊ�������洢
            index++;
            token = strtok(NULL, ",");
        }
    }

    // ����������Դ洢��int����һά�����е����ݽ��в����������ӡ�����鿴
    for (int i = 0; i < index; i++) {
        printf("data[%d]: %d\n", i, data[i]);
    }

    // �ر��ļ�
    fclose(fp);
    return 0;
}
#endif



