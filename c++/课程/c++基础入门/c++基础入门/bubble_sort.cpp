//#include <iostream>
//using namespace std;
//#include <string>
//
///*ц╟ещеепР*/
//int main(void)
//{
//	int arr[] = { 4,2,8,0,5,7,1,3,9 };
//	int temp = 0;
//	int sz = sizeof(arr) / sizeof(arr[0]);
//	for (int i = 0; i < sz - 1; i++)
//	{
//		for (int j = i+1; j < sz; j++)
//		{
//			if (arr[i] > arr[j])
//			{
//				temp = arr[i];
//				arr[i] = arr[j];
//				arr[j] = temp;
//			}
//		}
//	}
//	for (int i = 0; i < sz; i++)
//	{
//		cout << arr[i] << " ";
//	}
//	cout << endl;
//	system("pausew");
//	return 0;
//}