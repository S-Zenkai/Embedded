#include <iostream>
using namespace std;

/*ˮ�ɻ���*/
int main()
{
	int num = 100;
	do
	{
		if (powf(num % 10, 3) + powf((num % 100) / 10, 3) + powf(num / 100, 3) == num)
		{
			cout << num << endl;
		}
		num++;
	} while (num<1000);
	system("pause");
	return 0;
}