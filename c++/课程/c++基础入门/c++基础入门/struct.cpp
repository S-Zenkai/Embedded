#include <iostream>
using namespace std;
#include <string>

struct student
{
	string name;
	int score;
};
struct teature
{
	string name;
	struct student sdarr[5];
};

string tea_name_arr[] = { "����", "����", "����" };
string stu_name_arr[3][5] = {{ "����һ", "������", "������" ,"������","������"},
							 { "����һ", "���Ķ�", "������" ,"������","������"},
							 { "����һ", "�����", "������" ,"������","������"}};
int score_arr[3][5] = { 100, 90, 80, 70, 60,100, 90, 80, 70, 60,100, 90, 80, 70, 60 };

/*ͨ���������ṹ�������ֵ*/
void init_stru(struct teature* t)
{
	for (int i = 0; i < 3; i++)
	{
		t[i].name = tea_name_arr[i];
		for(int j = 0; j < 5; j++)
		{
			t[i].sdarr[j].name = stu_name_arr[i][j];
			t[i].sdarr[j].score = score_arr[i][j];
		}
		
	}
}

void printf_stru(struct teature* t)
{
	for (int i = 0; i < 3; i++)
	{
		cout << "��ʦ������" << t[i].name << endl;
		for (int j = 0; j < 5; j++)
		{
			cout << "ѧ��������" << t[i].sdarr[j].name << " ѧ���ɼ���" << t[i].sdarr[j].score << endl;
		}
		cout << endl << endl;
	}
}

int main(void)
{
	struct teature t[3];
	init_stru(t);
	printf_stru(t);
	system("pause");
	return 0;
}