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

string tea_name_arr[] = { "张三", "李四", "王五" };
string stu_name_arr[3][5] = {{ "张三一", "张三二", "张三三" ,"张三四","张三五"},
							 { "李四一", "李四二", "李四三" ,"李四四","李四五"},
							 { "王五一", "王五二", "王五三" ,"王五四","王五五"}};
int score_arr[3][5] = { 100, 90, 80, 70, 60,100, 90, 80, 70, 60,100, 90, 80, 70, 60 };

/*通过函数给结构体数组幅值*/
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
		cout << "老师姓名：" << t[i].name << endl;
		for (int j = 0; j < 5; j++)
		{
			cout << "学生姓名：" << t[i].sdarr[j].name << " 学生成绩：" << t[i].sdarr[j].score << endl;
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