#define _CRT_SECURE_NO_WARNINGS
#include <stdio.h>
#include "game-五子棋.h"
void Begin()
{
	printf("*************************\n");
	printf("****是否进入游戏<0/1>****\n");
	printf("*************************\n");
}
void Game(char chessboard[ROW][COL],int row,int col)
{
	int ret = 0;
	//初始化棋盘
	Initialization_brard(chessboard, row, col);
	//打印棋盘
	Print_board(chessboard, row, col);
	while(1)
	{
		//人下棋
		Men_chess(chessboard, row, col);
		Print_board(chessboard, row, col);
		ret=Is_win(chessboard, row, col);
		if (ret == 1)
		{
			printf("你赢了");
			break;
		}
		else if (ret == 2)
		{
			printf("平");
			break;
		}
		//机器下棋
		Computer_chess(chessboard, row, col);
		Print_board(chessboard, row, col);
		ret = Is_win(chessboard, row, col);
		if (ret == 0)
		{
			printf("你输了");
			break;
		}
		else if (ret == 2)
		{
			printf("平\n");
			break;
		}
	}
}
int main()
{
	int a = 0;
	char chessboard[ROW][COL] = { ' '};
	srand(time(NULL));
	//进入游戏
	do
	{
		Begin();
		scanf("%d", &a);
		switch(a)
		{
		case 0:
			printf("退出游戏");
			break;
		case 1:
			Game(chessboard, ROW, COL);
			break;
		defult:
			printf("输入错误，请重新输入");
			break;
		}
	} while (a);
	return 0;
}