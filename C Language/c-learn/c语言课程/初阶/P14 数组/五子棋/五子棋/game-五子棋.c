#define _CRT_SECURE_NO_WARNINGS
#include "game-五子棋.h"

//   |   |   
//---|---|---
//   |   |   
//---|---|---
//   |   |   
void Initialization_brard(char chessboard[ROW][COL], int row, int col)
{
	int i = 0;
	int j = 0;
	for (i = 0; i < row; i++)
	{
		for (j = 0; j < col; j++)
		{
			chessboard[i][j] = ' ';
		}
	}
}


void Print_board(char chessboard[ROW][COL], int row, int col)
{
	int i = 0;
	int j = 0;
	for (i = 0; i < row; i++)
	{
		if (i < row - 1)
		{
			for (j = 0; j < col; j++)
			{
				if (j < col - 1)
				{
					{
						printf(" %c |", chessboard[i][j]);
					}
				}
				else
				{
					printf(" %c \n", chessboard[i][j]);
				}
			}
			for (j = 0; j < col; j++)
			{
				if (j < col - 1)
				{
					{
						printf("---|");
					}
				}
				else
				{
					printf("---\n");
				}
			}
		}
		else
		{
			for (j = 0; j < col; j++)
			{
				if (j < col - 1)
				{
					{
						printf(" %c |", chessboard[i][j]);
					}
				}
				else
				{
					printf(" %c \n", chessboard[i][j]);
				}
			}
		}
	}
}


void Men_chess(char chessboard[ROW][COL], int row, int col)
{
	while (1)
	{
		int i = 0;
		int j = 0;
		printf("请输入坐标：");
		scanf("%d%d", &i,&j);
		if (i >= 1 && i <= 3 && j >= 1 && j <= 3 )
		{
			if (chessboard[i - 1][j - 1] == ' ')
			{
				chessboard[i - 1][j - 1] = '*';
				break;
			}
			else
				printf("此坐标已有棋子，请重新输入\n");
		}
		else
			printf("坐标非法，请重新输入\n");
	}
}


void Computer_chess(char chessboard[ROW][COL], int row, int col)
{
	int i = 0;
	int j = 0;
	while(1)
	{
		i = rand() % row;
		j = rand() % col;
		if (chessboard[i][j] == ' ')
		{
			printf("机器下棋\n");
			chessboard[i][j] = '#';
			break;
		}
	}
}


int Is_win(char chessboard[ROW][COL], int row, int col)
{
	/*if (chessboard[i][1] == chessboard[i][2] == chessboard[i][3] == '*'
		&& chessboard[1][i]== chessboard[2][i]== chessboard[3][i]
		&& chessboard[1][1]== chessboard[2][2]==)*/
	int i = 0;
	int j = 0;
	//人判赢
	for (i = 0; i < row; i++)
	{
		for (j = 0; j < row; j++)
		{
			if (chessboard[i][j] != '*')
			{
				break;
			}
		}
		if (j == row)
		{
			return(1);
			break;
		}
	}
	for (i = 0; i < row; i++)
	{
		for (j = 0; j < row; j++)
		{
			if (chessboard[j][i] != '*')
			{
				break;
			}
		}
		if (j == row)
		{
			return(1);
			break;
		}
	}
	for (i = 0; i < row; i++)
	{
		if (chessboard[i][i] != '*')
		{
			break;
		}
	}
	if (j == row)
	{
		return(1);
	}
	for (i = 0,j=row-1; i < row; i++,j--)
	{
		if (chessboard[i][i] != '*')
		{
			break;
		}
	}
	if (i == row)
	{
		return(1);
	}
	//机器判赢
	for (i = 0; i < row; i++)
	{
		for (j = 0; j < row; j++)
		{
			if (chessboard[i][j] != '#')
			{
				break;
			}
		}
		if (j == row)
		{
			return(0);
			break;
		}
	}
	for (i = 0; i < row; i++)
	{
		for (j = 0; j < row; j++)
		{
			if (chessboard[j][i] != '#')
			{
				break;
			}
		}
		if (j == row)
		{
			return(0);
			break;
		}
	}
	for (i = 0; i < row; i++)
	{
		if (chessboard[i][i] != '#')
		{
			break;
		}
	}
	if (j == row)
	{
		return(0);
	}
	for (i = 0, j = row - 1; i < row; i++, j--)
	{
		if (chessboard[i][i] != '#')
		{
			break;
		}
	}
	if (i == row)
	{
		return(0);
	}
	//判平
	for(i=0;i<row;i++)
	{
		for (j = 0; j < col; j++)
		{
			if (chessboard[i][j] == ' ')
				break;
		}
		if (chessboard[i][j] == ' ')
			break;
	}
	if (i == row && j == col)
		return 2;
}