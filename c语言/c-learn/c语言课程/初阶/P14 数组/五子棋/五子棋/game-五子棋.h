#define _CRT_SECURE_NO_WARNINGS
#define ROW 3
#define COL 3
#include <stdio.h>
#include <time.h>
#include <stdlib.h>
void Initialization_brard(char chessboard[ROW][COL], int row, int col);
void Print_board(char chessboard[ROW][COL], int row, int col);
void Men_chess(char chessboard[ROW][COL], int row, int col);
void Computer_chess(char chessboard[ROW][COL], int row, int col);
int Is_win(char chessboard[ROW][COL], int row, int col);