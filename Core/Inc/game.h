/*
 * game.h
 *
 *  Created on: May 4, 2020
 *      Author: Anmol
 */
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#ifndef INC_GAME_H_
#define INC_GAME_H_

#define NUM_OF_BALLS (4)
#define BOARD_ROWS (22)
#define BOARD_COLS (44)



typedef struct Ball
{
	int x[NUM_OF_BALLS];
	int y[NUM_OF_BALLS];

} Ball;

typedef struct Goal
{
	int x;
	int y;

} Goal;

typedef struct Game
{
	int rows;
	int cols;
	uint8_t board[BOARD_ROWS][BOARD_COLS];
	Ball *b;
	Goal *goal;
	int lvl;
	int score;

} Game;

void init_ball(Ball *b);
void move_ball(Game *g, int x, int y);
void init_goal(Goal *g);
void init_game(Game *g, Ball *b, Goal *goal);
void display_board(Game *g);
void clear_board(Game *g);
void update_board(Game *g);

#endif /* INC_GAME_H_ */
