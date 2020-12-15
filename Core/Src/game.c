/*
 * game.c
 *
 *  Created on: May 4, 2020
 *      Author: Anmol
 */

#include "game.h"
#include "UART_Helpers.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>

extern int scores[];
extern RNG_HandleTypeDef hrng;

void display_board(Game *g) {
	uint8_t esc[1] = { 27 };
	HAL_UART_Transmit(&huart2, esc, strlen(esc), HAL_MAX_DELAY);
	HAL_UART_Transmit(&huart2, "[2J", strlen("[2J"), HAL_MAX_DELAY);
	HAL_UART_Transmit(&huart2, esc, strlen(esc), HAL_MAX_DELAY);
	HAL_UART_Transmit(&huart2, "[H", strlen("[H"), HAL_MAX_DELAY);

	UART_prints("\r\n");
	for (int i = 0; i < g->rows; i++) {
		HAL_UART_Transmit(&huart2, g->board[i], BOARD_COLS, HAL_MAX_DELAY);
		UART_prints("\r\n");
	}
}

void clear_board(Game *g) {

}

// x -> col
// y -> row

void update_board(Game *g) {
	for (int i = 0; i < g->rows; i++) {
		for (int j = 0; j < g->cols; j++) {
			// If first or last row
			if (i == 0 || i == g->rows - 1) {
				g->board[i][j] = '=';
			} else if (j == 0 || j == g->cols - 1) {
				g->board[i][j] = '|';
			} else {
				g->board[i][j] = ' ';
				if ((i == g->b->y[3] + 1) && (j == g->b->x[3] + 1)) {
					g->board[i][j] = '.';
				}
				if ((i == g->b->y[2] + 1) && (j == g->b->x[2] + 1)) {
					g->board[i][j] = 'o';
				}
				if ((i == g->b->y[1] + 1) && (j == g->b->x[1] + 1)) {
					g->board[i][j] = 'O';
				}
				if ((i == g->b->y[0] + 1) && (j == g->b->x[0] + 1)) {
					g->board[i][j] = '*';
				}
				if ((i == g->goal->y + 1) && (j == g->goal->x + 1)) {
					g->board[i][j] = '+';
				}

			}
		}
	}
}

void move_ball(Game *g, int x, int y) {
	x += (BOARD_COLS - 2) / 2;
	y += (BOARD_ROWS - 2) / 2;

	x = x >= BOARD_COLS ? BOARD_COLS - 1 : x;
	x = x < 0 ? 0 : x;
	y = y >= BOARD_ROWS ? BOARD_ROWS - 1 : y;
	y = y < 0 ? 0 : y;
	g->b->x[3] = g->b->x[2];
	g->b->y[3] = g->b->y[2];
	g->b->x[2] = g->b->x[1];
	g->b->y[2] = g->b->y[1];
	g->b->x[1] = g->b->x[0];
	g->b->y[1] = g->b->y[0];
	g->b->x[0] = x;
	g->b->y[0] = y;
}

void init_ball(Ball *b) {
	for (int i = 0; i < NUM_OF_BALLS; i++) {
		b->x[i] = 20;
		b->y[i] = 10;
	}
}

void init_goal(Goal *g) {
	g->x = 0;
	g->y = 0;
}

void random_goal(Goal *g) {
	srand(HAL_GetTick());
	uint32_t randvar = 0;
	HAL_RNG_GenerateRandomNumber(&hrng, &randvar);
	g->x = randvar % (BOARD_COLS -2);
	HAL_RNG_GenerateRandomNumber(&hrng, &randvar);
	g->y = randvar % (BOARD_ROWS -2);
}

int game_check(Game *g) {
	if (abs(g->b->x[0] - g->goal->x) < 2)
		if (abs(g->b->y[0] - g->goal->y) < 2) {
			// Reached Goal
			scores[g->lvl - 1] = g->score;
			g->score = 0;
			g->lvl++;
			HAL_GPIO_TogglePin(GPIOE, GPIO_PIN_8);
			HAL_Delay(200);
			HAL_GPIO_TogglePin(GPIOE, GPIO_PIN_8);
			random_goal(g->goal);
			if (g->lvl > 6) {
				return 1;
			}
		}
	return 0;
}

void set_score(Game *g, int score) {
	g->score = score;
}

void print_info(Game *g) {
	HAL_UART_Transmit(&huart2, "Level: ", strlen("Level: "), HAL_MAX_DELAY);
	UART_print(g->lvl);
	HAL_UART_Transmit(&huart2, "\r\nScore: ", strlen("\r\nScore: "),
			HAL_MAX_DELAY);
	UART_print(g->score);
	HAL_UART_Transmit(&huart2, "\r\nGoal x: ", strlen("\r\nGoal x: "), HAL_MAX_DELAY);
	UART_print(g->goal->x);
	HAL_UART_Transmit(&huart2, "\t\tGoal y: ", strlen("\t\tGoal y: "), HAL_MAX_DELAY);
	UART_print(g->goal->y);
}

void init_game(Game *g, Ball *b, Goal *goal) {
	g->rows = BOARD_ROWS;
	g->cols = BOARD_COLS;
	g->lvl = 1;
	g->score = 0;
//	g->board = malloc(rows*sizeof(uint8_t*));
//	for(int i = 0; i < rows; i++)
//		g->board[i] = malloc(cols*sizeof(uint8_t));
	g->goal = goal;
	g->b = b;
	update_board(g);
}
