/*
 * mygyro.h
 *
 *  Created on: May 4, 2020
 *      Author: Anmol
 */

#ifndef INC_MYGYRO_H_
#define INC_MYGYRO_H_

#ifndef GYRO_CAL_SIZE
#define GYRO_CAL_SIZE (50)
#endif

#include <stdio.h>
#include <string.h>
#include <math.h>

typedef struct Gyro {
	int32_t x;
	int32_t y;
	int32_t z;
	int32_t dx;
	int32_t dy;
	int32_t dz;
	int32_t x_offset;
	int32_t y_offset;
	int32_t z_offset;
	int32_t cnt;
} Gyro;

void init_gyro(Gyro *g);
void update_gyro(Gyro *g);
int calibrate_gyro(Gyro *g);
void get_measurement(Gyro *g);
void print_measurement(Gyro *g);

#endif /* INC_MYGYRO_H_ */
