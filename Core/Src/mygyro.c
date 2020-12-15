/*
 * mygyro.c
 *
 *  Created on: May 4, 2020
 *      Author: Anmol
 */

#include "mygyro.h"
#include "UART_Helpers.h"
#include "stm32l476g_discovery.h"
#include "stm32l476g_discovery_gyroscope.h"


void init_gyro(Gyro *g){
	g->x = 0;
	g->y = 0;
	g->z = 0;
	g->dx = 0;
	g->dy = 0;
	g->dz = 0;
	g->x_offset = 0;
	g->y_offset = 0;
	g->z_offset = 0;
	g->cnt = 0;
}

void update_gyro(Gyro *g){
	float buffer[3] = {0};
	BSP_GYRO_GetXYZ(buffer);
	int32_t xscale = 5000;
	int32_t yscale = 2500;
	int32_t zscale = 5000;
	g->dx = buffer[0]/xscale;
	g->dy = buffer[1]/yscale;
	g->dz = buffer[2]/zscale;
}

void get_measurement(Gyro *g){
	update_gyro(g);
	g->dx = g->dx - g->x_offset;
	g->dy = g->dy - g->y_offset;
	g->dz = g->dz - g->z_offset;

	g->x += g->dx;
	g->y += g->dy;
	g->z += g->dz;
	g->cnt++;
}

void print_measurement(Gyro *g){
	uint8_t buffer[80];
	sprintf((char*) buffer, "$%d %d %d;\r\n", g->x, g->y, g->z);
	HAL_UART_Transmit(&huart2, buffer, strlen((char *)buffer),100);
}

void print_dmeasurement(Gyro *g){
	uint8_t buffer[80];
	sprintf((char*) buffer, "$%d %d %d;\r\n", g->dx, g->dy, g->dz);
	HAL_UART_Transmit(&huart2, buffer, strlen((char *)buffer),100);
}

int calibrate_gyro(Gyro *g){
	UART_prints("\r\n\nPress Enter to Calibrate Gyro...\r\n");
	wait_for_key(13);

	g->x_offset = 0;
	int32_t xmin = 10000000;
	int32_t xmax = 0;
	int32_t xsum = 0;
	g->y_offset = 0;
	int32_t ymin = 10000000;
	int32_t ymax = 0;
	int32_t ysum = 0;
	g->z_offset = 0;
	int32_t zmin = 10000000;
	int32_t zmax = 0;
	int32_t zsum = 0;

	for (int i=0; i<GYRO_CAL_SIZE; i++){
		get_measurement(g);
		xsum += g->dx;
		ysum += g->dy;
		zsum += g->dz;
		if (g->dx < xmin) xmin = g->dx;
		if (g->dx > xmax) xmax = g->dx;
		if (g->dy < ymin) ymin = g->dy;
		if (g->dy > ymax) ymax = g->dy;
		if (g->dz < zmin) zmin = g->dz;
		if (g->dz > zmax) zmax = g->dz;
	}

	g->x_offset = xsum/GYRO_CAL_SIZE;
	g->y_offset = ysum/GYRO_CAL_SIZE;
	g->z_offset = zsum/GYRO_CAL_SIZE;
#ifdef DEBUG
	UART_println(xmax-xmin);
	UART_println(ymax-ymin);
	UART_println(zmax-zmin);
#endif
	if (xmax-xmin>1000 || ymax-ymin>1000 || zmax-zmin>1000){
		UART_prints("Calibration Failed. Please keep the gyro still ...\r\n");
		return 1;
	}
	UART_prints("Calibration Success...\r\n");
	return 0;
}
