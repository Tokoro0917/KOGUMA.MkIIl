/*
 * Wallsensor.h
 *
 *  Created on: Jun 21, 2024
 *      Author: akihi
 */

#ifndef INC_WALLSENSOR_H_
#define INC_WALLSENSOR_H_

extern int G_Wall_data[];
extern int G_WallCtrl_Use_mm;

void Wall_search();

float Wall_Flont_Av();

int Sensor_Enter();
int Sensor_Start();

void Wall_search_LED();
void Wall_Distance_Calibration();
void Wall_Center_Calibration();

float calWallConrol();
float calWallConrol_NANAME();
float calWallConrol_Flontwall_ST();
float calWallConrol_Flontwall_Turn();

#endif /* INC_WALLSENSOR_H_ */
