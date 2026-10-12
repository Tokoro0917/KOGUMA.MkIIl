/*
 * LOG.c
 *
 *  Created on: Jun 30, 2024
 *      Author: akihi
 */

#include"LOG.h"
#include "stdio.h"

#include "PL_encoder.h"
#include "lsm6dsr.h"
#include "motor.h"
#include "PL_sensor.h"
#include "Wallsensor.h"

/* 3000(3秒/1kHz)から2000(2秒)に削減。32x32迷路のDijkstraノードグラフ用に
 * 約32KB(8本×4byte×1000サンプル)のSRAMを確保するため */
#define LOG_MAX 2000

float Tire_Speed_LOG[LOG_MAX];
float G_Motor_V_Target_LOG[LOG_MAX];

float Gyro_z_LOG[LOG_MAX];
float G_Motor_W_Target_LOG[LOG_MAX];

float Sensor_L_LOG[LOG_MAX];
float Sensor_R_LOG[LOG_MAX];

float Motor_Voltage_L_LOG[LOG_MAX];
float Motor_Voltage_R_LOG[LOG_MAX];

/* 横壁制御の調整用(2026-10-07追加): 誤差(Wall_error)と出力(calWallConrol())。
 * 直進中(PID_Mode == 1)以外は、最後に計算した値が残っている */
float Wall_Error_LOG[LOG_MAX];
float Wall_PID_LOG[LOG_MAX];

int LOG_flag = 0;
int LOG_count = 0;
static int LOG_used = 0;	//最後に取ったログの行数(LOG_get_end で残す。0 ならわからないので全部出す)

void LOG_get_start() {
	for (int i = 0; i < LOG_MAX; i++) {
		Tire_Speed_LOG[i] = 0;
		G_Motor_V_Target_LOG[i] = 0;
		Gyro_z_LOG[i] = 0;
		G_Motor_W_Target_LOG[i] = 0;
//		Sensor_L_LOG[i] = 0;
//		Sensor_R_LOG[i] = 0;
	}
	LOG_flag = 1;
	LOG_count = 0;
	LOG_used = 0;
}

void LOG_get_end() {
	LOG_flag = 0;
	LOG_used = LOG_count;
	LOG_count = 0;
}

void LOG_get_interrupt() {
	if (LOG_count >= LOG_MAX) {
		LOG_flag = 0;
	}
	if (LOG_flag == 1) {
		Tire_Speed_LOG[LOG_count] = (G_Tire_Speed_L + G_Tire_Speed_R) / 2;
		G_Motor_V_Target_LOG[LOG_count] = G_Motor_V_Target;
		Gyro_z_LOG[LOG_count] = read_NoiseCut_gyro_z();
		G_Motor_W_Target_LOG[LOG_count] = G_Motor_W_Target;
		Sensor_L_LOG[LOG_count] = g_sensor_av[2];
		Sensor_R_LOG[LOG_count] = g_sensor_av[1];
		Motor_Voltage_L_LOG[LOG_count]=Motor_Voltage_L;
		Motor_Voltage_R_LOG[LOG_count]=Motor_Voltage_R;
		Wall_Error_LOG[LOG_count] = Wall_error;
		Wall_PID_LOG[LOG_count] = calWallConrol();
		LOG_count++;
	}
}

/* ログをUARTに出す(2026-10-12 から tools/viewer.html で読める形)。
 *   #BEGIN LOG dt=0.001 n=行数
 *   列名の行(t,v,...)
 *   1ms ごとの値の行
 *   #END LOG
 * 列: 時刻[s], 速度, 目標速度[mm/s], 角速度, 目標角速度[deg/s], 横センサ L, R, 電圧 L, R, 横壁の誤差, 横壁制御の出力 */
void LOG_print() {
	int n = (LOG_used > 0) ? LOG_used : LOG_MAX;
	printf("\r\n#BEGIN LOG dt=0.001 n=%d\r\n", n);
	printf("t,v,v_target,w,w_target,sensor_L,sensor_R,volt_L,volt_R,wall_err,wall_out\r\n");
	for (int i = 0; i < n; i++) {
		printf("%.3f,%.1f,%.1f,%.2f,%.2f,%.0f,%.0f,%.1f,%.1f,%.2f,%.3f\r\n", i * 0.001, Tire_Speed_LOG[i],
				G_Motor_V_Target_LOG[i], Gyro_z_LOG[i], G_Motor_W_Target_LOG[i],
				Sensor_L_LOG[i], Sensor_R_LOG[i], Motor_Voltage_L_LOG[i], Motor_Voltage_R_LOG[i],
				Wall_Error_LOG[i], Wall_PID_LOG[i]);
	}
	printf("#END LOG\r\n");
}
