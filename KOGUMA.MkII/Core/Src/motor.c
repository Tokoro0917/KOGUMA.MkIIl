/*
 * motor.c
 *
 *  Created on: Jun 12, 2025
 *      Author: akihi
 */

#include "motor.h"
#include "tim.h"
#include "math.h"
#include "PL_encoder.h"
#include "PL_timer.h"
#include "Define.h"
#include "lsm6dsr.h"
#include "PL_sensor.h"
#include "Wallsensor.h"
#include "Maze.h"
#include "UI.h"
#include "stdlib.h"

float G_Robot_speed = 0;

float G_Motor_X = 0;
float G_Motor_Ac = 0;
float G_Motor_V_Target = 0;

float G_Motor_Angle = 0;
float G_Motor_W_Ac = 0;
float G_Motor_W_Target = 0;

int G_Motor_Flag = 0;
float G_Motor_Count = 0;

float Motor_PWM_L, Motor_PWM_R;

float Enc_error = 0;
float Enc_Old_error = 0;
float Enc_Sigma_error = 0;
float Enc_Delta_error = 0;

float Gyro_error = 0;
float Gyro_Old_error = 0;
float Gyro_Sigma_error = 0;
float Gyro_Delta_error = 0;

float Ksp = 0.2; //0.8
float Ksi = 0.002; //0.002
float Ksd = 0.4; //0.5

float Ksp_Fun = 0.7; //0.1
float Ksi_Fun = 0.003; //0.00105
float Ksd_Fun = 0.01; //0.1

float Ksp_AT = 0.3; //0.3
float Ksi_AT = 0.0002; ////
float Ksd_AT = 0.01;

float Ktp = 1.3; //
float Kti = 0.004; //
float Ktd = 0.0; //

float Ktp_sula = 0.7; //0.12
float Kti_sula = 0.005; //0.06
float Ktd_sula = 0.001; //0.01

float Ktp_sula_20 = 1.2; //0.12
float Kti_sula_20 = 0.02; //0.06
float Ktd_sula_20 = 0.01; //0.01

float Ktp_sula_24 = 1.4; //0.12
float Kti_sula_24 = 0.02; //0.06
float Ktd_sula_24 = 0.01; //0.01

float Motor_FB_ST = 0;
float Motor_FB_Turn = 0;
float Motor_FF_ST = 0;
float Motor_FF_Turn = 0;

int PID_Mode = 0;
int Turn_Mode = 0;
int Fun_Flag = 0;

int Sula_Flag = 0;

float Alignment_TIME = 0.5;

int Flont_th = 210; //310

float Cut_R = 150;
float Cut_L = 150;

float Cut_R_NA = 150;
float Cut_L_NA = 150;

float Run_Voltage;

void Motor_Setup() {
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);
	Enc_Sigma_error = 0;
	Gyro_Sigma_error = 0;
}

void Motor_Setup_Voltage() {
	Run_Voltage = g_V_batt;
}

void Motor_Stop() {
	G_Motor_Flag = 1;
	G_Motor_V_Target = 0;
	G_Motor_Ac = 0;
	Enc_Sigma_error = 0;
	Gyro_Sigma_error = 0;
	Motor_PWM_L = 0;
	Motor_PWM_R = 0;
	wait_ms(50);
	Motor_PWM_L = 0;
	Motor_PWM_R = 0;
	Enc_Sigma_error = 0;
	Gyro_Sigma_error = 0;
	G_Motor_Flag = 0;

}

void Suction_Start(int duty) {
	Fun_Flag = 1;
	if (duty == 50) {
		Sula_Flag = 1;
	} else if (duty >= 70) {
		Sula_Flag = 2;
	} else if (duty <= 20) {
		Fun_Flag = 0;
	}
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
	__HAL_TIM_SET_AUTORELOAD(&htim2, 100);
	duty = duty * (15.8 / Run_Voltage);
	for (int i = 1; duty > i; i += 1) {
		__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, i);
		HAL_Delay(10);
	}

	HAL_Delay(500);
}

void Fun_Flag_OFF() {
	Fun_Flag = 0;
}

void Suction_change(int duty){
	duty = duty * (15.8 / Run_Voltage);
	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, duty);
}

void Suction_Stop() {
	HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_1);
	Sula_Flag = 0;
	Fun_Flag = 0;
}

float cal_turnV(float W) {
	float TurnV = ((float) TIREBETWEEN * PI / 360) * W;
	return TurnV;
}

void Motor_PWM_Generate() { //MP6550モーター出力

	float Motor_Voltage_L = Motor_PWM_L * (REFERENCE_V / Run_Voltage);
	float Motor_Voltage_R = Motor_PWM_R * (REFERENCE_V / Run_Voltage);

	if (Motor_Voltage_L > 0) {
		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 200 - Motor_Voltage_L);
		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 200);
	} else {
		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 200);
		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2,
				200 - fabs(Motor_Voltage_L));
	}
	if (Motor_Voltage_R > 0) {
		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, 200 - Motor_Voltage_R);
		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, 200);
	} else {
		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, 200);
		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4,
				200 - fabs(Motor_Voltage_R));
	}
}

void Motor_Free() {
	G_Motor_V_Target = 0;
	G_Motor_Ac = 0;
	Suction_Stop();
	HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_1);
	HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_2);
	HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_3);
	HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_4);
}

void Motor_Back() {
	G_Motor_Flag = 1;
	G_Motor_V_Target = -200;
	G_Motor_Ac = 0;
	wait_ms(250);
	Enc_Sigma_error = 0;
	Motor_PWM_L = 0;
	Motor_PWM_R = 0;
	G_Motor_Flag = 0;
}

void Motor_Speed_PID_ST(int V, int Ac) { // mm/s

	float T = ((TIREDIAMETER / 2000) * ROBOT_M * (Ac / 1000)) / (2 * GEARRATIO);
	float w = 0;
//	float w = (60 * GEARRATIO * (V / 1000)) / (2 * PI * (TIREDIAMETER / 2000))
//			* Bff;
	Motor_FF_ST = (((MOTORR * T) / KT) + (w * KE)) / Run_Voltage;
	//Motor_FF_ST = 0;

	G_Robot_speed = ((G_Tire_Speed_L + G_Tire_Speed_R) / 2); //)+ (read_average_acc_x()*0.01/2)

	Enc_error = V - G_Robot_speed;
	Enc_Delta_error = Enc_error - Enc_Old_error;
	Enc_Sigma_error += Enc_error;

	if (Fun_Flag == 1) {
		Motor_FB_ST = Ksp_Fun * Enc_error + Ksi_Fun * Enc_Sigma_error
				+ Ksd_Fun * Enc_Delta_error;
	} else {
		Motor_FB_ST = (Ksp * Enc_error + Ksi * Enc_Sigma_error
				+ Ksd * Enc_Delta_error);
	}

	if (PID_Mode == 0) {
		Motor_PWM_L = Motor_FB_ST + Motor_FF_ST;
		Motor_PWM_R = Motor_FB_ST + Motor_FF_ST;
	} else if (PID_Mode == 1) {
//		if (Fun_Flag == 1) {
//			Motor_PWM_L = Motor_FF_ST + Motor_FB_ST + calWallConrol(); //
//			Motor_PWM_R = Motor_FF_ST + Motor_FB_ST - calWallConrol(); //
//		} else {
		Motor_PWM_L = Motor_FF_ST + Motor_FB_ST + calWallConrol(); //
		Motor_PWM_R = Motor_FF_ST + Motor_FB_ST - calWallConrol(); //
//		}
	} else if (PID_Mode == 2) {
		Motor_PWM_L = Motor_FB_ST + Motor_FF_ST + calWallConrol_NANAME();
		Motor_PWM_R = Motor_FB_ST + Motor_FF_ST - calWallConrol_NANAME();
	} else if (PID_Mode == 3) {
		Motor_PWM_L = calWallConrol_Flontwall_ST(); //
		Motor_PWM_R = calWallConrol_Flontwall_ST(); //
	}

	Enc_Old_error = Enc_error;

}

void Motor_Speed_PID_Turn(float W, float W_Ac) {
	float T = ((TIREDIAMETER / 2000) * ROBOT_M * (W_Ac / 1000))
			/ (2 * GEARRATIO);
//	float w = (60 * GEARRATIO * (W / 1000)) / (2 * PI * (TIREDIAMETER / 2000))
//			* Bff_Turn;
	float w = 0;
	Motor_FF_ST = (((MOTORR * T) / KT) + (w * KE)) / Run_Voltage * 7;
	//Motor_FF_Turn = 0;

	if (Turn_Mode == 1) { //右向き
		Gyro_error = -W - G_Gyro_Z;
	} else { //左向き
		Gyro_error = W - G_Gyro_Z;
	}

	Gyro_Delta_error = Gyro_error - Gyro_Old_error;
	Gyro_Sigma_error += Gyro_error;

	if (G_Motor_Flag == 1) { //直線
		Motor_FB_Turn = Ksp_AT * Gyro_error; //+ Ksi_AT * Gyro_Sigma_error+ Ksd_AT * Gyro_Delta_error
		Motor_PWM_L -= cal_turnV(Motor_FB_Turn);
		Motor_PWM_R += cal_turnV(Motor_FB_Turn);
	} else if (G_Motor_Flag == 2) { //超シンチ
		Motor_FB_Turn = Ktp * Gyro_error + Kti * Gyro_Sigma_error
				+ Ktd * Gyro_Delta_error;
		Motor_PWM_L -= cal_turnV(Motor_FB_Turn);
		Motor_PWM_R += cal_turnV(Motor_FB_Turn);
	} else if (G_Motor_Flag == 3) { //スラローム
		if (Sula_Flag == 1) {
			Motor_FB_Turn = Ktp_sula_20 * Gyro_error
					+ Kti_sula_20 * Gyro_Sigma_error
					+ Ktd_sula_20 * Gyro_Delta_error;
		} else if (Sula_Flag == 2) {
			Motor_FB_Turn = Ktp_sula_24 * Gyro_error
					+ Kti_sula_24 * Gyro_Sigma_error
					+ Ktd_sula_24 * Gyro_Delta_error; //
		} else {
			Motor_FB_Turn = Ktp_sula * Gyro_error + Kti_sula * Gyro_Sigma_error
					+ Ktd_sula * Gyro_Delta_error; //
		}
		Motor_PWM_L -= cal_turnV(Motor_FB_Turn + Motor_FF_Turn);
		Motor_PWM_R += cal_turnV(Motor_FB_Turn + Motor_FF_Turn);
	} else if (G_Motor_Flag == 4) {
		Motor_PWM_L -= calWallConrol_Flontwall_Turn();
		Motor_PWM_R += calWallConrol_Flontwall_Turn();
	}

	Gyro_Old_error = Gyro_error;
}

void Motor_trapezoid(float Vst, float Vmax, float Vend, float Ac, float X) {
	G_Motor_Flag = 1;

	G_Motor_V_Target = Vst;
	G_Motor_X = 0;
	G_Motor_Ac = Ac;

	G_Motor_Angle = 0;
	G_Motor_W_Ac = 0;
	G_Motor_W_Target = 0;

	Enc_Sigma_error = 0;
	Gyro_Sigma_error = 0;

	G_Motor_Count = 0;

	float Acceleration_X = (Vmax * Vmax - Vst * Vst) / (4 * Ac) * PI;
	float Deceleration_X = 0;
	float Constant_X = 0;

	float x13 = PI / (2 * Ac) * (Vmax * Vmax - ((Vst * Vst + Vend * Vend) / 2));
	if (x13 > X) {
		Vmax = sqrt((2 * Ac * X / PI) + ((Vst * Vst + Vend * Vend) / 2));
	}

	float Df1 = Vmax - Vst;
	float t1 = PI * Df1 / 2 / Ac;
	while (1) {

		if (Df1 > 0) {
			G_Motor_V_Target = Df1 / 2 * (1 - cos(2 * Ac / Df1 * G_Motor_Count))
					+ Vst;
		}

		if (Vmax <= Vend) {
			Deceleration_X = 0;
		} else {
			Deceleration_X = (G_Motor_V_Target * G_Motor_V_Target - Vend * Vend)
					/ (4 * Ac) * PI;
		}

		if (t1 <= G_Motor_Count) {
			G_Motor_V_Target = Vmax;
			Deceleration_X = (G_Motor_V_Target * G_Motor_V_Target - Vend * Vend)
					/ (4 * Ac) * PI;
			break;
		}
		if (X - G_Motor_X < Deceleration_X) {
			Acceleration_X = G_Motor_X;
			break;
		}

	}

	Constant_X = X - (Acceleration_X + Deceleration_X);

	G_Motor_X = 0;
	G_Motor_Ac = 0;

	while (1) {
		if (G_Motor_X > Constant_X) {
			break;
		}

	}
	G_Motor_Count = 0;
	float Df3 = Vmax - Vend;
	float t3 = PI * Df3 / 2 / Ac;
	G_Motor_Ac = -Ac;
	while (1) {
		//Df3 / 2* (1 - cos(2 * Ac / Df3 * (G_Motor_Count - t1))) + Vend;
		G_Motor_V_Target = Df3 / 2
				- (Df3 / 2) * (1 - cos(G_Motor_Count * ((2 * Ac) / Df3))) + Vend
				+ (Df3 / 2);

		if (t3 <= G_Motor_Count) {
			break;
		}
	}
	G_Motor_V_Target = Vend;
	PID_Mode = 0;
	G_Motor_Flag = 0;
}

void Motor_trapezoid_PID(float Vst, float Vmax, float Vend, float Ac, float X) {
	PID_Mode = 1;
	Motor_trapezoid(Vst, Vmax, Vend, Ac, X);
}

void Motor_NANAME_PID(float Vst, float Vmax, float Vend, float Ac, float X) {
	PID_Mode = 2;
	Motor_trapezoid(Vst, Vmax, Vend, Ac, X);
}

void Motor_Sula_before(float Vst, float Vmax, float Vend, float Ac, float X) {
	G_Motor_Flag = 1;
	PID_Mode = 1;

	G_Motor_V_Target = Vst;
	G_Motor_X = 0;
	G_Motor_Ac = 0;

	G_Motor_Angle = 0;
	G_Motor_W_Ac = 0;
	G_Motor_W_Target = 0;

	Gyro_Sigma_error = 0;
	//Enc_Sigma_error = 0;

	if (Wall_Flont_Av() > Flont_th - 60) {		//壁補正前進用
		X = X + 15;
	}

	Wall_search();
	Maze_Wall_Update();

	while (1) {
		if (G_Motor_X > X) {
			break;
		}

		if (Wall_Flont_Av() > Flont_th) {
			break;
		}
	}

	PID_Mode = 0;
	G_Motor_Flag = 0;

}

void Motor_Sula_ST(float Vst, float Vmax, float Vend, float Ac, float X) {
	G_Motor_Flag = 1;
	PID_Mode = 1;

	G_Motor_V_Target = Vst;
	G_Motor_X = 0;
	G_Motor_Ac = 0;

	G_Motor_Angle = 0;
	G_Motor_W_Ac = 0;
	G_Motor_W_Target = 0;

	Gyro_Sigma_error = 0;
	//Enc_Sigma_error = 0;

	if (Wall_Flont_Av() > Flont_th - 60) {		//壁補正前進用
		X = X + 15;
	}

	while (1) {
		if (G_Motor_X > X) {
			break;
		}

		if (Wall_Flont_Av() > Flont_th) {
			break;
		}
	}

	PID_Mode = 0;
	G_Motor_Flag = 0;

}

void Motor_trapezoid_Turn(float Angle, float Wmax, float W_Ac) {
	int SANKAKU = 0;
	G_Motor_Flag = 2;
	G_Motor_V_Target = 0;
	G_Motor_X = 0;
	G_Motor_Ac = 0;

	G_Motor_Angle = 0;
	G_Motor_W_Ac = W_Ac;
	G_Motor_W_Target = 0;

	if (Angle < 0) {
		Angle = -Angle;
		Turn_Mode = 1;				//右回転
	} else {
		Turn_Mode = 0;				//左回転
	}

	Gyro_Sigma_error = 0;
	Enc_Sigma_error = 0;

	float Acceleration_X = (Wmax * Wmax) / (4 * W_Ac) * PI;
	float Deceleration_X = 0;
	float Constant_X = 0;

	float x13 = PI / (2 * W_Ac) * (Wmax * Wmax);
	if (x13 > Angle) {
		Wmax = sqrt(2 * W_Ac * Angle / PI);
		SANKAKU = 1;
	}
	float Df1 = Wmax;
	float t1 = PI * Df1 / 2 / W_Ac;
	G_Motor_Count = 0;
	while (1) {
		G_Motor_W_Target = Df1 / 2 * (1 - cos(2 * W_Ac / Df1 * G_Motor_Count));
		if (t1 <= G_Motor_Count) {
			G_Motor_W_Target = Wmax;
			Deceleration_X = (G_Motor_W_Target * G_Motor_W_Target)
					/ (4 * W_Ac)* PI;
			break;
		}
		if (Angle - G_Motor_Angle < Deceleration_X) {
			Acceleration_X = fabs(G_Motor_Angle);
			break;
		}

	}
	Constant_X = fabs(Angle - (Acceleration_X + Deceleration_X));

	G_Motor_W_Ac = 0;
	G_Motor_Angle = 0;

	while (1) {
		if (SANKAKU == 1) {
			SANKAKU = 0;
			break;
		}
		if (fabs(G_Motor_Angle) > Constant_X) {
			break;
		}
	}

	G_Motor_Count = 0;
	float Df3 = G_Motor_W_Target;
	float t3 = PI * Df3 / 2 / W_Ac;
	while (1) {
		G_Motor_W_Target = Df3 / 2
				* (1 - cos(2 * W_Ac / Df3 * (G_Motor_Count - t1)));
		if (t3 <= G_Motor_Count) {
			break;
		}
	}
	G_Motor_Flag = 0;
}

void Motor_Sula_COS(float V, float Angle, float Wmax, float W_Ac) {
	int SANKAKU = 0;
	G_Motor_Flag = 3;
	G_Motor_V_Target = V;
	G_Motor_X = 0;
	G_Motor_Ac = 0;

	G_Motor_Angle = 0;
	G_Motor_W_Ac = 0;
	G_Motor_W_Target = 0;

	if (Angle < 0) {
		Angle = -Angle;
		Turn_Mode = 1;				//右回転
	} else {
		Turn_Mode = 0;				//左回転
	}

	//Gyro_Sigma_error = 0;
	Enc_Sigma_error = 0;
	G_Motor_W_Ac = W_Ac;
	float Acceleration_X = (Wmax * Wmax) / (4 * W_Ac) * PI;
	float Deceleration_X = 0;
	float Constant_X = 0;

	float x13 = PI / (2 * W_Ac) * (Wmax * Wmax);
	if (x13 > Angle) {
		Wmax = sqrt(2 * W_Ac * Angle / PI);
		SANKAKU = 1;
	}
	float Df1 = Wmax;
	float t1 = PI * Df1 / 2 / W_Ac;
	G_Motor_Count = 0;
	while (1) {
		//printf("Ac____G_Motor_W_Target:%f----G_Motor_Angle:%f\n\r",
		//G_Motor_W_Target, G_Motor_Angle);
		G_Motor_W_Target = Df1 / 2 * (1 - cos(2 * W_Ac / Df1 * G_Motor_Count));
		if (t1 <= G_Motor_Count) {
			G_Motor_W_Target = Wmax;
			Deceleration_X = (G_Motor_W_Target * G_Motor_W_Target)
					/ (4 * W_Ac)* PI;
			break;
		}
		if (Angle - G_Motor_Angle < Deceleration_X) {
			Acceleration_X = fabs(G_Motor_Angle); //加速時の補填
			break;
		}

	}
	Constant_X = fabs(Angle - (Acceleration_X + Deceleration_X));

	G_Motor_W_Ac = 0;
	G_Motor_Angle = 0;

	while (1) {
		//printf("Co____G_Motor_W_Target:%f----G_Motor_Angle:%f\n\r",
		//G_Motor_W_Target, G_Motor_Angle + Acceleration_X);
		if (SANKAKU == 1) {
			SANKAKU = 0;
			break;
		}
		if (fabs(G_Motor_Angle) > Constant_X) {
			break;
		}
	}

	G_Motor_Count = 0;
	G_Motor_W_Ac = -W_Ac;
	float Df3 = G_Motor_W_Target;
	float t3 = PI * Df3 / 2 / W_Ac;
	while (1) {
		G_Motor_W_Target = Df3 / 2
				* (1 - cos(2 * W_Ac / Df3 * (G_Motor_Count - t1)));
		//printf("%f\n\r",G_Motor_V_Target);
		if (t3 <= G_Motor_Count) {
			break;
		}
	}
	Enc_Sigma_error = 0;
	//Gyro_Sigma_error = 0;
	G_Motor_Flag = 0;
}

void Motor_Wallcut_ST(float Vmax, float X, int direction) {		//壁の有無を変数に入れる
	G_Motor_Flag = 1;
	PID_Mode = 1;

	G_Motor_V_Target = Vmax;

	G_Motor_Ac = 0;
	float X_act = 0;

	//Enc_Sigma_error = 0;
	//Gyro_Sigma_error = 0;

	G_Motor_X = 0;
	while (1) {
		if (G_Motor_X > X * 0.50) {
			X_act = G_Motor_X;
			break;
		}
	}
	if (direction == 0) { //左旋回
		if (G_Wall_data[1] == 1) {
			while (1) {
				if (g_sensor_av[2] < Cut_L) {
					break;
				}
				LED_ON_L();
				if (Wall_Flont_Av() > 100) {
					break;
				}
			}
		}
	} else if (direction == 1) { //右旋回
		if (G_Wall_data[2] == 1) {
			while (1) {
				if (g_sensor_av[1] < Cut_R) {
					break;
				}
				if (Wall_Flont_Av() > 100) {
					break;
				}
				LED_ON_R();
			}
		}
	}
	LED_Reset();
	G_Motor_X = 0;
	while (1) {
		if (G_Motor_X > X - X_act) {
			break;
		}
		if (Wall_Flont_Av() > 100) {
			break;
		}
	}
	PID_Mode = 0;
}

void Motor_Wallcut_END(float Vmax, float X, int direction) {
	G_Motor_Flag = 1;
	PID_Mode = 1;

	G_Motor_V_Target = Vmax;

	G_Motor_Ac = 0;

	Enc_Sigma_error = 0;
	Gyro_Sigma_error = 0;

	Wall_search();
	int Wall_L = G_Wall_data[1];
	int Wall_R = G_Wall_data[2];

	G_Motor_X = 0;

	while (1) {
		if (G_Motor_X > X) {
			break;
		}
		if (Wall_L == 1) {
			if (g_sensor_av[2] < Cut_L) {
				break;
			}
		}
		if (Wall_R == 1) {
			if (g_sensor_av[1] < Cut_R) {
				break;
			}
		}
	}
	PID_Mode = 0;
}

void Motor_Wallcut_ST_NANAME(float Vmax, float X, int direction) {
	G_Motor_Flag = 1;
	PID_Mode = 2; //2

	G_Motor_V_Target = Vmax;

	G_Motor_Ac = 0;

	Enc_Sigma_error = 0;
	Gyro_Sigma_error = 0;
	Wall_search();
	G_Motor_X = 0;
	if (direction == 0) { //左旋回
		while (1) {
			if (g_sensor_av[2] < Cut_L_NA) {
				break;
			}
//			if (G_Motor_X > X * 0.4) {
//				break;
//			}
		}
	} else if (direction == 1) { //右旋回
		while (1) {
			if (g_sensor_av[1] < Cut_R_NA) {
				break;
			}
//			if (G_Motor_X > X * 0.4) {
//				break;
//			}
		}
	}
	G_Motor_X = 0;
	while (1) {
		if (G_Motor_X > X) {
			break;
		}
	}
	PID_Mode = 0;
}

void Motor_Wallcut_END_NANAME(float Vmax, float X, int direction) {
	G_Motor_Flag = 1;
	PID_Mode = 2; //2

	G_Motor_V_Target = Vmax;

	G_Motor_Ac = 0;

	Enc_Sigma_error = 0;
	Gyro_Sigma_error = 0;

	G_Motor_X = 0;
	Wall_search();
	int Flag = 0;
	if (G_Wall_data[2] == 1 && direction == 0) {
		Flag = 1;
	} else if (G_Wall_data[1] == 1 && direction == 1) {
		Flag = 1;
	}

	while (1) {
		if (G_Motor_X > X) {
			break;
		}
	}

//	while (1) {
//		if (G_Motor_X > X) {
//			break;
//		}
//		if (direction == 0) { //左旋回
//			if (Flag == 1) {
//				if (g_sensor_av[1] < Cut_R_NA) {
//					break;
//				}
//			}
//		} else if (direction == 1) {
//			if (Flag == 1) {
//				if (g_sensor_av[2] < Cut_L_NA) {
//					break;
//				}
//			}
//		}
//	}
	PID_Mode = 0;
}

void Motor_Robot_Alignment() {
	G_Motor_Flag = 4;
	PID_Mode = 3;
	G_Motor_V_Target = 0;
	G_Motor_X = 0;
	G_Motor_Ac = 0;

	G_Motor_Angle = 0;
	G_Motor_W_Ac = 0;
	G_Motor_W_Target = 0;

	Enc_Sigma_error = 0;
	G_Motor_Count = 0;
	while (1) {
		if (G_Motor_Count > Alignment_TIME) {
			break;
		}
	}
	Enc_Sigma_error = 0;
	Gyro_Sigma_error = 0;
	PID_Mode = 0;
	G_Motor_Flag = 0;
}

/*
 #include "tim.h"
 #include "PL_encoder.h"
 #include "PL_timer.h"
 #include "PL_sensor.h"
 #include "Define.h"
 #include "lsm6dsr.h"
 #include "math.h"
 #include "Wallsensor.h"
 #include "Maze.h"

 float G_Motor_X = 0;
 float G_Motor_Ac = 0;
 float G_Motor_V_Target = 0;

 float G_Motor_Angle = 0;
 float G_Motor_W_Ac = 0;
 float G_Motor_W_Target = 0;

 float G_Robot_Angle = 0;
 float G_Robot_Target_Angle = 0;
 float G_Attitude_Mode = 0;

 float G_Gyro_Z = 0;

 float G_Robot_speed = 0;

 float Motor_PWM_L, Motor_PWM_R;

 float Motor_FB_ST;
 float Motor_FB_Turn;

 float Enc_error = 0;
 float Enc_Old_error = 0;
 float Enc_Sigma_error = 0;
 float Enc_Delta_error = 0;

 float Gyro_error = 0;
 float Gyro_Old_error = 0;
 float Gyro_Sigma_error = 0;
 float Gyro_Delta_error = 0;

 float Angle_error = 0;
 float Angle_Old_error = 0;
 float Angle_Sigma_error = 0;
 float Angle_Delta_error = 0;

 float Aff = 1;
 float Bff = 0;
 float Cff = 0;

 float Ksp = 0.2; //0.3
 float Ksi = 0.008; //0.005
 float Ksd = 0.005; //0.1

 float Ksp_Fun = 0.05; //0.1
 float Ksi_Fun = 0.004; //0.00105
 float Ksd_Fun = 0.2; //0.1

 float Ksp_turn = 0.0; //
 float Ksi_turn = 0.0; //
 float Ksd_turn = 0.0; //

 float Ktp = 0.07; //
 float Kti = 0.01; //
 float Ktd = 0.005; //

 float Ksp_AT = 0.2; //23
 float Ksi_AT = 0.0001; ////
 float Ksd_AT = 0.01;

 float Ksp_Angle = 0.5; //1.7  09
 float Ksi_Angle = 0.0;
 float Ksd_Angle = 4.0; //6.0

 float Aff_Turn = 3;
 float Bff_Turn = 1.8;
 float Cff_Turn = 0;

 float Ktp_sula = 0.25; //0.12
 float Kti_sula = 0.025; //0.06
 float Ktd_sula = 0.001; //0.01

 float Ktp_sula_Fun = 0.24; //
 float Kti_sula_Fun = 0.03; //
 float Ktd_sula_Fun = 0.2; //

 int PID_Mode = 0;
 int Turn_Mode = 0;

 int Fun_Flag = 0;
 int G_Motor_Flag = 0;
 float G_Motor_Count = 0;

 int Flont_th = 490; //310

 float Cut_R = 80;
 float Cut_L = 80;

 float Cut_R_NA = 500;
 float Cut_L_NA = 500;

 float Motor_FF_ST = 0;
 float Motor_FF_Turn = 0;

 float Motor_Back_Flag = 0;

 void Motor_Setup() {
 HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
 HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_2);
 HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3);
 HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_4);
 Enc_Sigma_error = 0;
 Gyro_Sigma_error = 0;
 }

 void Suction_Start(int duty) {
 HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
 __HAL_TIM_SET_AUTORELOAD(&htim2, 100);
 duty = duty * (7.4 / g_V_batt);
 for (int i = 1; 80 > i; i += 1) {
 __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, i);
 HAL_Delay(30);
 }
 for (int i = 80; duty < i; i -= 1) {
 __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, i);
 HAL_Delay(30);
 }
 HAL_Delay(1000);
 Fun_Flag = 1;
 }

 void Suction_Stop() {
 HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_2);
 Fun_Flag = 0;
 }

 void Motor_Free() {
 G_Motor_V_Target = 0;
 G_Motor_Ac = 0;
 Suction_Stop();
 HAL_TIM_PWM_Stop(&htim4, TIM_CHANNEL_1);
 HAL_TIM_PWM_Stop(&htim4, TIM_CHANNEL_2);
 HAL_TIM_PWM_Stop(&htim4, TIM_CHANNEL_3);
 HAL_TIM_PWM_Stop(&htim4, TIM_CHANNEL_4);
 }

 void Motor_Stop() {
 G_Motor_Flag = 1;
 G_Motor_V_Target = 0;
 G_Motor_Ac = 0;
 Enc_Sigma_error = 0;
 Gyro_Sigma_error = 0;
 Motor_PWM_L = 0;
 Motor_PWM_R = 0;
 wait_ms(100);
 Motor_PWM_L = 0;
 Motor_PWM_R = 0;
 Enc_Sigma_error = 0;
 Gyro_Sigma_error = 0;
 G_Motor_Flag = 0;

 }

 void Motor_Back() {
 G_Motor_Flag = 1;
 Motor_Back_Flag = 1;
 G_Motor_V_Target = -150;
 G_Motor_Ac = 0;
 wait_ms(250);
 Enc_Sigma_error = 0;
 Motor_PWM_L = 0;
 Motor_PWM_R = 0;
 G_Motor_Flag = 0;
 Motor_Back_Flag = 0;
 }




 void Motor_seach_Wallcut(float Vmax, float X) {
 G_Motor_Flag = 1;
 PID_Mode = 1;

 G_Motor_V_Target = Vmax;
 G_Motor_Ac = 0;
 G_Motor_X = 0;

 Wall_search();

 //Enc_Sigma_error = 0;
 Gyro_Sigma_error = 0;

 while (1) {
 if (G_Motor_X > X) {
 break;
 } else if (G_Motor_X < 20) {
 if (G_Wall_data[1] == 1) {
 if (g_sensor[1][0] < Cut_L) {
 X = 45;
 break;
 }
 } else if (G_Wall_data[2] == 1) {
 if (g_sensor[2][0] < Cut_R) {
 X = 45;

 }
 }
 }
 }

 }

 void Motor_Sula_ST(float Vst, float Vmax, float Vend, float Ac, float X) {
 G_Motor_Flag = 1;
 PID_Mode = 0;

 G_Motor_V_Target = Vst;
 G_Motor_X = 0;
 G_Motor_Ac = 0;

 G_Motor_Angle = 0;
 G_Motor_W_Ac = 0;
 G_Motor_W_Target = 0;

 //		Wall_search();
 //		Maze_Wall_Update();

 Enc_Sigma_error = 0;
 Gyro_Sigma_error = 0;

 if (Wall_Flont_Av() > Flont_th - 200) {		//壁補正前進用
 X = X + 15;
 }

 while (1) {
 if (G_Motor_X > X) {
 break;
 }

 if (Wall_Flont_Av() > Flont_th) {
 break;
 }
 }

 PID_Mode = 0;
 G_Motor_Flag = 0;
 }





 void Motor_Wallcut_ST(float Vmax, float X, int direction) {
 G_Motor_Flag = 1;
 PID_Mode = 1;

 G_Motor_V_Target = Vmax;

 G_Motor_Ac = 0;

 //Enc_Sigma_error = 0;
 Gyro_Sigma_error = 0;

 Wall_search();
 G_Motor_X = 0;
 while (1) {
 if (G_Motor_X > X * 0.50) {
 break;
 }
 }
 int time = 0;
 if (direction == 0) { //左旋回
 if (G_Wall_data[1] == 1) {
 while (1) {
 if (g_sensor[1][0] < Cut_L) {
 break;
 }
 time++;
 //				if (time > 10000000) {
 //					break;
 //				}
 }
 //		} else if (Wall_Flont_Av() > 55) {
 //			while (Wall_Flont_Av() < 55)
 //				;
 }
 } else if (direction == 1) { //右旋回
 if (G_Wall_data[2] == 1) {
 while (1) {
 if (g_sensor[2][0] < Cut_R) {
 break;
 }
 time++;
 //				if (time > 10000000) {
 //					break;
 //				}
 }
 //		} else if (Wall_Flont_Av() > 55) {
 //			while (Wall_Flont_Av() < 55)
 //				;
 }
 }
 G_Motor_X = 0;
 while (1) {
 if (G_Motor_X > X * 0.50) {
 break;
 }
 }
 PID_Mode = 0;
 }

 void Motor_Wallcut_END(float Vmax, float X, int direction) {
 G_Motor_Flag = 1;
 PID_Mode = 1;

 G_Motor_V_Target = Vmax;

 G_Motor_Ac = 0;

 //Enc_Sigma_error = 0;
 Gyro_Sigma_error = 0;

 Wall_search();
 G_Motor_X = 0;

 while (1) {
 if (G_Motor_X > X) {
 break;
 }
 if (G_Wall_data[1] == 1) {
 if (g_sensor[1][0] < Cut_L) {
 break;
 }
 }
 if (G_Wall_data[2] == 1) {
 if (g_sensor[2][0] < Cut_R) {
 break;
 }
 }
 }
 PID_Mode = 0;
 }

 void Motor_Wallcut_ST_NANAME(float Vmax, float X, int direction) {
 G_Motor_Flag = 1;
 PID_Mode = 2;

 G_Motor_V_Target = Vmax;

 G_Motor_Ac = 0;

 //Enc_Sigma_error = 0;
 Gyro_Sigma_error = 0;
 Wall_search();
 G_Motor_X = 0;
 while (1) {
 if (G_Motor_X > X * 0.40) {
 break;
 }
 }
 G_Motor_X = 0;
 if (direction == 0) { //左旋回
 while (1) {
 if (g_sensor[1][0] < Cut_L_NA) {
 break;
 }
 //			if (G_Motor_X > X * 0.4) {
 //				break;
 //			}
 }
 } else if (direction == 1) { //右旋回
 while (1) {
 if (g_sensor[2][0] < Cut_R_NA) {
 break;
 }
 //			if (G_Motor_X > X * 0.4) {
 //				break;
 //			}
 }
 }
 G_Motor_X = 0;
 while (1) {
 if (G_Motor_X > X * 0.6) {
 break;
 }
 }
 PID_Mode = 0;
 }

 void Motor_Wallcut_END_NANAME(float Vmax, float X, int direction) {
 G_Motor_Flag = 1;
 PID_Mode = 2;

 G_Motor_V_Target = Vmax;

 G_Motor_Ac = 0;

 //Enc_Sigma_error = 0;
 Gyro_Sigma_error = 0;
 Wall_search();
 int Flag = 0;
 if (G_Wall_data[2] == 1 && direction == 0) {
 Flag = 1;
 } else if (G_Wall_data[1] == 1 && direction == 1) {
 Flag = 1;
 }
 G_Motor_X = 0;
 while (1) {
 if (G_Motor_X > X) {
 break;
 }
 if (direction == 0) { //左旋回
 if (Flag == 1) {
 if (g_sensor[2][0] < Cut_R_NA) {
 break;
 }
 }
 } else if (direction == 1) {
 if (Flag == 1) {
 if (g_sensor[1][0] < Cut_L_NA) {
 break;
 }
 }
 }
 }
 PID_Mode = 0;
 }

 */
