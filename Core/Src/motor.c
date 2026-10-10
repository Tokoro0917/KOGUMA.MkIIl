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
#include "WallDistance.h"
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

float Ksp = 0.25; //0.8
float Ksi = 0.0015; //0.002
float Ksd = 0.01; //0.5

float Ksp_Fun = 0.23; //0.5
float Ksi_Fun = 0.0015; //0.005
float Ksd_Fun = 0.1; //0.01

float Ksp_AT = 0.3; //0.3
float Ksi_AT = 0.003; ////
float Ksd_AT = 0.01;

float Ktp = 0.7; //
float Kti = 0.01; //
float Ktd = 0.001; //

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

float Motor_Voltage_L = 0;
float Motor_Voltage_R = 0;

int PID_Mode = 0;
int Turn_Mode = 0;
int Fun_Flag = 0;

int Sula_Flag = 0;
static int Suction_Duty = 0;	//今の吸引のduty(Suction_Start/Suction_changeで指定した値)

float Alignment_TIME = 0.5;


float Cut_R = 80;
float Cut_L = 80;

float Cut_R_NA = 150;
float Cut_L_NA = 150;

/* 壁切れを柱で見る(2026-10-10)。0 は従来どおり「壁があるときだけ、センサ値が Cut_* を下回ったら」。
 * 1 は「壁の有無に関係なく、柱の後ろの端でセンサ値が Pillar_TH 以上減ったら」(Wallsensor.c の Pillar_Edge)。
 * 区画の境目には必ず柱があるので、壁のない所でも壁切れできる。
 * 斜めの壁切れ(Cut_L_NA / Cut_R_NA を使うもの)は柱版でも従来どおり。
 * 最短走行の間だけ main.c の Short_WallCut_Pillar に従って立てる */
int G_WallCut_Pillar = 0;
int Pillar_TH = 40;	//柱の切れ目とみなす減り方(センサ値、Pillar_Len 進む間)
float Pillar_MaxX = 140;	//後距離で、柱が見つからないまま、これだけ進んだら諦める[mm]
#define PILLAR_WARM_MS 9	//ターン直後は、ターン中のセンサ値が履歴に残っているので見ない[ms]

float Run_Voltage;

float Aff = 0.65;
float Bff = 0.50;

float FF_offset = 15.0;

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
	PID_Mode = 0;
	G_Motor_V_Target = 0;
	G_Motor_Ac = 0;
	Enc_Sigma_error = 0;
	Gyro_Sigma_error = 0;
//	Motor_PWM_L = 0;
//	Motor_PWM_R = 0;
	wait_ms(50);
	Motor_PWM_L = 0;
	Motor_PWM_R = 0;
	Enc_Sigma_error = 0;
	Gyro_Sigma_error = 0;
	G_Motor_Flag = 0;

}

void Suction_Start(int duty) {
	Suction_Duty = duty;
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

void Suction_change(int duty) {
	Suction_Duty = duty;
	duty = duty * (15.8 / Run_Voltage);
	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, duty);
}

void Suction_Stop() {
	HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_1);
	Sula_Flag = 0;
	Fun_Flag = 0;
	Suction_Duty = 0;
}

/* 一時的に吸引を上げる(最短走行の最初のターンなど)。今の吸引が duty 以上なら何もしない。
 * 吸引と一緒に、直線とターンのPIDゲインの組(Fun_Flag / Sula_Flag)も Suction_Start(duty) と同じにする。
 * 吸引が上がりきるまで wait[ms] 待つので、止まっているときに呼ぶ。Suction_Boost_End() で元に戻す */
static int Boost_On = 0;
static int Boost_Duty_Prev = 0;
static int Boost_Fun_Prev = 0;
static int Boost_Sula_Prev = 0;

void Suction_Boost_Start(int duty, int wait) {
	Boost_On = 0;
	if (Suction_Duty >= duty) {
		return;
	}
	Boost_Duty_Prev = Suction_Duty;
	Boost_Fun_Prev = Fun_Flag;
	Boost_Sula_Prev = Sula_Flag;
	Suction_change(duty);
	Fun_Flag = 1;
	if (duty >= 70) {
		Sula_Flag = 2;
	} else if (duty == 50) {
		Sula_Flag = 1;
	}
	Boost_On = 1;
	HAL_Delay(wait);
}

void Suction_Boost_End() {
	if (!Boost_On) {
		return;
	}
	Suction_change(Boost_Duty_Prev);
	Fun_Flag = Boost_Fun_Prev;
	Sula_Flag = Boost_Sula_Prev;
	Boost_On = 0;
}

float cal_turnV(float W) {
	float TurnV = ((float) TIREBETWEEN * PI / 360) * W;
	return TurnV;
}

void Motor_PWM_Generate() { //MP6550モーター出力

	Motor_Voltage_L = Motor_PWM_L * (REFERENCE_V / Run_Voltage);
	Motor_Voltage_R = Motor_PWM_R * (REFERENCE_V / Run_Voltage);

	if (Motor_Voltage_L > 200) {
		Motor_Voltage_L = 200;
	} else if (Motor_Voltage_L < -200) {
		Motor_Voltage_L = -200;
	}

	if (Motor_Voltage_R > 200) {
		Motor_Voltage_R = 200;
	} else if (Motor_Voltage_R < -200) {
		Motor_Voltage_R = -200;
	}

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
	G_Motor_Flag = 5;
	G_Motor_V_Target = -300;
	G_Motor_Ac = 0;
	wait_ms(250);
	Enc_Sigma_error = 0;
	Motor_PWM_L = 0;
	Motor_PWM_R = 0;
	G_Motor_Flag = 0;
}

void Motor_Speed_PID_ST(int V, int Ac) { // mm/s

	float T = ((TIREDIAMETER / 2000) * ROBOT_M * (Ac / 1000)) / (2 * GEARRATIO)
			* Aff;
	//float w = 0;
	float w = (60 * GEARRATIO * (V / 1000)) / (2 * PI * (TIREDIAMETER / 2000))
			* Bff;
	Motor_FF_ST = (((MOTORR * T) / KT) + (w * KE)) / Run_Voltage * 200.0;

	if (V > 1) {
		Motor_FF_ST += FF_offset;
	} else if (V < -1) {
		Motor_FF_ST -= FF_offset;
	}

	G_Robot_speed = ((G_Tire_Speed_L + G_Tire_Speed_R) / 2); //)+ (read_average_acc_x()*0.01/2)

	Enc_error = V - G_Robot_speed;
	Enc_Delta_error = Enc_error - Enc_Old_error;
	Enc_Sigma_error += Enc_error;

	if (Enc_Sigma_error > 2000)
		Enc_Sigma_error = 2000;
	else if (Enc_Sigma_error < -2000)
		Enc_Sigma_error = -2000;

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
		float wall = Wall_Control_Update();	//横壁制御はここで1msに1回だけ計算する
		Motor_PWM_L = Motor_FF_ST + Motor_FB_ST + wall; //
		Motor_PWM_R = Motor_FF_ST + Motor_FB_ST - wall; //
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
	float w = 0;
	if (PID_Mode == 1) {
		W = calWallConrol() * 0.4;
	} else if (PID_Mode == 2) {
		//W = calWallConrol() * 0.5;
	}
	float T = ((TIREDIAMETER / 2000) * ROBOT_M * (W_Ac / 1000))
			/ (2 * GEARRATIO);
//	float w = (60 * GEARRATIO * (W / 1000)) / (2 * PI * (TIREDIAMETER / 2000))
//			* Bff_Turn;
	Motor_FF_ST = (((MOTORR * T) / KT) + (w * KE)) / Run_Voltage * 7;
	//Motor_FF_Turn = 0;

	if (G_Motor_Flag == 1) { //直線: W は横壁制御(左の壁に近いと正 -> 右へ回す)
		/* 以前は直前のターンの向き(Turn_Mode)で符号が決まり、左ターンのあとは
		 * 壁に寄る向きに効いて、左右のPWMへの横壁制御と打ち消し合っていた */
		Gyro_error = -W - G_Gyro_Z;
	} else if (Turn_Mode == 1) { //右向き
		Gyro_error = -W - G_Gyro_Z;
	} else { //左向き
		Gyro_error = W - G_Gyro_Z;
	}

	Gyro_Delta_error = Gyro_error - Gyro_Old_error;
	Gyro_Sigma_error += Gyro_error;

	if (G_Motor_Flag == 1) { //直線
		Motor_FB_Turn = Ksp_AT * Gyro_error + Ksi_AT * Gyro_Sigma_error
				+ Ksd_AT * Gyro_Delta_error;
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
	} else if (G_Motor_Flag == 5) {
		Motor_FB_Turn = Ksp_AT * Gyro_error;
		Motor_PWM_L -= cal_turnV(Motor_FB_Turn);
		Motor_PWM_R += cal_turnV(Motor_FB_Turn);
	}

	Gyro_Old_error = Gyro_error;
}

void Motor_trapezoid(float Vst, float Vmax, float Vend, float Ac, float X) {
	G_Motor_Flag = 1;

	G_Motor_V_Target = Vst;
	G_Motor_X = 0;
	G_Motor_Ac = 0;

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
			G_Motor_Ac = Ac * sin(2 * Ac / Df1 * G_Motor_Count);
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

		if (Df3 > 0) {
			G_Motor_Ac = -Ac * sin(G_Motor_Count * ((2 * Ac) / Df3));
		}

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

void Motor_Multistage_PID(float Vst, float Vmax, float Vend, float Ac_base,
		float X) {
	// --- 加速度と閾値の設定（例） ---
	float Ac_1 = Ac_base * 1.2;  // 低速域：超加速
	float Ac_2 = Ac_base * 1.0;  // 中速域：標準
	float Ac_3 = Ac_base * 0.6;  // 高速域：飽和防止
	float Ac_De = Ac_base * 2;

	float V_th1 = 2000.0f;        // 1段階目の切り替え速度
	float V_th2 = 3500.0f;        // 2段階目の切り替え速度

	G_Motor_Flag = 1;
	PID_Mode = 1;
	G_Motor_V_Target = Vst;
	G_Motor_X = 0;
	G_Motor_Count = 0;

	float prev_count = 0;
	float dt = 0;
	float Deceleration_X = 0;

	// ==========================================
	// 1. 加速フェーズ（3段階切り替え）
	// ==========================================
	while (G_Motor_V_Target < Vmax) {
		dt = G_Motor_Count - prev_count;
		prev_count = G_Motor_Count;

		// ★3段階の加速度判定
		if (G_Motor_V_Target < V_th1) {
			G_Motor_Ac = Ac_1;
		} else if (G_Motor_V_Target < V_th2) {
			G_Motor_Ac = Ac_2;
		} else {
			G_Motor_Ac = Ac_3;
		}

		G_Motor_V_Target += G_Motor_Ac * dt;
		if (G_Motor_V_Target > Vmax)
			G_Motor_V_Target = Vmax;

		// 常に減速開始距離をチェック（一定の Ac_base で減速すると仮定）
		Deceleration_X = (G_Motor_V_Target * G_Motor_V_Target - Vend * Vend)
				/ (2 * Ac_De);
		if (X - G_Motor_X <= Deceleration_X)
			goto DECEL_START;
	}

	// ==========================================
	// 2. 等速フェーズ
	// ==========================================
	G_Motor_Ac = 0;
	while (1) {
		prev_count = G_Motor_Count;
		Deceleration_X = (Vmax * Vmax - Vend * Vend) / (2 * Ac_De);
		if (X - G_Motor_X <= Deceleration_X)
			break;
	}

	DECEL_START:
	// ==========================================
	// 3. 減速フェーズ（一定）
	// ==========================================
	G_Motor_Ac = -Ac_De;
	while (G_Motor_V_Target > Vend) {
		dt = G_Motor_Count - prev_count;
		prev_count = G_Motor_Count;
		G_Motor_V_Target += G_Motor_Ac * dt;
		if (G_Motor_V_Target <= Vend) {
			G_Motor_V_Target = Vend;
			break;
		}
	}

	G_Motor_V_Target = Vend;
	PID_Mode = 0;
	G_Motor_Flag = 0;
}

void Motor_trapezoid_Asymmetric_PID(float Vst, float Vmax, float Vend, float Ac,
		float X) {
	G_Motor_Flag = 1;
	PID_Mode = 1;
	G_Motor_V_Target = Vst;
	G_Motor_X = 0;
	G_Motor_Ac = 0;

	G_Motor_Angle = 0;
	G_Motor_W_Ac = 0;
	G_Motor_W_Target = 0;

	Enc_Sigma_error = 0;
	Gyro_Sigma_error = 0;

	G_Motor_Count = 0;

	// ★減速加速度を加速の2倍に固定
	float Ac_dec = Ac * 2.0f;

	// 加速・減速それぞれの理論上の必要距離
	float Acceleration_X = (Vmax * Vmax - Vst * Vst) / (4 * Ac) * PI;
	float Deceleration_X = (Vmax * Vmax - Vend * Vend) / (4 * Ac_dec) * PI;
	float Constant_X = 0;

	// 三角加速（等速区間がない場合）の判定とVmax再計算
	if (Acceleration_X + Deceleration_X > X) {
		// 非対称加速度（Acと2Ac）の場合の到達可能最高速度を算出
		Vmax = sqrt(
				((8.0f * Ac * X / PI) + 2.0f * Vst * Vst + Vend * Vend) / 3.0f);
	}

	float Df1 = Vmax - Vst;
	float t1 = PI * Df1 / 2 / Ac;

	// --- 1. 加速フェーズ ---
	while (1) {
		if (Df1 > 0) {
			G_Motor_V_Target = Df1 / 2 * (1 - cos(2 * Ac / Df1 * G_Motor_Count))
					+ Vst;
			G_Motor_Ac = Ac * sin(2 * Ac / Df1 * G_Motor_Count);
		}

		// 走行中に常に減速距離（Ac_decを使用）をチェック
		float current_Decel_X = (G_Motor_V_Target * G_Motor_V_Target
				- Vend * Vend) / (4 * Ac_dec) * PI;

		if (t1 <= G_Motor_Count) {
			G_Motor_V_Target = Vmax;
			G_Motor_Ac = 0;
			Deceleration_X = current_Decel_X; // 確定
			break;
		}
		if (X - G_Motor_X < current_Decel_X) {
			Acceleration_X = G_Motor_X;
			Deceleration_X = current_Decel_X; // 確定
			break;
		}
	}

	// 等速区間の計算
	Constant_X = X - (Acceleration_X + Deceleration_X);
	G_Motor_X = 0;
	G_Motor_Ac = 0;

	// --- 2. 等速フェーズ ---
	while (1) {
		if (G_Motor_X > Constant_X) {
			break;
		}
	}

	// --- 3. 減速フェーズ（加速度は2倍の Ac_dec を使用） ---
	G_Motor_Count = 0;
	float Df3 = Vmax - Vend;
	float t3 = PI * Df3 / 2 / Ac_dec;

	while (1) {
		if (Df3 > 0) {
			// 減速のcosカーブ。終了点(Vend)に向かって滑らかに着地
			G_Motor_V_Target = Df3 / 2
					* (1 + cos(2 * Ac_dec / Df3 * G_Motor_Count)) + Vend;
			G_Motor_Ac = -Ac_dec * sin(2 * Ac_dec / Df3 * G_Motor_Count);
		}

		if (t3 <= G_Motor_Count) {
			break;
		}
	}

	G_Motor_V_Target = Vend;
	G_Motor_Ac = 0;
	PID_Mode = 0;
	G_Motor_Flag = 0;
}

/* スラロームの前の直進(Motor_Sula_before / Motor_Sula_ST)で、前壁が見えているときは、
 * 前センサで測った前壁までの距離からターンを始める位置を決める。
 * 区画の境目にいるとき、その区画の前壁までは 84 + 90 = 174mm。
 * 前距離 X なら、前壁までが 174 - X mm になったところでターンに入る
 * (以前はセンサ値130で打ち切っていたが、これは境目より手前の約178mmで、
 *  前壁があるとすぐ打ち切られて境目からターンしていた) */
#define SULA_BOUNDARY_FRONT_MM 174.0f
#define SULA_FRONT_WALL_TH 60	//境目でこれより大きければ前壁あり(境目で前壁は約140)

static float Front_Wall_mm(void) {
	return (WallDist_mm(WALLDIST_FL, g_sensor[0][0])
			+ WallDist_mm(WALLDIST_FR, g_sensor[3][0])) / 2;
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

	float X0 = X;
	int front_wall = (Wall_Flont_Av() > SULA_FRONT_WALL_TH);
	if (front_wall) {		//壁補正前進用
		X = X + 15;
	}

	Wall_search();
	Maze_Wall_Update();

	while (1) {
		if (G_Motor_X > X) {
			break;
		}

		if (front_wall && Front_Wall_mm() <= SULA_BOUNDARY_FRONT_MM - X0) {
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

	float X0 = X;
	int front_wall = (Wall_Flont_Av() > SULA_FRONT_WALL_TH);
	if (front_wall) {		//壁補正前進用
		X = X + 15;
	}

	while (1) {
		if (G_Motor_X > X) {
			break;
		}

		if (front_wall && Front_Wall_mm() <= SULA_BOUNDARY_FRONT_MM - X0) {
			break;
		}
	}

	PID_Mode = 0;
	G_Motor_Flag = 0;

}

void Motor_trapezoid_Turn(float Angle, float Wmax, float W_Ac) {
	int SANKAKU = 0;
	PID_Mode = 0;
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

	Gyro_Sigma_error = 0;
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

/* 壁切れの待ち方(G_WallCut_Pillar で従来版と柱版を切り替える)
 * Cut_Need: 待つかどうか。従来版は壁があるときだけ、柱版はいつも待つ
 * Cut_Begin: 待ち始め。warm = 1 ならターン直後なので PILLAR_WARM_MS は柱を見ない。
 *   max_x: 柱版で、柱が見つからないまま諦めるまでの距離。前距離では X の残り半分、後距離では Pillar_MaxX
 *   (柱をすでに過ぎていたとき、次の柱まで行ってしまわないように)
 * Cut_Hit: 切れ目が来たら 1(i: 1 右 / 2 左、cut: 従来版の閾値、th: 柱版の閾値) */
static float Cut_X0 = 0;
static float Cut_MaxX = 0;
static uint32_t Cut_T0 = 0;
static int Cut_Warm = 0;
static int Cut_Need(int wall) {
	return G_WallCut_Pillar ? 1 : wall;
}
static void Cut_Begin(int warm, float max_x) {
	Cut_X0 = G_Motor_X;
	Cut_MaxX = max_x;
	Cut_T0 = HAL_GetTick();
	Cut_Warm = warm;
}
static int Cut_Hit(int i, float cut, int th) {
	if (!G_WallCut_Pillar) {
		return g_sensor_av[i] < cut;
	}
	if (G_Motor_X - Cut_X0 > Cut_MaxX) {
		return 1;
	}
	if (Cut_Warm && (HAL_GetTick() - Cut_T0 < PILLAR_WARM_MS)) {
		return 0;
	}
	return Pillar_Edge(i, th);
}

void Motor_Wallcut_ST(float Vmax, float X, int direction) {		//壁の有無を変数に入れる
	G_Motor_Flag = 1;
	PID_Mode = 1;

	G_Motor_V_Target = Vmax;

	G_Motor_Ac = 0;
	float X_act = 0;

	Enc_Sigma_error = 0;
	//Gyro_Sigma_error = 0;

	G_Motor_X = 0;
	while (1) {
		if (G_Motor_X > X * 0.50) {
			X_act = G_Motor_X;
			break;
		}
	}
	Cut_Begin(0, X_act);
	if (direction == 0) { //左旋回
		if (Cut_Need(G_Wall_data[1])) {
			while (1) {
				if (Cut_Hit(2, Cut_L, Pillar_TH)) {
					break;
				}
				LED_ON_L();
			}
		}
	} else if (direction == 1) { //右旋回
		if (Cut_Need(G_Wall_data[2])) {
			while (1) {
				if (Cut_Hit(1, Cut_R, Pillar_TH)) {
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
	Cut_Begin(1, Pillar_MaxX);

	while (1) {
		if (G_Motor_X > X) {
			break;
		}
		if (Cut_Need(Wall_L) && Cut_Hit(2, Cut_L, Pillar_TH)) {
			break;
		}
		if (Cut_Need(Wall_R) && Cut_Hit(1, Cut_R, Pillar_TH)) {
			break;
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
		}
	} else if (direction == 1) { //右旋回
		while (1) {
			if (g_sensor_av[1] < Cut_R_NA) {
				break;
			}
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

	while (1) {
		if (G_Motor_X > X) {
			break;
		}
	}
	Wall_search();
	if (G_Wall_data[2] == 1 && direction == 0) {
		while (1) {
			if (g_sensor_av[1] < Cut_R_NA) {
				break;
			}
		}
	} else if (G_Wall_data[1] == 1 && direction == 1) {
		while (1) {
			if (g_sensor_av[2] < Cut_L_NA) {
				break;
			}
		}
	}

	PID_Mode = 0;
}

/* ---- 加速しながらの前距離・後距離(2026-10-07、最短走行の最初のターン用) ----
 * Motor_Wallcut_ST / Motor_Wallcut_END / Motor_Wallcut_END_NANAME と同じ動きを、
 * Vst から Vmax まで cos加速(Motor_trapezoid の加速と同じ形、加速度 Ac)しながら行う。
 * Vmax < Vst なら同じ形で減速する(2026-10-07、ターンごとの速度で後距離のうちに次のターンの速度へ変える用)。
 * 加減速は関数の最初からの経過時間で決める。終わったときの目標速度は G_Motor_V_Target に残る */
static void Accel_Target_Update(float Vst, float Vmax, float Ac) {
	float Df = Vmax - Vst;
	if (Df == 0 || Ac <= 0) {
		G_Motor_V_Target = Vmax;
		G_Motor_Ac = 0;
		return;
	}
	float Dabs = fabs(Df);
	float t1 = PI * Dabs / 2 / Ac;
	if (G_Motor_Count < t1) {
		G_Motor_V_Target = Df / 2 * (1 - cos(2 * Ac / Dabs * G_Motor_Count)) + Vst;
		G_Motor_Ac = (Df > 0 ? Ac : -Ac) * sin(2 * Ac / Dabs * G_Motor_Count);
	} else {
		G_Motor_V_Target = Vmax;
		G_Motor_Ac = 0;
	}
}

static void Accel_Start(float Vst, int pid_mode) {
	G_Motor_Flag = 1;
	PID_Mode = pid_mode;
	G_Motor_V_Target = Vst;
	G_Motor_Ac = 0;
	G_Motor_Angle = 0;
	G_Motor_W_Ac = 0;
	G_Motor_W_Target = 0;
	Enc_Sigma_error = 0;
	Gyro_Sigma_error = 0;
	G_Motor_Count = 0;
	G_Motor_X = 0;
}

/* Motor_Wallcut_ST の加速版。X_pre は前距離の前に足す距離(スタート位置から区画中心までの24mmなど) */
void Motor_Wallcut_ST_Accel(float Vst, float Vmax, float Ac, float X_pre,
		float X, int direction) {
	Accel_Start(Vst, 1);
	float X_act = 0;
	while (1) {
		Accel_Target_Update(Vst, Vmax, Ac);
		if (G_Motor_X > X_pre + X * 0.50) {
			X_act = G_Motor_X - X_pre;
			break;
		}
	}
	Cut_Begin(0, X_act);
	if (direction == 0) { //左旋回
		if (Cut_Need(G_Wall_data[1])) {
			while (1) {
				Accel_Target_Update(Vst, Vmax, Ac);
				if (Cut_Hit(2, Cut_L, Pillar_TH)) {
					break;
				}
				LED_ON_L();
			}
		}
	} else if (direction == 1) { //右旋回
		if (Cut_Need(G_Wall_data[2])) {
			while (1) {
				Accel_Target_Update(Vst, Vmax, Ac);
				if (Cut_Hit(1, Cut_R, Pillar_TH)) {
					break;
				}
				LED_ON_R();
			}
		}
	}
	LED_Reset();
	float X0 = G_Motor_X;
	while (1) {
		Accel_Target_Update(Vst, Vmax, Ac);
		if (G_Motor_X - X0 > X - X_act) {
			break;
		}
	}
	PID_Mode = 0;
}

/* Motor_Wallcut_END の加速版 */
void Motor_Wallcut_END_Accel(float Vst, float Vmax, float Ac, float X,
		int direction) {
	Accel_Start(Vst, 1);
	Wall_search();
	int Wall_L = G_Wall_data[1];
	int Wall_R = G_Wall_data[2];
	Cut_Begin(1, Pillar_MaxX);
	while (1) {
		Accel_Target_Update(Vst, Vmax, Ac);
		if (G_Motor_X > X) {
			break;
		}
		if (Cut_Need(Wall_L) && Cut_Hit(2, Cut_L, Pillar_TH)) {
			break;
		}
		if (Cut_Need(Wall_R) && Cut_Hit(1, Cut_R, Pillar_TH)) {
			break;
		}
	}
	PID_Mode = 0;
}

/* Motor_Wallcut_END_NANAME の加速版 */
void Motor_Wallcut_END_NANAME_Accel(float Vst, float Vmax, float Ac, float X,
		int direction) {
	Accel_Start(Vst, 2);
	while (1) {
		Accel_Target_Update(Vst, Vmax, Ac);
		if (G_Motor_X > X) {
			break;
		}
	}
	Wall_search();
	if (G_Wall_data[2] == 1 && direction == 0) {
		while (1) {
			Accel_Target_Update(Vst, Vmax, Ac);
			if (g_sensor_av[1] < Cut_R_NA) {
				break;
			}
		}
	} else if (G_Wall_data[1] == 1 && direction == 1) {
		while (1) {
			Accel_Target_Update(Vst, Vmax, Ac);
			if (g_sensor_av[2] < Cut_L_NA) {
				break;
			}
		}
	}
	PID_Mode = 0;
}

/* ---- 加速しながら曲がる最初のターン(2026-10-09) ----
 * スタートから後距離の終わりまで、目標速度を 0 から Vt まで1本の cos加速(加速度 Ac)で上げ続ける。
 * ターン中も加速するので、ターンは時間ではなく走った距離で角度を決める:
 *   基準の速度 v_ref で Motor_Sula_COS(v_ref, ang, w, w_ac) を走ったときの軌跡(曲率 = 角速度 / v_ref)を、
 *   ターンに入ってから走った距離の関数として使い、目標角速度 = 曲率 × 今の速度 にする。
 * こうすると速度に関係なく、v_ref で合わせたのと同じ軌跡を通る(G_First_Turn の前距離・角度・後距離をそのまま使える)。
 * 前距離・後距離の壁切れは Motor_Wallcut_ST_Accel / Motor_Wallcut_END_Accel / Motor_Wallcut_END_NANAME_Accel と同じ */

/* Motor_Sula_COS(v, A, W, Wac) の角速度の形(時間 t の関数)。台形にならないときは三角形 */
typedef struct {
	float W, Wac, t1, tc, T;
} SulaRef;

static void SulaRef_Init(SulaRef *r, float A, float W, float Wac) {
	float acc = PI * W * W / (4 * Wac);
	if (2 * acc > A) {
		W = sqrt(2 * Wac * A / PI);
		acc = A / 2;
	}
	r->W = W;
	r->Wac = Wac;
	r->t1 = PI * W / 2 / Wac;
	r->tc = (A - 2 * acc) / W;
	r->T = 2 * r->t1 + r->tc;
}

/* 時間 t での角速度 w [deg/s] と角加速度 wa [deg/s^2] */
static void SulaRef_At(const SulaRef *r, float t, float *w, float *wa) {
	float k = 2 * r->Wac / r->W;
	if (t < r->t1) {
		*w = r->W / 2 * (1 - cos(k * t));
		*wa = r->Wac * sin(k * t);
	} else if (t < r->t1 + r->tc) {
		*w = r->W;
		*wa = 0;
	} else if (t < r->T) {
		float u = t - r->t1 - r->tc;
		*w = r->W / 2 * (1 + cos(k * u));
		*wa = -r->Wac * sin(k * u);
	} else {
		*w = 0;
		*wa = 0;
	}
}

/* v_ref で Motor_Sula_COS(v_ref, A, W, Wac) を走ったときのターンの長さ[mm] */
float Sula_Ref_Length(float v_ref, float A, float W, float Wac) {
	SulaRef r;
	SulaRef_Init(&r, fabs(A), W, Wac);
	return v_ref * r.T;
}

static float FT_T0, FT_Vt, FT_Ac;

/* 目標速度: 0 から FT_Vt までの cos加速(Accel_Target_Update と同じ形)。時間は関数の最初から通しで数える */
static void FT_Target_Update(void) {
	float t = G_Motor_Count - FT_T0;
	float t1 = PI * FT_Vt / 2 / FT_Ac;
	if (t < t1) {
		G_Motor_V_Target = FT_Vt / 2 * (1 - cos(2 * FT_Ac / FT_Vt * t));
		G_Motor_Ac = FT_Ac * sin(2 * FT_Ac / FT_Vt * t);
	} else {
		G_Motor_V_Target = FT_Vt;
		G_Motor_Ac = 0;
	}
}

/* 最初のターンを、スタートから後距離の終わりまで加速しながら走る。
 * X_pre: 前距離の前に足す距離(スタート位置から区画中心まで)、pre/post: 前距離・後距離[mm]、
 * v_ref・Angle(左が正)・W・Wac: 軌跡を決める基準のターン、naname: 1 なら後距離は斜め、
 * Vt: 目標速度の上限、Ac: 加速度。戻り値は後距離の終わりの目標速度 */
float Motor_First_Turn_Accel(float X_pre, float pre, float post, float v_ref,
		float Angle, float W, float Wac, int naname, int direction, float Vt,
		float Ac) {
	SulaRef r;
	SulaRef_Init(&r, fabs(Angle), W, Wac);

	FT_Vt = Vt;
	FT_Ac = Ac;

	/* 前距離(Motor_Wallcut_ST_Accel と同じ) */
	Accel_Start(0, 1);
	FT_T0 = G_Motor_Count;
	float X_act = 0;
	while (1) {
		FT_Target_Update();
		if (G_Motor_X > X_pre + pre * 0.50) {
			X_act = G_Motor_X - X_pre;
			break;
		}
	}
	Cut_Begin(0, X_act);
	if (direction == 0) { //左旋回
		if (Cut_Need(G_Wall_data[1])) {
			while (1) {
				FT_Target_Update();
				if (Cut_Hit(2, Cut_L, Pillar_TH)) {
					break;
				}
				LED_ON_L();
			}
		}
	} else if (direction == 1) { //右旋回
		if (Cut_Need(G_Wall_data[2])) {
			while (1) {
				FT_Target_Update();
				if (Cut_Hit(1, Cut_R, Pillar_TH)) {
					break;
				}
				LED_ON_R();
			}
		}
	}
	LED_Reset();
	float X0 = G_Motor_X;
	while (1) {
		FT_Target_Update();
		if (G_Motor_X - X0 > pre - X_act) {
			break;
		}
	}

	/* ターン: 走った距離 s から基準の時間 s / v_ref を求め、その曲率で曲がる */
	PID_Mode = 0;
	Turn_Mode = (direction == 1) ? 1 : 0;
	Gyro_Sigma_error = 0;
	Enc_Sigma_error = 0;
	G_Motor_Angle = 0;
	G_Motor_X = 0;
	G_Motor_W_Target = 0;
	G_Motor_W_Ac = 0;
	G_Motor_Flag = 3;
	float L = v_ref * r.T;
	while (1) {
		FT_Target_Update();
		float s = G_Motor_X;
		if (s >= L) {
			break;
		}
		float w, wa;
		SulaRef_At(&r, s / v_ref, &w, &wa);
		float v = (G_Tire_Speed_L + G_Tire_Speed_R) / 2;
		if (v < 100) {
			v = 100;
		}
		float kappa = w / v_ref;					//曲率 [deg/mm]
		float dkappa = wa / (v_ref * v_ref);		//曲率の変化 [deg/mm^2]
		G_Motor_W_Target = kappa * v;
		G_Motor_W_Ac = dkappa * v * v + kappa * G_Motor_Ac;
	}
	G_Motor_W_Target = 0;
	G_Motor_W_Ac = 0;

	/* 後距離(Motor_Wallcut_END_Accel / Motor_Wallcut_END_NANAME_Accel と同じ) */
	G_Motor_Flag = 1;
	Enc_Sigma_error = 0;
	Gyro_Sigma_error = 0;
	G_Motor_Angle = 0;
	G_Motor_X = 0;
	if (naname) {
		PID_Mode = 2;
		while (1) {
			FT_Target_Update();
			if (G_Motor_X > post) {
				break;
			}
		}
		Wall_search();
		if (G_Wall_data[2] == 1 && direction == 0) {
			while (1) {
				FT_Target_Update();
				if (g_sensor_av[1] < Cut_R_NA) {
					break;
				}
			}
		} else if (G_Wall_data[1] == 1 && direction == 1) {
			while (1) {
				FT_Target_Update();
				if (g_sensor_av[2] < Cut_L_NA) {
					break;
				}
			}
		}
	} else {
		PID_Mode = 1;
		Wall_search();
		int Wall_L = G_Wall_data[1];
		int Wall_R = G_Wall_data[2];
		Cut_Begin(1, Pillar_MaxX);
		while (1) {
			FT_Target_Update();
			if (G_Motor_X > post) {
				break;
			}
			if (Cut_Need(Wall_L) && Cut_Hit(2, Cut_L, Pillar_TH)) {
				break;
			}
			if (Cut_Need(Wall_R) && Cut_Hit(1, Cut_R, Pillar_TH)) {
				break;
			}
		}
	}
	PID_Mode = 0;
	return G_Motor_V_Target;
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
	Gyro_Sigma_error = 0;
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

