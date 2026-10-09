/*
 * Wallsensor.c
 *
 *  Created on: Jun 21, 2024
 *      Author: akihi
 */

#include "Wallsensor.h"
#include "PL_sensor.h"
#include "UI.h"
#include "gpio.h"
#include "motor.h"
#include "math.h"
#include "WallDistance.h"
#include <stdio.h>

int G_Wall_data[] = { 0, 0, 0, 0 };
int Wall_threshold[] = { 60, 90, 90, 60 }; //80, 150, 150, 80

float Kp = 0.05; //0.025
float Kd = 0.01;

/* 横壁制御を距離[mm]で行うか (0: 従来のセンサ値, 1: 距離)
 * 2026-10-07 から 1 が標準(直進テストのログで、センサ値版より角速度の振れが小さかった)。
 * センサ値版に戻すときは 0 にする(モード4 No.8 はセンサ値版で走る) */
int G_WallCtrl_Use_mm = 1;
/* 区画中心付近で従来のKp, Kdと同じ効きになる値から始める
 * (仮の表では84mm付近の傾きが約5.2[値/mm] -> 0.05*5.2, 0.01*5.2)。
 * 2026-10-09: 壁に寄ったときの戻りが弱い(センサ値版は寄るほど強く効いていた)ので Kp_mm を 0.26 -> 0.35 */
float Kp_mm = 0.35;
float Kd_mm = 0.07;	//2026-10-09 0.05 -> 0.07(横壁制御で左右に揺れながら走り、ターンに入るときの向きがずれるのを抑える)

float Kp_Na = 0.1;
float Kd_Na = 0.00;

float Kp_FW = 0.1;
float Kd_FW = 0.0;

float Kp_FW_dire = 0.1;
float Kd_FW_dire = 0.0;

float KP, KD;

/* 区画中心(壁まで84mm)での横センサの値。2026-10-06の実測の表(WallDistance.c)から
 * L=243、R=206(以前は220/200で、片側の壁だけのときは左壁から約89mm/右壁から
 * 約85mm、両側の壁では中心から右へ約1.7mmずれた位置に合わせていた) */
int Wall_L = 243; //220 //370
int Wall_R = 206; //200 //250

int Wall_TH_L = 80;
int Wall_TH_R = 80;

/* 前壁合わせの目標(前センサFL/FRの平均)。区画中心(前の壁まで84mm)で
 * FL=645、FR=627(2026-10-06実測)の平均。以前の530は前の壁から約92mmで、
 * 区画中心より約8mm手前で止まっていた */
float FlontWall_Distance = 636; //530

int Sensor_diff_TH = 50;

/* 距離版(G_WallCtrl_Use_mm = 1)の横壁制御だけで使う値(2026-10-07、ログから決めた)。
 * 遠い壁や壁の切れ目では mm の誤差が大きくなり、センサ値版の約2倍のキックが出ていた */
float WallCtrl_mm_MaxDist = 104;	//これより遠い壁は使わない(区画中心から±20mm)
float WallCtrl_mm_ErrMax = 20;	//片側の誤差の頭打ち[mm]
int Sensor_diff_TH_mm = 25;	//壁の切れ目の判定(1msの変化)。3000mm/sでは切れ目で20〜40しか変わらない

float Wall_error = 0;
float Wall_old_error = 0;
float Wall_Delta_error = 0;

float Wall_error_NA = 0;
float Wall_old_error_NA = 0;
float Wall_Delta_error_NA = 0;

float Wall_error_FW = 0;
float Wall_old_error_FW = 0;
float Wall_Delta_error_FW = 0;

void Wall_search() {
	for (int i = 0; i < 4; i++) {
		G_Wall_data[i] = 0;
	}

	if (g_sensor_av10[0] > Wall_threshold[0]) { //正面左//g_sensor_av
		G_Wall_data[0] = 1;
	}
	if (g_sensor_av10[2] > Wall_threshold[2]) { //左
		G_Wall_data[1] = 1;
	}

	if (g_sensor_av10[1] > Wall_threshold[1]) { //右
		G_Wall_data[2] = 1;
	}
	if (g_sensor_av10[3] > Wall_threshold[3]) { //正面右
		G_Wall_data[3] = 1;
	}

}

int Sensor_Enter() {
	int i = 0;
	if (g_sensor_av[0] > 1400) {
		i = 1;
	}
	return i;
}

int Sensor_Start() {
	while (1) {
		LED_StartWait();
		if ((g_sensor_av[1] + (g_sensor_av[2])) > 2000) {
			break;
		}
	}
	Buzzer_Start();

}

float Wall_Flont_Av() {
	float av = 0;
	av = (g_sensor[0][0] + g_sensor[3][0]) / 2;
	return av;
}

void Wall_search_LED() {
	if (G_Wall_data[0] == 1) {
		HAL_GPIO_WritePin(LEDF1_GPIO_Port, LEDF1_Pin, 1);
	} else {
		HAL_GPIO_WritePin(LEDF1_GPIO_Port, LEDF1_Pin, 0);
	}
	if (G_Wall_data[1] == 1) {
		HAL_GPIO_WritePin(LEDF3_GPIO_Port, LEDF3_Pin, 1);
	} else {
		HAL_GPIO_WritePin(LEDF3_GPIO_Port, LEDF3_Pin, 0);
	}
	if (G_Wall_data[2] == 1) {
		HAL_GPIO_WritePin(LEDF2_GPIO_Port, LEDF2_Pin, 1);
	} else {
		HAL_GPIO_WritePin(LEDF2_GPIO_Port, LEDF2_Pin, 0);
	}
	if (G_Wall_data[3] == 1) {
		HAL_GPIO_WritePin(LEDF4_GPIO_Port, LEDF4_Pin, 1);
	} else {
		HAL_GPIO_WritePin(LEDF4_GPIO_Port, LEDF4_Pin, 0);
	}

}

/* 距離キャリブレーション(モード3-4)
 * 機体を壁から既知の距離に置き、表示された値を WallDistance.c の表に書き写す。
 * 約0.2秒ごとに100回平均したセンサ値と、現在の表で変換した距離[mm]をCSVで出す。
 * 手で機体を動かすとモード選択が変わるが、このループの中にいる間は影響しない。
 * 抜けるときはリセット。 */
void Wall_Distance_Calibration() {
	printf("CAL,FL,L,R,FR,FL_mm,L_mm,R_mm,FR_mm\n\r");
	while (1) {
		long sum[4] = { 0, 0, 0, 0 };
		for (int k = 0; k < 100; k++) {
			for (int i = 0; i < 4; i++) {
				sum[i] += g_sensor[i][0];
			}
			HAL_Delay(2);
		}
		int v[4];
		for (int i = 0; i < 4; i++) {
			v[i] = sum[i] / 100;
		}
		printf("CAL,%d,%d,%d,%d,%.1f,%.1f,%.1f,%.1f\n\r", v[0], v[2], v[1],
				v[3], WallDist_mm(WALLDIST_FL, v[0]),
				WallDist_mm(WALLDIST_L, v[2]), WallDist_mm(WALLDIST_R, v[1]),
				WallDist_mm(WALLDIST_FR, v[3]));
	}
}

/* 会場ごとの感度合わせ(モード3 No.5)
 * 機体を区画中心に置き、前・左・右に壁がある状態で測る
 * (スタート区画なら、後ろの壁のほうを向けて置く)。
 * 約1秒ごとに500回平均した値と、手で書き写す行をシリアルに出す。抜けるときはリセット。
 * 倍率 = 測った値 / 表の区画中心(84mm)の値。閾値は今の値に倍率を掛けたもの
 * (書き写す前の値、つまり倍率1のときの値が基準。2回目以降も元の値に掛ける) */
void Wall_Center_Calibration() {
	/* 倍率1のときの閾値(書き換えたら、ここも基準として残す) */
	const int th_base[4] = { 60, 90, 90, 60 };	//Wall_threshold
	const float align_base = 200;	//Move.c ALIGN_FRONT_MIN
	const float cut_base = 80;	//motor.c Cut_L / Cut_R
	const float cut_na_base = 150;	//motor.c Cut_L_NA / Cut_R_NA

	while (1) {
		long sum[4] = { 0, 0, 0, 0 };
		for (int k = 0; k < 500; k++) {
			for (int i = 0; i < 4; i++) {
				sum[i] += g_sensor[i][0];
			}
			HAL_Delay(2);
		}
		float v[4], ref[4], k[4];
		for (int i = 0; i < 4; i++) {
			v[i] = sum[i] / 500.0f;
			ref[i] = WallDist_Value(i, WALLDIST_CENTER_MM);
			k[i] = v[i] / ref[i];
		}
		float kf = (k[0] + k[3]) / 2;
		printf("\n\r---- CENTER  FL=%.0f L=%.0f R=%.0f FR=%.0f  (table FL=%.0f L=%.0f R=%.0f FR=%.0f)\n\r",
				v[0], v[2], v[1], v[3], ref[0], ref[2], ref[1], ref[3]);
		printf("WallDistance.c: float WallDist_Scale[4] = { %.2ff, %.2ff, %.2ff, %.2ff };\n\r",
				k[0], k[1], k[2], k[3]);
		printf("Wallsensor.c:   int Wall_L = %.0f;  int Wall_R = %.0f;  float FlontWall_Distance = %.0f;\n\r",
				v[2], v[1], (v[0] + v[3]) / 2);
		printf("Wallsensor.c:   int Wall_threshold[] = { %.0f, %.0f, %.0f, %.0f };\n\r",
				th_base[0] * k[0], th_base[1] * k[1], th_base[2] * k[2],
				th_base[3] * k[3]);
		printf("Move.c:         #define ALIGN_FRONT_MIN %.0f\n\r", align_base * kf);
		printf("motor.c:        float Cut_R = %.0f;  float Cut_L = %.0f;  float Cut_R_NA = %.0f;  float Cut_L_NA = %.0f;\n\r",
				cut_base * k[1], cut_base * k[2], cut_na_base * k[1],
				cut_na_base * k[2]);
	}
}

/* 横壁制御の出力。Wall_Control_Update() が1msに1回計算し、
 * calWallConrol() はその値を返す */
static float G_Wall_PID = 0;

/* 直進中の横壁制御の値を返す(1msの中で何回呼んでも同じ値)。
 * 計算は Motor_Speed_PID_ST() の中の Wall_Control_Update() で1回だけ行う */
float calWallConrol() {
	return G_Wall_PID;
}

/* 横壁制御を計算する。1msに1回だけ呼ぶこと。
 * 以前は calWallConrol() の中で計算していて、1msに3回(左PWM・右PWM・ジャイロ目標)
 * 呼ばれていたため、2回目以降は Wall_old_error が更新済みでD項が0になり、
 * D項が左のPWMにしか効いていなかった */
float Wall_Control_Update() {

	int Sensor_diff_L = abs(g_sensor[2][0] - g_sensor[2][1]);
	int Sensor_diff_R = abs(g_sensor[1][0] - g_sensor[1][1]);
	float PID_Wall = 0;
	int Wall_st = 0;
	if (G_WallCtrl_Use_mm) {
		/* 距離版: 近い壁(WallCtrl_mm_MaxDist以内)で、切れ目でないものだけ使う */
		float dL = WallDist_mm(WALLDIST_L, g_sensor[2][0]);
		float dR = WallDist_mm(WALLDIST_R, g_sensor[1][0]);
		int use_L = (dL <= WallCtrl_mm_MaxDist) && (Sensor_diff_L < Sensor_diff_TH_mm);
		int use_R = (dR <= WallCtrl_mm_MaxDist) && (Sensor_diff_R < Sensor_diff_TH_mm);
		Wall_st = (use_L ? 1 : 0) + (use_R ? 2 : 0);
	} else if ((g_sensor_av[2] > Wall_TH_L) && (Sensor_diff_L < Sensor_diff_TH)) { //左あり
		if ((g_sensor_av[1] > Wall_TH_R) && (Sensor_diff_R < Sensor_diff_TH)) { //右あり
			Wall_st = 3;
		} else { //右なし
			Wall_st = 1;
		}
	} else {
		if ((g_sensor_av[1] > Wall_TH_R) && (Sensor_diff_R < Sensor_diff_TH)) {
			Wall_st = 2;
		} else {
			Wall_st = 0;
		}
	}

	float Kp_base = G_WallCtrl_Use_mm ? Kp_mm : Kp;
	float Kd_base = G_WallCtrl_Use_mm ? Kd_mm : Kd;
	if (G_Motor_V_Target >= 2000) {
		KP = Kp_base * (2000 / 500);
		KD = Kd_base * (2000 / 500);
	} else {
		KP = Kp_base * (G_Motor_V_Target / 500);
		KD = Kd_base * (G_Motor_V_Target / 500);
	}

	int Sensor_L = g_sensor[2][0];
	int Sensor_R = g_sensor[1][0];

	if (Sensor_L > 600) {
		Sensor_L = 600;
	}
	if (Sensor_R > 600) {
		Sensor_R = 600;
	}

	/* 距離モードでは「壁に近いほど正」になるよう (中心距離 - 実距離) を使う。
	 * 近づいたときの値の急増は変換で吸収されるので600での頭打ちは不要 */
	float Err_L, Err_R;
	if (G_WallCtrl_Use_mm) {
		Err_L = WALLDIST_CENTER_MM - WallDist_mm(WALLDIST_L, g_sensor[2][0]);
		Err_R = WALLDIST_CENTER_MM - WallDist_mm(WALLDIST_R, g_sensor[1][0]);
		if (Err_L > WallCtrl_mm_ErrMax) Err_L = WallCtrl_mm_ErrMax;
		if (Err_L < -WallCtrl_mm_ErrMax) Err_L = -WallCtrl_mm_ErrMax;
		if (Err_R > WallCtrl_mm_ErrMax) Err_R = WallCtrl_mm_ErrMax;
		if (Err_R < -WallCtrl_mm_ErrMax) Err_R = -WallCtrl_mm_ErrMax;
	} else {
		Err_L = Sensor_L - Wall_L;
		Err_R = Sensor_R - Wall_R;
	}

	LED_Reset();
	if (Wall_Flont_Av() > 800) {
		Wall_error = 0;
		Wall_old_error = 0;
	} else if (Wall_st == 3) {
		Wall_error = Err_L - Err_R;
		LED_ON_L();
		LED_ON_R();
	} else if (Wall_st == 2) {
		Wall_error = 2.0 * -Err_R;
		LED_ON_R();
	} else if (Wall_st == 1) {
		Wall_error = 2.0 * Err_L;
		LED_ON_L();
	} else {
		Wall_error = 0;
		Wall_old_error = 0;
	}

	/* 距離版: 壁の有無が切り替わった瞬間は誤差が段差で変わるので、D項を出さない */
	static int Wall_st_prev = 0;
	if (G_WallCtrl_Use_mm && Wall_st != Wall_st_prev) {
		Wall_old_error = Wall_error;
	}
	Wall_st_prev = Wall_st;

	Wall_Delta_error = Wall_error - Wall_old_error;
	Wall_old_error = Wall_error;

	PID_Wall = KP * Wall_error + KD * Wall_Delta_error;

	G_Wall_PID = PID_Wall;
	return PID_Wall;
}

float calWallConrol_NANAME() {
	float PID_Wall = 0;

	KP = Kp_Na;
	KD = Kd_Na;

	KP = Kp_Na * (G_Motor_V_Target / 1000);
	KD = Kd_Na * (G_Motor_V_Target / 1000);
	LED_Reset();
	if (g_sensor[0][0] > 110) {
		Wall_error_NA = g_sensor[0][0] - 110;
		LED_ON_L();
	} else if (g_sensor[3][0] > 110) {
		Wall_error_NA = -(g_sensor[3][0] - 110);
		LED_ON_R();
	} else {
		Wall_error_NA = 0;
		Wall_old_error_NA=0;
	}

	Wall_Delta_error_NA = Wall_error_NA - Wall_old_error_NA;
	Wall_old_error_NA = Wall_error_NA;
	PID_Wall = KP * Wall_error_NA + KD * Wall_Delta_error_NA;

	return PID_Wall;
}

float calWallConrol_Flontwall_ST() {
	float PID_Wall = 0;

	if (Wall_Flont_Av() > 1000) {
		Wall_error_FW = FlontWall_Distance - 1000;
	} else {
		Wall_error_FW = FlontWall_Distance - Wall_Flont_Av();
	}

	Wall_Delta_error_FW = Wall_error_FW - Wall_old_error_FW;
	Wall_old_error_FW = Wall_error_FW;
	PID_Wall = Kp_FW * Wall_error_FW + Kd_FW * Wall_Delta_error_FW;

	return PID_Wall;
}

float calWallConrol_Flontwall_Turn() {
	float PID_Wall = 0;

	Wall_error_FW = (g_sensor[0][0]) - (g_sensor[3][0]);

	Wall_Delta_error_FW = Wall_error_FW - Wall_old_error_FW;
	Wall_old_error_FW = Wall_error_FW;
	PID_Wall = Kp_FW_dire * Wall_error_FW + Kd_FW_dire * Wall_Delta_error_FW;

	return PID_Wall;
}
