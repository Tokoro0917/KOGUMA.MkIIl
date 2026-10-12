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
#include "DistHist.h"
#include "PL_encoder.h"
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
float Kp_mm = 0.30;	//2026-10-10 0.35 -> 0.30(センサ値版の Kp=0.05 と区画中心付近で同じ効き。中心付近は約6[値/mm])
float Kd_mm = 0.06;	//2026-10-10 0.07 -> 0.06(センサ値版の Kd=0.01 と同じ効き)。2026-10-09 0.05 -> 0.07(横壁制御で左右に揺れながら走り、ターンに入るときの向きがずれるのを抑える)

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
float WallCtrl_mm_MaxDist = 120;	//これより遠い壁は使わない(区画中心から約±36mm)。104では片寄ったときに使う/使わないが切り替わり続けて中心に戻れなかった
float WallCtrl_mm_ErrMax = 20;	//片側の誤差の頭打ち[mm]
/* 壁の切れ目の判定(2026-10-10)。壁が見え始め・消えかけのときはセンサ値が20mmほどかけて
 * 変わり、その途中では壁を実際より遠いと読んで誤差が±20に張り付いていた。
 * 1msの変化で見ると速度で効き方が変わり、1000mm/sでは一度も効かなかったので、
 * WallCtrl_mm_StableLen 進む間の距離の変化が WallCtrl_mm_StableTH を超えた壁は使わない */
float WallCtrl_mm_StableLen = 10;	//何mm前の距離と比べるか
float WallCtrl_mm_StableTH = 15;	//この間に距離がこれ以上変わった壁は使わない[mm]。2026-10-10 8 -> 15(左センサのノイズで壁を使う/使わないが数msおきに切り替わっていた。ノイズが直れば8に戻す)
float WallCtrl_mm_GainMin = 0;	//距離版のゲイン倍率(速度/500)の下限。減速中に効かなくなって片寄っていたので0.5にしたが、2026-10-10 センサ値版と比べるため0(下限なし)

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

/* 横センサの直近3回の中央値(2026-10-10、距離版の横壁制御用)。
 * 左センサだけに4msおきの1回だけの落ち込みがあり、そのたびに「壁が遠い」と読んで
 * 誤差が -20〜-33 に飛んでいたので、1回だけの飛びを消す。
 * g_sensor[i][1] は更新直後のずらしで [0] と同じ値なので、[0](今回)・[2](1ms前)・[3](2ms前)を使う */
static int Median3(int a, int b, int c) {
	if (a > b) { int t = a; a = b; b = t; }
	if (b > c) { b = c; }
	return (a > b) ? a : b;
}

static int Wall_Sensor_Median3(int i) {
	return Median3(g_sensor[i][0], g_sensor[i][2], g_sensor[i][3]);
}

/* 3回の中央値(壁切れを遅れの少ない値で見るとき用。motor.c の G_WallCut_Fast) */
int Wall_Sensor_Med3(int i) {
	return Wall_Sensor_Median3(i);
}

/* ---- 柱の切れ目の判定(壁切れを柱で見るとき用。motor.c の G_WallCut_Pillar) ----
 * 「Pillar_Len 進む前と今とで、センサ値(3回の中央値)が th 以上減ったら」柱の後ろの端とみなす。
 * 壁の終わりも、壁のない所の柱も、柱の後ろの端でセンサ値が下がるので同じ位置で見つかる。
 *
 * G_Pillar_DistHist = 1(2026-10-12、既定): センサ値を走った距離ごと(DistHist.h、1mm間隔)に残し、
 *   ちょうど Pillar_Len 前の値と比べる。速度に関係なく同じ距離で比べるので、閾値の効き方が速度で変わらない。
 *   距離はエンコーダから求めた実際の走行距離(G_Odo_X)を使う。
 * G_Pillar_DistHist = 0(2026-10-10 の版): Pillar_Len を目標速度から何ms前かに直し、1〜6msに丸めて比べる。
 *   丸めと上限のため、実際に比べる距離は 500で3mm、1000で5mm、2000で6mm、2400で4.8mm と速度で変わっていた。
 *
 * Pillar_Len(5mm)はクラシックの値。センサ値が柱で何mmかけて下がるかの1/3〜1/2を目安にする
 * (モード3 No.6〜9 のログで確かめる)。ハーフでは柱・壁までの距離がおおよそ半分なので、2.5mm から測り直す */
int G_Pillar_DistHist = 1;
float Pillar_Len = 5;	//何mm前のセンサ値と比べるか

/* 距離ごとの履歴と、1msごとに求めた「Pillar_Len 進む間の減り方」。0: 右(g_sensor[1])、1: 左(g_sensor[2]) */
static DistHist Edge_Hist[2];
static volatile float Edge_Drop[2] = { 0, 0 };
static volatile float Edge_PastX[2] = { -1e9f, -1e9f };	//比べた過去の点の距離(G_Odo_X)

/* 壁切れの位置のログ(モード3 No.6〜9)。ログを取っているあいだ、3つの方法それぞれで反応した位置を残す */
#define EDGE_LOG_N 48
#define EDGE_KIND_AV 0		//従来: g_sensor_av < Cut_*
#define EDGE_KIND_MED 1		//遅れの少ない値: 3回の中央値 < Cut_*
#define EDGE_KIND_PILLAR 2	//柱: Pillar_Len 進む間に Pillar_TH 以上減った
#define EDGE_REARM 20		//閾値の判定は、いちど Cut_* + これ を超えてから次を数える
typedef struct {
	float x;	//ログ開始からの距離[mm]
	float v;	//そのときの速度[mm/s]
	signed char side;	//0: 右、1: 左
	signed char kind;
} EdgeEvent;
static EdgeEvent Edge_Log[EDGE_LOG_N];
static volatile int Edge_Log_Count = 0;
static volatile int Edge_Log_On = 0;
static float Edge_Log_X0 = 0;
static int Edge_Arm[2][3];

static void Edge_Log_Add(int side, int kind, float x) {
	if (Edge_Log_Count >= EDGE_LOG_N) return;
	EdgeEvent *e = &Edge_Log[Edge_Log_Count];
	e->x = x - Edge_Log_X0;
	e->v = (G_Tire_Speed_L + G_Tire_Speed_R) / 2;
	e->side = side;
	e->kind = kind;
	Edge_Log_Count++;
}

/* 1msに1回、割り込みで呼ぶ(G_Odo_X とセンサ値を更新したあと) */
void Wall_Edge_Update() {
	for (int s = 0; s < 2; s++) {
		int i = (s == 0) ? 1 : 2;
		float now = Wall_Sensor_Median3(i);
		DistHist_Push(&Edge_Hist[s], G_Odo_X, now);
		float x = DistHist_LastX(&Edge_Hist[s]);
		float past;
		if (DistHist_At(&Edge_Hist[s], x - Pillar_Len, &past)) {
			Edge_Drop[s] = past - now;
			Edge_PastX[s] = x - Pillar_Len;
		} else {
			Edge_Drop[s] = 0;
			Edge_PastX[s] = -1e9f;
		}

		if (Edge_Log_On) {
			float cut = (s == 0) ? Cut_R : Cut_L;
			float lv[2] = { g_sensor_av[i], now };
			for (int k = 0; k < 2; k++) {
				if (lv[k] > cut + EDGE_REARM) {
					Edge_Arm[s][k] = 1;
				} else if (Edge_Arm[s][k] && lv[k] < cut) {
					Edge_Log_Add(s, k, G_Odo_X);
					Edge_Arm[s][k] = 0;
				}
			}
			if (Edge_Drop[s] < Pillar_TH / 2) {
				Edge_Arm[s][EDGE_KIND_PILLAR] = 1;
			} else if (Edge_Arm[s][EDGE_KIND_PILLAR] && Edge_Drop[s] >= Pillar_TH) {
				Edge_Log_Add(s, EDGE_KIND_PILLAR, G_Odo_X);
				Edge_Arm[s][EDGE_KIND_PILLAR] = 0;
			}
		}
	}
}

/* i: 1 右 / 2 左。x_from: 比べる過去の点がこの距離(G_Odo_X)より前なら見ない(ターン直後用。距離ごとの判定のときだけ) */
int Pillar_Edge(int i, int th, float x_from) {
	if (G_Pillar_DistHist) {
		int s = (i == 1) ? 0 : 1;
		return (Edge_PastX[s] >= x_from) && (Edge_Drop[s] >= th);
	}
	/* 以前の版(時間ごと)。g_sensor[i][] は1msごとに更新され、[1] は [0] と同じ値なので、kms前は [k+1] */
	int k = 6;
	if (G_Motor_V_Target > 0) {
		k = (int) (Pillar_Len * 1000 / G_Motor_V_Target + 0.5f);
	}
	if (k < 1) k = 1;
	if (k > 6) k = 6;	//[k+3] が履歴(10個)に収まる範囲
	int now = Wall_Sensor_Median3(i);
	int past = Median3(g_sensor[i][k + 1], g_sensor[i][k + 2], g_sensor[i][k + 3]);
	return (past - now) >= th;
}

void Wall_Edge_Log_Start() {
	Edge_Log_On = 0;
	Edge_Log_Count = 0;
	Edge_Log_X0 = G_Odo_X;
	for (int s = 0; s < 2; s++) {
		for (int k = 0; k < 3; k++) {
			Edge_Arm[s][k] = 0;
		}
	}
	Edge_Log_On = 1;
}

void Wall_Edge_Log_Stop() {
	Edge_Log_On = 0;
}

/* first_boundary: ログ開始の位置から最初の区画の境目までの距離[mm]。境目は first_boundary + 180k */
void Wall_Edge_Log_Print(float first_boundary) {
	static const char *kind_name[3] = { "av ", "med", "pil" };
	printf("\n\r---- EDGE  Cut_L=%.0f Cut_R=%.0f Pillar_TH=%d Pillar_Len=%.1f DistHist=%d Fast=%d  (border = %.0f + 180k)\n\r",
			Cut_L, Cut_R, Pillar_TH, Pillar_Len, G_Pillar_DistHist, G_WallCut_Fast, first_boundary);
	printf("side,kind,x,v,border,x-border\n\r");
	for (int n = 0; n < Edge_Log_Count; n++) {
		const EdgeEvent *e = &Edge_Log[n];
		int k = (int) floorf((e->x - first_boundary) / 180.0f + 0.5f);
		float border = first_boundary + 180.0f * k;
		printf("%s,%s,%.1f,%.0f,%.0f,%.1f\n\r", e->side ? "L" : "R", kind_name[(int) e->kind],
				e->x, e->v, border, e->x - border);
	}
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
	int Sensor_L_med = Wall_Sensor_Median3(2);
	int Sensor_R_med = Wall_Sensor_Median3(1);
	/* 距離版の切れ目判定用に、横壁までの距離を1msごとに残す。
	 * 上の Sensor_diff_L/R は g_sensor[i][1] がずらし後に [0] と同じ値なので常に0で、
	 * 切れ目判定が一度も効いていなかった(センサ値版はそのまま) */
	#define WALL_HIST_N 16
	static float hist_L[WALL_HIST_N], hist_R[WALL_HIST_N];
	static int hist_i = 0;
	float dL = WallDist_mm(WALLDIST_L, Sensor_L_med);
	float dR = WallDist_mm(WALLDIST_R, Sensor_R_med);
	hist_i = (hist_i + 1) % WALL_HIST_N;
	hist_L[hist_i] = dL;
	hist_R[hist_i] = dR;
	if (G_WallCtrl_Use_mm) {
		/* 距離版: 近い壁(WallCtrl_mm_MaxDist以内)で、WallCtrl_mm_StableLen 進む間に
		 * 距離があまり変わっていないものだけ使う。比べる相手は速度から何ms前かを決める */
		int back = WALL_HIST_N - 1;
		if (G_Motor_V_Target > 0) {
			back = (int) (WallCtrl_mm_StableLen * 1000 / G_Motor_V_Target + 0.5f);
		}
		if (back < 2) back = 2;
		if (back > WALL_HIST_N - 1) back = WALL_HIST_N - 1;
		int j = (hist_i - back + WALL_HIST_N) % WALL_HIST_N;
		int use_L = (dL <= WallCtrl_mm_MaxDist) && (fabsf(dL - hist_L[j]) < WallCtrl_mm_StableTH);
		int use_R = (dR <= WallCtrl_mm_MaxDist) && (fabsf(dR - hist_R[j]) < WallCtrl_mm_StableTH);
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
		float gain = G_Motor_V_Target / 500;
		if (G_WallCtrl_Use_mm && (G_Motor_V_Target > 0) && (gain < WallCtrl_mm_GainMin)) {
			gain = WallCtrl_mm_GainMin;
		}
		KP = Kp_base * gain;
		KD = Kd_base * gain;
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
		Err_L = WALLDIST_CENTER_MM - dL;
		Err_R = WALLDIST_CENTER_MM - dR;
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
