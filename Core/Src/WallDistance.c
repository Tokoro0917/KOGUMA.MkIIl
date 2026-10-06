/*
 * WallDistance.c
 *
 *  壁センサ値 -> 距離[mm] 変換 (表引き + 直線補間)
 *
 *  横(L/R)は実測値(2026-10-06、モード3 No.4)。区画の内のり168mm(区画18cm、
 *  壁の厚さ1.2cm)、機体の幅70mm(中心線から側面まで35mm)として、機体の側面を
 *  反対側の壁につけ、壁の板(12mm)を n 枚はさんで、測る側の壁までを
 *  133 - 12n mm(n = 0〜8)にして測った。
 *
 *  !!! 前(FL/FR)はまだ仮の値 !!!
 *  既存の制御定数(FlontWall_Distance=530 を区画中心 84mm での値とみなす)から、
 *  センサ値が距離の2乗に反比例すると仮定して作っただけ。実測値に置き換えること。
 *
 *  表の書き方:
 *    - 距離は小さい順(近い順)に並べる
 *    - センサ値は距離が増えるほど小さくなること(単調減少)
 *    - 点の数はセンサごとに変えてよい(最大 WALLDIST_TABLE_MAX)
 */

#include "WallDistance.h"

#define WALLDIST_TABLE_MAX 12

typedef struct {
	int n;
	float mm[WALLDIST_TABLE_MAX];
	int v[WALLDIST_TABLE_MAX];
} WallDistTable;

static const WallDistTable wall_dist_table[4] = {
	/* FL */
	{ 10,
	  { 34, 44, 54, 64, 74, 84, 104, 124, 144, 174 },
	  { 3235, 1932, 1282, 913, 683, 530, 346, 243, 180, 124 } },
	/* R (実測) */
	{ 9,
	  { 37, 49, 61, 73, 85, 97, 109, 121, 133 },
	  { 780, 522, 356, 262, 201, 160, 131, 110, 93 } },
	/* L (実測) */
	{ 9,
	  { 37, 49, 61, 73, 85, 97, 109, 121, 133 },
	  { 910, 603, 415, 308, 237, 183, 150, 125, 105 } },
	/* FR */
	{ 10,
	  { 34, 44, 54, 64, 74, 84, 104, 124, 144, 174 },
	  { 3235, 1932, 1282, 913, 683, 530, 346, 243, 180, 124 } },
};

float WallDist_mm(int sensor, int v) {
	const WallDistTable *t = &wall_dist_table[sensor];

	if (v >= t->v[0]) {
		return t->mm[0];
	}
	for (int i = 0; i < t->n - 1; i++) {
		if (v >= t->v[i + 1]) {
			float r = (float) (v - t->v[i + 1]) / (float) (t->v[i] - t->v[i + 1]);
			return t->mm[i + 1] + r * (t->mm[i] - t->mm[i + 1]);
		}
	}
	return t->mm[t->n - 1];
}
