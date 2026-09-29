/*
 * WallDistance.c
 *
 *  壁センサ値 -> 距離[mm] 変換 (表引き + 直線補間)
 *
 *  !!! 下の表は仮の値 !!!
 *  既存の制御定数(Wall_L=220, Wall_R=200, FlontWall_Distance=530 を
 *  区画中心 84mm での値とみなす)から、センサ値が距離の2乗に反比例すると
 *  仮定して作っただけ。モード3-4(距離キャリブレーション)で実測した値に
 *  置き換えること。
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
	/* R */
	{ 9,
	  { 44, 54, 64, 74, 84, 94, 104, 114, 124 },
	  { 729, 484, 345, 258, 200, 160, 130, 109, 92 } },
	/* L */
	{ 9,
	  { 44, 54, 64, 74, 84, 94, 104, 114, 124 },
	  { 802, 532, 379, 283, 220, 176, 144, 119, 101 } },
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
