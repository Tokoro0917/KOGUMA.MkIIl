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
 *  前(FL/FR)も実測値(2026-10-06)。全長120mm(中心からお尻まで60mm)として、
 *  お尻を後ろの壁につけ、壁の板を n 枚はさんで、前の壁までを 108 - 12n mm
 *  (n = 0〜4、60〜108mm)にして測った。60mmより近い点と108mmより遠い点は
 *  まだ測っていない(表の範囲外は端の値になる)。
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
	/* FL (実測) */
	{ 5,
	  { 60, 72, 84, 96, 108 },
	  { 1366, 902, 645, 470, 356 } },
	/* R (実測) */
	{ 9,
	  { 37, 49, 61, 73, 85, 97, 109, 121, 133 },
	  { 780, 522, 356, 262, 201, 160, 131, 110, 93 } },
	/* L (実測) */
	{ 9,
	  { 37, 49, 61, 73, 85, 97, 109, 121, 133 },
	  { 910, 603, 415, 308, 237, 183, 150, 125, 105 } },
	/* FR (実測) */
	{ 5,
	  { 60, 72, 84, 96, 108 },
	  { 1377, 940, 627, 461, 358 } },
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
