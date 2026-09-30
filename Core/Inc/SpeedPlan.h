/*
 * SpeedPlan.h
 *
 * 最短走行の速度計画。ターンごとに通過速度を変えたときに、
 * 直線区間の始点・終点速度と、ターンの通過速度を決める。
 * HALに依存しないので、PC上でテストできる(test/test_speedplan.c)。
 */

#ifndef INC_SPEEDPLAN_H_
#define INC_SPEEDPLAN_H_

#include <stdint.h>

/* G_Short_Pass_NANAME[] の1要素の種類 */
#define SP_SKIP      0	/* -1などの穴埋め。走行ループでも何もしない */
#define SP_STRAIGHT  1	/* 区間前進(1以上、半区画=90mm単位) */
#define SP_DIAGONAL  2	/* 斜め直線(-50の倍数、127.3mm単位) */
#define SP_TURN      3	/* 大回り・斜め入り/出・V90 */

int SpeedPlan_Kind(int16_t code);

#define SPEEDPLAN_MAX_ELEM 256	/* 圧縮後のパスの要素数の上限 */

typedef struct {
	/* 入力 */
	float (*turn_v)(int16_t code);	/* ターンの希望通過速度[mm/s] */
	float v_max;		/* 直線の最高速度(= MAX)。ターン速度もこれを超えない */
	float ac;			/* 直線の加速度(= AC) */
	float v_start;		/* スタート区間の終わりの速度の上限 */
	float v_goal;		/* ゴール停止区間に入る速度の上限 */
	/* 出力(要素ごと。ターンは v_in == v_out) */
	float v_in[SPEEDPLAN_MAX_ELEM];
	float v_out[SPEEDPLAN_MAX_ELEM];
	float v_start_out;	/* スタート区間の終わりの速度 */
	float v_goal_in;	/* ゴール停止区間に入る速度 */
} SpeedPlan_t;

/* pass[] を 0 終端まで見て速度を決める。要素数が上限を超えたら -1 を返す */
int SpeedPlan_Make(const int16_t *pass, SpeedPlan_t *plan);

#endif /* INC_SPEEDPLAN_H_ */
