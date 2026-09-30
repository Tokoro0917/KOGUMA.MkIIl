/*
 * SpeedPlan.c
 *
 * ターンごとに通過速度が違うときの速度計画。
 *
 * パスを「節点(速度が決まる点)」と「辺(直線区間)」の列として見る。
 *   節点: スタート区間の終わり / 各ターン / ゴール停止区間の入口
 *   辺  : 節点の間の直線。ターンどうしが直接つながるときは長さ0の辺
 * 各節点の速度を上限(ターンの希望速度など)から出発して、
 * 「直線の長さで加減速しきれる」範囲まで下げる。
 * 前向き(加速側)と後ろ向き(減速側)に1回ずつ制限をかけて、小さい方を取る。
 * 長さ0の辺では前後の速度が等しくなる(=連続ターンは遅い方にそろう)。
 *
 * 加減速距離の式はmotor.cの台形(cos)加速に合わせてある:
 *   速度 v1 -> v2 に必要な距離 = |v2^2 - v1^2| / (4*Ac) * PI
 * これを超える速度差を渡すと、Motor_trapezoid系は目標速度を段差で飛ばしてしまう。
 */

#include <math.h>
#include "SpeedPlan.h"

#define SP_PI 3.14159265f
#define SP_MARGIN 0.9f	/* 直線の長さのうち加減速に使ってよい割合 */

#define SP_MAX_NODE (SPEEDPLAN_MAX_ELEM + 2)

int SpeedPlan_Kind(int16_t code) {
	/* Move.cの走行ループと同じ判定順 */
	if (code > 0) {
		return SP_STRAIGHT;
	}
	if (code <= -4 && code > -50) {
		return (code >= -7) ? SP_TURN : SP_SKIP;
	}
	if (code <= -50) {
		if ((code <= -51 && code >= -54) || (code <= -61 && code >= -66)) {
			return SP_TURN;
		}
		if (code % 50 == 0) {
			return SP_DIAGONAL;
		}
	}
	return SP_SKIP;
}

/* 距離Lで v から加(減)速して到達できる最大の速度 */
static float reach(float v, float a, float L) {
	return sqrtf(v * v + 4.0f * a * L * SP_MARGIN / SP_PI);
}

static float minf(float a, float b) {
	return (a < b) ? a : b;
}

static float ub[SP_MAX_NODE];		/* 節点の速度の上限 */
static float fw[SP_MAX_NODE];
static float bw[SP_MAX_NODE];
static float e_len[SP_MAX_NODE];	/* 辺k: 節点k -> k+1 */
static float e_acc[SP_MAX_NODE];
static float e_dec[SP_MAX_NODE];
static int node_of[SPEEDPLAN_MAX_ELEM];	/* ターン要素 -> 節点番号 */
static int from_of[SPEEDPLAN_MAX_ELEM];	/* 直線要素 -> 始点の節点番号 */

/* 辺kに直線codeの長さと加減速度を入れる。code=0は長さ0(ターンの直結) */
static void set_edge(int k, int16_t code, float ac) {
	if (code > 0) {		/* Motor_trapezoid_Asymmetric_PID: 減速は加速の2倍 */
		e_len[k] = 90.0f * code;
		e_acc[k] = ac;
		e_dec[k] = ac * 2.0f;
	} else if (code < 0) {	/* 斜め直線: Motor_NANAME_PIDをAC-5000で走る */
		float a = ac - 5000.0f;
		if (a < 1.0f) {
			a = 1.0f;
		}
		e_len[k] = 127.3f * (code / -50);
		e_acc[k] = a;
		e_dec[k] = a;
	} else {
		e_len[k] = 0.0f;
		e_acc[k] = 1.0f;
		e_dec[k] = 1.0f;
	}
}

int SpeedPlan_Make(const int16_t *pass, SpeedPlan_t *plan) {
	int n = 0;			/* 節点数 */
	int pending = -1;	/* まだ終点が決まっていない直線要素 */
	int i;

	ub[n++] = minf(plan->v_start, plan->v_max);

	for (i = 0; pass[i] != 0; i++) {
		if (i >= SPEEDPLAN_MAX_ELEM) {
			return -1;
		}
		int kind = SpeedPlan_Kind(pass[i]);
		if (kind == SP_SKIP) {
			continue;
		}
		if (kind == SP_TURN || pending >= 0) {
			/* 新しい節点を作り、直前の節点との間の辺を張る。
			 * 直線が2本続いたときは、その間に上限v_maxの節点を置く */
			set_edge(n - 1, (pending >= 0) ? pass[pending] : 0, plan->ac);
			if (pending >= 0) {
				from_of[pending] = n - 1;
				pending = -1;
			}
			ub[n] = plan->v_max;
			n++;
		}
		if (kind == SP_TURN) {
			float v = plan->turn_v(pass[i]);
			ub[n - 1] = minf(v, plan->v_max);
			node_of[i] = n - 1;
		} else {
			pending = i;
		}
	}
	int len = i;

	/* ゴール停止区間の入口 */
	set_edge(n - 1, (pending >= 0) ? pass[pending] : 0, plan->ac);
	if (pending >= 0) {
		from_of[pending] = n - 1;
	}
	ub[n++] = minf(plan->v_goal, plan->v_max);

	/* 前向き: 加速しきれる速度まで下げる */
	fw[0] = ub[0];
	for (int k = 0; k + 1 < n; k++) {
		fw[k + 1] = minf(ub[k + 1], reach(fw[k], e_acc[k], e_len[k]));
	}
	/* 後ろ向き: 減速しきれる速度まで下げる */
	bw[n - 1] = ub[n - 1];
	for (int k = n - 2; k >= 0; k--) {
		bw[k] = minf(ub[k], reach(bw[k + 1], e_dec[k], e_len[k]));
	}
	for (int k = 0; k < n; k++) {
		fw[k] = minf(fw[k], bw[k]);
	}

	/* 要素ごとの速度に戻す */
	for (i = 0; i < len; i++) {
		int kind = SpeedPlan_Kind(pass[i]);
		if (kind == SP_TURN) {
			plan->v_in[i] = fw[node_of[i]];
			plan->v_out[i] = fw[node_of[i]];
		} else if (kind == SP_SKIP) {
			plan->v_in[i] = 0.0f;
			plan->v_out[i] = 0.0f;
		} else {
			plan->v_in[i] = fw[from_of[i]];
			plan->v_out[i] = fw[from_of[i] + 1];
		}
	}
	plan->v_start_out = fw[0];
	plan->v_goal_in = fw[n - 1];
	return 0;
}
