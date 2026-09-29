/* ターンごとの通過速度の速度計画(SpeedPlan_Make)のPC上での検証。
 *
 * ビルドと実行 (リポジトリ直下で):
 *   gcc -std=c11 -Wall -ICore/Inc -o /tmp/tsp test/test_speedplan.c Core/Src/SpeedPlan.c -lm && /tmp/tsp
 */
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include "SpeedPlan.h"

static int fail_count = 0;
#define CHECK(cond, ...)                                                      \
	do {                                                                      \
		if (!(cond)) {                                                        \
			fail_count++;                                                     \
			printf("  FAIL: ");                                               \
			printf(__VA_ARGS__);                                              \
			printf("\n");                                                     \
		}                                                                     \
	} while (0)

#define NEAR(a, b) (fabsf((a) - (b)) < 0.5f)

/* テスト用のターン速度表: コード -> 速度 */
static float tv[128];
static float turn_v(int16_t code) {
	return tv[-code];
}
static void tv_all(float v) {
	for (int i = 0; i < 128; i++) {
		tv[i] = v;
	}
}

static SpeedPlan_t plan;

static void setup(float v_max, float ac) {
	plan.turn_v = turn_v;
	plan.v_max = v_max;
	plan.ac = ac;
	plan.v_start = 2000;
	plan.v_goal = 2000;
}

/* motor.cの加減速で飛ばずに走れるか(速度差が直線の長さに収まるか)と、
 * 速度が上限を超えていないか、連続ターンの速度がそろっているかを確かめる */
static void check_feasible(const char *name, const int16_t *pass) {
	float prev = plan.v_start_out;	/* 直前の節点の速度 */
	CHECK(prev <= plan.v_start + 1e-3f && prev <= plan.v_max + 1e-3f,
			"%s: スタート速度 %.1f が上限超え", name, prev);
	int i;
	for (i = 0; pass[i] != 0; i++) {
		int k = SpeedPlan_Kind(pass[i]);
		if (k == SP_SKIP) {
			continue;
		}
		float vi = plan.v_in[i], vo = plan.v_out[i];
		CHECK(NEAR(vi, prev), "%s: 要素%d の始点速度 %.1f が直前 %.1f と不一致",
				name, i, vi, prev);
		if (k == SP_TURN) {
			CHECK(vi == vo, "%s: ターン%d の入口と出口の速度が違う", name, i);
			CHECK(vi <= turn_v(pass[i]) + 1e-3f && vi <= plan.v_max + 1e-3f,
					"%s: ターン%d の速度 %.1f が上限超え", name, i, vi);
		} else {
			float L, acc, dec;
			if (k == SP_STRAIGHT) {
				L = 90.0f * pass[i];
				acc = plan.ac;
				dec = plan.ac * 2;
			} else {
				L = 127.3f * (pass[i] / -50);
				acc = dec = plan.ac - 5000;
			}
			float need = (vo > vi) ? (vo * vo - vi * vi) / (4 * acc) * 3.14159265f
					: (vi * vi - vo * vo) / (4 * dec) * 3.14159265f;
			CHECK(need <= L + 1e-2f, "%s: 直線%d (%.0fmm) で %.0f -> %.0f に加減速しきれない(必要%.1fmm)",
					name, i, L, vi, vo, need);
		}
		prev = vo;
	}
	CHECK(NEAR(plan.v_goal_in, prev), "%s: ゴール速度 %.1f が直前 %.1f と不一致",
			name, plan.v_goal_in, prev);
	CHECK(plan.v_goal_in <= plan.v_goal + 1e-3f, "%s: ゴール速度が上限超え", name);
}

int main(void) {
	/* 1. 全ターン2000なら Short_NANAME_Move2000 と同じ速度になる */
	{
		int16_t pass[] = { -1, 4, -4, 6, -5, 3, -51, -100, -61, 2, -6, -52, -65, -62, 5, 0 };
		setup(6000, 20000);
		tv_all(2000);
		SpeedPlan_Make(pass, &plan);
		check_feasible("全2000", pass);
		for (int i = 0; pass[i] != 0; i++) {
			if (SpeedPlan_Kind(pass[i]) != SP_SKIP) {
				CHECK(plan.v_in[i] == 2000 && plan.v_out[i] == 2000,
						"全2000: 要素%d が %.1f -> %.1f", i, plan.v_in[i], plan.v_out[i]);
			}
		}
	}

	/* 2. 長い直線を挟むと、ターンごとの速度がそのまま使われる */
	{
		int16_t pass[] = { 8, -5, 8, -4, 8, 0 };
		setup(6000, 20000);
		tv_all(2000);
		tv[5] = 2600;	/* 大回り180 */
		tv[4] = 1500;	/* 大回り90 */
		SpeedPlan_Make(pass, &plan);
		check_feasible("長い直線", pass);
		CHECK(plan.v_in[1] == 2600, "長い直線: 大回り180 = %.1f", plan.v_in[1]);
		CHECK(plan.v_in[3] == 1500, "長い直線: 大回り90 = %.1f", plan.v_in[3]);
		CHECK(plan.v_in[2] == 2600 && plan.v_out[2] == 1500,
				"長い直線: 直線 %.1f -> %.1f", plan.v_in[2], plan.v_out[2]);
	}

	/* 3. ターンが直接つながる(間が-1だけ)と、遅い方にそろう */
	{
		int16_t pass[] = { 6, -52, -1, -65, -1, -62, 6, 0 };
		setup(6000, 20000);
		tv_all(2000);
		tv[52] = 1800;
		tv[65] = 2500;
		tv[62] = 1600;
		SpeedPlan_Make(pass, &plan);
		check_feasible("連続ターン", pass);
		CHECK(plan.v_in[1] == 1600 && plan.v_in[3] == 1600 && plan.v_in[5] == 1600,
				"連続ターン: %.1f %.1f %.1f", plan.v_in[1], plan.v_in[3], plan.v_in[5]);
	}

	/* 4. 短い直線では加減速しきれる速度まで下がる */
	{
		int16_t pass[] = { 4, -4, 1, -5, 1, -4, 4, 0 };
		setup(6000, 20000);
		tv_all(2000);
		tv[4] = 1200;
		tv[5] = 3000;
		SpeedPlan_Make(pass, &plan);
		check_feasible("短い直線", pass);
		CHECK(plan.v_in[3] < 3000 && plan.v_in[3] > 1200,
				"短い直線: 大回り180 = %.1f (1200〜3000の間のはず)", plan.v_in[3]);
	}

	/* 5. ターン速度はMAXを超えない */
	{
		int16_t pass[] = { 4, -5, 4, 0 };
		setup(2000, 20000);
		tv_all(2600);
		SpeedPlan_Make(pass, &plan);
		check_feasible("MAX", pass);
		CHECK(plan.v_in[1] == 2000, "MAX: 大回り180 = %.1f", plan.v_in[1]);
	}

	/* 6. スタート直後・ゴール直前のターンは v_start / v_goal を超えない */
	{
		int16_t pass[] = { -1, -4, 3, -6, 0 };
		setup(6000, 20000);
		tv_all(2600);
		SpeedPlan_Make(pass, &plan);
		check_feasible("端のターン", pass);
		CHECK(plan.v_in[1] == 2000 && plan.v_in[3] == 2000,
				"端のターン: %.1f %.1f", plan.v_in[1], plan.v_in[3]);
	}

	/* 7. ランダムなパスで、常に走れる速度になっている */
	{
		const int16_t turns[] = { -4, -5, -6, -7, -51, -52, -53, -54, -61, -62, -63, -64, -65, -66 };
		srand(1);
		for (int t = 0; t < 2000; t++) {
			int16_t pass[64];
			int n = 1 + rand() % 60;
			for (int i = 0; i < n; i++) {
				int r = rand() % 10;
				if (r < 4) {
					pass[i] = 1 + rand() % 10;
				} else if (r < 6) {
					pass[i] = -50 * (1 + rand() % 6);
				} else if (r < 7) {
					pass[i] = -1;
				} else {
					pass[i] = turns[rand() % 14];
				}
			}
			pass[n] = 0;
			for (int i = 0; i < 128; i++) {
				tv[i] = 1000 + rand() % 2500;
			}
			setup(3000 + rand() % 4000, 10000 + rand() % 25000);
			SpeedPlan_Make(pass, &plan);
			char name[32];
			snprintf(name, sizeof name, "ランダム#%d", t);
			check_feasible(name, pass);
			if (fail_count > 20) {
				break;
			}
		}
	}

	if (fail_count == 0) {
		printf("OK\n");
		return 0;
	}
	printf("%d件の失敗\n", fail_count);
	return 1;
}
