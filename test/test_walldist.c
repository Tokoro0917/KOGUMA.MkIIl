/* 壁センサ値 -> 距離変換(WallDist_mm)のPC上での検証。
 * 表を実測値に書き換えたあとも、このテストで表の書き間違いを検出できる。
 *
 * ビルドと実行 (リポジトリ直下で):
 *   gcc -std=c11 -Wall -ICore/Inc -o /tmp/tw test/test_walldist.c Core/Src/WallDistance.c && /tmp/tw
 */
#include <stdio.h>
#include "WallDistance.h"

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

static const char *name[4] = { "FL", "R", "L", "FR" };

int main(void) {
	for (int s = 0; s < 4; s++) {
		/* センサ値が増えるほど距離は縮む(増えない)。表の並び間違いはここで引っかかる */
		float prev = WallDist_mm(s, 0);
		for (int v = 1; v <= 4095; v++) {
			float d = WallDist_mm(s, v);
			CHECK(d <= prev + 1e-4f, "%s: v=%d で距離が増えた (%.2f -> %.2f)",
					name[s], v, prev, d);
			prev = d;
		}
		/* 区画中心(84mm)が表の範囲内に入っていること */
		CHECK(WallDist_mm(s, 4095) < WALLDIST_CENTER_MM
				&& WallDist_mm(s, 0) > WALLDIST_CENTER_MM,
				"%s: 表の範囲が 84mm を含んでいない", name[s]);
	}

	/* 仮の表の値そのものと、その中点での補間 */
	CHECK(WallDist_mm(WALLDIST_L, 220) == 84.0f, "L: 220 -> 84mm");
	CHECK(WallDist_mm(WALLDIST_R, 200) == 84.0f, "R: 200 -> 84mm");
	CHECK(WallDist_mm(WALLDIST_FL, 530) == 84.0f, "FL: 530 -> 84mm");
	float mid = WallDist_mm(WALLDIST_L, (220 + 176) / 2);
	CHECK(mid > 88.9f && mid < 89.1f, "L: 198 -> 89mm (got %.2f)", mid);

	if (fail_count == 0) {
		printf("OK\n");
		return 0;
	}
	printf("%d failures\n", fail_count);
	return 1;
}
