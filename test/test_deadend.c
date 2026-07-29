/* 行き止まり潰し(Maze_DeadEnd_Fill)のPC上での検証。
 * Core/Src/Maze.c をそのままリンクして、実装が満たすべき不変条件を確認する。
 *
 * ビルドと実行 (リポジトリ直下で):
 *   gcc -std=c11 -Wall -ICore/Inc -o /tmp/t test/test_deadend.c Core/Src/Maze.c && /tmp/t
 *   gcc -std=c11 -Wall -DMAZE_SIZE=32 -ICore/Inc -o /tmp/t32 test/test_deadend.c Core/Src/Maze.c && /tmp/t32
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "Maze.h"
#include "Define.h"

/* Maze.c が参照する外部シンボルのスタブ */
int G_Wall_data[4];
extern int g_deadend_closed_count;	//Maze.c のデバッグ用カウンタ

#define N MAZE_SIZE

static int true_open_n[N][N]; /* 北へ抜けられるか(検証側の真の迷路) */
static int true_open_e[N][N]; /* 東へ抜けられるか */

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

static void walls_all_set(void) {
	for (int i = 0; i < N + 1; i++) {
		G_Maze_Row[i] = 0xFFFFFFFFu;
		G_Maze_Column[i] = 0xFFFFFFFFu;
	}
	memset(true_open_n, 0, sizeof(true_open_n));
	memset(true_open_e, 0, sizeof(true_open_e));
}

/* (x,y)の北の壁を壊す */
static void carve_n(int x, int y) {
	G_Maze_Row[y + 1] &= ~(1u << x);
	true_open_n[x][y] = 1;
}

/* (x,y)の東の壁を壊す */
static void carve_e(int x, int y) {
	G_Maze_Column[x + 1] &= ~(1u << y);
	true_open_e[x][y] = 1;
}

static int true_passable(int x, int y, int d) {
	switch (d) {
	case 0:
		return (y + 1 < N) && true_open_n[x][y];
	case 1:
		return (x + 1 < N) && true_open_e[x][y];
	case 2:
		return (y - 1 >= 0) && true_open_n[x][y - 1];
	default:
		return (x - 1 >= 0) && true_open_e[x - 1][y];
	}
}

static const int DX[4] = { 0, 1, 0, -1 };
static const int DY[4] = { 1, 0, -1, 0 };

/* skip_closed=1 なら閉鎖済みマスを通行不可として最短距離を求める */
static int shortest(int sx, int sy, int gx, int gy, int skip_closed) {
	static int dist[N][N];
	static int qx[N * N], qy[N * N];
	int head = 0, tail = 0;
	for (int x = 0; x < N; x++)
		for (int y = 0; y < N; y++)
			dist[x][y] = -1;
	dist[sx][sy] = 0;
	qx[tail] = sx;
	qy[tail] = sy;
	tail++;
	while (head < tail) {
		int cx = qx[head], cy = qy[head];
		head++;
		if (cx == gx && cy == gy)
			return dist[cx][cy];
		for (int d = 0; d < 4; d++) {
			if (!true_passable(cx, cy, d))
				continue;
			int nx = cx + DX[d], ny = cy + DY[d];
			if (dist[nx][ny] >= 0)
				continue;
			if (skip_closed && G_MAZE_Closed[nx][ny])
				continue;
			dist[nx][ny] = dist[cx][cy] + 1;
			qx[tail] = nx;
			qy[tail] = ny;
			tail++;
		}
	}
	return -1;
}

/* 深さ優先で全域木(完全迷路)を作る。行き止まりだらけになるので連鎖の検証に向く */
static void gen_perfect_maze(unsigned seed) {
	static int visited[N][N];
	static int stx[N * N], sty[N * N];
	int sp = 0;
	srand(seed);
	walls_all_set();
	memset(visited, 0, sizeof(visited));
	visited[0][0] = 1;
	stx[sp] = 0;
	sty[sp] = 0;
	sp++;
	while (sp > 0) {
		int cx = stx[sp - 1], cy = sty[sp - 1];
		int cand[4], nc = 0;
		for (int d = 0; d < 4; d++) {
			int nx = cx + DX[d], ny = cy + DY[d];
			if (nx < 0 || nx >= N || ny < 0 || ny >= N)
				continue;
			if (visited[nx][ny])
				continue;
			cand[nc++] = d;
		}
		if (nc == 0) {
			sp--;
			continue;
		}
		int d = cand[rand() % nc];
		int nx = cx + DX[d], ny = cy + DY[d];
		if (d == 0)
			carve_n(cx, cy);
		else if (d == 1)
			carve_e(cx, cy);
		else if (d == 2)
			carve_n(cx, cy - 1);
		else
			carve_e(cx - 1, cy);
		visited[nx][ny] = 1;
		stx[sp] = nx;
		sty[sp] = ny;
		sp++;
	}
}

/* 完全迷路に余分な通路をランダムに開けてループを作る(現実の競技迷路に近い) */
static void add_loops(int count) {
	for (int k = 0; k < count; k++) {
		int x = rand() % N, y = rand() % N;
		if (rand() % 2) {
			if (y + 1 < N)
				carve_n(x, y);
		} else {
			if (x + 1 < N)
				carve_e(x, y);
		}
	}
}

/* 既知の壁を真の迷路の部分集合にする(探索途中の状態を模擬)。
 * ratio%の壁だけ「未確認」として落とす */
static void forget_walls(int ratio) {
	for (int y = 0; y < N; y++) {
		for (int x = 0; x < N; x++) {
			if ((y > 0) && (y < N) && (rand() % 100 < ratio))
				G_Maze_Row[y] &= ~(1u << x);
			if ((x > 0) && (x < N) && (rand() % 100 < ratio))
				G_Maze_Column[x] &= ~(1u << y);
		}
	}
}

static void print_maze(void) {
	for (int y = N - 1; y >= 0; y--) {
		for (int x = 0; x < N; x++) {
			printf("+");
			printf((G_Maze_Row[y + 1] & (1u << x)) ? "---" : "   ");
		}
		printf("+\n");
		for (int x = 0; x < N; x++) {
			printf((G_Maze_Column[x] & (1u << y)) ? "|" : " ");
			if (x == 0 && y == 0)
				printf(" S ");
			else if (x == G_Gool_X && y == G_Gool_Y)
				printf(" G ");
			else
				printf(G_MAZE_Closed[x][y] ? " # " : "   ");
		}
		printf("%s\n", (G_Maze_Column[N] & (1u << y)) ? "|" : " ");
	}
	for (int x = 0; x < N; x++) {
		printf("+");
		printf((G_Maze_Row[0] & (1u << x)) ? "---" : "   ");
	}
	printf("+\n");
}

int main(void) {
	printf("=== 1. 完全迷路: 潰した後に残るのはスタート-ゴール間の唯一の経路だけか ===\n");
	for (unsigned seed = 1; seed <= 200; seed++) {
		gen_perfect_maze(seed);
		G_Gool_X = MAZE_GOOL_X;
		G_Gool_Y = MAZE_GOOL_Y;
		int before = shortest(0, 0, G_Gool_X, G_Gool_Y, 0);
		Maze_DeadEnd_Fill();
		int after = shortest(0, 0, G_Gool_X, G_Gool_Y, 1);
		CHECK(before == after,
				"seed=%u 最短距離が変わった before=%d after=%d", seed, before,
				after);
		/* 木なのでスタート-ゴール経路は一意。残る開マスはちょうどその経路 */
		int open_cells = 0;
		for (int x = 0; x < N; x++)
			for (int y = 0; y < N; y++)
				if (!G_MAZE_Closed[x][y])
					open_cells++;
		CHECK(open_cells == before + 1,
				"seed=%u 経路長+1=%d に対し残った開マス=%d", seed, before + 1,
				open_cells);
		CHECK(G_MAZE_Closed[0][0] == 0, "seed=%u スタートが閉じられた", seed);
		CHECK(G_MAZE_Closed[G_Gool_X][G_Gool_Y] == 0,
				"seed=%u ゴールが閉じられた", seed);
	}
	printf("  %s\n", fail_count == 0 ? "OK (200 seeds)" : "NG");

	int base = fail_count;
	printf("=== 2. ループあり迷路: 最短距離が変わらないか ===\n");
	for (unsigned seed = 1; seed <= 300; seed++) {
		gen_perfect_maze(seed);
		add_loops(60);
		G_Gool_X = MAZE_GOOL_X;
		G_Gool_Y = MAZE_GOOL_Y;
		int before = shortest(0, 0, G_Gool_X, G_Gool_Y, 0);
		Maze_DeadEnd_Fill();
		int after = shortest(0, 0, G_Gool_X, G_Gool_Y, 1);
		CHECK(before == after,
				"seed=%u 最短距離が変わった before=%d after=%d", seed, before,
				after);
	}
	printf("  %s\n", fail_count == base ? "OK (300 seeds)" : "NG");

	base = fail_count;
	printf("=== 3. 探索途中(壁が部分的にしか分かっていない)でも安全か ===\n");
	for (unsigned seed = 1; seed <= 300; seed++) {
		gen_perfect_maze(seed);
		add_loops(60);
		G_Gool_X = MAZE_GOOL_X;
		G_Gool_Y = MAZE_GOOL_Y;
		int before = shortest(0, 0, G_Gool_X, G_Gool_Y, 0);
		forget_walls(40);	//既知の壁を減らす=探索途中の状態
		Maze_DeadEnd_Fill();
		int after = shortest(0, 0, G_Gool_X, G_Gool_Y, 1);
		CHECK(before == after,
				"seed=%u 最短距離が変わった before=%d after=%d", seed, before,
				after);
	}
	printf("  %s\n", fail_count == base ? "OK (300 seeds)" : "NG");

	base = fail_count;
	printf("=== 4. 初期状態(外周だけ既知)で誤爆しないか ===\n");
	G_Gool_X = MAZE_GOOL_X;
	G_Gool_Y = MAZE_GOOL_Y;
	G_Robot_MAZE_X = 0;
	G_Robot_MAZE_Y = 0;
	Maze_Initialization();
	Maze_DeadEnd_Fill();
	CHECK(g_deadend_closed_count == 0, "初期状態で%dマス閉じてしまった",
			g_deadend_closed_count);
	printf("  %s\n", fail_count == base ? "OK" : "NG");

	base = fail_count;
	printf("=== 5. 無効化スイッチ ===\n");
	gen_perfect_maze(7);
	G_Gool_X = MAZE_GOOL_X;
	G_Gool_Y = MAZE_GOOL_Y;
	G_DeadEnd_Fill_Enable = 0;
	Maze_DeadEnd_Fill();
	CHECK(g_deadend_closed_count == 0, "無効化しても%dマス閉じた",
			g_deadend_closed_count);
	G_DeadEnd_Fill_Enable = 1;
	Maze_DeadEnd_Fill();
	CHECK(g_deadend_closed_count > 0, "有効化しても1マスも閉じない");
	printf("  %s\n", fail_count == base ? "OK" : "NG");

	printf("\n=== 見本 (seed=3, ループあり, # = 潰した行き止まり) ===\n");
	gen_perfect_maze(3);
	add_loops(60);
	G_Gool_X = MAZE_GOOL_X;
	G_Gool_Y = MAZE_GOOL_Y;
	Maze_DeadEnd_Fill();
	print_maze();
	printf("閉じたマス: %d / %d\n", g_deadend_closed_count, N * N);

	printf("\n%s\n", fail_count == 0 ? "ALL PASS" : "FAILURES PRESENT");
	return fail_count != 0;
}
