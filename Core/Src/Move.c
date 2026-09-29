/*
 * Move.c
 *
 *  Created on: 2023/08/29
 *      Author: akihi
 */

/*
 * Move.c
 *
 *  Created on: 2022/08/20
 *      Author: akihi
 */

#include "motor.h"
#include "Define.h"
#include "Move.h"
#include "Maze.h"
#include "Wallsensor.h"
#include "stdio.h"
#include "Failsafe.h"
#include "UI.h"
#include"Failsafe.h"
#include "SpeedPlan.h"

int G_Pass_before;
int G_Pass_after;

int Pass_NM = 0;

void Robot_adjustment() {
	Motor_Stop();
	if (G_Wall_data[0] == 1 && G_Wall_data[3] == 1) {
		Motor_Robot_Alignment();
		Motor_Stop();
	}
	Motor_trapezoid_Turn(90, 600, 20000);
	Motor_Stop();
	if (G_Wall_data[0] == 1 && G_Wall_data[3] == 1) {
		Motor_Robot_Alignment();
		Motor_Stop();
	}
	Motor_trapezoid_Turn(90, 600, 20000);
	Motor_Stop();
}

void Robot_adjustment_180() {
	Suction_change(20);
	if (G_Wall_data[0] == 1 && G_Wall_data[3] == 1) {
		Motor_Robot_Alignment();
		Motor_Stop();
	}
	Motor_trapezoid_Turn(180, 2000, 50000);
	Motor_Stop();
	Suction_change(35);
}

void Robot_adjustment_Back() {
	Motor_trapezoid_Turn(90, 400, 25000);
	Motor_Stop();
	//Motor_Back();
	Motor_Stop();
	Motor_trapezoid(0, 200, 0, 5000, 19);
	Motor_Stop();
	Motor_trapezoid_Turn(90, 400, 25000);
	Motor_Stop();
	//Motor_Back();
	Motor_Stop();
	Motor_trapezoid_PID(0, 300, 300, 5000, 13);
}

void Robot_Maze_Sula_Action() {
	Motor_Sula_before(500, 500, 500, 5000, 20);
	if ((G_Maze_Flont <= G_Maze_Left) && (G_Maze_Flont <= G_Maze_Right)	//前進
			&& (G_Maze_Flont <= G_Maze_Back)) {
		Motor_trapezoid_PID(500, 500, 500, 5000, 160);
		G_Robot_Direction += 0;
		G_Robot_Lastaction = 0;

	} else if ((G_Maze_Left <= G_Maze_Right)	//左折
	&& (G_Maze_Left <= G_Maze_Back)) {
		Motor_Sula_COS(500, 90, 700, 10000);
		Motor_trapezoid_PID(500, 500, 500, 5000, 25);
		G_Robot_Direction += 3;
		G_Robot_Lastaction = 3;
	} else if (G_Maze_Right <= G_Maze_Back) {	//	右折
		Motor_Sula_COS(500, -90, 700, 10000);
		Motor_trapezoid_PID(500, 500, 500, 5000, 25);
		G_Robot_Direction += 1;
		G_Robot_Lastaction = 1;
	} else if ((G_Maze_Flont == MAX_STEP) && (G_Maze_Left == MAX_STEP)
			&& (G_Maze_Right == MAX_STEP)) {	//Uターン　全部壁あり
		if ((G_Robot_MAZE_X == 0) && (G_Robot_MAZE_Y == 0)) {	//初期位置に戻ってきたとき
			Motor_trapezoid(500, 500, 0, 5000, 70);
			Motor_Stop();
			G_Robot_Direction += 2;
			G_Robot_Lastaction = 2;
			Robot_adjustment();
			Motor_Back();
			Motor_Stop();
		} else {
			Motor_trapezoid(500, 500, 0, 5000, 70);
			Motor_Stop();
			G_Robot_Direction += 2;
			G_Robot_Lastaction = 2;
			Robot_adjustment();
			Motor_trapezoid_PID(0, 500, 500, 5000, 90);
		}
	} else {	//Uターン　一部壁無し
		Motor_trapezoid(500, 500, 0, 5000, 70);
		Motor_Stop();
		G_Robot_Direction += 2;
		G_Robot_Lastaction = 2;
		Robot_adjustment();
		Motor_trapezoid_PID(0, 500, 500, 5000, 90);
	}

}

void Robot_Maze_Suction_Action() {
	Motor_Sula_before(1000, 1000, 1000, 5000, 15);
	if ((G_Maze_Flont <= G_Maze_Left) && (G_Maze_Flont <= G_Maze_Right)	//前進
			&& (G_Maze_Flont <= G_Maze_Back)) {
		Motor_trapezoid_PID(1000, 1000, 1000, 5000, 165);
		G_Robot_Direction += 0;
		G_Robot_Lastaction = 0;

	} else if ((G_Maze_Left <= G_Maze_Right)	//左折
	&& (G_Maze_Left <= G_Maze_Back)) {
		Motor_Sula_COS(1000, 90, 1640, 60000);
		Motor_trapezoid_PID(1000, 1000, 1000, 5000, 50);
		G_Robot_Direction += 3;
		G_Robot_Lastaction = 3;
	} else if (G_Maze_Right <= G_Maze_Back) {	//	右折
		Motor_Sula_COS(1000, -90, 1640, 60000);
		Motor_trapezoid_PID(1000, 1000, 1000, 5000, 50);
		G_Robot_Direction += 1;
		G_Robot_Lastaction = 1;
	} else if ((G_Maze_Flont == MAX_STEP) && (G_Maze_Left == MAX_STEP)
			&& (G_Maze_Right == MAX_STEP)) {	//Uターン　全部壁あり
		if ((G_Robot_MAZE_X == 0) && (G_Robot_MAZE_Y == 0)) {	//初期位置に戻ってきたとき
			Motor_trapezoid(1000, 1000, 0, 10000, 75);
			Motor_Stop();
			Suction_change(20);
			G_Robot_Direction += 2;
			G_Robot_Lastaction = 2;
			G_Just_UTurned = 1;
			Robot_adjustment();
			Motor_Back();
			Motor_Stop();
			Motor_trapezoid_PID(0, 300, 0, 10000, 45);
			Motor_Stop();
			Motor_Back();
			Motor_Stop();
		} else {
			Motor_trapezoid(1000, 1000, 0, 10000, 75);
			Motor_Stop();
			G_Robot_Direction += 2;
			G_Robot_Lastaction = 2;
			G_Just_UTurned = 1;
			Robot_adjustment();
			Motor_trapezoid_PID(0, 1000, 1000, 10000, 90);
		}
	} else {	//Uターン　一部壁無し
		Motor_trapezoid(1000, 1000, 0, 10000, 75);
		Motor_Stop();
		G_Robot_Direction += 2;
		G_Robot_Lastaction = 2;
		G_Just_UTurned = 1;
		Robot_adjustment();
		Motor_trapezoid_PID(0, 1000, 1000, 10000, 90);
	}

}

void Robot_Maze_Pass_Action() {
	int Known_X = G_Robot_MAZE_X;
	int Known_Y = G_Robot_MAZE_Y;
	if (G_Robot_Direction % 4 == 0) {
		Known_X += 0;
		Known_Y += 1;
	} else if (G_Robot_Direction % 4 == 1) {
		Known_X += 1;
		Known_Y += 0;
	} else if (G_Robot_Direction % 4 == 2) {
		Known_X += 0;
		Known_Y += -1;
	} else {
		Known_X += -1;
		Known_Y += 0;
	}
	if (G_MAZE_Explored[Known_X][Known_Y] == 1) {
		Known_Pass_Generation();
		for (int i = 0; G_Known_Pass[i] != 0; i++) {
			if (Failsafe_Flag() == 1) {
				break;
			}
			if (G_Known_Pass[i] > 0) {			//1区間前進
				Motor_trapezoid_PID(1000, 4000, 1000, 15000,
						90 * G_Known_Pass[i]);
			} else if (G_Known_Pass[i] == -2) {			//左
				Motor_Sula_ST(1000, 1000, 1000, 5000, 20);
				Motor_Sula_COS(1000, 90, 1630, 60000);
				Motor_trapezoid_PID(1000, 1000, 1000, 5000, 48);
			} else if (G_Known_Pass[i] == -3) {			//右
				Motor_Sula_ST(1000, 1000, 1000, 5000, 20);
				Motor_Sula_COS(1000, -90, 1630, 60000);
				Motor_trapezoid_PID(1000, 1000, 1000, 5000, 48);
			} else if (G_Known_Pass[i] == -8) {			//行き止まりUターン
				Motor_trapezoid(1000, 1000, 0, 10000, 75);
				Motor_Stop();
				G_Just_UTurned = 1;
				Robot_adjustment();
				Motor_trapezoid_PID(0, 1000, 1000, 10000, 90);
			} else if (G_Known_Pass[i] == -4) {			//左大廻９０
				Motor_Wallcut_ST(1000, 50, 0);
				Motor_Sula_COS(1000, 90, 750, 20000);
				Motor_Wallcut_END(1000, 85, 0);
			} else if (G_Known_Pass[i] == -6) {			//右大廻９０
				Motor_Wallcut_ST(1000, 50, 1);
				Motor_Sula_COS(1000, -90, 750, 20000);
				Motor_Wallcut_END(1000, 85, 1);
			} else if (G_Known_Pass[i] == -5) {			//左大廻１８０
				Motor_Wallcut_ST(1000, 40, 0);
				Motor_Sula_COS(1000, 180, 680, 13000);
				Motor_Wallcut_END(1000, 70, 0);
			} else if (G_Known_Pass[i] == -7) {			//右大廻１８０
				Motor_Wallcut_ST(1000, 40, 1);
				Motor_Sula_COS(1000, -180, 680, 13000);
				Motor_Wallcut_END(1000, 70, 1);
			}
			// 斜め(NANAME)区間の実行は未実装・未チューニングのためコメントアウトのまま。
			// 有効化する際は (1) このforループの走査対象をG_Known_Pass[]ではなく
			// Known_Pass_NANAME[]に切り替え、(2) 下記の速度・距離・角度の各定数を
			// 実機で再チューニングすること。生成側はKnown_Pass_Compression_NANAME()
			// (Maze.c)で既にKnown_Pass_NANAME[]として計算済み。
			/* else if (Known_Pass_NANAME[i] == -51) {			//入り　左４５
			 Motor_Wallcut_ST(600, 35, 0);
			 Motor_Sula_COS(600, 45, 450, 8000);
			 Motor_NANAME_PID(600, 600, 600, 5000, 67);
			 } else if (Known_Pass_NANAME[i] == -52) {			//入り　左１３５
			 Motor_Wallcut_ST(600, 80, 0);
			 Motor_Sula_COS(600, 135, 550, 8000);
			 Motor_NANAME_PID(600, 600, 600, 5000, 78);
			 } else if (Known_Pass_NANAME[i] == -53) {			//入り　右４５
			 Motor_Wallcut_ST(600, 35, 1);
			 Motor_Sula_COS(600, -45, 450, 8000);
			 Motor_NANAME_PID(600, 600, 600, 5000, 67);
			 } else if (Known_Pass_NANAME[i] == -54) {			//入り　右１３５
			 Motor_Wallcut_ST(600, 80, 1);
			 Motor_Sula_COS(600, -135, 550, 8000);
			 Motor_NANAME_PID(600, 600, 600, 5000, 78);
			 } else if (Known_Pass_NANAME[i] == -61) {			//出　左４５
			 Motor_Wallcut_ST_NANAME(600, 65, 0);
			 Motor_Sula_COS(600, 45, 600, 10000);
			 Motor_Wallcut_END(600, 43, 0);
			 } else if (Known_Pass_NANAME[i] == -62) {			//出　左１３５
			 Motor_Wallcut_ST_NANAME(600, 45, 0);
			 Motor_Sula_COS(600, 135, 500, 8000);
			 Motor_Wallcut_END(600, 85, 0);
			 } else if (Known_Pass_NANAME[i] == -63) {			//出　右４５
			 Motor_Wallcut_ST_NANAME(600, 65, 1);
			 Motor_Sula_COS(600, -45, 600, 10000);
			 Motor_Wallcut_END(600, 43, 1);
			 } else if (Known_Pass_NANAME[i] == -64) {			//出　右１３５
			 Motor_Wallcut_ST_NANAME(600, 45, 1);
			 Motor_Sula_COS(600, -135, 500, 8000);
			 Motor_Wallcut_END(600, 85, 1);
			 } else if (Known_Pass_NANAME[i] == -65) {			//V90左
			 Motor_Wallcut_ST_NANAME(600, 30, 0);
			 Motor_Sula_COS(600, 90, 600, 10000);
			 Motor_NANAME_PID(600, 600, 600, 5000, 53);
			 } else if (Known_Pass_NANAME[i] == -66) {			//V90右
			 Motor_Wallcut_ST_NANAME(600, 30, 1);
			 Motor_Sula_COS(600, -90, 600, 10000);
			 Motor_NANAME_PID(600, 600, 600, 5000, 53);
			 } else if (Known_Pass_NANAME[i] % 50 == 0) {			//直線
			 Motor_NANAME_PID(600, 1000, 600, 5000,
			 127.3 * Known_Pass_NANAME[i] / -50);

			 }*/
		}
		if (G_Robot_MAZE_X == 0 && G_Robot_MAZE_Y == 0) {
			Motor_trapezoid(1000, 1000, 0, 10000, 90);
			//Robot_adjustment_Back_Only();
		}
	} else {
		Robot_Maze_Suction_Action();
	}
}

void Short_NANAME_Move1000(int MAX, int AC) {
	Maze_Road();
	Maze_Wall_fill();
	G_Gool_X = MAZE_GOOL_X;
	G_Gool_Y = MAZE_GOOL_Y;
	G_Robot_MAZE_X = 0;
	G_Robot_MAZE_Y = 0;
	G_Robot_Direction = 0;
	G_MAZE_Explored[G_Gool_X][G_Gool_Y] = 0;
	Maze_Step_Calculate();
	Maze_Shortest_Calculation();
	Shortest_Pass_Compression();
	Shortest_Pass_Compression_NANAME();

	Motor_Setup_Voltage();
	Suction_Start(15);
	Fun_Flag_OFF();
	HAL_Delay(500);

	Motor_Setup();
	if (G_Short_Pass_NANAME[0] == 1) {
		Motor_trapezoid_PID(0, 1000, 1000, 15000, 90 + 24);
		G_Short_Pass_NANAME[0] = -1;
	} else {
		Motor_trapezoid_PID(0, 1000, 1000, 20000, 24);			//24
	}

	for (int i = 0; G_Short_Pass_NANAME[i] != 0; i++) {
		if (Failsafe_Flag() == 1) {
			break;
		}
		if (G_Short_Pass_NANAME[i] > 0) {			//区間前進
			Motor_trapezoid_PID(1000, MAX, 1000, AC,
					90 * G_Short_Pass_NANAME[i]);
		} else if ((G_Short_Pass_NANAME[i] <= -4)
				&& (G_Short_Pass_NANAME[i] > -50)) {
			if (G_Short_Pass_CP[i] == -4) {			//左大廻９０
				Motor_Wallcut_ST(1000, 48, 0);
				Motor_Sula_COS(1000, 88, 600, 10000);
				Motor_Wallcut_END(1000, 35, 0);
			} else if (G_Short_Pass_CP[i] == -6) {			//右大廻９０
				Motor_Wallcut_ST(1000, 48, 1);
				Motor_Sula_COS(1000, -88, 600, 10000);
				Motor_Wallcut_END(1000, 35, 1);
			} else if (G_Short_Pass_CP[i] == -5) {			//左大廻１８０
				Motor_Wallcut_ST(1000, 60, 0);
				Motor_Sula_COS(1000, 179, 650, 10000);
				Motor_Wallcut_END(1000, 40, 0);
			} else if (G_Short_Pass_CP[i] == -7) {			//右大廻１８０
				Motor_Wallcut_ST(1000, 60, 1);
				Motor_Sula_COS(1000, -179, 650, 10000);
				Motor_Wallcut_END(1000, 40, 1);

			}
		} else if (G_Short_Pass_NANAME[i] <= -50) {			//斜め
			if (G_Short_Pass_NANAME[i] == -51) {			//入り　左４５
				Motor_Wallcut_ST(1000, 15, 0);
				Motor_Sula_COS(1000, 44, 750, 10000);
				Motor_Wallcut_END_NANAME(1000, 42, 0);
			} else if (G_Short_Pass_NANAME[i] == -52) {			//入り　左１３５
				Motor_Wallcut_ST(1000, 38, 0);
				Motor_Sula_COS(1000, 134, 800, 10000);
				Motor_Wallcut_END_NANAME(1000, 8, 0);
			} else if (G_Short_Pass_NANAME[i] == -53) {			//入り　右４５
				Motor_Wallcut_ST(1000, 15, 1);
				Motor_Sula_COS(1000, -44, 750, 10000);
				Motor_Wallcut_END_NANAME(1000, 42, 1);
			} else if (G_Short_Pass_NANAME[i] == -54) {			//入り　右１３５
				Motor_Wallcut_ST(1000, 38, 1);
				Motor_Sula_COS(1000, -134, 800, 10000);
				Motor_Wallcut_END_NANAME(1000, 8, 1);
			} else if (G_Short_Pass_NANAME[i] == -61) {			//出　左４５
				Motor_Wallcut_ST_NANAME(1000, 73, 0);
				Motor_Sula_COS(1000, 41.5, 700, 20000);
				Motor_Wallcut_END(1000, 30, 0);
			} else if (G_Short_Pass_NANAME[i] == -62) {			//出　左１３５
				Motor_Wallcut_ST_NANAME(1000, 25, 0);
				Motor_Sula_COS(1000, 132, 700, 14000);
				Motor_Wallcut_END(1000, 25, 0);
			} else if (G_Short_Pass_NANAME[i] == -63) {			//出　右４５
				Motor_Wallcut_ST_NANAME(1000, 73, 1);
				Motor_Sula_COS(1000, -41.5, 700, 20000);
				Motor_Wallcut_END(1000, 30, 1);
			} else if (G_Short_Pass_NANAME[i] == -64) {			//出　右１３５
				Motor_Wallcut_ST_NANAME(1000, 25, 1);
				Motor_Sula_COS(1000, -132, 700, 14000);
				Motor_Wallcut_END(1000, 25, 1);
			} else if (G_Short_Pass_NANAME[i] == -65) {			//V90左
				Motor_Wallcut_ST_NANAME(1000, 37, 0);
				//Motor_NANAME_PID(500, 500, 500, 5000, 10);
				Motor_Sula_COS(1000, 88.5, 800, 20000);
				Motor_Wallcut_END_NANAME(1000, 27, 0);
			} else if (G_Short_Pass_NANAME[i] == -66) {			//V90右
				Motor_Wallcut_ST_NANAME(1000, 37, 1);
				//Motor_NANAME_PID(500, 500, 500, 5000, 10);
				Motor_Sula_COS(1000, -88.5, 800, 20000);
				Motor_Wallcut_END_NANAME(1000, 27, 1);
			} else if (G_Short_Pass_NANAME[i] % 50 == 0) {			//直線
				Motor_NANAME_PID(1000, MAX, 1000, AC - 5000,
						127.3 * G_Short_Pass_NANAME[i] / -50);

			}
		}

	}
	if (Failsafe_Flag() == 0) {
		Motor_trapezoid_PID(1000, 1000, 0, 10000, 180);
		Motor_Stop();
		Suction_Stop();

		Robot_adjustment();
		LED_Goal();
	} else {
		Failsafe_Flag_OFF();
	}

}

void Short_NANAME_Move2000(int MAX, int AC) {
	Maze_Road();
	Maze_Wall_fill();
	G_Gool_X = MAZE_GOOL_X;
	G_Gool_Y = MAZE_GOOL_Y;
	G_Robot_MAZE_X = 0;
	G_Robot_MAZE_Y = 0;
	G_Robot_Direction = 0;
	G_MAZE_Explored[G_Gool_X][G_Gool_Y] = 0;
	Maze_Step_Calculate();

	Maze_Shortest_Calculation();
	//Maze_Dijkstra_Calculation();

	Shortest_Pass_Compression();
	Shortest_Pass_Compression_NANAME();
	Suction_Start(50);
	HAL_Delay(500);

	Motor_Setup();
	if (G_Short_Pass_NANAME[0] == 1) {
		Motor_trapezoid_PID(0, 2000, 2000, 30000, 90 + 24);
		G_Short_Pass_NANAME[0] = -1;
	} else {
		Motor_trapezoid_PID(0, 2000, 2000, 70000, 10);			//24
	}

	for (int i = 0; G_Short_Pass_NANAME[i] != 0; i++) {
		if (Failsafe_Flag() == 1) {
			break;
		}
		if (G_Short_Pass_NANAME[i] > 0) {			//区間前進
			//Suction_change(30);
			Motor_trapezoid_Asymmetric_PID(2000, MAX, 2000, AC,
					90 * G_Short_Pass_NANAME[i]);
			//Suction_change(50);
		} else if ((G_Short_Pass_NANAME[i] <= -4)
				&& (G_Short_Pass_NANAME[i] > -50)) {
			if (G_Short_Pass_NANAME[i] == -4) {			//左大廻９０
				Motor_Wallcut_ST(2000, 10, 0);
				Motor_Sula_COS(2000, 90, 2000, 40000);
				Motor_Wallcut_END(2000, 75, 0);
			} else if (G_Short_Pass_NANAME[i] == -6) {			//右大廻９０
				Motor_Wallcut_ST(2000, 10, 1);
				Motor_Sula_COS(2000, -90, 2000, 40000);
				Motor_Wallcut_END(2000, 75, 1);
			} else if (G_Short_Pass_NANAME[i] == -5) {			//左大廻１８０
				Motor_Wallcut_ST(2000, 5, 0);
				Motor_Sula_COS(2000, 180, 1220, 40000);
				Motor_Wallcut_END(2000, 73, 0);
			} else if (G_Short_Pass_NANAME[i] == -7) {			//右大廻１８０
				Motor_Wallcut_ST(2000, 5, 1);
				Motor_Sula_COS(2000, -180, 1220, 40000);
				Motor_Wallcut_END(2000, 73, 1);

			}
		} else if (G_Short_Pass_NANAME[i] <= -50) {			//斜め
			if (G_Short_Pass_NANAME[i] == -51) {			//入り　左４５
				Motor_Wallcut_ST(2000, 5, 0);
				Motor_Sula_COS(2000, 45, 1900, 120000);
				Motor_Wallcut_END_NANAME(2000, 112, 0);
			} else if (G_Short_Pass_NANAME[i] == -52) {			//入り　左１３５
				Motor_Wallcut_ST(2000, 31, 0);
				Motor_Sula_COS(2000, 135, 1500, 80000);
				Motor_Wallcut_END_NANAME(2000, 98, 0);
			} else if (G_Short_Pass_NANAME[i] == -53) {			//入り　右４５
				Motor_Wallcut_ST(2000, 5, 1);
				Motor_Sula_COS(2000, -45, 1900, 120000);
				Motor_Wallcut_END_NANAME(2000, 112, 1);
			} else if (G_Short_Pass_NANAME[i] == -54) {			//入り　右１３５
				Motor_Wallcut_ST(2000, 31, 1);
				Motor_Sula_COS(2000, -135, 1500, 80000);
				Motor_Wallcut_END_NANAME(2000, 98, 1);
			} else if (G_Short_Pass_NANAME[i] == -61) {			//出　左４５
				Motor_Wallcut_ST_NANAME(2000, 13, 0);
				Motor_Sula_COS(2000, 45, 1500, 40000);
				Motor_Wallcut_END(2000, 25, 0);
			} else if (G_Short_Pass_NANAME[i] == -62) {			//出　左１３５
				Motor_Wallcut_ST_NANAME(2000, 10, 0);
				Motor_Sula_COS(2000, 135, 1350, 70000);
				Motor_Wallcut_END(2000, 90, 0);
			} else if (G_Short_Pass_NANAME[i] == -63) {			//出　右４５
				Motor_Wallcut_ST_NANAME(2000, 13, 1);
				Motor_Sula_COS(2000, -45, 1500, 40000);
				Motor_Wallcut_END(2000, 25, 1);
			} else if (G_Short_Pass_NANAME[i] == -64) {			//出　右１３５
				Motor_Wallcut_ST_NANAME(2000, 10, 1);
				Motor_Sula_COS(2000, -135, 1350, 70000);
				Motor_Wallcut_END(2000, 90, 1);
			} else if (G_Short_Pass_NANAME[i] == -65) {			//V90左
				Motor_Wallcut_ST_NANAME(2000, 13, 0);
				Motor_Sula_COS(2000, 90, 2000, 130000);
				Motor_Wallcut_END_NANAME(2000, 80, 0);
			} else if (G_Short_Pass_NANAME[i] == -66) {			//V90右
				Motor_Wallcut_ST_NANAME(2000, 13, 1);
				Motor_Sula_COS(2000, -90, 2000, 130000);
				Motor_Wallcut_END_NANAME(2000, 80, 1);
			} else if (G_Short_Pass_NANAME[i] % 50 == 0) {			//直線
				Motor_NANAME_PID(2000, MAX, 2000, AC - 5000,
						127.3 * G_Short_Pass_NANAME[i] / -50);

			}
		}

	}
	if (Failsafe_Flag() == 0) {
		Motor_trapezoid_PID(2000, 2000, 0, 15000, 180);
		Motor_Stop();
		Suction_Stop();

		Robot_adjustment();
		LED_Goal();
	} else {
		Failsafe_Flag_OFF();
	}

}

void Short_NANAME_Move2400(int MAX, int AC) {
	Maze_Road();
	Maze_Wall_fill();
	G_Gool_X = MAZE_GOOL_X;
	G_Gool_Y = MAZE_GOOL_Y;
	G_Robot_MAZE_X = 0;
	G_Robot_MAZE_Y = 0;
	G_Robot_Direction = 0;
	G_MAZE_Explored[G_Gool_X][G_Gool_Y] = 0;
	Maze_Step_Calculate();
	Maze_Shortest_Calculation();
	Shortest_Pass_Compression();
	Shortest_Pass_Compression_NANAME();
	Suction_Start(70);
	HAL_Delay(500);

	Motor_Setup();
	if (G_Short_Pass_NANAME[0] == 1) {
		Motor_trapezoid_PID(0, 2400, 2400, 30000, 90 + 24);			//24
		G_Short_Pass_NANAME[0] = -1;
	} else {
		Motor_trapezoid_PID(0, 2400, 2400, 60000, 10);			//24
	}
	for (int i = 0; G_Short_Pass_NANAME[i] != 0; i++) {
		if (Failsafe_Flag() == 1) {
			break;
		}
		if (G_Short_Pass_NANAME[i] > 0) {			//区間前進
			Motor_trapezoid_Asymmetric_PID(2400, MAX, 2400, AC,
					90 * G_Short_Pass_NANAME[i]);
		} else if ((G_Short_Pass_NANAME[i] <= -4)
				&& (G_Short_Pass_NANAME[i] > -50)) {
			if (G_Short_Pass_CP[i] == -4) {			//左大廻９０
				Motor_Wallcut_ST(2400, 25, 0);
				Motor_Sula_COS(2400, 90, 1600, 80000);
				Motor_Wallcut_END(2400, 100, 0);
			} else if (G_Short_Pass_CP[i] == -6) {			//右大廻９０
				Motor_Wallcut_ST(2400, 25, 1);
				Motor_Sula_COS(2400, -90, 1600, 80000);
				Motor_Wallcut_END(2400, 100, 1);
			} else if (G_Short_Pass_CP[i] == -5) {			//左大廻１８０
				Motor_Wallcut_ST(2400, 15, 0);
				Motor_Sula_COS(2400, 180, 1500, 70000);
				Motor_Wallcut_END(2400, 88, 0);
			} else if (G_Short_Pass_CP[i] == -7) {			//右大廻１８０
				Motor_Wallcut_ST(2400, 15, 1);
				Motor_Sula_COS(2400, -180, 1500, 70000);
				Motor_Wallcut_END(2400, 88, 1);

			}
		} else if (G_Short_Pass_NANAME[i] <= -50) {			//斜め
			if (G_Short_Pass_NANAME[i] == -51) {			//入り　左４５
				Motor_Wallcut_ST(2400, 3, 0);
				Motor_Sula_COS(2400, 45, 2500, 160000);
				Motor_Wallcut_END_NANAME(2400, 113, 0);
			} else if (G_Short_Pass_NANAME[i] == -52) {			//入り　左１３５
				Motor_Wallcut_ST(2400, 15, 0);
				Motor_Sula_COS(2400, 135, 2000, 55000);
				Motor_Wallcut_END_NANAME(2400, 85, 0);
			} else if (G_Short_Pass_NANAME[i] == -53) {			//入り　右４５
				Motor_Wallcut_ST(2400, 3, 1);
				Motor_Sula_COS(2400, -45, 2500, 160000);
				Motor_Wallcut_END_NANAME(2400, 113, 1);
			} else if (G_Short_Pass_NANAME[i] == -54) {			//入り　右１３５
				Motor_Wallcut_ST(2400, 15, 1);
				Motor_Sula_COS(2400, -135, 2000, 55000);
				Motor_Wallcut_END_NANAME(2400, 85, 1);
			} else if (G_Short_Pass_NANAME[i] == -61) {			//出　左４５
				Motor_Wallcut_ST_NANAME(2400, 8, 0);
				Motor_Sula_COS(2400, 45, 1550, 70000);
				Motor_Wallcut_END(2400, 40, 0);
			} else if (G_Short_Pass_NANAME[i] == -62) {			//出　左１３５
				Motor_Wallcut_ST_NANAME(2400, 17, 0);
				Motor_Sula_COS(2400, 135, 2100, 80000);
				Motor_Wallcut_END(2400, 123, 0);
			} else if (G_Short_Pass_NANAME[i] == -63) {			//出　右４５
				Motor_Wallcut_ST_NANAME(2400, 8, 1);
				Motor_Sula_COS(2400, -45, 1550, 70000);
				Motor_Wallcut_END(2400, 40, 1);
			} else if (G_Short_Pass_NANAME[i] == -64) {			//出　右１３５
				Motor_Wallcut_ST_NANAME(2400, 17, 1);
				Motor_Sula_COS(2400, -135, 2100, 80000);
				Motor_Wallcut_END(2400, 123, 1);
			} else if (G_Short_Pass_NANAME[i] == -65) {			//V90左
				Motor_Wallcut_ST_NANAME(2400, 5, 0);
				//Motor_NANAME_PID(500, 500, 500, 5000, 10);
				Motor_Sula_COS(2400, 87, 2600, 150000);
				Motor_Wallcut_END_NANAME(2400, 90, 0);
			} else if (G_Short_Pass_NANAME[i] == -66) {			//V90右
				Motor_Wallcut_ST_NANAME(2400, 5, 1);
				//Motor_NANAME_PID(500, 500, 500, 5000, 10);
				Motor_Sula_COS(2400, -87, 2600, 150000);
				Motor_Wallcut_END_NANAME(2400, 90, 1);
			} else if (G_Short_Pass_NANAME[i] % 50 == 0) {			//直線
				Motor_NANAME_PID(2400, MAX, 2400, AC - 10000,
						127.3 * G_Short_Pass_NANAME[i] / -50);

			}
		}

	}
	if (Failsafe_Flag() == 0) {
		Motor_trapezoid_PID(2400, 2400, 0, 30000, 180);
		Motor_Stop();
		Suction_Stop();

		Robot_adjustment();
		LED_Goal();
	} else {
		Failsafe_Flag_OFF();
	}

}

void Short_NANAME_Move2700(int MAX, int AC) {
	Maze_Road();
	Maze_Wall_fill();
	G_Gool_X = MAZE_GOOL_X;
	G_Gool_Y = MAZE_GOOL_Y;
	G_Robot_MAZE_X = 0;
	G_Robot_MAZE_Y = 0;
	G_Robot_Direction = 0;
	G_MAZE_Explored[G_Gool_X][G_Gool_Y] = 0;
	Maze_Step_Calculate();
	Maze_Shortest_Calculation();
	Shortest_Pass_Compression();
	Shortest_Pass_Compression_NANAME();
	Suction_Start(85);
	HAL_Delay(500);

	Motor_Setup();
	if (G_Short_Pass_NANAME[0] == 1) {
		Motor_trapezoid_PID(0, 2700, 2700, 25000, 90 + 24);			//24
		G_Short_Pass_NANAME[0] = -1;
	} else {
		Motor_trapezoid_PID(0, 2700, 2700, 60000, 10);			//24
	}
	for (int i = 0; G_Short_Pass_NANAME[i] != 0; i++) {
		if (Failsafe_Flag() == 1) {
			break;
		}
		if (G_Short_Pass_NANAME[i] > 0) {			//区間前進
			Motor_trapezoid_PID(2700, MAX, 2700, AC,
					90 * G_Short_Pass_NANAME[i]);
		} else if ((G_Short_Pass_NANAME[i] <= -4)
				&& (G_Short_Pass_NANAME[i] > -50)) {
			if (G_Short_Pass_CP[i] == -4) {			//左大廻９０
				Motor_Wallcut_ST(2700, 25, 0);
				Motor_Sula_COS(2700, 90, 2300, 70000);
				Motor_Wallcut_END(2700, 68, 0);
			} else if (G_Short_Pass_CP[i] == -6) {			//右大廻９０
				Motor_Wallcut_ST(2700, 25, 1);
				Motor_Sula_COS(2700, -90, 2300, 70000);
				Motor_Wallcut_END(2700, 68, 1);
			} else if (G_Short_Pass_CP[i] == -5) {			//左大廻１８０
				Motor_Wallcut_ST(2700, 10, 0);
				Motor_Sula_COS(2700, 180, 1700, 40000);
				Motor_Wallcut_END(2700, 58, 0);
			} else if (G_Short_Pass_CP[i] == -7) {			//右大廻１８０
				Motor_Wallcut_ST(2700, 10, 1);
				Motor_Sula_COS(2700, -180, 1700, 40000);
				Motor_Wallcut_END(2700, 58, 1);

			}
		} else if (G_Short_Pass_NANAME[i] <= -50) {			//斜め
			if (G_Short_Pass_NANAME[i] == -51) {			//入り　左４５
				Motor_Wallcut_ST(2700, 2, 0);
				Motor_Sula_COS(2700, 45, 2850, 100000);
				Motor_Wallcut_END_NANAME(2700, 85, 0);
			} else if (G_Short_Pass_NANAME[i] == -52) {			//入り　左１３５
				Motor_Wallcut_ST(2700, 11, 0);
				Motor_Sula_COS(2700, 135, 2200, 60000);
				Motor_Wallcut_END_NANAME(2700, 70, 0);
			} else if (G_Short_Pass_NANAME[i] == -53) {			//入り　右４５
				Motor_Wallcut_ST(2700, 2, 1);
				Motor_Sula_COS(2700, -45, 2850, 100000);
				Motor_Wallcut_END_NANAME(2700, 85, 1);
			} else if (G_Short_Pass_NANAME[i] == -54) {			//入り　右１３５
				Motor_Wallcut_ST(2700, 11, 1);
				Motor_Sula_COS(2700, -135, 2200, 60000);
				Motor_Wallcut_END_NANAME(2700, 70, 1);
			} else if (G_Short_Pass_NANAME[i] == -61) {			//出　左４５
				Motor_Wallcut_ST_NANAME(2700, 12, 0);
				Motor_Sula_COS(2700, 45, 1800, 70000);
				Motor_Wallcut_END(2700, 10, 0);
			} else if (G_Short_Pass_NANAME[i] == -62) {			//出　左１３５
				Motor_Wallcut_ST_NANAME(2700, 23, 0);
				Motor_Sula_COS(2700, 135, 2000, 100000);
				Motor_Wallcut_END(2700, 117, 0);
			} else if (G_Short_Pass_NANAME[i] == -63) {			//出　右４５
				Motor_Wallcut_ST_NANAME(2700, 12, 1);
				Motor_Sula_COS(2700, -45, 1800, 70000);
				Motor_Wallcut_END(2700, 10, 1);
			} else if (G_Short_Pass_NANAME[i] == -64) {			//出　右１３５
				Motor_Wallcut_ST_NANAME(2700, 23, 1);
				Motor_Sula_COS(2700, -135, 2000, 100000);
				Motor_Wallcut_END(2700, 117, 1);
			} else if (G_Short_Pass_NANAME[i] == -65) {			//V90左
				Motor_Wallcut_ST_NANAME(2700, 18, 0);
				Motor_Sula_COS(2700, 82, 2800, 180000);
				Motor_Wallcut_END_NANAME(2700, 97, 0);
			} else if (G_Short_Pass_NANAME[i] == -66) {			//V90右
				Motor_Wallcut_ST_NANAME(2700, 18, 1);
				Motor_Sula_COS(2700, -82, 2800, 180000);
				Motor_Wallcut_END_NANAME(2700, 97, 1);
			} else if (G_Short_Pass_NANAME[i] % 50 == 0) {			//直線
				Motor_NANAME_PID(2700, MAX, 2700, AC - 10000,
						127.3 * G_Short_Pass_NANAME[i] / -50);

			}
		}

	}
	if (Failsafe_Flag() == 0) {
		Motor_trapezoid_PID(2700, 2700, 0, 33000, 180);
		Motor_Stop();
		Suction_Stop();

		Robot_adjustment();
		LED_Goal();
	} else {
		Failsafe_Flag_OFF();
	}

}

void Short_Dijkstra_Move2000(int MAX, int AC) {
	Maze_Road();
	Maze_Wall_fill();
	G_Gool_X = MAZE_GOOL_X;
	G_Gool_Y = MAZE_GOOL_Y;
	G_Robot_MAZE_X = 0;
	G_Robot_MAZE_Y = 0;
	G_Robot_Direction = 0;
	G_MAZE_Explored[G_Gool_X][G_Gool_Y] = 0;
	Maze_Step_Calculate();

	//Maze_Shortest_Calculation();
	Maze_Dijkstra_Calculation();

	Shortest_Pass_Compression();
	Shortest_Pass_Compression_NANAME();
	Suction_Start(50);
	HAL_Delay(500);

	Motor_Setup();
	if (G_Short_Pass_NANAME[0] == 1) {
		Motor_trapezoid_PID(0, 2000, 2000, 30000, 90 + 24);
		G_Short_Pass_NANAME[0] = -1;
	} else {
		Motor_trapezoid_PID(0, 2000, 2000, 70000, 10);			//24
	}

	for (int i = 0; G_Short_Pass_NANAME[i] != 0; i++) {
		if (Failsafe_Flag() == 1) {
			break;
		}
		if (G_Short_Pass_NANAME[i] > 0) {			//区間前進
			Motor_trapezoid_PID(2000, MAX, 2000, AC,
					90 * G_Short_Pass_NANAME[i]);
		} else if ((G_Short_Pass_NANAME[i] <= -4)
				&& (G_Short_Pass_NANAME[i] > -50)) {
			if (G_Short_Pass_CP[i] == -4) {			//左大廻９０
				Motor_Wallcut_ST(2000, 15, 0);
				Motor_Sula_COS(2000, 88.5, 1200, 30000);
				Motor_Wallcut_END(2000, 35, 0);
			} else if (G_Short_Pass_CP[i] == -6) {			//右大廻９０
				Motor_Wallcut_ST(2000, 15, 1);
				Motor_Sula_COS(2000, -88.5, 1200, 30000);
				Motor_Wallcut_END(2000, 35, 1);
			} else if (G_Short_Pass_CP[i] == -5) {			//左大廻１８０
				Motor_Wallcut_ST(2000, 30, 0);
				Motor_Sula_COS(2000, 178, 1280, 30000);
				Motor_Wallcut_END(2000, 60, 0);
			} else if (G_Short_Pass_CP[i] == -7) {			//右大廻１８０
				Motor_Wallcut_ST(2000, 30, 1);
				Motor_Sula_COS(2000, -178, 1280, 30000);
				Motor_Wallcut_END(2000, 60, 1);

			}
		} else if (G_Short_Pass_NANAME[i] <= -50) {			//斜め
			if (G_Short_Pass_NANAME[i] == -51) {			//入り　左４５
				Motor_Wallcut_ST(2000, 8, 0);
				Motor_Sula_COS(2000, 43, 1600, 55000);
				Motor_Wallcut_END_NANAME(2000, 55, 0);
			} else if (G_Short_Pass_NANAME[i] == -52) {			//入り　左１３５
				Motor_Wallcut_ST(2000, 10, 0);
				Motor_Sula_COS(2000, 134, 1300, 50000);
				Motor_Wallcut_END_NANAME(2000, 9, 0);
			} else if (G_Short_Pass_NANAME[i] == -53) {			//入り　右４５
				Motor_Wallcut_ST(2000, 8, 1);
				Motor_Sula_COS(2000, -43, 1600, 55000);
				Motor_Wallcut_END_NANAME(2000, 55, 1);
			} else if (G_Short_Pass_NANAME[i] == -54) {			//入り　右１３５
				Motor_Wallcut_ST(2000, 10, 1);
				Motor_Sula_COS(2000, -134, 1300, 50000);
				Motor_Wallcut_END_NANAME(2000, 9, 1);
			} else if (G_Short_Pass_NANAME[i] == -61) {			//出　左４５
				Motor_Wallcut_ST_NANAME(2000, 22, 0);
				Motor_Sula_COS(2000, 42, 1500, 40000);
				Motor_Wallcut_END(2000, 21, 0);
			} else if (G_Short_Pass_NANAME[i] == -62) {			//出　左１３５
				Motor_Wallcut_ST_NANAME(2000, 8, 0);
				Motor_Sula_COS(2000, 131.5, 1700, 50000);
				Motor_Wallcut_END(2000, 70, 0);
			} else if (G_Short_Pass_NANAME[i] == -63) {			//出　右４５
				Motor_Wallcut_ST_NANAME(2000, 22, 1);
				Motor_Sula_COS(2000, -42, 1500, 40000);
				Motor_Wallcut_END(2000, 21, 1);
			} else if (G_Short_Pass_NANAME[i] == -64) {			//出　右１３５
				Motor_Wallcut_ST_NANAME(2000, 8, 1);
				Motor_Sula_COS(2000, -131.5, 1700, 50000);
				Motor_Wallcut_END(2000, 70, 1);
			} else if (G_Short_Pass_NANAME[i] == -65) {			//V90左
				Motor_Wallcut_ST_NANAME(2000, 7, 0);
				Motor_Sula_COS(2000, 84, 2000, 80000);
				Motor_Wallcut_END_NANAME(2000, 35, 0);
			} else if (G_Short_Pass_NANAME[i] == -66) {			//V90右
				Motor_Wallcut_ST_NANAME(2000, 7, 1);
				Motor_Sula_COS(2000, -84, 2000, 80000);
				Motor_Wallcut_END_NANAME(2000, 35, 1);
			} else if (G_Short_Pass_NANAME[i] % 50 == 0) {			//直線
				Motor_NANAME_PID(2000, MAX, 2000, AC - 5000,
						127.3 * G_Short_Pass_NANAME[i] / -50);

			}
		}

	}
	if (Failsafe_Flag() == 0) {
		Motor_trapezoid_PID(2000, 2000, 0, 20000, 180);
		Motor_Stop();
		Suction_Stop();

		Robot_adjustment();
		LED_Goal();
	} else {
		Failsafe_Flag_OFF();
	}

}


/*
 * ターンごとに通過速度を変える最短走行。
 *
 * Short_NANAME_Move2000 と同じ走り方で、ターンの種類ごとに通過速度を持つ。
 * 直線の始点・終点速度は、前後のターンの速度に合わせる(SpeedPlan.c)。
 * 直線が短くて加減速しきれないときや、ターンが直接つながるときは、
 * 速度計画が遅い方に合わせて速度を下げる。
 *
 * ターンのパラメータは Short_NANAME_Move2000 の値(V_BASE=2000で調整済み)を基準にし、
 * 通過速度Vに合わせて 角速度 x (V/V_BASE)、角加速度 x (V/V_BASE)^2 に換算する。
 * こうすると理想的には同じ軌跡(同じ旋回半径)になる。横加速度は (V/V_BASE)^2 倍になる。
 * 前後のオフセット距離は理想的には変わらないが、実機では遅れで変わるので、
 * 速度を変えたターンは offset_st / offset_end の調整が必要。
 *
 * TurnV_Table の v を全部 2000 にすると Short_NANAME_Move2000 と同じ走りになる。
 */
#define TURNV_V_BASE 2000.0f
#define TURNV_V_START 2000.0f	/* スタート区間の終わりの速度の上限 */
#define TURNV_V_GOAL 2000.0f	/* ゴール停止区間に入る速度の上限 */

#define WC_NORMAL 0
#define WC_NANAME 1

typedef struct {
	float v;			/* 通過速度[mm/s]。これを調整する */
	float angle;		/* 左旋回の角度[deg]。右は符号を反転 */
	float w_max;		/* V_BASEでの最大角速度 */
	float w_ac;			/* V_BASEでの角加速度 */
	int st_kind;		/* 前オフセットの壁切れ: WC_NORMAL / WC_NANAME */
	float offset_st;
	int end_kind;		/* 後オフセット */
	float offset_end;
} TurnV_Param;

enum {
	TV_BIG90,		/* 大回り90  (-4 / -6) */
	TV_BIG180,		/* 大回り180 (-5 / -7) */
	TV_IN45,		/* 斜め入り45  (-51 / -53) */
	TV_IN135,		/* 斜め入り135 (-52 / -54) */
	TV_OUT45,		/* 斜め出45  (-61 / -63) */
	TV_OUT135,		/* 斜め出135 (-62 / -64) */
	TV_V90,			/* V90 (-65 / -66) */
	TV_NUM
};

static TurnV_Param TurnV_Table[TV_NUM] = {
	/*            v     angle  w_max  w_ac     st_kind    st   end_kind   end */
	[TV_BIG90]  = {2000,  90, 2000,  40000, WC_NORMAL, 10, WC_NORMAL,  75},
	[TV_BIG180] = {2000, 180, 1220,  40000, WC_NORMAL,  5, WC_NORMAL,  73},
	[TV_IN45]   = {2000,  45, 1900, 120000, WC_NORMAL,  5, WC_NANAME, 112},
	[TV_IN135]  = {2000, 135, 1500,  80000, WC_NORMAL, 31, WC_NANAME,  98},
	[TV_OUT45]  = {2000,  45, 1500,  40000, WC_NANAME, 13, WC_NORMAL,  25},
	[TV_OUT135] = {2000, 135, 1350,  70000, WC_NANAME, 10, WC_NORMAL,  90},
	[TV_V90]    = {2000,  90, 2000, 130000, WC_NANAME, 13, WC_NANAME,  80},
};

/* パスのコード -> テーブル番号と向き(0:左 1:右)。ターンでなければ -1 */
static int TurnV_Lookup(int16_t code, int *dir) {
	switch (code) {
	case -4:  *dir = 0; return TV_BIG90;
	case -6:  *dir = 1; return TV_BIG90;
	case -5:  *dir = 0; return TV_BIG180;
	case -7:  *dir = 1; return TV_BIG180;
	case -51: *dir = 0; return TV_IN45;
	case -53: *dir = 1; return TV_IN45;
	case -52: *dir = 0; return TV_IN135;
	case -54: *dir = 1; return TV_IN135;
	case -61: *dir = 0; return TV_OUT45;
	case -63: *dir = 1; return TV_OUT45;
	case -62: *dir = 0; return TV_OUT135;
	case -64: *dir = 1; return TV_OUT135;
	case -65: *dir = 0; return TV_V90;
	case -66: *dir = 1; return TV_V90;
	default:  return -1;
	}
}

static float TurnV_Speed(int16_t code) {
	int dir;
	int k = TurnV_Lookup(code, &dir);
	return (k >= 0) ? TurnV_Table[k].v : TURNV_V_BASE;
}

static void TurnV_Run(int16_t code, float V) {
	int dir;
	int k = TurnV_Lookup(code, &dir);
	if (k < 0) {
		return;
	}
	const TurnV_Param *p = &TurnV_Table[k];
	float s = V / TURNV_V_BASE;
	float angle = (dir == 0) ? p->angle : -p->angle;

	if (p->st_kind == WC_NANAME) {
		Motor_Wallcut_ST_NANAME(V, p->offset_st, dir);
	} else {
		Motor_Wallcut_ST(V, p->offset_st, dir);
	}
	Motor_Sula_COS(V, angle, p->w_max * s, p->w_ac * s * s);
	if (p->end_kind == WC_NANAME) {
		Motor_Wallcut_END_NANAME(V, p->offset_end, dir);
	} else {
		Motor_Wallcut_END(V, p->offset_end, dir);
	}
}

void Short_NANAME_MoveTurnV(int MAX, int AC) {
	static SpeedPlan_t plan;

	Maze_Road();
	Maze_Wall_fill();
	G_Gool_X = MAZE_GOOL_X;
	G_Gool_Y = MAZE_GOOL_Y;
	G_Robot_MAZE_X = 0;
	G_Robot_MAZE_Y = 0;
	G_Robot_Direction = 0;
	G_MAZE_Explored[G_Gool_X][G_Gool_Y] = 0;
	Maze_Step_Calculate();

	Maze_Shortest_Calculation();

	Shortest_Pass_Compression();
	Shortest_Pass_Compression_NANAME();

	/* 最初の半区画はスタート区間に含める(Short_NANAME_Move2000と同じ) */
	int start_half = (G_Short_Pass_NANAME[0] == 1);
	if (start_half) {
		G_Short_Pass_NANAME[0] = -1;
	}

	plan.turn_v = TurnV_Speed;
	plan.v_max = MAX;
	plan.ac = AC;
	plan.v_start = TURNV_V_START;
	plan.v_goal = TURNV_V_GOAL;
	if (SpeedPlan_Make(G_Short_Pass_NANAME, &plan) != 0) {
		return;
	}

	Suction_Start(50);
	HAL_Delay(500);

	Motor_Setup();
	float v0 = plan.v_start_out;
	if (start_half) {
		Motor_trapezoid_PID(0, v0, v0, 30000, 90 + 24);
	} else {
		Motor_trapezoid_PID(0, v0, v0, 70000, 10);
	}

	for (int i = 0; G_Short_Pass_NANAME[i] != 0; i++) {
		if (Failsafe_Flag() == 1) {
			break;
		}
		int16_t code = G_Short_Pass_NANAME[i];
		switch (SpeedPlan_Kind(code)) {
		case SP_STRAIGHT:			//区間前進
			Motor_trapezoid_Asymmetric_PID(plan.v_in[i], MAX, plan.v_out[i],
					AC, 90 * code);
			break;
		case SP_DIAGONAL:			//斜め直線
			Motor_NANAME_PID(plan.v_in[i], MAX, plan.v_out[i], AC - 5000,
					127.3 * code / -50);
			break;
		case SP_TURN:
			TurnV_Run(code, plan.v_in[i]);
			break;
		default:
			break;
		}
	}
	if (Failsafe_Flag() == 0) {
		float vg = plan.v_goal_in;
		Motor_trapezoid_PID(vg, vg, 0, 15000, 180);
		Motor_Stop();
		Suction_Stop();

		Robot_adjustment();
		LED_Goal();
	} else {
		Failsafe_Flag_OFF();
	}
}
