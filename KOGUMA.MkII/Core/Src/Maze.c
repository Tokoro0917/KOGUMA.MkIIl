/*
 * Maze.c
 *
 *  Created on: 2023/08/18
 *      Author: akihi
 */

#include"Maze.h"
#include"stdio.h"
#include"Wallsensor.h"
#include"stdio.h"
#include "Define.h"

int G_Gool_X; /////////////////////////////////////////
int G_Gool_Y; /////////////////////////////////////////

int G_MAZE_Explored[MAZE_SIZE][MAZE_SIZE];
int i, j;

int G_Step_Map[MAZE_SIZE][MAZE_SIZE];
int Step_N = 0;

int Maze_Wall_Left;
int Maze_Wall_Right;
int Maze_Wall_Flont;

int G_Maze_Row[MAZE_SIZE + 1];
int G_Maze_Column[MAZE_SIZE + 1];

int Maze_Row_Look[MAZE_SIZE + 1];
int Maze_Column_Look[MAZE_SIZE + 1];

int G_Maze_Row_Save[MAZE_SIZE + 1];
int G_Maze_Column_Save[MAZE_SIZE + 1];

int Maze_Row_Look_Save[MAZE_SIZE + 1];
int Maze_Column_Look_Save[MAZE_SIZE + 1];

int G_Robot_Direction = 4; //0~3　前　右　後ろ　左　%4
int G_Robot_Lastaction = 5; //0~3 直進　右　後ろ　左

int G_Maze_Flont;
int G_Maze_Back;
int G_Maze_Left;
int G_Maze_Right;

int G_Robot_MAZE_X = 0;
int G_Robot_MAZE_Y = 0;

int G_Short_Pass[MAX_STEP];
int G_Short_Pass_CP[MAX_STEP];
int G_Short_Pass_NANAME[MAX_STEP];

int G_Known_Pass[MAX_STEP];
int Known_Pass_CP[MAX_STEP];

int NANAME_Flag = 0;
int Known_Flag = 0;

int ALL_MODE = 0;

void pushQueue_walk(QUEUE_T *queue, unsigned short input) {
	/* データをデータの最後尾の１つ後ろに格納*/
	queue->data[queue->tail] = input;
	/* データの最後尾を１つ後ろに移動*/
	queue->tail = queue->tail + 1;
	/* 巡回シフト*/
	if (queue->tail == MAX_QUEUE_NUM)
		queue->tail = 0;
	/* スタックが満杯なら何もせず関数終了*/
	if (queue->tail == queue->head) {
		//printf("stack_full\n");return;
	}
}

unsigned short popQueue_walk(QUEUE_T *queue) {
	unsigned short ret = 0;
	/* スタックが空なら何もせずに関数終了*/
	if (queue->tail == queue->head) {
		//printf("stack_empty\n");
		return 65535;
	}
	/* データの最前列からデータを取得*/
	ret = queue->data[queue->head];
	/* データの最前列を１つ前にずらす*/
	queue->head = queue->head + 1;
	/* 巡回シフト*/
	if (queue->head == MAX_QUEUE_NUM)
		queue->head = 0;
	/* 取得したデータを返却*/
	return ret;
}

void pushStack_walk(STACK_T *stack, unsigned short input) {
	/* データをデータの最後尾の１つ後ろに格納*/
	stack->data[stack->tail] = input;
	/* データの最後尾を１つ後ろに移動*/
	stack->tail = stack->tail + 1;
	/* 巡回シフト*/
	if (stack->tail == MAX_QUEUE_NUM)
		stack->tail = 0;
	/* スタックが満杯なら何もせず関数終了*/
	if (stack->tail == stack->head) {
		//printf("stack_full\n");return;
	}
}

unsigned short popStack_walk(STACK_T *stack) {
	unsigned short ret = 0;
	/* スタックが空なら何もせずに関数終了*/
	if (stack->tail == stack->head) {
		//printf("stack_empty\n");
		return 65535;
	}
	/* データの最前列からデータを取得*/
	ret = stack->data[stack->head];
	/* データの最前列を１つ前にずらす*/
	stack->head = stack->head + 1;
	/* 巡回シフト*/
	if (stack->head == MAX_QUEUE_NUM)
		stack->head = 0;
	/* 取得したデータを返却*/
	return ret;
}

typedef struct {
	int cost;
	int x;
	int y;
	int direction;
	int isRow;
	int isConfirm;
} NODE_T;

typedef struct {
	int head;
	int tail;
	NODE_T *data[MAX_QUEUE_NUM];
} Queue_T;

NODE_T node_Row[17][17];
NODE_T node_Column[17][17];

void pushQueue_walk_node(Queue_T *queue, NODE_T *input) {
	/* データをデータの最後尾の１つ後ろに格納*/
	queue->data[queue->tail] = input;
	/* データの最後尾を１つ後ろに移動*/
	queue->tail = queue->tail + 1;
	/* 巡回シフト*/
	if (queue->tail == MAX_QUEUE_NUM)
		queue->tail = 0;
	/* スタックが満杯なら何もせず関数終了*/
	if (queue->tail == queue->head) {
		//printf("queue_full\n");return;
	}
}

NODE_T* popqueue_walk_node(Queue_T *queue) {
	NODE_T *ret = NULL;
	/* スタックが空なら何もせずに関数終了*/
	if (queue->tail == queue->head) {
		//printf("queue_empty\n");
		// ret->cost=65535;
		return ret;
	}
	/* データの最前列からデータを取得*/
	ret = queue->data[queue->head];
	/* データの最前列を１つ前にずらす*/
	queue->head = queue->head + 1;
	/* 巡回シフト*/
	if (queue->head == MAX_QUEUE_NUM)
		queue->head = 0;
	/* 取得したデータを返却*/
	return ret;
}

void Maze_Initialization() {
	for (i = 0; i < MAZE_SIZE; i++) {
		for (j = 0; j < MAZE_SIZE; j++) {
			G_MAZE_Explored[i][j] = 0;
		}
	}
	G_MAZE_Explored[G_Robot_MAZE_X][G_Robot_MAZE_Y] = 1;
	for (i = 0; i < MAZE_SIZE; i++) {
		for (j = 0; j < MAZE_SIZE; j++) {
			G_Step_Map[i][j] = MAX_STEP;
		}
	}
	G_Step_Map[G_Gool_X][G_Gool_Y] = 0;
	for (i = 0; i < MAZE_SIZE + 1; i++) {
		G_Maze_Row[i] = 0;
		G_Maze_Column[i] = 0;
		Maze_Row_Look[i] = 0;
		Maze_Column_Look[i] = 0;
	}

	for (i = 0; i < MAX_STEP; i++) {
		G_Short_Pass[i] = 0;
	}

	G_Maze_Row[0] = 0b11111111111111111;
	G_Maze_Row[MAZE_SIZE] = 0b11111111111111111;
	G_Maze_Column[0] = 0b11111111111111111;
	G_Maze_Column[1] = 1;
	G_Maze_Column[MAZE_SIZE] = 0b11111111111111111;

	Maze_Row_Look[0] = 0b11111111111111111;
	Maze_Row_Look[1] = 1;
	Maze_Row_Look[MAZE_SIZE] = 0b11111111111111111;
	Maze_Column_Look[0] = 0b11111111111111111;
	Maze_Column_Look[1] = 1;
	Maze_Column_Look[MAZE_SIZE] = 0b11111111111111111;

	/*G_Maze_Row[0] = 0b1111111111111111111111111111111111;
	 G_Maze_Row[MAZE_SIZE] = 0b1111111111111111111111111111111111;
	 G_Maze_Column[0] = 0b1111111111111111111111111111111111;
	 G_Maze_Column[1] = 1;
	 G_Maze_Column[MAZE_SIZE] = 0b1111111111111111111111111111111111;

	 Maze_Row_Look[0] = 0b1111111111111111111111111111111111;
	 Maze_Row_Look[1] = 1;
	 Maze_Row_Look[MAZE_SIZE] = 0b1111111111111111111111111111111111;
	 Maze_Column_Look[0] = 0b1111111111111111111111111111111111;
	 Maze_Column_Look[1] = 1;
	 Maze_Column_Look[MAZE_SIZE] = 0b1111111111111111111111111111111111;*/

}

void Maze_Wall_Search(int X, int Y, int Direction) {
	Maze_Wall_Flont = 0;
	Maze_Wall_Left = 0;
	Maze_Wall_Right = 0;
	if (Direction == 0) {
		if ((G_Maze_Row[Y + 1] & (1 << X)) == (1 << X)) {
			Maze_Wall_Flont = 1;
		}
		if ((G_Maze_Column[X] & (1 << Y)) == (1 << Y)) {
			Maze_Wall_Left = 1;
		}
		if ((G_Maze_Column[X + 1] & (1 << Y)) == (1 << Y)) {
			Maze_Wall_Right = 1;
		}
	} else if (Direction == 1) {
		if ((G_Maze_Column[X + 1] & (1 << Y)) == (1 << Y)) {
			Maze_Wall_Flont = 1;
		}
		if ((G_Maze_Row[Y + 1] & (1 << X)) == (1 << X)) {
			Maze_Wall_Left = 1;
		}
		if ((G_Maze_Row[Y] & (1 << X)) == (1 << X)) {
			Maze_Wall_Right = 1;
		}
	} else if (Direction == 2) {
		if ((G_Maze_Row[Y] & (1 << X)) == (1 << X)) {
			Maze_Wall_Flont = 1;
		}
		if ((G_Maze_Column[X + 1] & (1 << Y)) == (1 << Y)) {
			Maze_Wall_Left = 1;
		}
		if ((G_Maze_Column[X] & (1 << Y)) == (1 << Y)) {
			Maze_Wall_Right = 1;
		}
	} else if (Direction == 3) {
		if ((G_Maze_Column[X] & (1 << Y)) == (1 << Y)) {
			Maze_Wall_Flont = 1;
		}
		if ((G_Maze_Row[Y] & (1 << X)) == (1 << X)) {
			Maze_Wall_Left = 1;
		}
		if ((G_Maze_Row[Y + 1] & (1 << X)) == (1 << X)) {
			Maze_Wall_Right = 1;
		}
	}

}

void Maze_Wall_Update() {
	if (G_Robot_Direction % 4 == 0) { //////////////////////////////////////////////////////////北に移動
		G_Robot_MAZE_X += 0;
		G_Robot_MAZE_Y += 1;
		if (Known_Flag == 0) {
			if (G_Wall_data[1] == 1) {					//左壁がある
				G_Maze_Column[G_Robot_MAZE_X] = G_Maze_Column[G_Robot_MAZE_X]
						| (1 << (G_Robot_MAZE_Y));
			}
			if (G_Wall_data[0] == 1 && G_Wall_data[3] == 1) {		//前壁がある//
				G_Maze_Row[G_Robot_MAZE_Y + 1] = G_Maze_Row[G_Robot_MAZE_Y + 1]
						| (1 << (G_Robot_MAZE_X));
			}
			if (G_Wall_data[2] == 1) {					//右壁がある
				G_Maze_Column[G_Robot_MAZE_X + 1] = G_Maze_Column[G_Robot_MAZE_X
						+ 1] | (1 << (G_Robot_MAZE_Y));
			}
			Maze_Column_Look[G_Robot_MAZE_X] = Maze_Column_Look[G_Robot_MAZE_X]
					| (1 << (G_Robot_MAZE_Y));
			Maze_Row_Look[G_Robot_MAZE_Y + 1] =
					Maze_Row_Look[G_Robot_MAZE_Y + 1] | (1 << (G_Robot_MAZE_X));
			Maze_Column_Look[G_Robot_MAZE_X + 1] =
					Maze_Column_Look[G_Robot_MAZE_X + 1]
							| (1 << (G_Robot_MAZE_Y));

			Maze_Step_Calculate();			//歩数マップ更新
		}
		G_Maze_Flont = G_Step_Map[G_Robot_MAZE_X][G_Robot_MAZE_Y + 1];
		G_Maze_Back = G_Step_Map[G_Robot_MAZE_X][G_Robot_MAZE_Y - 1];
		G_Maze_Left = G_Step_Map[G_Robot_MAZE_X - 1][G_Robot_MAZE_Y];
		G_Maze_Right = G_Step_Map[G_Robot_MAZE_X + 1][G_Robot_MAZE_Y];

		if ((G_Maze_Row[G_Robot_MAZE_Y + 1] & (1 << G_Robot_MAZE_X))
				== (1 << G_Robot_MAZE_X)) {
			G_Maze_Flont = MAX_STEP;
		}
		if ((G_Maze_Row[G_Robot_MAZE_Y] & (1 << G_Robot_MAZE_X))
				== (1 << G_Robot_MAZE_X)) {
			G_Maze_Back = MAX_STEP;
		}
		if ((G_Maze_Column[G_Robot_MAZE_X] & (1 << G_Robot_MAZE_Y))
				== (1 << G_Robot_MAZE_Y)) {
			G_Maze_Left = MAX_STEP;
		}
		if ((G_Maze_Column[G_Robot_MAZE_X + 1] & (1 << G_Robot_MAZE_Y))
				== (1 << G_Robot_MAZE_Y)) {
			G_Maze_Right = MAX_STEP;
		}

	} else if (G_Robot_Direction % 4 == 1) {//////////////////////////////////////////////////////////////東に移動
		G_Robot_MAZE_X += 1;
		G_Robot_MAZE_Y += 0;
		if (Known_Flag == 0) {
			if (G_Wall_data[1] == 1) {					//左壁がある
				G_Maze_Row[G_Robot_MAZE_Y + 1] = G_Maze_Row[G_Robot_MAZE_Y + 1]
						| (1 << (G_Robot_MAZE_X));
			}
			if (G_Wall_data[0] == 1 && G_Wall_data[3] == 1) {		//前壁がある
				G_Maze_Column[G_Robot_MAZE_X + 1] = G_Maze_Column[G_Robot_MAZE_X
						+ 1] | (1 << (G_Robot_MAZE_Y));
			}
			if (G_Wall_data[2] == 1) {					//右壁がある
				G_Maze_Row[G_Robot_MAZE_Y] = G_Maze_Row[G_Robot_MAZE_Y]
						| (1 << (G_Robot_MAZE_X));
			}
			Maze_Row_Look[G_Robot_MAZE_Y + 1] =
					Maze_Row_Look[G_Robot_MAZE_Y + 1] | (1 << (G_Robot_MAZE_X));
			Maze_Column_Look[G_Robot_MAZE_X + 1] =
					Maze_Column_Look[G_Robot_MAZE_X + 1]
							| (1 << (G_Robot_MAZE_Y));
			Maze_Row_Look[G_Robot_MAZE_Y] = Maze_Row_Look[G_Robot_MAZE_Y]
					| (1 << (G_Robot_MAZE_X));

			Maze_Step_Calculate();			//歩数マップ更新
		}
		G_Maze_Flont = G_Step_Map[G_Robot_MAZE_X + 1][G_Robot_MAZE_Y];
		G_Maze_Back = G_Step_Map[G_Robot_MAZE_X - 1][G_Robot_MAZE_Y];
		G_Maze_Left = G_Step_Map[G_Robot_MAZE_X][G_Robot_MAZE_Y + 1];
		G_Maze_Right = G_Step_Map[G_Robot_MAZE_X][G_Robot_MAZE_Y - 1];

		if ((G_Maze_Column[G_Robot_MAZE_X + 1] & (1 << G_Robot_MAZE_Y))
				== (1 << G_Robot_MAZE_Y)) {
			G_Maze_Flont = MAX_STEP;
		}
		if ((G_Maze_Column[G_Robot_MAZE_X] & (1 << G_Robot_MAZE_Y))
				== (1 << G_Robot_MAZE_Y)) {
			G_Maze_Back = MAX_STEP;
		}
		if ((G_Maze_Row[G_Robot_MAZE_Y + 1] & (1 << G_Robot_MAZE_X))
				== (1 << G_Robot_MAZE_X)) {
			G_Maze_Left = MAX_STEP;
		}
		if ((G_Maze_Row[G_Robot_MAZE_Y] & (1 << G_Robot_MAZE_X))
				== (1 << G_Robot_MAZE_X)) {
			G_Maze_Right = MAX_STEP;
		}
	} else if (G_Robot_Direction % 4 == 2) {//////////////////////////////////////////////南に移動
		G_Robot_MAZE_X += 0;
		G_Robot_MAZE_Y += -1;
		if (Known_Flag == 0) {
			if (G_Wall_data[1] == 1) {					//左壁がある
				G_Maze_Column[G_Robot_MAZE_X + 1] = G_Maze_Column[G_Robot_MAZE_X
						+ 1] | (1 << (G_Robot_MAZE_Y));
			}
			if (G_Wall_data[0] == 1 && G_Wall_data[3] == 1) {		//前壁がある
				G_Maze_Row[G_Robot_MAZE_Y] = G_Maze_Row[G_Robot_MAZE_Y]
						| (1 << (G_Robot_MAZE_X));
			}
			if (G_Wall_data[2] == 1) {					//右壁がある
				G_Maze_Column[G_Robot_MAZE_X] = G_Maze_Column[G_Robot_MAZE_X]
						| (1 << (G_Robot_MAZE_Y));
			}
			Maze_Column_Look[G_Robot_MAZE_X + 1] =
					Maze_Column_Look[G_Robot_MAZE_X + 1]
							| (1 << (G_Robot_MAZE_Y));
			Maze_Row_Look[G_Robot_MAZE_Y] = Maze_Row_Look[G_Robot_MAZE_Y]
					| (1 << (G_Robot_MAZE_X));
			Maze_Column_Look[G_Robot_MAZE_X] = Maze_Column_Look[G_Robot_MAZE_X]
					| (1 << (G_Robot_MAZE_Y));

			Maze_Step_Calculate();			//歩数マップ更新

		}
		G_Maze_Flont = G_Step_Map[G_Robot_MAZE_X][G_Robot_MAZE_Y - 1];
		G_Maze_Back = G_Step_Map[G_Robot_MAZE_X][G_Robot_MAZE_Y + 1];
		G_Maze_Left = G_Step_Map[G_Robot_MAZE_X + 1][G_Robot_MAZE_Y];
		G_Maze_Right = G_Step_Map[G_Robot_MAZE_X - 1][G_Robot_MAZE_Y];

		if ((G_Maze_Row[G_Robot_MAZE_Y] & (1 << G_Robot_MAZE_X))
				== (1 << G_Robot_MAZE_X)) {
			G_Maze_Flont = MAX_STEP;
		}
		if ((G_Maze_Row[G_Robot_MAZE_Y + 1] & (1 << G_Robot_MAZE_X))
				== (1 << G_Robot_MAZE_X)) {
			G_Maze_Back = MAX_STEP;
		}

		if ((G_Maze_Column[G_Robot_MAZE_X + 1] & (1 << G_Robot_MAZE_Y))
				== (1 << G_Robot_MAZE_Y)) {
			G_Maze_Left = MAX_STEP;
		}
		if ((G_Maze_Column[G_Robot_MAZE_X] & (1 << G_Robot_MAZE_Y))
				== (1 << G_Robot_MAZE_Y)) {
			G_Maze_Right = MAX_STEP;
		}

	} else {			///////////////////////////////////////////////////西に移動
		G_Robot_MAZE_X += -1;
		G_Robot_MAZE_Y += 0;
		if (Known_Flag == 0) {
			if (G_Wall_data[1] == 1) {					//左壁がある
				G_Maze_Row[G_Robot_MAZE_Y] = G_Maze_Row[G_Robot_MAZE_Y]
						| (1 << (G_Robot_MAZE_X));
			}
			if (G_Wall_data[0] == 1 && G_Wall_data[3] == 1) {		//前壁がある
				G_Maze_Column[G_Robot_MAZE_X] = G_Maze_Column[G_Robot_MAZE_X]
						| (1 << (G_Robot_MAZE_Y));
			}
			if (G_Wall_data[2] == 1) {					//右壁がある
				G_Maze_Row[G_Robot_MAZE_Y + 1] = G_Maze_Row[G_Robot_MAZE_Y + 1]
						| (1 << (G_Robot_MAZE_X));
			}
			Maze_Row_Look[G_Robot_MAZE_Y] = Maze_Row_Look[G_Robot_MAZE_Y]
					| (1 << (G_Robot_MAZE_X));
			Maze_Column_Look[G_Robot_MAZE_X] = Maze_Column_Look[G_Robot_MAZE_X]
					| (1 << (G_Robot_MAZE_Y));
			Maze_Row_Look[G_Robot_MAZE_Y + 1] =
					Maze_Row_Look[G_Robot_MAZE_Y + 1] | (1 << (G_Robot_MAZE_X));

			Maze_Step_Calculate();			//歩数マップ更新
		}
		G_Maze_Flont = G_Step_Map[G_Robot_MAZE_X - 1][G_Robot_MAZE_Y];
		G_Maze_Back = G_Step_Map[G_Robot_MAZE_X + 1][G_Robot_MAZE_Y];
		G_Maze_Left = G_Step_Map[G_Robot_MAZE_X][G_Robot_MAZE_Y - 1];
		G_Maze_Right = G_Step_Map[G_Robot_MAZE_X][G_Robot_MAZE_Y + 1];

		if ((G_Maze_Column[G_Robot_MAZE_X + 1] & (1 << G_Robot_MAZE_Y))
				== (1 << G_Robot_MAZE_Y)) {
			G_Maze_Back = MAX_STEP;
		}
		if ((G_Maze_Column[G_Robot_MAZE_X] & (1 << G_Robot_MAZE_Y))
				== (1 << G_Robot_MAZE_Y)) {
			G_Maze_Flont = MAX_STEP;
		}
		if ((G_Maze_Row[G_Robot_MAZE_Y + 1] & (1 << G_Robot_MAZE_X))
				== (1 << G_Robot_MAZE_X)) {
			G_Maze_Right = MAX_STEP;
		}
		if ((G_Maze_Row[G_Robot_MAZE_Y] & (1 << G_Robot_MAZE_X))
				== (1 << G_Robot_MAZE_X)) {
			G_Maze_Left = MAX_STEP;
		}
	}
}

void Known_Pass_Generation() {
	int N = 0;

	Known_Flag = 1;
	//Maze_Step_Calculate();
	for (int i = 0; i < MAX_STEP; i++) {
		G_Known_Pass[i] = 0;
		Known_Pass_CP[i] = 0;
	}
	G_Known_Pass[0] = -1;
	while (1) {
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
		if (G_MAZE_Explored[Known_X][Known_Y] == 0) {
			break;
		}
		Maze_Wall_Update();
		if ((G_Maze_Flont <= G_Maze_Left) && (G_Maze_Flont <= G_Maze_Right)	//前進
				&& (G_Maze_Flont <= G_Maze_Back)) {
			if (G_Known_Pass[N] < 0) {
				N++;
			}
			G_Known_Pass[N] += 2;
			G_Robot_Direction += 0;
		} else if ((G_Maze_Left <= G_Maze_Right)	//左折
		&& (G_Maze_Left <= G_Maze_Back)) {
			N++;
			G_Known_Pass[N] = -2;
			G_Robot_Direction += 3;
		} else if (G_Maze_Right <= G_Maze_Back) {	//	右折
			N++;
			G_Known_Pass[N] = -3;
			G_Robot_Direction += 1;
		} else {
			N++;
			G_Known_Pass[N] = -1;
		}

	}

	for (i = 0; G_Known_Pass[i] != 0; i++) {
		Known_Pass_CP[i] = G_Known_Pass[i];
	}
//	for (i = 0; Known_Pass_CP[i] != 0; i++) {
//		if (Known_Pass_CP[i] == -2) {
//			if (Known_Pass_CP[i - 1] > 0) {
//				if (Known_Pass_CP[i + 1] > 0) { //左９０おおまわり
//					Known_Pass_CP[i - 1] -= 1;
//					Known_Pass_CP[i] = -4;
//					Known_Pass_CP[i + 1] -= 1;
//				} else if ((Known_Pass_CP[i + 1] == -2)
//						&& (Known_Pass_CP[i + 2] > 0)) { //左１８０おおまわり
//					Known_Pass_CP[i - 1] -= 1;
//					Known_Pass_CP[i] = -5;
//					Known_Pass_CP[i + 1] = -1;
//					Known_Pass_CP[i + 2] -= 1;
//				}
//				if (Known_Pass_CP[i - 1] == 0) {
//					Known_Pass_CP[i - 1] = -1;
//				}
//				if (Known_Pass_CP[i + 1] == 0) {
//					Known_Pass_CP[i + 1] = -1;
//				}
//			}
//		} else if (Known_Pass_CP[i] == -3) {
//			if (Known_Pass_CP[i - 1] > 0) {
//				if (Known_Pass_CP[i + 1] > 0) { //右９０おおまわり
//					Known_Pass_CP[i - 1] -= 1;
//					Known_Pass_CP[i] = -6;
//					Known_Pass_CP[i + 1] -= 1;
//				} else if ((Known_Pass_CP[i + 1] == -3)
//						&& (Known_Pass_CP[i + 2] > 0)) { //右１８０おおまわり
//					Known_Pass_CP[i - 1] -= 1;
//					Known_Pass_CP[i] = -7;
//					Known_Pass_CP[i + 1] = -1;
//					Known_Pass_CP[i + 2] -= 1;
//				}
//				if (Known_Pass_CP[i - 1] == 0) {
//					Known_Pass_CP[i - 1] = -1;
//				}
//				if (Known_Pass_CP[i + 1] == 0) {
//					Known_Pass_CP[i + 1] = -1;
//				}
//			}
//		}
//
//	}

//	for (i = 0; Known_Pass_CP[i] != 0; i++) {
//		if ((Known_Pass_CP[i] == -2) || (Known_Pass_CP[i] == -3)) {
//			if (Known_Pass_CP[i - 1] > 0) { //斜め入り
//				if (Known_Pass_CP[i] == -2) { //左
//
//					if (Known_Pass_CP[i + 1] == -3) {
//						//入り４５
//						Known_Pass_CP[i - 1] -= 1;
//						Known_Pass_CP[i] = -51;
//						NANAME_Flag = 1;
//						Pass_zero_act();
//					} else if (Known_Pass_CP[i + 1] == -2) {
//						//入り135
//						Known_Pass_CP[i - 1] -= 1;
//						Known_Pass_CP[i] = -52;
//						Known_Pass_CP[i + 1] = -1;
//						//G_Short_Pass_NANAME[i + 2] = -1;
//						NANAME_Flag = 1;
//						Pass_zero_act();
//					}
//				} else if (Known_Pass_CP[i] == -3) { //右
//
//					if (Known_Pass_CP[i + 1] == -2) {
//						//入り４５
//						Known_Pass_CP[i - 1] -= 1;
//						Known_Pass_CP[i] = -53;
//						NANAME_Flag = 1;
//						Pass_zero_act();
//					} else if (Known_Pass_CP[i + 1] == -3) {
//						//入り135
//						Known_Pass_CP[i - 1] -= 1;
//						Known_Pass_CP[i] = -54;
//						Known_Pass_CP[i + 1] = -1;
//						//G_Short_Pass_NANAME[i + 2] = -1;
//						NANAME_Flag = 1;
//						Pass_zero_act();
//					}
//				}
//			} else if ((Known_Pass_CP[i + 1] >= 0) && (NANAME_Flag == 1)) { //斜め出 45
//				if (Known_Pass_CP[i] == -3) {
//					//右４５
//					//G_Short_Pass_NANAME[i - 1] = -1;
//					Known_Pass_CP[i] = -63;
//					Known_Pass_CP[i + 1] -= 1;
//					Pass_zero_act();
//				} else if (Known_Pass_CP[i] == -2) {
//					//左４５
//					//G_Short_Pass_NANAME[i - 1] = -1;
//					Known_Pass_CP[i] = -61;
//					Known_Pass_CP[i + 1] -= 1;
//					Pass_zero_act();
//				}
//				NANAME_Flag = 0;
//			} else if ((Known_Pass_CP[i + 2] >= 0) && (NANAME_Flag == 1)) {
//				if (Known_Pass_CP[i] == Known_Pass_CP[i + 1]) {
//					if (Known_Pass_CP[i] == -3) {
//						//右135
//						Known_Pass_CP[i] = -64;
//						Known_Pass_CP[i + 1] = -1;
//						Known_Pass_CP[i + 2] -= 1;
//						Pass_zero_act();
//					} else if (Known_Pass_CP[i] == -2) {
//						//左135
//						Known_Pass_CP[i] = -62;
//						Known_Pass_CP[i + 1] = -1;
//						Known_Pass_CP[i + 2] -= 1;
//						Pass_zero_act();
//					}
//					NANAME_Flag = 0;
//				} else {
//					if ((Known_Pass_CP[i] == -2) || (Known_Pass_CP[i] == -3)) {
//						if (Known_Pass_CP[i] == Known_Pass_CP[i + 1]) {
//							if (Known_Pass_CP[i] == -2) {
//								Known_Pass_CP[i] = -65;
//								Known_Pass_CP[i + 1] = -1;
//							} else {
//								Known_Pass_CP[i] = -66;
//								Known_Pass_CP[i + 1] = -1;
//							}
//
//						} else {
//							Known_Pass_CP[i] = -50;
//							//G_Short_Pass_NANAME[i + 1] = -1;
//						}
//					}
//				}
//			} else if (NANAME_Flag == 1) {
//				if ((Known_Pass_CP[i] == -2) || (Known_Pass_CP[i] == -3)) {
//					if (Known_Pass_CP[i] == Known_Pass_CP[i + 1]) {
//						if (Known_Pass_CP[i] == -2) {
//							Known_Pass_CP[i] = -65;
//							Known_Pass_CP[i + 1] = -1;
//						} else {
//							Known_Pass_CP[i] = -66;
//							Known_Pass_CP[i + 1] = -1;
//						}
//
//					} else {
//						Known_Pass_CP[i] = -50;
//						//G_Short_Pass_NANAME[i + 1] = -1;
//					}
//				}
//			}
//		}
//	}
//
//	for (i = 0; Known_Pass_CP[i] != 0; i++) {
//		if (Known_Pass_CP[i] == -50) {
//			for (int j = 1; Known_Pass_CP[i + j] == -50; j++) {
//				Known_Pass_CP[i + j] = -1;
//				Known_Pass_CP[i] -= 50;
//			}
//		}
//	}

	for (i = 0; G_Known_Pass[i] != 0; i++) {
		G_Known_Pass[i] = Known_Pass_CP[i];
	}
	Known_Flag = 0;
}

void Maze_Unkown_ALL_ModeSet() {
	ALL_MODE = 1;
}

int Maze_All_MODE_Check() {
	return ALL_MODE;
}

void Maze_Gool_Setting(int mode) {
	int N = 0;

	if (mode == 1) {
		for (int x = 0; x < MAZE_SIZE; x++) {
			for (int y = 0; y < MAZE_SIZE; y++) {
				if (G_MAZE_Explored[x][y] == 0) {

					if ((x == 0) && (y == 0)) {
						G_Step_Map[x][y] = 100;
						N = 0;
					} else {
						G_Step_Map[x][y] = 0;
						N++;
					}
				}
			}
		}
		if (N == 0) {
			ALL_MODE = 0;
		}
	} else {
		G_Step_Map[G_Gool_X][G_Gool_Y] = 0;
	}
}

void Maze_Unkown_ALL_ModeOFF() {
	ALL_MODE = 0;
}

void Maze_Step_Calculate() {
	QUEUE_T queue_x;
	QUEUE_T queue_y;
	queue_x.head = 0;
	queue_x.tail = 0;
	queue_y.head = 0;
	queue_y.tail = 0;
	unsigned short X;
	unsigned short Y;
	Step_N = 0;
	for (i = 0; i < MAZE_SIZE; i++) {
		for (j = 0; j < MAZE_SIZE; j++) {
			G_Step_Map[i][j] = MAX_STEP;
		}
	}
	Maze_Gool_Setting(ALL_MODE);
	pushQueue_walk(&queue_x, G_Gool_X);
	pushQueue_walk(&queue_y, G_Gool_Y);
	while (Step_N < MAX_STEP) {
		X = popQueue_walk(&queue_x);
		Y = popQueue_walk(&queue_y);
		if ((X == 65535) || (Y == 65535)) {
			break;
		}
		if (((G_Maze_Column[X + 1] & (1 << Y)) == 0)
				&& (G_Step_Map[X + 1][Y] == MAX_STEP)) { //右
			G_Step_Map[X + 1][Y] = G_Step_Map[X][Y] + 1;
			pushQueue_walk(&queue_x, X + 1);
			pushQueue_walk(&queue_y, Y);
		}
		if (((G_Maze_Column[X] & (1 << Y)) == 0)
				&& (G_Step_Map[X - 1][Y] == MAX_STEP)) { //左
			G_Step_Map[X - 1][Y] = G_Step_Map[X][Y] + 1;
			pushQueue_walk(&queue_x, X - 1);
			pushQueue_walk(&queue_y, Y);
		}
		if (((G_Maze_Row[Y + 1] & (1 << X)) == 0)
				&& (G_Step_Map[X][Y + 1] == MAX_STEP)) { //上
			G_Step_Map[X][Y + 1] = G_Step_Map[X][Y] + 1;
			pushQueue_walk(&queue_x, X);
			pushQueue_walk(&queue_y, Y + 1);
		}
		if (((G_Maze_Row[Y] & (1 << X)) == 0)
				&& (G_Step_Map[X][Y - 1] == MAX_STEP)) { //下
			G_Step_Map[X][Y - 1] = G_Step_Map[X][Y] + 1;
			pushQueue_walk(&queue_x, X);
			pushQueue_walk(&queue_y, Y - 1);
		}
		Step_N++;
	}
}

void Maze_Shortest_Calculation() {
	int Short_MAZE_X = 0;
	int Short_MAZE_Y = 0;
	int Step = 0;
	int N = 0;

	G_Step_Map[G_Gool_X][G_Gool_Y] = 0;
	G_Step_Map[G_Gool_X + 1][G_Gool_Y] = 0;
	G_Step_Map[G_Gool_X][G_Gool_Y + 1] = 0;
	G_Step_Map[G_Gool_X + 1][G_Gool_Y + 1] = 0;

	for (i = 0; i < MAX_STEP; i++) {
		G_Short_Pass[i] = 0;
	}
	G_Short_Pass[0] = -1;
	while ((G_MAZE_Explored[G_Gool_X][G_Gool_Y] == 0)
			|| (G_MAZE_Explored[G_Gool_X][G_Gool_Y] == 0)
			|| (G_MAZE_Explored[G_Gool_X][G_Gool_Y] == 0)
			|| (G_MAZE_Explored[G_Gool_X][G_Gool_Y] == 0)) {
		Step = G_Step_Map[Short_MAZE_X][Short_MAZE_Y];
		if (G_Robot_Direction % 4 == 0) { //北向き
			Short_MAZE_X += 0;
			Short_MAZE_Y += 1;

			if (G_Step_Map[Short_MAZE_X][Short_MAZE_Y] == 0) {
				break;
			}
			Maze_Wall_Search(Short_MAZE_X, Short_MAZE_Y, 0);
			while (1) {
				if (Maze_Wall_Flont == 0) {
					if (G_Step_Map[Short_MAZE_X][Short_MAZE_Y + 1]
							< G_Step_Map[Short_MAZE_X][Short_MAZE_Y]) { //前
						if (G_Short_Pass[N] < 0) {
							N++;
						}
						G_Short_Pass[N] += 2;
						G_Robot_Direction += 0;
						break;
					}
					printf("1____WHILE\n\r");
				}
				if (Maze_Wall_Left == 0) {
					if (G_Step_Map[Short_MAZE_X - 1][Short_MAZE_Y]
							< G_Step_Map[Short_MAZE_X][Short_MAZE_Y]) { //左
						N++;
						G_Short_Pass[N] = -2;
						G_Robot_Direction += 3;
						break;
					}
					printf("2____WHILE\n\r");
				}
				if (Maze_Wall_Right == 0) {
					if (G_Step_Map[Short_MAZE_X + 1][Short_MAZE_Y]
							< G_Step_Map[Short_MAZE_X][Short_MAZE_Y]) { //右
						N++;
						G_Short_Pass[N] = -3;
						G_Robot_Direction += 1;
						break;
					}
					printf("3____WHILE\n\r");
				}
			}

		} else if (G_Robot_Direction % 4 == 1) { //東向き
			Short_MAZE_X += 1;
			Short_MAZE_Y += 0;
			if (G_Step_Map[Short_MAZE_X][Short_MAZE_Y] == 0) {
				break;
			}
			Maze_Wall_Search(Short_MAZE_X, Short_MAZE_Y, 1);
			while (1) {
				if (Maze_Wall_Flont == 0) {
					if (G_Step_Map[Short_MAZE_X + 1][Short_MAZE_Y]
							< G_Step_Map[Short_MAZE_X][Short_MAZE_Y]) { //前
						if (G_Short_Pass[N] < 0) {
							N++;
						}
						G_Short_Pass[N] += 2;
						G_Robot_Direction += 0;
						break;
					}
					printf("4____WHILE\n\r");
				}
				if (Maze_Wall_Left == 0) {
					if (G_Step_Map[Short_MAZE_X][Short_MAZE_Y + 1]
							< G_Step_Map[Short_MAZE_X][Short_MAZE_Y]) { //左
						N++;
						G_Short_Pass[N] = -2;
						G_Robot_Direction += 3;
						break;
					}
					printf("5____WHILE\n\r");
				}
				if (Maze_Wall_Right == 0) {
					if (G_Step_Map[Short_MAZE_X][Short_MAZE_Y - 1]
							< G_Step_Map[Short_MAZE_X][Short_MAZE_Y]) { //右
						N++;
						G_Short_Pass[N] = -3;
						G_Robot_Direction += 1;
						break;
					}
					printf("6____WHILE\n\r");
				}

			}
		} else if (G_Robot_Direction % 4 == 2) { //南
			Short_MAZE_X += 0;
			Short_MAZE_Y += -1;
			if (G_Step_Map[Short_MAZE_X][Short_MAZE_Y] == 0) {
				break;
			}
			Maze_Wall_Search(Short_MAZE_X, Short_MAZE_Y, 2);
			while (1) {
				if (Maze_Wall_Flont == 0) {
					if (G_Step_Map[Short_MAZE_X][Short_MAZE_Y - 1]
							< G_Step_Map[Short_MAZE_X][Short_MAZE_Y]) { //前
						if (G_Short_Pass[N] < 0) {
							N++;
						}
						G_Short_Pass[N] += 2;
						G_Robot_Direction += 0;
						break;
					}
					printf("7____WHILE\n\r");
				}
				if (Maze_Wall_Left == 0) {
					if (G_Step_Map[Short_MAZE_X + 1][Short_MAZE_Y]
							< G_Step_Map[Short_MAZE_X][Short_MAZE_Y]) { //左
						N++;
						G_Short_Pass[N] = -2;
						G_Robot_Direction += 3;
						break;
					}
					printf("8____WHILE\n\r");
				}
				if (Maze_Wall_Right == 0) {
					if (G_Step_Map[Short_MAZE_X - 1][Short_MAZE_Y]
							< G_Step_Map[Short_MAZE_X][Short_MAZE_Y]) { //右
						N++;
						G_Short_Pass[N] = -3;
						G_Robot_Direction += 1;
						break;
					}
					printf("9____WHILE\n\r");
				}
			}
		} else if (G_Robot_Direction % 4 == 3) { //西
			Short_MAZE_X += -1;
			Short_MAZE_Y += 0;
			if (G_Step_Map[Short_MAZE_X][Short_MAZE_Y] == 0) {
				break;
			}
			Maze_Wall_Search(Short_MAZE_X, Short_MAZE_Y, 3);
			while (1) {
				if (Maze_Wall_Flont == 0) {
					if (G_Step_Map[Short_MAZE_X - 1][Short_MAZE_Y]
							< G_Step_Map[Short_MAZE_X][Short_MAZE_Y]) { //前
						if (G_Short_Pass[N] < 0) {
							N++;
						}
						G_Short_Pass[N] += 2;
						G_Robot_Direction += 0;
						break;
					}
					printf("10____WHILE\n\r");
				}
				if (Maze_Wall_Left == 0) {
					if (G_Step_Map[Short_MAZE_X][Short_MAZE_Y - 1]
							< G_Step_Map[Short_MAZE_X][Short_MAZE_Y]) { //左
						N++;
						G_Short_Pass[N] = -2;
						G_Robot_Direction += 3;
						break;
					}
					printf("11____WHILE\n\r");
				}
				if (Maze_Wall_Right == 0) {
					if (G_Step_Map[Short_MAZE_X][Short_MAZE_Y + 1]
							< G_Step_Map[Short_MAZE_X][Short_MAZE_Y]) { //右
						N++;
						G_Short_Pass[N] = -3;
						G_Robot_Direction += 1;
						break;
					}
					printf("12____WHILE\n\r");
				}
			}
		}
		G_MAZE_Explored[Short_MAZE_X][Short_MAZE_Y] = 1;
	}
}

void Maze_Dijkstra_Calculation() {
	Queue_T queue_node;
	queue_node.head = 0;
	queue_node.tail = 0;

	unsigned short Row_X;
	unsigned short Row_Y;
	unsigned short Column_X;
	unsigned short Column_Y;

	for (int i = 0; i < 17; i++) { //dijk 初期化
		for (int j = 0; j < 17; j++) {
			node_Row[i][j].cost = DIJK_MAXCOST;
			node_Row[i][j].isRow = 1;
			node_Row[i][j].isConfirm = 0;
			node_Row[i][j].x = i;
			node_Row[i][j].y = j;
			node_Column[i][j].cost = DIJK_MAXCOST;
			node_Column[i][j].isRow = 0;
			node_Column[i][j].isConfirm = 0;
			node_Column[i][j].x = i;
			node_Column[i][j].y = j;
		}
	}

	for (int i = 0; i < 17; i++) { //dijk 壁入れ
		for (int j = 0; j < 17; j++) {
			if ((G_Maze_Row[j] & (1 << i)) == (1 << i)) {
				node_Row[i][j].cost = DIJK_WALLCOST;
			}
			if ((G_Maze_Column[i] & (1 << j)) == (1 << j)) {
				node_Column[i][j].cost = DIJK_WALLCOST;
			}
		}
	}

	node_Row[G_Gool_X][G_Gool_Y + 1].cost = 0;

	pushQueue_walk_node(&queue_node, &node_Row[G_Gool_X][G_Gool_Y + 1]);

	while (1) {
		NODE_T *popNode;
		popNode = popqueue_walk_node(&queue_node);

		if (popNode == NULL) { //END
			break;
		}

		popNode->isConfirm = 1;

		if (popNode->isRow == 1) { //Row
			if (node_Row[popNode->x][popNode->y + 1].cost != DIJK_WALLCOST
					&& node_Row[popNode->x][popNode->y + 1].isConfirm != 1) { //no wall
				if (node_Row[popNode->x][popNode->y + 1].cost > popNode->cost) {
					if (popNode->direction == 0) {
						if (node_Row[popNode->x][popNode->y + 1].cost
								> popNode->cost + CON_COST) {
							node_Row[popNode->x][popNode->y + 1].cost =
									popNode->cost + CON_COST;
						}
					} else {
						if (node_Row[popNode->x][popNode->y + 1].cost
								> popNode->cost + ST_COST) {
							node_Row[popNode->x][popNode->y + 1].cost =
									popNode->cost + ST_COST;
						}
					}
					node_Row[popNode->x][popNode->y + 1].direction = 0; //N
					pushQueue_walk_node(&queue_node,
							&node_Row[popNode->x][popNode->y + 1]);
				}
			}
			if (node_Row[popNode->x][popNode->y - 1].cost != DIJK_WALLCOST
					&& node_Row[popNode->x][popNode->y - 1].isConfirm != 1) { //no wall
				if (node_Row[popNode->x][popNode->y - 1].cost > popNode->cost) {
					if (popNode->direction == 4) {
						if (node_Row[popNode->x][popNode->y - 1].cost
								> popNode->cost + CON_COST) {
							node_Row[popNode->x][popNode->y - 1].cost =
									popNode->cost + CON_COST;
						}
					} else {
						if (node_Row[popNode->x][popNode->y - 1].cost
								> popNode->cost + ST_COST) {
							node_Row[popNode->x][popNode->y - 1].cost =
									popNode->cost + ST_COST;
						}
					}
					node_Row[popNode->x][popNode->y - 1].direction = 4; //S
					pushQueue_walk_node(&queue_node,
							&node_Row[popNode->x][popNode->y - 1]);
				}
			}
			if (node_Column[popNode->x][popNode->y].cost != DIJK_WALLCOST
					&& node_Column[popNode->x][popNode->y].isConfirm != 1) { //no wall
				if (node_Column[popNode->x][popNode->y].cost > popNode->cost) {
					if (popNode->direction == 7) {
						if (node_Column[popNode->x][popNode->y].cost
								> popNode->cost + CON_COST) {
							node_Column[popNode->x][popNode->y].cost =
									popNode->cost + CON_COST;
						}
					} else {
						if (node_Column[popNode->x][popNode->y].cost
								> popNode->cost + DIAG_COST) {
							node_Column[popNode->x][popNode->y].cost =
									popNode->cost + DIAG_COST;
						}
					}
					node_Column[popNode->x][popNode->y].direction = 7; //NW
					pushQueue_walk_node(&queue_node,
							&node_Column[popNode->x][popNode->y]);
				}
			}
			if (node_Column[popNode->x + 1][popNode->y].cost != DIJK_WALLCOST
					&& node_Column[popNode->x + 1][popNode->y].isConfirm != 1) { //no wall
				if (node_Column[popNode->x + 1][popNode->y].cost
						> popNode->cost) {
					if (popNode->direction == 1) {
						if (node_Column[popNode->x + 1][popNode->y].cost
								> popNode->cost + CON_COST) {
							node_Column[popNode->x + 1][popNode->y].cost =
									popNode->cost + CON_COST;
						}
					} else {
						if (node_Column[popNode->x + 1][popNode->y].cost
								> popNode->cost + DIAG_COST) {
							node_Column[popNode->x + 1][popNode->y].cost =
									popNode->cost + DIAG_COST;
						}
					}
					node_Column[popNode->x + 1][popNode->y].direction = 1; //NE
					pushQueue_walk_node(&queue_node,
							&node_Column[popNode->x + 1][popNode->y]);
				}
			}
			if (node_Column[popNode->x][popNode->y - 1].cost != DIJK_WALLCOST
					&& node_Column[popNode->x][popNode->y - 1].isConfirm != 1) { //no wall
				if (node_Column[popNode->x][popNode->y - 1].cost
						> popNode->cost) {
					if (popNode->direction == 5) {
						if (node_Column[popNode->x][popNode->y - 1].cost
								> popNode->cost + CON_COST) {
							node_Column[popNode->x][popNode->y - 1].cost =
									popNode->cost + CON_COST;
						}
					} else {
						if (node_Column[popNode->x][popNode->y - 1].cost
								> popNode->cost + DIAG_COST) {
							node_Column[popNode->x][popNode->y - 1].cost =
									popNode->cost + DIAG_COST;
						}
					}
					node_Column[popNode->x][popNode->y - 1].direction = 5; //SW
					pushQueue_walk_node(&queue_node,
							&node_Column[popNode->x][popNode->y - 1]);
				}
			}
			if (node_Column[popNode->x + 1][popNode->y - 1].cost
					!= DIJK_WALLCOST
					&& node_Column[popNode->x + 1][popNode->y - 1].isConfirm
							!= 1) { //no wall
				if (node_Column[popNode->x + 1][popNode->y - 1].cost
						> popNode->cost) {
					if (popNode->direction == 3) {
						if (node_Column[popNode->x + 1][popNode->y - 1].cost
								> popNode->cost + CON_COST) {
							node_Column[popNode->x + 1][popNode->y - 1].cost =
									popNode->cost + CON_COST;
						}
					} else {
						if (node_Column[popNode->x + 1][popNode->y - 1].cost
								> popNode->cost + DIAG_COST) {
							node_Column[popNode->x + 1][popNode->y - 1].cost =
									popNode->cost + DIAG_COST;
						}
					}
					node_Column[popNode->x + 1][popNode->y - 1].direction = 3; //SE
					pushQueue_walk_node(&queue_node,
							&node_Column[popNode->x + 1][popNode->y - 1]);
				}
			}
		} else { //column
			if (node_Column[popNode->x + 1][popNode->y].cost != DIJK_WALLCOST
					&& node_Column[popNode->x + 1][popNode->y].isConfirm != 1) { //no wall
				if (node_Column[popNode->x + 1][popNode->y].cost
						> popNode->cost) {
					if (popNode->direction == 2) {
						if (node_Column[popNode->x + 1][popNode->y].cost
								> popNode->cost + CON_COST) {
							node_Column[popNode->x + 1][popNode->y].cost =
									popNode->cost + CON_COST;
						}
					} else {
						if (node_Column[popNode->x + 1][popNode->y].cost
								> popNode->cost + ST_COST) {
							node_Column[popNode->x + 1][popNode->y].cost =
									popNode->cost + ST_COST;
						}
					}
					node_Column[popNode->x + 1][popNode->y].direction = 2; //E
					pushQueue_walk_node(&queue_node,
							&node_Column[popNode->x + 1][popNode->y]);
				}
			}
			if (node_Column[popNode->x - 1][popNode->y].cost != DIJK_WALLCOST
					&& node_Column[popNode->x - 1][popNode->y].isConfirm != 1) { //no wall
				if (node_Column[popNode->x - 1][popNode->y].cost
						> popNode->cost) {
					if (popNode->direction == 6) {
						if (node_Column[popNode->x - 1][popNode->y].cost
								> popNode->cost + CON_COST) {
							node_Column[popNode->x - 1][popNode->y].cost =
									popNode->cost + CON_COST;
						}
					} else {
						if (node_Column[popNode->x - 1][popNode->y].cost
								> popNode->cost + ST_COST) {
							node_Column[popNode->x - 1][popNode->y].cost =
									popNode->cost + ST_COST;
						}
					}
					node_Column[popNode->x - 1][popNode->y].direction = 6; //E
					pushQueue_walk_node(&queue_node,
							&node_Column[popNode->x - 1][popNode->y]);
				}
			}
			if (node_Row[popNode->x][popNode->y + 1].cost != DIJK_WALLCOST
					&& node_Row[popNode->x][popNode->y + 1].isConfirm != 1) { //no wall
				if (node_Row[popNode->x][popNode->y + 1].cost > popNode->cost) {
					if (popNode->direction == 1) {
						if (node_Row[popNode->x][popNode->y + 1].cost
								> popNode->cost + CON_COST) {
							node_Row[popNode->x][popNode->y + 1].cost =
									popNode->cost + CON_COST;
						}
					} else {
						if (node_Row[popNode->x][popNode->y + 1].cost
								> popNode->cost + DIAG_COST) {
							node_Row[popNode->x][popNode->y + 1].cost =
									popNode->cost + DIAG_COST;
						}
					}
					node_Row[popNode->x][popNode->y + 1].direction = 1; //NE
					pushQueue_walk_node(&queue_node,
							&node_Row[popNode->x][popNode->y + 1]);
				}
			}
			if (node_Row[popNode->x][popNode->y].cost != DIJK_WALLCOST
					&& node_Row[popNode->x][popNode->y].isConfirm != 1) { //no wall
				if (node_Row[popNode->x][popNode->y].cost > popNode->cost) {
					if (popNode->direction == 3) {
						if (node_Row[popNode->x][popNode->y].cost
								> popNode->cost + CON_COST) {
							node_Row[popNode->x][popNode->y].cost =
									popNode->cost + CON_COST;
						}
					} else {
						if (node_Row[popNode->x][popNode->y].cost
								> popNode->cost + DIAG_COST) {
							node_Row[popNode->x][popNode->y].cost =
									popNode->cost + DIAG_COST;
						}
					}
					node_Row[popNode->x][popNode->y].direction = 3; //SE
					pushQueue_walk_node(&queue_node,
							&node_Row[popNode->x][popNode->y]);
				}
			}
			if (node_Row[popNode->x - 1][popNode->y + 1].cost != DIJK_WALLCOST
					&& node_Row[popNode->x - 1][popNode->y + 1].isConfirm
							!= 1) { //no wall
				if (node_Row[popNode->x - 1][popNode->y + 1].cost
						> popNode->cost) {
					if (popNode->direction == 7) {
						if (node_Row[popNode->x - 1][popNode->y + 1].cost
								> popNode->cost + CON_COST) {
							node_Row[popNode->x - 1][popNode->y + 1].cost =
									popNode->cost + CON_COST;
						}
					} else {
						if (node_Row[popNode->x - 1][popNode->y + 1].cost
								> popNode->cost + DIAG_COST) {
							node_Row[popNode->x - 1][popNode->y + 1].cost =
									popNode->cost + DIAG_COST;
						}
					}
					node_Row[popNode->x - 1][popNode->y + 1].direction = 7; //NW
					pushQueue_walk_node(&queue_node,
							&node_Row[popNode->x - 1][popNode->y + 1]);
				}
			}
			if (node_Row[popNode->x - 1][popNode->y].cost != DIJK_WALLCOST
					&& node_Row[popNode->x - 1][popNode->y].isConfirm != 1) { //no wall
				if (node_Row[popNode->x - 1][popNode->y].cost > popNode->cost) {
					if (popNode->direction == 5) {
						if (node_Row[popNode->x - 1][popNode->y].cost
								> popNode->cost + CON_COST) {
							node_Row[popNode->x - 1][popNode->y].cost =
									popNode->cost + CON_COST;
						}
					} else {
						if (node_Row[popNode->x - 1][popNode->y].cost
								> popNode->cost + DIAG_COST) {
							node_Row[popNode->x - 1][popNode->y].cost =
									popNode->cost + DIAG_COST;
						}
					}
					node_Row[popNode->x - 1][popNode->y].direction = 5; //SW
					pushQueue_walk_node(&queue_node,
							&node_Row[popNode->x - 1][popNode->y]);
				}
			}
		}

	}

	int toGool_direction = 0;
	int N = 0;

	NODE_T *short_node;
	short_node = &node_Row[0][1]; //&node_Row[0][1];
	toGool_direction = (short_node->direction + 4) % 8; //

	for (int i = 0; i < 255; i++) {
		G_Short_Pass[i] = 0;
	}


	G_Short_Pass[0] -= 1;
	//N++;


	while (1) {
		if (short_node->isRow == 1) {
			short_node = &node_Row[short_node->x][short_node->y];
		} else {
			short_node = &node_Column[short_node->x][short_node->y];
		}

		if (short_node->cost == 0) {
			break;
		}

		toGool_direction = (short_node->direction + 4) % 8;
		printf("%d direction\r\n", toGool_direction);
		if (toGool_direction == 0) {
			if (G_Short_Pass[N] < 0) {
				N++;
			}
			G_Short_Pass[N] += 2;
			short_node->y++;
		} else if (toGool_direction == 1) {
			N++;
			if (short_node->isRow == 1) {
				G_Short_Pass[N] = -3;
				short_node->x++;
				short_node->isRow = 0;
			} else {
				G_Short_Pass[N] = -2;
				short_node->y++;
				short_node->isRow = 1;
			}
		} else if (toGool_direction == 2) {
			if (G_Short_Pass[N] < 0) {
				N++;
			}
			G_Short_Pass[N] += 2;
			short_node->x++;
		} else if (toGool_direction == 3) {
			N++;
			if (short_node->isRow == 1) {
				G_Short_Pass[N] = -2;
				short_node->x++;
				short_node->y--;
				short_node->isRow = 0;
			} else {
				G_Short_Pass[N] = -3;
				short_node->isRow = 1;
			}
		} else if (toGool_direction == 4) {
			if (G_Short_Pass[N] < 0) {
				N++;
			}
			G_Short_Pass[N] += 2;
			short_node->y--;
		} else if (toGool_direction == 5) {
			N++;
			if (short_node->isRow == 1) {
				G_Short_Pass[N] = -3;
				short_node->y--;
				short_node->isRow = 0;
			} else {
				G_Short_Pass[N] = -2;
				short_node->x--;
				short_node->isRow = 1;
			}
		} else if (toGool_direction == 6) {
			if (G_Short_Pass[N] < 0) {
				N++;
			}
			G_Short_Pass[N] += 2;
			short_node->x--;
		} else if (toGool_direction == 7) {
			N++;
			if (short_node->isRow == 1) {
				G_Short_Pass[N] = -2;
				short_node->isRow = 0;
			} else {
				G_Short_Pass[N] = -3;
				short_node->x--;
				short_node->y++;
				short_node->isRow = 1;
			}
		}

	}

	if (G_Short_Pass[0] == 0) {
		G_Short_Pass[0] = -1;
	}
}

void Shortest_Pass_Compression() {
	for (i = 0; G_Short_Pass[i] != 0; i++) {
		G_Short_Pass_CP[i] = G_Short_Pass[i];
	}
	G_Short_Pass[i] = 1;
	G_Short_Pass_CP[i] = 1;
	G_Short_Pass_CP[0] = 1;
	for (i = 0; G_Short_Pass_CP[i] != 0; i++) {
		if (G_Short_Pass_CP[i] == -2) {
			if (G_Short_Pass_CP[i - 1] > 0) {
				if (G_Short_Pass[i + 1] > 0) { //左９０おおまわり
					G_Short_Pass_CP[i - 1] -= 1;
					G_Short_Pass_CP[i] = -4;
					G_Short_Pass_CP[i + 1] -= 1;
				} else if ((G_Short_Pass[i + 1] == -2)
						&& (G_Short_Pass[i + 2] > 0)) { //左１８０おおまわり
					G_Short_Pass_CP[i - 1] -= 1;
					G_Short_Pass_CP[i] = -5;
					G_Short_Pass_CP[i + 1] = -1;
					G_Short_Pass_CP[i + 2] -= 1;
				}
				if (G_Short_Pass_CP[i - 1] == 0) {
					G_Short_Pass_CP[i - 1] = -1;
				}
				if (G_Short_Pass_CP[i + 1] == 0) {
					G_Short_Pass_CP[i + 1] = -1;
				}
			}
		} else if (G_Short_Pass_CP[i] == -3) {
			if (G_Short_Pass_CP[i - 1] > 0) {
				if (G_Short_Pass_CP[i + 1] > 0) { //右９０おおまわり
					G_Short_Pass_CP[i - 1] -= 1;
					G_Short_Pass_CP[i] = -6;
					G_Short_Pass_CP[i + 1] -= 1;
				} else if ((G_Short_Pass[i + 1] == -3)
						&& (G_Short_Pass[i + 2] > 0)) { //右１８０おおまわり
					G_Short_Pass_CP[i - 1] -= 1;
					G_Short_Pass_CP[i] = -7;
					G_Short_Pass_CP[i + 1] = -1;
					G_Short_Pass_CP[i + 2] -= 1;
				}
				if (G_Short_Pass_CP[i - 1] == 0) {
					G_Short_Pass_CP[i - 1] = -1;
				}
				if (G_Short_Pass_CP[i + 1] == 0) {
					G_Short_Pass_CP[i + 1] = -1;
				}
			}
		}

	}

}

void Shortest_Pass_Compression_NANAME() {
	for (i = 0; G_Short_Pass_CP[i] != 0; i++) {
		G_Short_Pass_NANAME[i] = G_Short_Pass_CP[i];
	}
	for (i = 0; G_Short_Pass_NANAME[i] != 0; i++) {
		if ((G_Short_Pass_NANAME[i] == -2) || (G_Short_Pass_NANAME[i] == -3)) {
			if (G_Short_Pass_NANAME[i - 1] > 0) { //斜め入り
				if (G_Short_Pass_NANAME[i] == -2) { //左

					if (G_Short_Pass_NANAME[i + 1] == -3) {
						//入り４５
						G_Short_Pass_NANAME[i - 1] -= 1;
						G_Short_Pass_NANAME[i] = -51;
						NANAME_Flag = 1;
						Pass_zero_act();
					} else if (G_Short_Pass_NANAME[i + 1] == -2) {
						//入り135
						G_Short_Pass_NANAME[i - 1] -= 1;
						G_Short_Pass_NANAME[i] = -52;
						G_Short_Pass_NANAME[i + 1] = -1;
						//G_Short_Pass_NANAME[i + 2] = -1;
						NANAME_Flag = 1;
						Pass_zero_act();
					}
				} else if (G_Short_Pass_NANAME[i] == -3) { //右

					if (G_Short_Pass_NANAME[i + 1] == -2) {
						//入り４５
						G_Short_Pass_NANAME[i - 1] -= 1;
						G_Short_Pass_NANAME[i] = -53;
						NANAME_Flag = 1;
						Pass_zero_act();
					} else if (G_Short_Pass_NANAME[i + 1] == -3) {
						//入り135
						G_Short_Pass_NANAME[i - 1] -= 1;
						G_Short_Pass_NANAME[i] = -54;
						G_Short_Pass_NANAME[i + 1] = -1;
						//G_Short_Pass_NANAME[i + 2] = -1;
						NANAME_Flag = 1;
						Pass_zero_act();
					}
				}
			} else if ((G_Short_Pass_NANAME[i + 1] >= 0)
					&& (NANAME_Flag == 1)) { //斜め出 45
				if (G_Short_Pass_NANAME[i] == -3) {
					//右４５
					//G_Short_Pass_NANAME[i - 1] = -1;
					G_Short_Pass_NANAME[i] = -63;
					G_Short_Pass_NANAME[i + 1] -= 1;
					Pass_zero_act();
				} else if (G_Short_Pass_NANAME[i] == -2) {
					//左４５
					//G_Short_Pass_NANAME[i - 1] = -1;
					G_Short_Pass_NANAME[i] = -61;
					G_Short_Pass_NANAME[i + 1] -= 1;
					Pass_zero_act();
				}
				NANAME_Flag = 0;
			} else if ((G_Short_Pass_NANAME[i + 2] >= 0)
					&& (NANAME_Flag == 1)) {
				if (G_Short_Pass_NANAME[i] == G_Short_Pass_NANAME[i + 1]) {
					if (G_Short_Pass_NANAME[i] == -3) {
						//右135
						G_Short_Pass_NANAME[i] = -64;
						G_Short_Pass_NANAME[i + 1] = -1;
						G_Short_Pass_NANAME[i + 2] -= 1;
						Pass_zero_act();
					} else if (G_Short_Pass_NANAME[i] == -2) {
						//左135
						G_Short_Pass_NANAME[i] = -62;
						G_Short_Pass_NANAME[i + 1] = -1;
						G_Short_Pass_NANAME[i + 2] -= 1;
						Pass_zero_act();
					}
					NANAME_Flag = 0;
				} else {
					if ((G_Short_Pass_NANAME[i] == -2)
							|| (G_Short_Pass_NANAME[i] == -3)) {
						if (G_Short_Pass_NANAME[i]
								== G_Short_Pass_NANAME[i + 1]) {
							if (G_Short_Pass_NANAME[i] == -2) {
								G_Short_Pass_NANAME[i] = -65;
								G_Short_Pass_NANAME[i + 1] = -1;
							} else {
								G_Short_Pass_NANAME[i] = -66;
								G_Short_Pass_NANAME[i + 1] = -1;
							}

						} else {
							G_Short_Pass_NANAME[i] = -50;
							//G_Short_Pass_NANAME[i + 1] = -1;
						}
					}
				}
			} else if (NANAME_Flag == 1) {
				if ((G_Short_Pass_NANAME[i] == -2)
						|| (G_Short_Pass_NANAME[i] == -3)) {
					if (G_Short_Pass_NANAME[i] == G_Short_Pass_NANAME[i + 1]) {
						if (G_Short_Pass_NANAME[i] == -2) {
							G_Short_Pass_NANAME[i] = -65;
							G_Short_Pass_NANAME[i + 1] = -1;
						} else {
							G_Short_Pass_NANAME[i] = -66;
							G_Short_Pass_NANAME[i + 1] = -1;
						}

					} else {
						G_Short_Pass_NANAME[i] = -50;
						//G_Short_Pass_NANAME[i + 1] = -1;
					}
				}
			}
		}
	}
	for (i = 0; G_Short_Pass_NANAME[i] != 0; i++) {
		if (G_Short_Pass_NANAME[i] == -50) {
			for (int j = 1; G_Short_Pass_NANAME[i + j] == -50; j++) {
				G_Short_Pass_NANAME[i + j] = -1;
				G_Short_Pass_NANAME[i] -= 50;
			}
		}
	}

}

void Maze_Wall_fill() {
	int Maze_Row_NoLook[MAZE_SIZE + 1];
	int Maze_Column_NoLook[MAZE_SIZE + 1];
	for (i = 0; i < MAZE_SIZE; i++) {
		Maze_Row_NoLook[i] = ~Maze_Row_Look[i];
		G_Maze_Row[i] = G_Maze_Row[i] | Maze_Row_NoLook[i];

		Maze_Column_NoLook[i] = ~Maze_Column_Look[i];
		G_Maze_Column[i] = G_Maze_Column[i] | Maze_Column_NoLook[i];
	}
}

void Maze_Save() {
	for (i = 0; i < MAZE_SIZE + 1; i++) {
		G_Maze_Column_Save[i] = G_Maze_Column[i];
		G_Maze_Row_Save[i] = G_Maze_Row[i];
		Maze_Row_Look_Save[i] = Maze_Row_Look[i];
		Maze_Column_Look_Save[i] = Maze_Column_Look[i];
	}
}

void Maze_Road() {
	for (i = 0; i < MAZE_SIZE + 1; i++) {
		G_Maze_Column[i] = G_Maze_Column_Save[i];
		G_Maze_Row[i] = G_Maze_Row_Save[i];
		Maze_Row_Look[i] = Maze_Row_Look_Save[i];
		Maze_Column_Look[i] = Maze_Column_Look_Save[i];
	}
}
void Maze_Mapping() {
	for (j = MAZE_SIZE; j > 0; j--) {
		for (i = 0; i < MAZE_SIZE; i++) {
			printf("+");
			if ((G_Maze_Row_Save[j] & (1 << i)) == (1 << i)) {
				printf("---");
			} else {
				printf("   ");
			}
		}
		printf("+\n\r");
		for (i = 0; i < MAZE_SIZE + 1; i++) {
			if ((G_Maze_Column_Save[i] & (1 << (j - 1))) == (1 << (j - 1))) {
				printf("|");
			} else {
				printf(" ");
			}
			printf("%3d", G_Step_Map[i][j - 1]);
			//printf("%3d", G_MAZE_Explored[i][j - 1]);
		}
		printf("\n\r");
	}
	for (i = 0; i < MAZE_SIZE; i++) {
		printf("+");
		if ((G_Maze_Row[0] & (1 << i)) == (1 << i)) {
			printf("---");
		} else {
			printf("   ");
		}
	}
	printf("＋\n\r");
}

void Maze_Look_Mapping() {
	for (j = MAZE_SIZE; j > 0; j--) {
		for (i = 0; i < MAZE_SIZE; i++) {
			printf("+");
			if ((Maze_Row_Look[j] & (1 << i)) == (1 << i)) {
				printf("---");
			} else {
				printf("   ");
			}
		}
		printf("+\n\r");
		for (i = 0; i < MAZE_SIZE + 1; i++) {
			if ((Maze_Column_Look[i] & (1 << (j - 1))) == (1 << (j - 1))) {
				printf("|");
			} else {
				printf(" ");
			}
			printf("   ");
		}
		printf("\n\r");
	}
	for (i = 0; i < MAZE_SIZE; i++) {
		printf("+");
		if ((Maze_Row_Look[0] & (1 << i)) == (1 << i)) {
			printf("---");
		} else {
			printf("   ");
		}
	}
	printf("＋\n\r");					////かによし
}

void Maze_Dijkstra_Mapping() {
	for (int j = 16; j > 0; j--) {
		printf("   ");
		for (int i = 0; i < 16; i++) {
			printf("+");
			if ((G_Maze_Row[j] & (1 << i)) == (1 << i)) {
				printf("---");
			} else {
				printf("%3d",node_Row[i][j].cost);
			}
		}
		printf("+\n\r");
		for (int i = 0; i < 17; i++) {
			if ((G_Maze_Column[i] & (1 << (j - 1))) == (1 << (j - 1))) {
				printf("   |");
			} else {
				printf(" %3d",node_Column[i][j-1].cost);//
			}
		}
		printf("\n\r");
	}
	printf("   ");
	for (int i = 0; i < 16; i++) {
		printf("+");
		if ((G_Maze_Row[0] & (1 << i)) == (1 << i)) {
			printf("---");
		} else {
			printf("   ");
		}
	}
	printf("＋\n\r");
}

void Pass_zero_act() {
	if (G_Short_Pass_NANAME[i - 1] == 0) {
		G_Short_Pass_NANAME[i - 1] = -1;
	}
	if (G_Short_Pass_NANAME[i + 1] == 0) {
		G_Short_Pass_NANAME[i + 1] = -1;
	}
	if (G_Short_Pass_NANAME[i + 2] == 0) {
		G_Short_Pass_NANAME[i + 2] = -1;
	}

	if (Known_Pass_CP[i - 1] == 0) {
		Known_Pass_CP[i - 1] = -1;
	}
	if (Known_Pass_CP[i + 1] == 0) {
		Known_Pass_CP[i + 1] = -1;
	}
	if (Known_Pass_CP[i + 2] == 0) {
		Known_Pass_CP[i + 2] = -1;
	}
}

