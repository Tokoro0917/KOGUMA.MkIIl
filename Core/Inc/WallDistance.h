/*
 * WallDistance.h
 *
 *  壁センサ値 -> 距離[mm] 変換
 *
 *  距離の定義(キャリブレーション時もこの定義で測る):
 *    横センサ(L/R): 機体の中心線から横壁の表面までの距離
 *    前センサ(FL/FR): 機体の中心から前壁の表面までの距離
 *  どちらも区画中心に置いたとき 84mm (= (180 - 12) / 2) になる。
 *
 *  HALに依存しないので、ホスト(PC)のgccでもそのままテストできる。
 */

#ifndef INC_WALLDISTANCE_H_
#define INC_WALLDISTANCE_H_

/* センサ番号は g_sensor[][] と同じ並び */
#define WALLDIST_FL 0
#define WALLDIST_R  1
#define WALLDIST_L  2
#define WALLDIST_FR 3

/* 区画中心にいるときの壁までの距離[mm] */
#define WALLDIST_CENTER_MM 84.0f

/* センサ値 v を距離[mm]に変換する。
 * 表の範囲外は端の距離に張り付く(近すぎ -> 最小距離、遠すぎ -> 最大距離) */
float WallDist_mm(int sensor, int v);

#endif /* INC_WALLDISTANCE_H_ */
