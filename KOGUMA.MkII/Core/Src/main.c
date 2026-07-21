/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2025 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "dma.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "lsm6dsr.h"
#include "motor.h"
#include "PL_encoder.h"
#include "PL_timer.h"
#include "Motor.h"
#include "LOG.h"
#include "PL_sensor.h"
#include "UI.h"
#include "Wallsensor.h"
#include "Maze.h"
#include "Move.h"
#include "Failsafe.h"
#include "Define.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void) {

	/* USER CODE BEGIN 1 */

	/* USER CODE END 1 */

	/* MCU Configuration--------------------------------------------------------*/

	/* Reset of all peripherals, Initializes the Flash interface and the Systick. */
	HAL_Init();

	/* USER CODE BEGIN Init */

	/* USER CODE END Init */

	/* Configure the system clock */
	SystemClock_Config();

	/* USER CODE BEGIN SysInit */

	/* USER CODE END SysInit */

	/* Initialize all configured peripherals */
	MX_GPIO_Init();
	MX_DMA_Init();
	MX_ADC1_Init();
	MX_SPI1_Init();
	MX_SPI3_Init();
	MX_TIM2_Init();
	MX_TIM3_Init();
	MX_TIM4_Init();
	MX_USART2_UART_Init();
	MX_TIM6_Init();
	/* USER CODE BEGIN 2 */

	//HAL_TIM_Base_Start_IT(&htim6);
	pl_timer_init();
	lsm6dsr_init();
	Maze_Initialization();
	if (g_V_batt < LIMITBATT) {
		while (1) {
			LED_Reset();
			HAL_Delay(500);
			LED_ALL_ON();
			HAL_Delay(500);
		}
	}

	LED_Setup_Robot();

//
	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_4); //buzeer
//
//	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);

	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1) {
		/* USER CODE END WHILE */

		/* USER CODE BEGIN 3 */

		LED_program_number(Encorder_number_out());
		LED_program_mode(Encorder_mode_out());
		Maze_Unkown_ALL_ModeOFF();
		Motor_Setup_Voltage();
		if (Encorder_mode_out() == 0) {		//大会用
			if (Encorder_number_out() == 0) {		//
				/////////////////////////////
			} else if (Encorder_number_out() == 1) {		//探索
				if (Sensor_Enter() == 1) {
					Buzzer_Enter();
					Sensor_Start();

					Motor_Setup();
					Motor_Stop();

					G_Gool_X = MAZE_GOOL_X;
					G_Gool_Y = MAZE_GOOL_Y;
					G_Robot_MAZE_X = 0;
					G_Robot_MAZE_Y = 0;
					G_Robot_Direction = 0;
					G_MAZE_Explored[G_Gool_X][G_Gool_Y] = 0;

					Motor_trapezoid_PID(0, 500, 500, 5000, 24 + 90);
					while (G_MAZE_Explored[G_Gool_X][G_Gool_Y] == 0) {
						if (Failsafe_Flag() == 1) {
							break;
						}
						Robot_Maze_Sula_Action();
						//Robot_Maze_Pass_Action();
						G_MAZE_Explored[G_Robot_MAZE_X][G_Robot_MAZE_Y] = 1;
					}
					Maze_Save();

					Maze_Unkown_ALL_ModeSet();

					G_Gool_X = 0;
					G_Gool_Y = 0;
					G_MAZE_Explored[G_Gool_X][G_Gool_Y] = 0;
					Maze_Step_Calculate();
					while (G_MAZE_Explored[G_Gool_X][G_Gool_Y] == 0) {
						if (Failsafe_Flag() == 1) {
							break;
						}
						Robot_Maze_Sula_Action();
						//Robot_Maze_Pass_Action();
						G_MAZE_Explored[G_Robot_MAZE_X][G_Robot_MAZE_Y] = 1;
					}
					Motor_Stop();
					if (Failsafe_Flag() == 0) {
						Maze_Save();
					} else {
						Failsafe_Flag_OFF();
					}
					Motor_Free();
					LED_Reset();
					Encorder_count_reset();

				}
			} else if (Encorder_number_out() == 2) {		//
				if (Sensor_Enter() == 1) {
					Buzzer_Enter();
					Sensor_Start();

					Motor_Setup();
					Motor_Stop();

					G_Gool_X = MAZE_GOOL_X;
					G_Gool_Y = MAZE_GOOL_Y;
					G_Robot_MAZE_X = 0;
					G_Robot_MAZE_Y = 0;
					G_Robot_Direction = 0;
					G_MAZE_Explored[G_Gool_X][G_Gool_Y] = 0;

					Motor_trapezoid_PID(0, 500, 500, 5000, 24 + 90);
					while (G_MAZE_Explored[G_Gool_X][G_Gool_Y] == 0) {
						if (Failsafe_Flag() == 1) {
							break;
						}
						Robot_Maze_Sula_Action();
						//Robot_Maze_Pass_Action();
						G_MAZE_Explored[G_Robot_MAZE_X][G_Robot_MAZE_Y] = 1;
					}
					Maze_Save();

					//Maze_Unkown_ALL_ModeSet();

					G_Gool_X = 0;
					G_Gool_Y = 0;
					G_MAZE_Explored[G_Gool_X][G_Gool_Y] = 0;
					Maze_Step_Calculate();
					while (G_MAZE_Explored[G_Gool_X][G_Gool_Y] == 0) {
						if (Failsafe_Flag() == 1) {
							break;
						}
						Robot_Maze_Sula_Action();
						//Robot_Maze_Pass_Action();
						G_MAZE_Explored[G_Robot_MAZE_X][G_Robot_MAZE_Y] = 1;
					}
					Motor_Stop();

					if (Failsafe_Flag() == 0) {
						Maze_Save();
					} else {
						Failsafe_Flag_OFF();
					}

					Motor_Setup();
					Motor_Stop();
					if (Failsafe_Flag() == 0) {
						Maze_Save();
					} else {
						Failsafe_Flag_OFF();
					}
					Motor_Free();
					LED_Reset();
					Encorder_count_reset();

//					Short_NANAME_Move1000(5000, 15000);
//
//					Motor_Free();
//					LED_Reset();
//					Encorder_count_reset();
				}
			} else if (Encorder_number_out() == 3) {		//1m吸?��?
				if (Sensor_Enter() == 1) {
					Buzzer_Enter();
					Sensor_Start();

					Motor_Setup();
					Motor_Stop();
					Short_NANAME_Move1000(5000, 15000);
					Motor_Free();
					LED_Reset();
					HAL_Delay(500);
					Encorder_count_reset();

				}
			} else if (Encorder_number_out() == 4) {		//
				if (Sensor_Enter() == 1) {
					Buzzer_Enter();
					Sensor_Start();

					Motor_Setup();
					Motor_Stop();
					Short_NANAME_Move2000(5000, 20000);
					Motor_Free();
					LED_Reset();
					HAL_Delay(500);
					Encorder_count_reset();
				}
			} else if (Encorder_number_out() == 5) {		//
				if (Sensor_Enter() == 1) {
					Buzzer_Enter();
					Sensor_Start();

					Motor_Setup();
					Motor_Stop();
					Short_NANAME_Move2000(5000, 25000);
					//Short_Dijkstra_Move2000(5500, 20000);
					Motor_Free();
					LED_Reset();
					HAL_Delay(500);
					Encorder_count_reset();
				}
			} else if (Encorder_number_out() == 6) {		//
				if (Sensor_Enter() == 1) {
					Buzzer_Enter();
					Sensor_Start();

					Motor_Setup();
					Motor_Stop();
					Short_NANAME_Move2400(5000, 20000);
					Motor_Free();
					LED_Reset();
					HAL_Delay(500);
					Encorder_count_reset();
				}
			} else if (Encorder_number_out() == 7) {		//
				if (Sensor_Enter() == 1) {
					Buzzer_Enter();
					Sensor_Start();

					Motor_Setup();
					Motor_Stop();
					Short_NANAME_Move2400(5000, 30000);
					Motor_Free();
					LED_Reset();
					HAL_Delay(500);
					Encorder_count_reset();
				}
			}
		} else if (Encorder_mode_out() == 1) {				//基本調整
			if (Encorder_number_out() == 0) {		//センサ
				Wall_search_LED();
//				__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 50);
//				__HAL_TIM_SET_AUTORELOAD(&htim2, 100);
//				HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
//				printf("FL = %4d //", g_sensor_av[0]);
//				printf(" L = %4d //", g_sensor_av[2]);
//				printf(" R = %4d //", g_sensor_av[1]);
//				printf("FR = %4d //", g_sensor_av[3]);
				printf("FL = %4d //", g_sensor[0][0]);
				printf(" L = %4d //", g_sensor[2][0]);
				printf(" R = %4d //", g_sensor[1][0]);
				printf("FR = %4d //", g_sensor[3][0]);
				printf("BATT = %.2f//", g_V_batt);
				printf("GyroZ = %.3f\n\r", read_NoiseCut_gyro_z());
			} else if (Encorder_number_out() == 1) {		//直進
				if (Sensor_Enter() == 1) {
					Buzzer_Enter();
					Sensor_Start();
					///
					Motor_Setup();
					//Suction_Start(50);
					LOG_get_start();
//					Motor_trapezoid_PID(0, 2000, 2000, 60000, 24);
//					Motor_trapezoid_PID(2000, 2000, 0, 35000, 90);
					Motor_trapezoid_PID(0, 1000, 0, 8000, 180 * 15);
					Motor_Stop();
					Suction_Stop();
					LOG_get_end();
					///
					//Motor_trapezoid_PID(0, 6000, 0, 50000, 180 * 4);
					Encorder_count_reset();
				}
			} else if (Encorder_number_out() == 2) {		//??��?��?シン??��?��?
				if (Sensor_Enter() == 1) {
					Buzzer_Enter();
					Sensor_Start();
					///
					Motor_Setup();
//					for (int i = 0; i < 20; i++) {
//						Motor_trapezoid_Turn(90, 600, 20000);
//						Motor_Stop();
//					}
					Motor_trapezoid_Turn(3600, 600, 20000);
					///

					Encorder_count_reset();
				}
			} else if (Encorder_number_out() == 3) {		//スラロー??��?��?
				if (Sensor_Enter() == 1) {
					Buzzer_Enter();
					Sensor_Start();
					//
					Motor_Setup();
					Motor_trapezoid_PID(0, 500, 500, 5000, 90 + 180 + 29);
					Motor_Sula_COS(500, 90, 600, 10000);
					Motor_trapezoid(500, 500, 0, 5000, 180 + 20);
					Motor_Stop();
					//
					Encorder_count_reset();
				}
			} else if (Encorder_number_out() == 4) {		//斜め
				if (Sensor_Enter() == 1) {
					Buzzer_Enter();
					Sensor_Start();

					Motor_Setup();
					Motor_NANAME_PID(0, 2000, 0, 5000, 127.3 * 6);
					Motor_Stop();

					Encorder_count_reset();
				}
			} else if (Encorder_number_out() == 5) {		//吸??��?��?
				if (Sensor_Enter() == 1) {
					Buzzer_Enter();
					Sensor_Start();
					__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 90);
					__HAL_TIM_SET_AUTORELOAD(&htim2, 100);
					HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
					while (1) {
						if (Sensor_Enter() == 1) {
							break;
						}
					}
					HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_1);

				}
			} else if (Encorder_number_out() == 6) {		//壁合わせ
				if (Sensor_Enter() == 1) {
					Buzzer_Enter();
					Sensor_Start();

					Motor_Setup();
					Motor_Robot_Alignment();
					Motor_Stop();

					Encorder_count_reset();
				}
			} else if (Encorder_number_out() == 7) {		//
				if (Sensor_Enter() == 1) {
					Buzzer_Enter();
					Sensor_Start();

					LOG_print();
					HAL_Delay(1000);

				}
			}
		} else if (Encorder_mode_out() == 2) {			//1m??��?��?低吸??��?��?
			if (Encorder_number_out() == 1) {		//90
				if (Sensor_Enter() == 1) {
					Buzzer_Enter();
					Sensor_Start();

					Motor_Setup();
					Suction_Start(15);

					HAL_Delay(1000);
					LOG_get_start();
					Motor_trapezoid_PID(0, 1000, 1000, 15000, 90 + 90);
					//Motor_trapezoid_PID(1000, 1000, 1000, 15000, 50);
					Motor_Wallcut_ST(1000, 48, 0);
					Motor_Sula_COS(1000, 88, 600, 10000);
					Motor_trapezoid(1000, 1000, 1000, 15000, 35);
					Motor_trapezoid(1000, 1000, 0, 15000, 180);

					Motor_Stop();

					HAL_Delay(500);
					Suction_Stop();
					LOG_get_end();

					Encorder_count_reset();

				}
			} else if (Encorder_number_out() == 2) {		//180
				if (Sensor_Enter() == 1) {
					Buzzer_Enter();
					Sensor_Start();

					Motor_Setup();
					Suction_Start(15);

					HAL_Delay(1000);
					LOG_get_start();
					Motor_trapezoid_PID(0, 1000, 1000, 15000, 90 + 90);
					Motor_trapezoid_PID(1000, 1000, 1000, 15000, 60);
					//Motor_Wallcut_ST(1000, 60, 0);
					Motor_Sula_COS(1000, 179, 650, 13000);
					Motor_trapezoid(1000, 1000, 1000, 15000, 40);
					Motor_trapezoid(1000, 1000, 0, 15000, 180);

					Motor_Stop();
					HAL_Delay(500);
					Suction_Stop();
					LOG_get_end();

					Encorder_count_reset();
				}
			} else if (Encorder_number_out() == 3) {		//in45
				if (Sensor_Enter() == 1) {
					Buzzer_Enter();
					Sensor_Start();

					Motor_Setup();
					Suction_Start(15);

					HAL_Delay(1000);
					LOG_get_start();
					Motor_trapezoid_PID(0, 1000, 1000, 15000, 90 + 90);
					Motor_trapezoid_PID(1000, 1000, 1000, 15000, 15);
					Motor_Sula_COS(1000, 44, 750, 10000);
					Motor_trapezoid(1000, 1000, 1000, 15000, 42);
					Motor_trapezoid(1000, 1000, 0, 15000, 127.3);

					Motor_Stop();
					HAL_Delay(500);
					Suction_Stop();
					LOG_get_end();

					Encorder_count_reset();
				}
			} else if (Encorder_number_out() == 4) {		//in135
				if (Sensor_Enter() == 1) {
					Buzzer_Enter();
					Sensor_Start();

					Motor_Setup();
					Suction_Start(15);

					HAL_Delay(1000);
					LOG_get_start();
					Motor_trapezoid_PID(0, 1000, 1000, 15000, 90 + 90);
					Motor_trapezoid_PID(1000, 1000, 1000, 15000, 38);
					Motor_Sula_COS(1000, 134, 800, 10000);
					Motor_trapezoid(1000, 1000, 1000, 15000, 8);
					Motor_trapezoid(1000, 1000, 0, 15000, 127.3);

					Motor_Stop();
					HAL_Delay(500);
					Suction_Stop();
					LOG_get_end();

					Encorder_count_reset();
				}
			} else if (Encorder_number_out() == 5) {		//out45
				if (Sensor_Enter() == 1) {
					Buzzer_Enter();
					Sensor_Start();

					Motor_Setup();
					Suction_Start(15);

					HAL_Delay(1000);
					LOG_get_start();
					Motor_trapezoid(0, 1000, 1000, 10000, 127.3);
					Motor_trapezoid(1000, 1000, 1000, 5000, 73);
					Motor_Sula_COS(1000, 41.5, 700, 20000);
					Motor_trapezoid(1000, 1000, 1000, 5000, 30);
					Motor_trapezoid(1000, 1000, 0, 10000, 180);

					Motor_Stop();
					HAL_Delay(500);
					Suction_Stop();
					LOG_get_end();

					Encorder_count_reset();
				}
			} else if (Encorder_number_out() == 6) {		//out135
				if (Sensor_Enter() == 1) {
					Buzzer_Enter();
					Sensor_Start();

					Motor_Setup();
					Suction_Start(15);

					HAL_Delay(1000);
					LOG_get_start();
					Motor_trapezoid(0, 1000, 1000, 10000, 127.3);
					Motor_trapezoid(1000, 1000, 1000, 5000, 25);
					Motor_Sula_COS(1000, 132, 700, 14000);
					Motor_trapezoid(1000, 1000, 1000, 5000, 25);
					Motor_trapezoid(1000, 1000, 0, 10000, 180);

					Motor_Stop();
					HAL_Delay(500);
					Suction_Stop();
					LOG_get_end();

					Encorder_count_reset();
				}
			} else if (Encorder_number_out() == 7) {		//V90
				if (Sensor_Enter() == 1) {
					Buzzer_Enter();
					Sensor_Start();

					Motor_Setup();
					Suction_Start(15);

					HAL_Delay(1000);
					LOG_get_start();
					Motor_trapezoid(0, 1000, 1000, 10000, 127.3);
					Motor_trapezoid(1000, 1000, 1000, 5000, 37);
					Motor_Sula_COS(1000, 88.5, 800, 20000);
					Motor_trapezoid(1000, 1000, 1000, 5000, 27);
					Motor_trapezoid(1000, 1000, 0, 10000, 127.3);

					Motor_Stop();
					HAL_Delay(500);
					Suction_Stop();
					LOG_get_end();

					Encorder_count_reset();
				}
			} else if (Encorder_number_out() == 0) {		//
				if (Sensor_Enter() == 1) {
					Buzzer_Enter();
					Sensor_Start();
				}
			}
		} else if (Encorder_mode_out() == 3) {				//2m 吸??��?��?
			if (Encorder_number_out() == 0) {		//
				if (Sensor_Enter() == 1) {
					Buzzer_Enter();
					Sensor_Start();

					Motor_Setup();
					Suction_Start(50);

					HAL_Delay(1000);
					LOG_get_start();
					Motor_trapezoid_PID(0, 6000, 0, 40000, 180 * 6);

					Motor_Stop();

					HAL_Delay(500);
					Suction_Stop();
					LOG_get_end();

					Encorder_count_reset();
				}
			} else if (Encorder_number_out() == 1) {		//90
				if (Sensor_Enter() == 1) {
					Buzzer_Enter();
					Sensor_Start();

					Motor_Setup();
					Suction_Start(50);

					HAL_Delay(1000);
					LOG_get_start();
					Motor_trapezoid_PID(0, 2000, 2000, 20000, 90 + 90);
					Motor_trapezoid_PID(2000, 2000, 2000, 15000, 15);
					//Motor_Wallcut_ST(2000, 50, 0);
					Motor_Sula_COS(2000, 88.5, 1200, 30000);
					Motor_trapezoid(2000, 2000, 2000, 15000, 35);
					Motor_trapezoid(2000, 2000, 0, 20000, 180);

					Motor_Stop();

					HAL_Delay(500);
					Suction_Stop();
					LOG_get_end();

					Encorder_count_reset();
				}
			} else if (Encorder_number_out() == 2) {		//180
				if (Sensor_Enter() == 1) {
					Buzzer_Enter();
					Sensor_Start();

					Motor_Setup();
					Suction_Start(50);

					HAL_Delay(1000);
					LOG_get_start();
					Motor_trapezoid_PID(0, 2000, 2000, 20000, 90 + 90);
					Motor_trapezoid_PID(2000, 2000, 2000, 15000, 30);
					//Motor_Wallcut_ST(1000, 60, 0);
					Motor_Sula_COS(2000, 178, 1280, 30000);
					Motor_trapezoid(2000, 2000, 2000, 15000, 60);
					Motor_trapezoid(2000, 2000, 0, 20000, 180);

					Motor_Stop();

					HAL_Delay(500);
					Suction_Stop();
					LOG_get_end();

					Encorder_count_reset();
				}
			} else if (Encorder_number_out() == 3) {		//in45
				if (Sensor_Enter() == 1) {
					Buzzer_Enter();
					Sensor_Start();

					Motor_Setup();
					Suction_Start(50);

					HAL_Delay(1000);
					LOG_get_start();
					Motor_trapezoid_PID(0, 2000, 2000, 20000, 90 + 90);
					Motor_trapezoid_PID(2000, 2000, 2000, 15000, 8);
					//Motor_Wallcut_ST(1000, 60, 0);
					Motor_Sula_COS(2000, 43, 1600, 55000);
					Motor_trapezoid(2000, 2000, 2000, 15000, 55);
					Motor_trapezoid(2000, 2000, 0, 20000, 127.3);

					Motor_Stop();

					HAL_Delay(500);
					Suction_Stop();
					LOG_get_end();

					Encorder_count_reset();
				}
			} else if (Encorder_number_out() == 4) {		//in135
				if (Sensor_Enter() == 1) {
					Buzzer_Enter();
					Sensor_Start();

					Motor_Setup();
					Suction_Start(50);

					HAL_Delay(1000);
					LOG_get_start();
					Motor_trapezoid_PID(0, 2000, 2000, 20000, 90 + 90);
					Motor_trapezoid_PID(2000, 2000, 2000, 15000, 10);
					//Motor_Wallcut_ST(1000, 60, 0);
					Motor_Sula_COS(2000, 134, 1300, 50000);
					Motor_trapezoid(2000, 2000, 2000, 15000, 9);
					Motor_trapezoid(2000, 2000, 0, 20000, 127.3);

					Motor_Stop();

					HAL_Delay(500);
					Suction_Stop();
					LOG_get_end();

					Encorder_count_reset();
				}
			} else if (Encorder_number_out() == 5) {		//out45
				if (Sensor_Enter() == 1) {
					Buzzer_Enter();
					Sensor_Start();

					Motor_Setup();
					Suction_Start(50);

					HAL_Delay(1000);
					LOG_get_start();
					Motor_trapezoid(0, 2000, 2000, 20000, 127.3);
					Motor_trapezoid(2000, 2000, 2000, 15000, 22);
					//Motor_Wallcut_ST(1000, 60, 0);
					Motor_Sula_COS(2000, 42, 1500, 40000);
					Motor_trapezoid(2000, 2000, 2000, 15000, 21);
					Motor_trapezoid(2000, 2000, 0, 20000, 180);

					Motor_Stop();

					HAL_Delay(500);
					Suction_Stop();
					LOG_get_end();

					Encorder_count_reset();
				}
			} else if (Encorder_number_out() == 6) {		//out135
				if (Sensor_Enter() == 1) {
					Buzzer_Enter();
					Sensor_Start();

					Motor_Setup();
					Suction_Start(50);

					HAL_Delay(1000);
					LOG_get_start();
					Motor_trapezoid(0, 2000, 2000, 20000, 127.3);
					Motor_trapezoid(2000, 2000, 2000, 15000, 8);
					//Motor_Wallcut_ST(1000, 60, 0);
					Motor_Sula_COS(2000, 131.5, 1700, 50000);
					Motor_trapezoid(2000, 2000, 2000, 15000, 70);
					Motor_trapezoid(2000, 2000, 0, 20000, 180);

					Motor_Stop();

					HAL_Delay(500);
					Suction_Stop();
					LOG_get_end();

					Encorder_count_reset();
				}
			} else if (Encorder_number_out() == 7) {		//V90
				if (Sensor_Enter() == 1) {
					Buzzer_Enter();
					Sensor_Start();

					Motor_Setup();
					Suction_Start(50);

					HAL_Delay(1000);
					LOG_get_start();
					Motor_trapezoid(0, 2000, 2000, 20000, 127.3);
					Motor_trapezoid(2000, 2000, 2000, 15000, 7);
					//Motor_Wallcut_ST(1000, 60, 0);
					Motor_Sula_COS(2000, 84, 2000, 80000);
					Motor_trapezoid(2000, 2000, 2000, 15000, 35);
					Motor_trapezoid(2000, 2000, 0, 20000, 127.3);

					Motor_Stop();

					HAL_Delay(500);
					Suction_Stop();
					LOG_get_end();

					Encorder_count_reset();
				}
			}
		} else if (Encorder_mode_out() == 4) {				//2.4m吸??��?��?
			if (Encorder_number_out() == 0) {		//
				if (Sensor_Enter() == 1) {
					Buzzer_Enter();
					Sensor_Start();

				}
			} else if (Encorder_number_out() == 1) {		//90
				if (Sensor_Enter() == 1) {
					Buzzer_Enter();
					Sensor_Start();

					Motor_Setup();
					Suction_Start(70);

					HAL_Delay(1000);
					LOG_get_start();
					Motor_trapezoid_PID(0, 2400, 2400, 25000, 90 + 90);
					Motor_trapezoid_PID(2400, 2400, 2400, 15000, 24);
					//Motor_Wallcut_ST(1000, 60, 0);
					Motor_Sula_COS(2400, 87, 1700, 50000);
					Motor_trapezoid(2400, 2400, 2400, 15000, 56);
					Motor_trapezoid(2400, 2400, 0, 25000, 180);

					Motor_Stop();

					HAL_Delay(500);
					Suction_Stop();
					LOG_get_end();

					Encorder_count_reset();
				}
			} else if (Encorder_number_out() == 2) {		//180
				if (Sensor_Enter() == 1) {
					Buzzer_Enter();
					Sensor_Start();

					Motor_Setup();
					Suction_Start(70);

					HAL_Delay(1000);
					LOG_get_start();
					Motor_trapezoid_PID(0, 2400, 2400, 25000, 90 + 90);
					Motor_trapezoid_PID(2400, 2400, 2400, 15000, 15);
					//Motor_Wallcut_ST(1000, 60, 0);
					Motor_Sula_COS(2400, 177, 1550, 50300);
					Motor_trapezoid(2400, 2400, 2400, 15000, 47);
					Motor_trapezoid(2400, 2400, 0, 25000, 180);

					Motor_Stop();

					HAL_Delay(500);
					Suction_Stop();
					LOG_get_end();

					Encorder_count_reset();
				}
			} else if (Encorder_number_out() == 3) {		//in45
				if (Sensor_Enter() == 1) {
					Buzzer_Enter();
					Sensor_Start();

					Motor_Setup();
					Suction_Start(70);

					HAL_Delay(1000);
					LOG_get_start();
					Motor_trapezoid_PID(0, 2400, 2400, 25000, 90 + 90);
					Motor_trapezoid_PID(2400, 2400, 2400, 15000, 5);
					Motor_Sula_COS(2400, 42.5, 2050, 70000);
					Motor_trapezoid(2400, 2400, 2400, 15000, 41);
					Motor_trapezoid(2400, 2400, 0, 25000, 127.3);

					Motor_Stop();

					HAL_Delay(500);
					Suction_Stop();
					LOG_get_end();

					Encorder_count_reset();
				}
			} else if (Encorder_number_out() == 4) {		//in135
				if (Sensor_Enter() == 1) {
					Buzzer_Enter();
					Sensor_Start();

					Motor_Setup();
					Suction_Start(70);

					HAL_Delay(1000);
					LOG_get_start();
					Motor_trapezoid_PID(0, 2400, 2400, 25000, 90 + 90);
					Motor_trapezoid_PID(2400, 2400, 2400, 15000, 28);
					Motor_Sula_COS(2400, 130, 2200, 60000);
					Motor_trapezoid(2400, 2400, 2400, 15000, 33);
					Motor_trapezoid(2400, 2400, 0, 25000, 127.3);

					Motor_Stop();

					HAL_Delay(500);
					Suction_Stop();
					LOG_get_end();

					Encorder_count_reset();
				}
			} else if (Encorder_number_out() == 5) {		//out45
				if (Sensor_Enter() == 1) {
					Buzzer_Enter();
					Sensor_Start();

					Motor_Setup();
					Suction_Start(70);

					HAL_Delay(1000);
					LOG_get_start();

//					Motor_trapezoid(0, 2400, 2400, 25000, 127.3);
//					Motor_trapezoid(2400, 2400, 2400, 15000, 31);
//					Motor_Sula_COS(2400, 41, 1550, 60000);
//					Motor_trapezoid(2400, 2400, 2400, 15000, 18);
//					Motor_trapezoid(2400, 2400, 0, 25000, 180);

					Motor_trapezoid_PID(0, 2400, 2400, 25000, 90 + 90);
					Motor_trapezoid_PID(2400, 2400, 2400, 15000, 5);
					Motor_Sula_COS(2400, 42.5, 2050, 70000);
					Motor_trapezoid(2400, 2400, 2400, 15000, 41);
					Motor_NANAME_PID(2400, 2400, 2400, 30000, 127.3);

					Motor_trapezoid(2400, 2400, 2400, 15000, 35);
					Motor_Sula_COS(2400, 41, 1550, 60000);
					Motor_trapezoid(2400, 2400, 2400, 15000, 43);
					Motor_trapezoid(2400, 2400, 0, 25000, 180);

					Motor_Stop();

					HAL_Delay(500);
					Suction_Stop();
					LOG_get_end();

					Encorder_count_reset();
				}
			} else if (Encorder_number_out() == 6) {		//out135
				if (Sensor_Enter() == 1) {
					Buzzer_Enter();
					Sensor_Start();

					Motor_Setup();
					Suction_Start(70);

//					HAL_Delay(1000);
//					LOG_get_start();
//					Motor_NANAME_PID(0, 2400, 2400, 30000, 127.3);
//					Motor_NANAME_PID(2400, 2400, 2400, 15000, 20);
//					Motor_Sula_COS(2400, 123, 2300, 70000);
//					Motor_trapezoid(2400, 2400, 2400, 15000, 80);
//					Motor_trapezoid(2400, 2400, 0, 30000, 180);

					Motor_trapezoid_PID(0, 2400, 2400, 25000, 90 + 90);
					Motor_trapezoid_PID(2400, 2400, 2400, 15000, 5);
					Motor_Sula_COS(2400, 42.5, 2050, 70000);
					Motor_trapezoid(2400, 2400, 2400, 15000, 41);
					Motor_NANAME_PID(2400, 2400, 2400, 30000, 127.3);

					Motor_NANAME_PID(2400, 2400, 2400, 15000, 11);
					Motor_Sula_COS(2400, 125, 2300, 70000);
					Motor_trapezoid(2400, 2400, 2400, 15000, 80);
					Motor_trapezoid(2400, 2400, 0, 30000, 180);

					Motor_Stop();

					HAL_Delay(500);
					Suction_Stop();
					LOG_get_end();

					Encorder_count_reset();
				}
			} else if (Encorder_number_out() == 7) {		//v90
				if (Sensor_Enter() == 1) {
					Buzzer_Enter();
					Sensor_Start();

					Motor_Setup();
					Suction_Start(70);

					HAL_Delay(1000);
					LOG_get_start();
//					Motor_trapezoid(0, 2400, 2400, 40000, 127.3);
//					Motor_trapezoid(2400, 2400, 2400, 15000, 19);
//					Motor_Sula_COS(2400, 83, 2500, 100000);
//					Motor_trapezoid(2400, 2400, 2400, 15000, 29);
//					Motor_trapezoid(2400, 2400, 0, 25000, 127.3);

					Motor_trapezoid_PID(0, 2400, 2400, 25000, 90 + 90);
					Motor_trapezoid_PID(2400, 2400, 2400, 15000, 5);
					Motor_Sula_COS(2400, 42.5, 2050, 70000);
					Motor_trapezoid(2400, 2400, 2400, 15000, 41);
					Motor_NANAME_PID(2400, 2400, 2400, 30000, 127.3);

					Motor_trapezoid(2400, 2400, 2400, 15000, 17);
					Motor_Sula_COS(2400, 83, 2500, 100000);
					Motor_trapezoid(2400, 2400, 2400, 15000, 29);
					Motor_trapezoid(2400, 2400, 0, 25000, 127.3);

					Motor_Stop();

					HAL_Delay(500);
					Suction_Stop();
					LOG_get_end();

					Encorder_count_reset();
				}
			}
		} else if (Encorder_mode_out() == 5) {				//2.7m吸??��?��?
			if (Encorder_number_out() == 0) {		//
				if (Sensor_Enter() == 1) {
					Buzzer_Enter();
					Sensor_Start();
				}
			} else if (Encorder_number_out() == 1) {		//
				if (Sensor_Enter() == 1) {
					Buzzer_Enter();
					Sensor_Start();

					Motor_Setup();
					Suction_Start(80);

					HAL_Delay(1000);
					LOG_get_start();
					Motor_trapezoid_PID(0, 2700, 2700, 20000, 180 + 90 + 90);
					Motor_trapezoid_PID(2700, 2700, 2700, 15000, 18);
					//Motor_Wallcut_ST(2000, 50, 0);
					Motor_Sula_COS(2700, 88, 2300, 70000);
					Motor_trapezoid(2700, 2700, 2700, 15000, 73);
					Motor_trapezoid(2700, 2700, 0, 20000, 270);

					Motor_Stop();

					HAL_Delay(500);
					Suction_Stop();
					LOG_get_end();

					Encorder_count_reset();
				}
			} else if (Encorder_number_out() == 2) {		//
				if (Sensor_Enter() == 1) {
					Buzzer_Enter();
					Sensor_Start();
				}
			} else if (Encorder_number_out() == 3) {		//
				if (Sensor_Enter() == 1) {
					Buzzer_Enter();
					Sensor_Start();
				}
			} else if (Encorder_number_out() == 4) {		//
				if (Sensor_Enter() == 1) {
					Buzzer_Enter();
					Sensor_Start();
				}
			} else if (Encorder_number_out() == 5) {		//
				if (Sensor_Enter() == 1) {
					Buzzer_Enter();
					Sensor_Start();
				}
			} else if (Encorder_number_out() == 6) {		//
				if (Sensor_Enter() == 1) {
					Buzzer_Enter();
					Sensor_Start();
				}
			} else if (Encorder_number_out() == 7) {		//
				if (Sensor_Enter() == 1) {
					Buzzer_Enter();
					Sensor_Start();
				}
			}
		} else if (Encorder_mode_out() == 6) {				//3m吸??��?��?
			if (Encorder_number_out() == 0) {		//
				if (Sensor_Enter() == 1) {
					Buzzer_Enter();
					Sensor_Start();
				}
			} else if (Encorder_number_out() == 1) {		//
				if (Sensor_Enter() == 1) {
					Buzzer_Enter();
					Sensor_Start();

					Motor_Setup();
					Suction_Start(90);

					HAL_Delay(1000);
					LOG_get_start();
					Motor_trapezoid_PID(0, 3000, 3000, 20000, 180 + 90 + 90);
					Motor_trapezoid_PID(3000, 3000, 3000, 15000, 5);
					//Motor_Wallcut_ST(2000, 50, 0);
					Motor_Sula_COS(3000, 87.5, 2500, 80000);
					Motor_trapezoid(3000, 3000, 3000, 15000, 85);
					Motor_trapezoid(3000, 3000, 0, 25000, 270);

					Motor_Stop();

					HAL_Delay(500);
					Suction_Stop();
					LOG_get_end();

					Encorder_count_reset();
				}
			} else if (Encorder_number_out() == 2) {		//
				if (Sensor_Enter() == 1) {
					Buzzer_Enter();
					Sensor_Start();
				}
			} else if (Encorder_number_out() == 3) {		//
				if (Sensor_Enter() == 1) {
					Buzzer_Enter();
					Sensor_Start();
				}
			} else if (Encorder_number_out() == 4) {		//
				if (Sensor_Enter() == 1) {
					Buzzer_Enter();
					Sensor_Start();
				}
			} else if (Encorder_number_out() == 5) {		//
				if (Sensor_Enter() == 1) {
					Buzzer_Enter();
					Sensor_Start();
				}
			} else if (Encorder_number_out() == 6) {		//
				if (Sensor_Enter() == 1) {
					Buzzer_Enter();
					Sensor_Start();
				}
			} else if (Encorder_number_out() == 7) {		//
				if (Sensor_Enter() == 1) {
					Buzzer_Enter();
					Sensor_Start();
				}
			}
		} else if (Encorder_mode_out() == 7) {			//サーキ??��?��???��?��?
			if (Encorder_number_out() == 0) {		//
				if (Sensor_Enter() == 1) {
					Buzzer_Enter();
					Sensor_Start();

				}
			} else if (Encorder_number_out() == 1) {		//2000
				if (Sensor_Enter() == 1) {
					Buzzer_Enter();
					Sensor_Start();

					HAL_Delay(1000);
					Suction_Start(50);
					Motor_Setup();
					HAL_Delay(1000);

					Motor_trapezoid_PID(0, 2000, 2000, 20000, 24);
					Suction_change(30);
					Motor_trapezoid_PID(2000, 5000, 2000, 20000, 180 * 14);
					for (int i = 0; i < 7; i++) {
						Suction_change(50);
						Motor_Wallcut_ST(2000, 15, 1);
						Motor_Sula_COS(2000, -88.5, 1200, 30000);
						Motor_Wallcut_END(2000, 35, 1);
						Suction_change(30);

						Motor_trapezoid_PID(2000, 5000, 2000, 20000, 180 * 13);
					}
					Suction_change(50);
					Motor_Wallcut_ST(2000, 15, 1);
					Motor_Sula_COS(2000, -88.5, 1200, 30000);
					Motor_Wallcut_END(2000, 35, 1);
					Suction_change(30);
					Motor_trapezoid_PID(2000, 2000, 0, 20000, 180 * 3);
					Motor_Stop();
					Suction_Stop();
					Motor_Stop();
					Motor_Stop();
					HAL_Delay(1000);
				}
			} else if (Encorder_number_out() == 2) {		//2000
				if (Sensor_Enter() == 1) {
					Buzzer_Enter();
					Sensor_Start();

					HAL_Delay(1000);
					Suction_Start(50);
					Motor_Setup();
					HAL_Delay(1000);

					Motor_trapezoid_PID(0, 2000, 2000, 20000, 24);
					Suction_change(30);
					Motor_trapezoid_PID(2000, 5000, 2000, 30000, 180 * 14);
					for (int i = 0; i < 7; i++) {
						Suction_change(50);
						Motor_Wallcut_ST(2000, 15, 1);
						Motor_Sula_COS(2000, -88.5, 1200, 30000);
						Motor_Wallcut_END(2000, 35, 1);
						Suction_change(30);

						Motor_trapezoid_PID(2000, 5000, 2000, 30000, 180 * 13);
					}
					Suction_change(50);
					Motor_Wallcut_ST(2000, 15, 1);
					Motor_Sula_COS(2000, -88.5, 1200, 30000);
					Motor_Wallcut_END(2000, 35, 1);
					Suction_change(30);
					Motor_trapezoid_PID(2000, 2000, 0, 20000, 180 * 3);
					Motor_Stop();
					Suction_Stop();
					Motor_Stop();
					Motor_Stop();
					HAL_Delay(1000);
				}
			} else if (Encorder_number_out() == 3) {		//2400
				if (Sensor_Enter() == 1) {
					Buzzer_Enter();
					Sensor_Start();

					HAL_Delay(1000);
					Suction_Start(70);
					Motor_Setup();
					HAL_Delay(1000);

					Motor_trapezoid_PID(0, 1500, 1500, 20000, 24);
					Suction_change(40);
					Motor_trapezoid_PID(1500, 5500, 2400, 20000, 180 * 14);
					for (int i = 0; i < 7; i++) {
						Suction_change(70);
						Motor_Wallcut_ST(2400, 24, 1);
						Motor_Sula_COS(2400, -87, 1700, 50000);
						Motor_Wallcut_END(2400, 56, 1);
						Suction_change(40);

						Motor_trapezoid_PID(2400, 5500, 2400, 20000, 180 * 13);
					}
					Suction_change(70);
					Motor_Wallcut_ST(2400, 24, 1);
					Motor_Sula_COS(2400, -87, 1700, 50000);
					Motor_Wallcut_END(2400, 56, 1);
					Suction_change(30);
					Motor_trapezoid_PID(2400, 2400, 0, 20000, 180 * 3);
					Motor_Stop();
					Suction_Stop();
					Motor_Stop();
					Motor_Stop();
					HAL_Delay(1000);
				}
			} else if (Encorder_number_out() == 4) {		//2400
				if (Sensor_Enter() == 1) {
					Buzzer_Enter();
					Sensor_Start();

					HAL_Delay(1000);
					Suction_Start(70);
					Motor_Setup();
					HAL_Delay(1000);

					Motor_trapezoid_PID(0, 1500, 1500, 20000, 24);
					Suction_change(40);
					Motor_trapezoid_PID(1500, 5500, 2400, 25000, 180 * 14);
					for (int i = 0; i < 7; i++) {
						Suction_change(70);
						Motor_Wallcut_ST(2400, 24, 1);
						Motor_Sula_COS(2400, -87, 1700, 50000);
						Motor_Wallcut_END(2400, 56, 1);
						Suction_change(40);

						Motor_trapezoid_PID(2400, 5500, 2400, 25000, 180 * 13);
					}
					Suction_change(70);
					Motor_Wallcut_ST(2400, 24, 1);
					Motor_Sula_COS(2400, -87, 1700, 50000);
					Motor_Wallcut_END(2400, 56, 1);
					Suction_change(30);
					Motor_trapezoid_PID(2400, 2400, 0, 25000, 180 * 3);
					Motor_Stop();
					Suction_Stop();
					Motor_Stop();
					Motor_Stop();
					HAL_Delay(1000);
				}
			} else if (Encorder_number_out() == 5) {		//2700
				if (Sensor_Enter() == 1) {
					Buzzer_Enter();
					Sensor_Start();

					HAL_Delay(1000);
					Suction_Start(70);
					Motor_Setup();
					HAL_Delay(1000);

					Motor_trapezoid_PID(0, 1500, 1500, 20000, 24);
					Suction_change(50);
					Motor_trapezoid_PID(1500, 5500, 2700, 25000, 180 * 14);
					for (int i = 0; i < 7; i++) {
						Suction_change(80);
						Motor_Wallcut_ST(2700, 18, 1);
						Motor_Sula_COS(2700, -88, 2300, 70000);
						Motor_Wallcut_END(2700, 73, 1);
						Suction_change(50);

						Motor_trapezoid_PID(2700, 5500, 2700, 25000, 180 * 13);
					}
					Suction_change(80);
					Motor_Wallcut_ST(2700, 18, 1);
					Motor_Sula_COS(2700, -88, 2300, 70000);
					Motor_Wallcut_END(2700, 73, 1);
					Suction_change(50);
					Motor_trapezoid_PID(2700, 2700, 0, 25000, 180 * 3);
					Motor_Stop();
					Suction_Stop();
					Motor_Stop();
					Motor_Stop();
					HAL_Delay(1000);
				}
			} else if (Encorder_number_out() == 6) {		//3000
				if (Sensor_Enter() == 1) {
					Buzzer_Enter();
					Sensor_Start();

					HAL_Delay(1000);
					Suction_Start(90);
					Motor_Setup();
					HAL_Delay(1000);

					Motor_trapezoid_PID(0, 1500, 1500, 20000, 24);
					Suction_change(90);
					Motor_trapezoid_PID(1500, 5500, 3000, 25000, 180 * 14);
					for (int i = 0; i < 7; i++) {
						Suction_change(90);
						Motor_Wallcut_ST(3000, 5, 1);
						Motor_Sula_COS(3000, -87.5, 2500, 80000);
						Motor_Wallcut_END(3000, 85, 1);
						Suction_change(60);

						Motor_trapezoid_PID(3000, 5500, 3000, 25000, 180 * 13);
					}
					Suction_change(90);
					Motor_Wallcut_ST(3000, 5, 1);
					Motor_Sula_COS(3000, -87.5, 2500, 80000);
					Motor_Wallcut_END(3000, 85, 1);
					Suction_change(50);
					Motor_trapezoid_PID(3000, 3000, 0, 25000, 180 * 3);
					Motor_Stop();
					Suction_Stop();
					Motor_Stop();
					Motor_Stop();
					HAL_Delay(1000);
				}
			} else if (Encorder_number_out() == 7) {		//ALL 3000
				if (Sensor_Enter() == 1) {
					Buzzer_Enter();
					Sensor_Start();

					HAL_Delay(1000);
					Suction_Start(90);
					Motor_Setup();
					HAL_Delay(1000);

					Motor_trapezoid_PID(0, 1500, 1500, 20000, 24);
					Suction_change(90);
					Motor_trapezoid_PID(1500, 6000, 3000, 25000, 180 * 14);
					for (int i = 0; i < 7; i++) {
						Suction_change(90);
						Motor_Wallcut_ST(3000, 5, 1);
						Motor_Sula_COS(3000, -87.5, 2500, 80000);
						Motor_Wallcut_END(3000, 85, 1);
						Suction_change(60);

						Motor_trapezoid_PID(3000, 6000, 3000, 25000, 180 * 13);
					}
					Suction_change(90);
					Motor_Wallcut_ST(3000, 5, 1);
					Motor_Sula_COS(3000, -87.5, 2500, 80000);
					Motor_Wallcut_END(3000, 85, 1);
					Suction_change(50);
					Motor_trapezoid_PID(3000, 3000, 0, 25000, 180 * 3);
					Motor_Stop();
					Suction_Stop();
					Motor_Stop();
					Motor_Stop();
					HAL_Delay(1000);
				}
			}
		}
	}
	/* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void) {
	RCC_OscInitTypeDef RCC_OscInitStruct = { 0 };
	RCC_ClkInitTypeDef RCC_ClkInitStruct = { 0 };

	/** Configure the main internal regulator output voltage
	 */
	__HAL_RCC_PWR_CLK_ENABLE();
	__HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

	/** Initializes the RCC Oscillators according to the specified parameters
	 * in the RCC_OscInitTypeDef structure.
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
	RCC_OscInitStruct.HSEState = RCC_HSE_ON;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
	RCC_OscInitStruct.PLL.PLLM = 5;
	RCC_OscInitStruct.PLL.PLLN = 180;
	RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
	RCC_OscInitStruct.PLL.PLLQ = 2;
	RCC_OscInitStruct.PLL.PLLR = 2;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
		Error_Handler();
	}

	/** Activate the Over-Drive mode
	 */
	if (HAL_PWREx_EnableOverDrive() != HAL_OK) {
		Error_Handler();
	}

	/** Initializes the CPU, AHB and APB buses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
			| RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK) {
		Error_Handler();
	}
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void) {
	/* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
	__disable_irq();
	while (1) {
	}
	/* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
