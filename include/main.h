/**
 * @file      main.h
 * @author    George Andrew Brindeiro
 * @date      03/12/2011
 *
 * @attention Copyright (C) 2011
 * @attention Laboratório de Automação e Robótica (LARA)
 * @attention Departamento de Engenharia Elétrica (ENE)
 * @attention Universidade de Brasília (UnB)
 */

// Project Options
#define EXPERIMENT_TIME	300
#define SAMPLING_PERIOD	0.1
#define DATA_SAMPLES	2*(EXPERIMENT_TIME*SAMPLING_PERIOD)

// Aux print-outs
#define FATAL(x)		printf("\033[31m\033[1mFATAL:\033[0;0m\t%s\n",x)		
#define ERROR(x)		printf("\033[33m\033[1mERROR:\033[0;0m\t%s\n",x)
#define WARN(x)			printf("\033[32m\033[1mWARN:\033[0;0m\t%s\n",x)
#define INFO(x)			printf("\033[0;0m\033[1mINFO:\033[0;0m\t%s\n",x)
#define DEBUG(x)		printf("\033[36m\033[1mDEBUG:\033[0;0m\t%s\n",x)
#define VERBOSE(x)		printf("\033[34m\033[1mVERBOSE:\033[0;0m\t%s\n",x)

#define GPS_IMU(x)		printf("\033[0;0m\033[1m\n[GPS/IMU]\033[0;0m:\t%s\n\n",x)
#define PRINT_TIME(x)	printf("\033[0;0m\033[1m\nT = %lf s\033[0;0m\n",x)

int init_all();
int datalogger_init();
int datalogger_close();
int close_all();

void timer_start (void);
void timer_stop (void);
void timer_function (union sigval sigval);

int kbhit();
void sigint();
