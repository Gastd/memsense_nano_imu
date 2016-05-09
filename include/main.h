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

int init_all();
int close_all();
void sigint();
