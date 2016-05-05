/**
 * @file      imu-bytes.h
 * @author    George Andrew Brindeiro
 * @date      17/09/2011
 *
 * @attention Copyright (C) 2011
 * @attention Laboratório de Automação e Robótica (LARA)
 * @attention Departamento de Engenharia Elétrica (ENE)
 * @attention Universidade de Brasília (UnB)
 */

// Sample byte order/format
#define SYNC0		0
#define SYNC1		1
#define SYNC2		2
#define SYNC3		3
#define MSG_SIZE	4
#define DEV_ID		5
#define MSG_ID		6
#define TIME_MSB	7
#define TIME_LSB	8
#define GYRX_MSB	13
#define GYRX_LSB	14
#define GYRY_MSB	15
#define GYRY_LSB	16
#define GYRZ_MSB	17
#define GYRZ_LSB	18
#define ACCX_MSB	19
#define ACCX_LSB	20
#define ACCY_MSB	21
#define ACCY_LSB	22
#define ACCZ_MSB	23
#define ACCZ_LSB	24
#define MAGX_MSB	25
#define MAGX_LSB	26
#define MAGY_MSB	27
#define MAGY_LSB	28
#define MAGZ_MSB	29
#define MAGZ_LSB	30
#define TMPX_MSB	31
#define TMPX_LSB	32
#define TMPY_MSB	33
#define TMPY_LSB	34
#define TMPZ_MSB	35
#define TMPZ_LSB	36
#define CHECKSUM	37

// Default values
#define D_SYNC		0xFF
#define D_MSG_SIZE	0x26
#define D_DEV_ID	0xFF
#define D_MSG_ID	0x14
