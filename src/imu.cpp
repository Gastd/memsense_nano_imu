/**
 * @file      imu.c
 * @author    George Andrew Brindeiro
 * @date      03/12/2011
 *
 * @attention Copyright (C) 2011
 * @attention Laboratório de Automação e Robótica (LARA)
 * @attention Departamento de Engenharia Elétrica (ENE)
 * @attention Universidade de Brasília (UnB)
 */

/* Includes */

// General
#include <math.h>
#include <pthread.h>
#include <signal.h>
#include <stdio.h>
#include <sys/time.h>
#include <time.h>

// ROS
 #include <ros/console.h>

// Serial Port Headers (serialcom-termios)
#include "serialcom.h"

// Project Headers
#include "main.h"
#include "imu.h"
#include "imu-bytes.h"

/* Definitions */

// IMU States
#define IMU_SYNC_ST			0
#define IMU_HEADER_ST		1
#define IMU_PAYLOAD_ST		2
#define IMU_CHECKSUM_ST		3

// Serial port
#define IMU_SERIAL_PORT	"/dev/ttyUSB1"
#define TIMEOUT_US		100000
#define BPS				115200
#define MAX_BYTES		100

SERIALPORTCONFIG imu_SerialPortConfig;

int imu_init(char* serial_port)
{
	int err;

	// Check whether a port was defined, and enforce default value IMU_SERIAL_PORT
	if(serial_port == NULL)
		serial_port = (char*)IMU_SERIAL_PORT;

	// Init serial port
	if((err = serialcom_init(&imu_SerialPortConfig, 1, serial_port, BPS)) != SERIALCOM_SUCCESS)
	{
		ROS_ERROR_STREAM("serialcom_init failed " << err);
		return 0;
	}

	return 1;
}

int imu_get_data(imu_t* data)
{
	int i;
	int err;
	int data_ready = 0;
	
	// State machine variables
	int b = 0, s = IMU_SYNC_ST;

	// Storage for data read from serial port
	unsigned char data_read;

	// IMU data packet
	unsigned char imu_data[IMU_PACKET_SIZE];

	// Flush port
	tcflush(imu_SerialPortConfig.fd, TCIOFLUSH);

	// Try to sync with IMU and get latest data packet, up to MAX_BYTES read until failure
	for(i = 0; (!data_ready)&&(i < MAX_BYTES); i++)
	{
		// Read data from serial port
		if((err = serialcom_receivebyte(&imu_SerialPortConfig, &data_read, TIMEOUT_US)) != SERIALCOM_SUCCESS)
		{
			ROS_ERROR_STREAM("serialcom_receivebyte failed " << err);
			continue;
		}

		// Parse IMU packet (User Guide, p.7)
		switch(s)
		{
			case IMU_SYNC_ST:
			{
				// State logic: Packet starts with 4 sync bytes with value 0xFF
				if(data_read == D_SYNC)
				{
					imu_data[b] = data_read;
					b++;
				}
				else
					// Out of sync, reset
					b = 0;

				// State transition: I have reached the MSG_SIZE byte without resetting
				if(b == MSG_SIZE)
					s = IMU_HEADER_ST;
			}
			break;

			case IMU_HEADER_ST:
			{
				// State logic: MSG_SIZE, DEV_ID and MSG_ID have default values
				if((b == MSG_SIZE) && (data_read == D_MSG_SIZE))
				{
					imu_data[b] = data_read;
					b++;
				}
				else if((b == DEV_ID) && (data_read == D_DEV_ID))
				{
					imu_data[b] = data_read;
					b++;
				}
				else if((b == MSG_ID) && (data_read == D_MSG_ID))
				{
					imu_data[b] = data_read;
					b++;
				}
				else
				{
					// Invalid MSG_SIZE, DEV_ID or MSG_ID, reset
					ROS_DEBUG("Invalid MSG_SIZE, DEV_ID or MSG_ID: is D_DEV_ID correctly set?");
					b = 0;
					s = IMU_SYNC_ST;
				}

				// State transition: I have reached the TIME_MSB byte without resetting
				if(b == TIME_MSB)
					s = IMU_PAYLOAD_ST;

			}
			break;

			case IMU_PAYLOAD_ST:
			{
				// State logic: Grab data until you reach the checksum byte
				imu_data[b] = data_read;
				b++;

				// State transition: I have reached the checksum byte
				if(b == CHECKSUM)
					s = IMU_CHECKSUM_ST;
			}
			break;

			case IMU_CHECKSUM_ST:
			{
				// State logic: If checksum is OK, grab data
				if(imu_checksum(data_read, imu_data))
				{
					imu_data[b] = data_read;
					imu_decode(data, imu_data);
					data_ready = 1;
				}

				// State transition: Unconditional reset
				b = 0;
				s = IMU_SYNC_ST;
			}
			break;
		}
	}

	return data_ready;
}

void imu_decode(imu_t* data, unsigned char imu_data[])
{
	data->t = SC_TMR*((signed short)((imu_data[TIME_MSB] << 8) | imu_data[TIME_LSB]));

	data->g[0] = DS_GYR*((short)((imu_data[GYRX_MSB] << 8) | imu_data[GYRX_LSB]));
	data->g[1] = DS_GYR*((short)((imu_data[GYRY_MSB] << 8) | imu_data[GYRY_LSB]));
	data->g[2] = DS_GYR*((short)((imu_data[GYRZ_MSB] << 8) | imu_data[GYRZ_LSB]));

	data->a[0] = DS_ACC*((short)((imu_data[ACCX_MSB] << 8) | imu_data[ACCX_LSB]));
	data->a[1] = DS_ACC*((short)((imu_data[ACCY_MSB] << 8) | imu_data[ACCY_LSB]));
	data->a[2] = DS_ACC*((short)((imu_data[ACCZ_MSB] << 8) | imu_data[ACCZ_LSB]));

	data->m[0] = DS_MAG*((short)((imu_data[MAGX_MSB] << 8) | imu_data[MAGX_LSB]));
	data->m[1] = DS_MAG*((short)((imu_data[MAGY_MSB] << 8) | imu_data[MAGY_LSB]));
	data->m[2] = DS_MAG*((short)((imu_data[MAGZ_MSB] << 8) | imu_data[MAGZ_LSB]));

	data->tmp[0] = DS_TMP*((short)((imu_data[TMPX_MSB] << 8) | imu_data[TMPX_LSB])) + 25;
	data->tmp[1] = DS_TMP*((short)((imu_data[TMPY_MSB] << 8) | imu_data[TMPY_LSB])) + 25;
	data->tmp[2] = DS_TMP*((short)((imu_data[TMPZ_MSB] << 8) | imu_data[TMPZ_LSB])) + 25;
}

int imu_close()
{
	int err;

	if((err = serialcom_close(&imu_SerialPortConfig)) != SERIALCOM_SUCCESS)
	{
		ROS_ERROR_STREAM("serialcom_close failed " << err);
		return 0;
	}

	return 1;
}

int imu_checksum(unsigned char chksum, unsigned char imu_data[])
{
	unsigned char i;
	unsigned char sum = 0;

	for(i = 0; i < D_MSG_SIZE-1; i++)
		sum += imu_data[i];

	if(sum != chksum)
	{
		ROS_DEBUG("sum != checksum (%d != %d)\n", sum, chksum);
		return 0;
	}
	else
		return 1;
}

void imu_print_raw(unsigned char imu_data[])
{
	int i;

	// ROS_INFO("IMU: ");
	for(i = 0; i < D_MSG_SIZE; i++)
	{
		ROS_INFO("%02X ", imu_data[i]);
	}
	// ROS_INFO("\n");
}

void imu_print_formatted(imu_t* data)
{
	ROS_INFO("t=%.6lf g(%.3lf %.3lf %.3lf) a(%.3lf %.3lf %.3lf) m(%.3lf %.3lf %.3lf) tmp(%.3lf %.3lf %.3lf)\n",
			data->t, data->g[0], data->g[1], data->g[2], data->a[0], data->a[1], data->a[2], data->m[0], data->m[1], data->m[2], data->tmp[0], data->tmp[1], data->tmp[2]);
}
