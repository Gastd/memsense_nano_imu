/**
 * @file      imu.h
 * @author    George Andrew Brindeiro
 * @date      03/12/2011
 *
 * @attention Copyright (C) 2011
 * @attention Laboratório de Automação e Robótica (LARA)
 * @attention Departamento de Engenharia Elétrica (ENE)
 * @attention Universidade de Brasília (UnB)
 */

/* Definitions */

// Size of IMU data packets
#define IMU_PACKET_SIZE 38

// Digital sensitivy for each of the sensors
#define DS_GYR			1.3733E-2
#define DS_ACC			9.1553E-5
#define DS_MAG			8.6975E-5

// Thermometer in the gyros also have an offset
#define DS_TMP			1.8165E-2
#define OS_TMP			25

// Time scale for 16-bit time in seconds/count
#define SC_TMR			2.1701E-6

/* IMU data struct */
typedef struct imu
{
   double t;
   double g[3];
   double a[3];
   double m[3];
   double tmp[3];
} imu_t;

/* Function Prototypes */

int imu_init(char* serial_port);
int imu_get_data(imu_t* data);
void imu_decode(imu_t* data, unsigned char imu_data[]);
int imu_close();

int imu_checksum(unsigned char chksum, unsigned char imu_data[]);

void imu_print_raw(unsigned char imu_data[]);
void imu_print_formatted(imu_t* data);

/** ### MEMSense NanoIMU Specifications ###
 *
 *  Serial port:
 *
 *  (#)	Speed					115200
 *  (#) Parity					None
 * 	(#)	Bits					8
 * 	(#) Stopbits				1
 * 	(#) Flow control			None
 *
 *  Gyrometer:
 *
 * 	(#) Dynamic Range 			+-300 º/s
 * 	(#) Offset					+-1.5 º/s
 * 	(#) Cross-axis sensitivity	+-1 %
 * 	(#) Nonlinearity			+-0.1 % of FS (Best fit straight line)
 *  (#) Noise					0.56 (max 0.95) º/s, sigma
 * 	(#) Digital Sensitivity		DS_GYR
 * 	(#) Bandwidth				50 Hz
 *
 *	Accelerometer:
 *
 * 	(#) Dynamic Range 			+-2 g
 * 	(#) Offset					+-30 mg
 * 	(#) Nonlinearity			+-0.4 (max +-1.0) % of FS
 *  (#) Noise					0.6 (max 0.8) mg, sigma
 * 	(#) Digital Sensitivity		DS_ACC
 * 	(#) Bandwidth				50 Hz
 *
 *	Magnetometer:
 *
 * 	(#) Dynamic Range 			+-1.9 gauss
 * 	(#) Drift					2700 ppm/ºC
 * 	(#) Nonlinearity			+-0.5 % of FS (Best fit straight line)
 *  (#) Noise					0.00056 (max 0.0015) gauss, sigma
 * 	(#) Digital Sensitivity		DS_MAG
 * 	(#) Bandwidth				50 Hz
 *
 *  Thermometer:
 *
 * 	(#) Digital Sensitivity		DS_TMP
 */

