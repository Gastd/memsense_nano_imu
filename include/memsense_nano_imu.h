/** ### MEMSense NanoIMU Specifications ###
 *
 *  Serial port:
 *
 *  (#) Speed                   115200
 *  (#) Parity                  None
 *  (#) Bits                    8
 *  (#) Stopbits                1
 *  (#) Flow control            None
 *
 *  Gyrometer:
 *
 *  (#) Dynamic Range           +-300 º/s
 *  (#) Offset                  +-1.5 º/s
 *  (#) Cross-axis sensitivity  +-1 %
 *  (#) Nonlinearity            +-0.1 % of FS (Best fit straight line)
 *  (#) Noise                   0.56 (max 0.95) º/s, sigma
 *  (#) Digital Sensitivity     DS_GYR
 *  (#) Bandwidth               50 Hz
 *
 *  Accelerometer:
 *
 *  (#) Dynamic Range           +-2 g
 *  (#) Offset                  +-30 mg
 *  (#) Nonlinearity            +-0.4 (max +-1.0) % of FS
 *  (#) Noise                   0.6 (max 0.8) mg, sigma
 *  (#) Digital Sensitivity     DS_ACC
 *  (#) Bandwidth               50 Hz
 *
 *  Magnetometer:
 *
 *  (#) Dynamic Range           +-1.9 gauss
 *  (#) Drift                   2700 ppm/ºC
 *  (#) Nonlinearity            +-0.5 % of FS (Best fit straight line)
 *  (#) Noise                   0.00056 (max 0.0015) gauss, sigma
 *  (#) Digital Sensitivity     DS_MAG
 *  (#) Bandwidth               50 Hz
 *
 *  Thermometer:
 *
 *  (#) Digital Sensitivity     DS_TMP
 */
// General
#include <errno.h>
#include <pthread.h>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <string>
#include <sys/stat.h>
#include <sys/time.h>
#include <stdexcept>
#include <exception>
#include <time.h>
#include <cmath>
#include <termios.h>
#include <vector>

// ROS
#include "ros/ros.h"
#include <ros/console.h>
#include "sensor_msgs/Imu.h"
#include "sensor_msgs/MagneticField.h"
#include <tf/transform_datatypes.h>

// Serial Port Headers (serialcom-termios)
#include "serialcom.h"
// #include "imu-bytes.h"

// namespace memsense_nano_imu {

    class Imu
    {
    public:
        double G;


        Imu();
        void init();
        void init(std::string&);
        void receiveDataFromImu(sensor_msgs::Imu&, sensor_msgs::MagneticField&);
        void close();
        ~Imu();
    private:
        // Sample byte order/format
        enum bytes
        {
            SYNC0, //       0
            SYNC1, //       1
            SYNC2, //       2
            SYNC3, //       3
            MSG_SIZE, //    4
            DEV_ID, //      5
            MSG_ID, //      6
            TIME_MSB, //    7
            TIME_LSB, //    8
            GYRX_MSB = 13,//13
            GYRX_LSB, //    14
            GYRY_MSB, //    15
            GYRY_LSB, //    16
            GYRZ_MSB, //    17
            GYRZ_LSB, //    18
            ACCX_MSB, //    19
            ACCX_LSB, //    20
            ACCY_MSB, //    21
            ACCY_LSB, //    22
            ACCZ_MSB, //    23
            ACCZ_LSB, //    24
            MAGX_MSB, //    25
            MAGX_LSB, //    26
            MAGY_MSB, //    27
            MAGY_LSB, //    28
            MAGZ_MSB, //    29
            MAGZ_LSB, //    30
            TMPX_MSB, //    31
            TMPX_LSB, //    32
            TMPY_MSB, //    33
            TMPY_LSB, //    34
            TMPZ_MSB, //    35
            TMPZ_LSB, //    36
            CHECKSUM  //    37
        };

        // Default values
        int D_SYNC;
        int D_MSG_SIZE;
        int D_DEV_ID;
        int D_MSG_ID;

        enum states
        {
            IMU_SYNC_ST,    //     0
            IMU_HEADER_ST,  //     1
            IMU_PAYLOAD_ST, //     2
            IMU_CHECKSUM_ST //     3
        };

        // Size of IMU data packets
        double IMU_PACKET_SIZE;

        // Digital sensitivy for each of the sensors
        double DS_GYR;
        double DS_ACC;
        double DS_MAG;

        // Thermometer in the gyros also have an offset
        double DS_TMP;
        double OS_TMP;

        // Time scale for 16-bit time in seconds/count
        double SC_TMR;

        // Serial port
        std::string imu_serial_port;
        int TIMEOUT_US;
        int BPS;
        int MAX_BYTES;
        
        //! Gravity (m/sec^2)

        double stamp;
        geometry_msgs::Vector3 gyro;
        geometry_msgs::Vector3 accel;
        geometry_msgs::Vector3 magnet;
        geometry_msgs::Vector3 thermo;
        std::vector<unsigned char> imu_data;

        SERIALPORTCONFIG imu_SerialPortConfig;

        void decode();
        int readDataFromImu();
        void throwSerialComException(int);
        int checksum(unsigned char);
    };

//} // memsense_nano_imu
