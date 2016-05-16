#ifndef MEMSENSE_NANO_IMU_H
#define MEMSENSE_NANO_IMU_H

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
#include <stdexcept>
#include <exception>
#include <cmath>
#include <vector>

// ROS
#include "ros/ros.h"
#include <ros/console.h>
#include "sensor_msgs/Imu.h"
#include "sensor_msgs/Temperature.h"
#include "sensor_msgs/MagneticField.h"
#include <tf/transform_datatypes.h>

// Serial Port Headers (serialcom-termios)
#include "serialcom.h"

namespace MEMSENSE_BYTES {
    // Sample byte order/format
    enum BYTES
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
} // MEMSENSE_BYTES

namespace MEMSENSE_STATES {
    enum STATES
    {
        IMU_SYNC_ST,    //     0
        IMU_HEADER_ST,  //     1
        IMU_PAYLOAD_ST, //     2
        IMU_CHECKSUM_ST //     3
    };
} // MEMSENSE_STATES


namespace memsense_nano_imu {

    class Imu
    {
    public:
        //! Gravity (m/sec^2)
        const double GRAVITY;


        Imu();
        void init();
        void init(std::string&);
        void receiveDataFromImu(sensor_msgs::Imu&, sensor_msgs::MagneticField&, sensor_msgs::Temperature&);
        void close();
        ~Imu();
    private:

        // Default values
        const int D_SYNC;
        const int D_MSG_SIZE;
        const int D_DEV_ID;
        const int D_MSG_ID;


        // Size of IMU data packets
        const double IMU_PACKET_SIZE;

        // Digital sensitivy for each of the sensors
        const double DS_GYR;
        const double DS_ACC;
        const double DS_MAG;

        // Thermometer in the gyros also have an offset
        const double DS_TMP;
        const double OS_TMP;

        // Time scale for 16-bit time in seconds/count
        const double SC_TMR;

        // Serial port
        std::string imu_serial_port;
        const int TIMEOUT_US;
        const int BPS;
        const int MAX_BYTES;
        

        double stamp;
        geometry_msgs::Vector3 gyro;
        geometry_msgs::Vector3 accel;
        geometry_msgs::Vector3 magnet;
        std::vector<double> thermo;
        std::vector<unsigned char> imu_data;

        SERIALPORTCONFIG imu_SerialPortConfig;

        void decode();
        int readDataFromImu();
        void throwSerialComException(int);
        int checksum(unsigned char);
    };

} // memsense_nano_imu

#endif // MEMSENSE_NANO_IMU_H
