#include "memsense_nano_imu.h"

Imu::Imu()
{
    thermo.resize(3);
    D_SYNC     = 0xFF;
    D_MSG_SIZE = 0x26;
    D_DEV_ID   = 0xFF;
    D_MSG_ID   = 0x14;

    IMU_PACKET_SIZE = 38;

    DS_GYR = 1.3733E-2;
    DS_ACC = 9.1553E-5;
    DS_MAG = 8.6975E-5;

    DS_TMP = 1.8165E-2;
    OS_TMP = 25;

    SC_TMR = 2.1701E-6;

    imu_serial_port  = (char*)"/dev/ttyUSB0";
    TIMEOUT_US   =   100000;
    BPS          =   115200;
    MAX_BYTES    =   100;

    G = 9.80665;

    imu_data.resize(IMU_PACKET_SIZE);
}

Imu::~Imu()
{
    close();
}

void Imu::init()
{
    int err;

    // Init serial port
    if((err = serialcom_init(&imu_SerialPortConfig, 1, (char*)imu_serial_port.c_str(), BPS)) != SERIALCOM_SUCCESS)
    {
        ROS_ERROR_STREAM("serialcom_init failed " << err);
        throwSerialComException(err);
    }
}

void Imu::init(std::string& serial_port)
{
    int err;

    // Check whether a port was defined, and enforce default value imu_serial_port
    if(serial_port != std::string())
        imu_serial_port = serial_port;

    // Init serial port
    if((err = serialcom_init(&imu_SerialPortConfig, 1, (char*)serial_port.c_str(), BPS)) != SERIALCOM_SUCCESS)
    {
        ROS_ERROR_STREAM("serialcom_init failed " << err);
        throwSerialComException(err);
    }
}

int Imu::readDataFromImu()
{
    int i;
    int err;
    int data_ready = 0;
    
    // State machine variables
    int b = 0, s = IMU_SYNC_ST;

    // Storage for data read from serial port
    unsigned char data_read;

    // IMU data packet
    // unsigned char imu_data[IMU_PACKET_SIZE];

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
                    imu_data.at(b) = data_read;
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
                    imu_data.at(b) = data_read;
                    b++;
                }
                else if((b == DEV_ID) && (data_read == D_DEV_ID))
                {
                    imu_data.at(b) = data_read;
                    b++;
                }
                else if((b == MSG_ID) && (data_read == D_MSG_ID))
                {
                    imu_data.at(b) = data_read;
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
                imu_data.at(b) = data_read;
                b++;

                // State transition: I have reached the checksum byte
                if(b == CHECKSUM)
                    s = IMU_CHECKSUM_ST;
            }
            break;

            case IMU_CHECKSUM_ST:
            {
                // State logic: If checksum is OK, grab data
                if(checksum(data_read))
                {
                    imu_data.at(b) = data_read;
                    decode();
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

void Imu::decode()
{
    // data->t = SC_TMR*((signed short)((imu_data[TIME_MSB] << 8) | imu_data[TIME_LSB]));

    gyro.x = DS_GYR*((short)((imu_data.at(GYRX_MSB) << 8) | imu_data.at(GYRX_LSB)));
    gyro.y = DS_GYR*((short)((imu_data.at(GYRY_MSB) << 8) | imu_data.at(GYRY_LSB)));
    gyro.z = DS_GYR*((short)((imu_data.at(GYRZ_MSB) << 8) | imu_data.at(GYRZ_LSB)));

    accel.x = DS_ACC*((short)((imu_data.at(ACCX_MSB) << 8) | imu_data.at(ACCX_LSB)));
    accel.y = DS_ACC*((short)((imu_data.at(ACCY_MSB) << 8) | imu_data.at(ACCY_LSB)));
    accel.z = DS_ACC*((short)((imu_data.at(ACCZ_MSB) << 8) | imu_data.at(ACCZ_LSB)));

    magnet.x = DS_MAG*((short)((imu_data.at(MAGX_MSB) << 8) | imu_data.at(MAGX_LSB)));
    magnet.y = DS_MAG*((short)((imu_data.at(MAGY_MSB) << 8) | imu_data.at(MAGY_LSB)));
    magnet.z = DS_MAG*((short)((imu_data.at(MAGZ_MSB) << 8) | imu_data.at(MAGZ_LSB)));

    thermo.at(0) = DS_TMP*((short)((imu_data.at(TMPX_MSB) << 8) | imu_data.at(TMPX_LSB))) + 25;
    thermo.at(1) = DS_TMP*((short)((imu_data.at(TMPY_MSB) << 8) | imu_data.at(TMPY_LSB))) + 25;
    thermo.at(2) = DS_TMP*((short)((imu_data.at(TMPZ_MSB) << 8) | imu_data.at(TMPZ_LSB))) + 25;
}

void Imu::close()
{
    int err;

    if((err = serialcom_close(&imu_SerialPortConfig)) != SERIALCOM_SUCCESS)
    {
        ROS_ERROR_STREAM("serialcom_close failed " << err);
        throwSerialComException(err);
    }
}

void Imu::receiveDataFromImu(sensor_msgs::Imu& imu_data, sensor_msgs::MagneticField& mag_data, sensor_msgs::Temperature& temp_data)
{
    readDataFromImu();
    imu_data.orientation = geometry_msgs::Quaternion();

    imu_data.angular_velocity.x = gyro.x * M_PI / 180.0;
    imu_data.angular_velocity.y = gyro.y * M_PI / 180.0;
    imu_data.angular_velocity.z = gyro.z * M_PI / 180.0;


    imu_data.linear_acceleration.x = accel.x * G;
    imu_data.linear_acceleration.y = accel.y * G;
    imu_data.linear_acceleration.z = accel.z * G;

    mag_data.magnetic_field.x = magnet.x;
    mag_data.magnetic_field.y = magnet.y;
    mag_data.magnetic_field.z = magnet.z;

    double temp_mean = (thermo.at(0) + thermo.at(1) + thermo.at(2))/3.0;

    double temp_var;

    for (int i = 0; i < 3; ++i)
    {
        temp_var += (thermo.at(i) - temp_mean) * (thermo.at(i) - temp_mean);
    }
    temp_var /= 3.0;

    temp_data.temperature = temp_mean;
    temp_data.variance = temp_var;
}

void Imu::throwSerialComException(int err)
{
    switch(err)
    {
        case SERIALCOM_ERROR_IOPL:
        {
            throw std::runtime_error("serialcom error iopl");
            break;
        }
        case SERIALCOM_ERROR_MAXWAITENDOFTRANSMISSION:
        {
            throw std::runtime_error("serialcom error max wait end of transmission");
            break;
        }
        case SERIALCOM_ERROR_MAXWAITFORRECEPTION:
        {
            throw std::runtime_error("serialcom error max wait for reception");
            break;
        }
        case SERIALCOM_ERROR_MAXBPSPRECISION:
        {
            throw std::runtime_error("serialcom error max bps precision");
            break;
        }
        case SERIALCOM_ERROR_INCORRECTPORTNUMBER:
        {
            throw std::runtime_error("serialcom error incorrect port number");
            break;
        }
        case SERIALCOM_ERROR_INVALIDBAUDRATE:
        {
            throw std::runtime_error("serialcom error invalid baudrate");
            break;
        }
        case SERIALCOM_ERROR_INVALIDDEVICE:
        {
            throw std::runtime_error("serialcom error invalid device");
            break;
        }
    }
}

int Imu::checksum(unsigned char chksum)
{
    unsigned char sum = 0;

    for(unsigned char i = 0; i < D_MSG_SIZE-1; i++)
        sum += imu_data.at(i);

    if(sum != chksum)
    {
        ROS_DEBUG("sum != checksum (%d != %d)\n", sum, chksum);
        return 0;
    }
    else
        return 1;
}
