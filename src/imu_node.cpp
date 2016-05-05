// General
#include <errno.h>
#include <pthread.h>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/stat.h>
#include <sys/time.h>
#include <time.h>
#include <cmath>
#include <termios.h>

// ROS
#include "ros/ros.h"
#include "sensor_msgs/Imu.h"
#include "sensor_msgs/MagneticField.h"
#include <tf/transform_datatypes.h>

// Serial Port Headers (serialcom-termios)
#include "serialcom.h"

// Project Headers
#include "main.h"
#include "imu.h"
// pthread period function
#define TASK_PERIOD_US  (SAMPLING_PERIOD*1000000)
#define PI 3.14159265
#define G 9.807 // m/s^2

// int gps_ok = 0;
int imu_ok = 0;

// gps_t gps_values;
imu_t imu_values;

void setup_sig_handler();
void sig_handler(int sig);

int main(int argc, char** argv)
{
    ros::init(argc, argv, "imu_node");
    
    ros::NodeHandle n;

    ros::Publisher imu_pub = n.advertise<sensor_msgs::Imu>("imu/data_raw", 1000);
    ros::Publisher imu_mag_pub = n.advertise<sensor_msgs::MagneticField>("imu/mag", 1000);

    sensor_msgs::Imu data;
    sensor_msgs::MagneticField mag;
    ros::Rate loop_rate(50);
    int seq = 0;

    setup_sig_handler();

    // Initialize GPS/IMU
    if(!init_all())
        return 1;
    
    while(ros::ok())
    {   
        imu_get_data(&imu_values);
        imu_print_formatted(&imu_values);

        data.header.stamp = ros::Time::now();
        data.header.frame_id = "imu_frame";

        mag.header = data.header;
        mag.magnetic_field.x = imu_values.m[0];
        mag.magnetic_field.y = imu_values.m[1];
        mag.magnetic_field.z = imu_values.m[2];
        mag.magnetic_field_covariance[0] = 0.0015;
        mag.magnetic_field_covariance[1] = 0;
        mag.magnetic_field_covariance[2] = 0;
        mag.magnetic_field_covariance[3] = 0;
        mag.magnetic_field_covariance[4] = 0.0015;
        mag.magnetic_field_covariance[5] = 0;
        mag.magnetic_field_covariance[6] = 0;
        mag.magnetic_field_covariance[7] = 0;
        mag.magnetic_field_covariance[8] = 0.0015;

        // tf::quaternionTFToMsg(quat, data.orientation);
        data.orientation = geometry_msgs::Quaternion();
        data.orientation_covariance[0] = -1;
        data.orientation_covariance[1] = -1;
        data.orientation_covariance[2] = -1;
        data.orientation_covariance[3] = -1;
        data.orientation_covariance[4] = -1;
        data.orientation_covariance[5] = -1;
        data.orientation_covariance[6] = -1;
        data.orientation_covariance[7] = -1;
        data.orientation_covariance[8] = -1;

        data.angular_velocity.x = imu_values.g[0] * PI / 180.0;
        data.angular_velocity.y = imu_values.g[1] * PI / 180.0;
        data.angular_velocity.z = imu_values.g[2] * PI / 180.0;
        data.angular_velocity_covariance[0] = 0.95;
        data.angular_velocity_covariance[1] = 0;
        data.angular_velocity_covariance[2] = 0;
        data.angular_velocity_covariance[3] = 0;
        data.angular_velocity_covariance[4] = 0.95;
        data.angular_velocity_covariance[5] = 0;
        data.angular_velocity_covariance[6] = 0;
        data.angular_velocity_covariance[7] = 0;
        data.angular_velocity_covariance[8] = 0.95;

        data.linear_acceleration.x = imu_values.a[0] * G;
        data.linear_acceleration.y = imu_values.a[1] * G;
        data.linear_acceleration.z = -imu_values.a[2] * G;
        data.linear_acceleration_covariance[0] = 0.8;
        data.linear_acceleration_covariance[1] = 0;
        data.linear_acceleration_covariance[2] = 0;
        data.linear_acceleration_covariance[3] = 0;
        data.linear_acceleration_covariance[4] = 0.8;
        data.linear_acceleration_covariance[5] = 0;
        data.linear_acceleration_covariance[6] = 0;
        data.linear_acceleration_covariance[7] = 0;
        data.linear_acceleration_covariance[8] = 0.8;

        imu_mag_pub.publish(mag);
        imu_pub.publish(data);

        seq++;
        loop_rate.sleep();
        ros::spinOnce();
    }
    
    // Clean up and close GPS/IMU
    if(!close_all())
        return 1;
    
    return 0;
}

int init_all()
{
    // Initialize and open IMU serial port
    if((imu_ok = imu_init("/dev/ttyUSB0")) != 1)
        WARN("could not initialize IMU!");
    else
        INFO("IMU initialized...");
    
    // If nothing is working, fails
    if(!imu_ok)
        return 0;
    
    return 1;
}

int close_all()
{
    // Clean up and close IMU serial port
    if(!imu_close())
        WARN("Could not close IMU!");
    else
        INFO("IMU closed.");

    GPS_IMU("Goodbye!");
    
    return 1;
}

void sig_handler(int sig)
{
    INFO("Interrupted! Closing...");
    
    // Close everything cleanly before exiting
    if(!close_all())
    {
        FATAL("could not close cleanly!");
        exit(1);
    }
    
    exit(0);
}

void setup_sig_handler()
{
    signal(SIGINT, &sig_handler);
    signal(SIGTSTP, &sig_handler);
    signal(SIGABRT, &sig_handler);
    signal(SIGTERM, &sig_handler);
}
