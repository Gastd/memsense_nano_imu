#include "ros/ros.h"
#include <ros/console.h>
#include <string>
#include "memsense_nano_imu.h"
#include "sensor_msgs/Imu.h"
#include "sensor_msgs/Temperature.h"
#include "sensor_msgs/MagneticField.h"
#include "std_srvs/Empty.h"
#include <tf/transform_datatypes.h>

class ImuNode 
{
private:
    memsense_nano_imu::Imu imu;
    sensor_msgs::Imu imu_reading_;
    sensor_msgs::Temperature tem_reading_;
    sensor_msgs::MagneticField mag_reading_;

    std::string port;

    ros::NodeHandle node_handle_;
    ros::NodeHandle private_node_handle_;
    ros::Publisher imu_data_pub_;
    ros::Publisher mag_data_pub_;
    ros::Publisher tem_data_pub_;
    ros::ServiceServer calibrate_serv_;

    bool running;

    bool autocalibrate_;
    bool calibrate_requested_;
    bool calibrated_;

    int error_count_;
    int slow_count_;
    std::string was_slow_;
    std::string error_status_;

    std::string frameid_;

    double offset_;

    double bias_x_;
    double bias_y_;
    double bias_z_;
    double abias_x_;
    double abias_y_;
    double abias_z_;

    double angular_velocity_stdev_, angular_velocity_covariance_;
    double linear_acceleration_covariance_, linear_acceleration_stdev_;
    double orientation_covariance_, orientation_stdev_;
    double magnetic_field_covariance_, magnetic_field_stddev_;

    double max_drift_rate_;

    double desired_freq_;
    // diagnostic_updater::FrequencyStatus freq_diag_;

public:
    ImuNode(ros::NodeHandle n) : node_handle_(n), private_node_handle_("~"), calibrate_requested_(false),
    error_count_(0), slow_count_(0), desired_freq_(50)
    // freq_diag_(diagnostic_updater::FrequencyStatusParam(&desired_freq_, &desired_freq_, 0.05))
    {
        ros::NodeHandle imu_node_handle(node_handle_, "imu");

        // Params
        private_node_handle_.param("autocalibrate", autocalibrate_, true);
        private_node_handle_.param("assume_calibrated", calibrated_, false);
        private_node_handle_.param("port", port, std::string("/dev/ttyUSB0"));
        private_node_handle_.param("max_drift_rate", max_drift_rate_, 0.0002);
        private_node_handle_.param("frame_id", frameid_, std::string("imu"));
        private_node_handle_.param("linear_acceleration_stdev", linear_acceleration_stdev_, 0.6 * 10e-3 * imu.GRAVITY);
        private_node_handle_.param("orientation_stdev", orientation_stdev_, -1.0);
        private_node_handle_.param("angular_velocity_stdev", angular_velocity_stdev_, 0.36 * M_PI / 180.0);
        private_node_handle_.param("magnetic_field_stddev", magnetic_field_stddev_, 0.00056);

        // Publishers
        imu_data_pub_ = imu_node_handle.advertise<sensor_msgs::Imu>("data_raw", 10);
        tem_data_pub_ = imu_node_handle.advertise<sensor_msgs::Temperature>("temperature", 10);
        mag_data_pub_ = imu_node_handle.advertise<sensor_msgs::MagneticField>("mag", 10);

        // Services
        calibrate_serv_ = imu_node_handle.advertiseService("calibrate", &ImuNode::calibrate, this);

        // Variables init
        running = false;
        bias_x_ = bias_y_ = bias_z_ = 0;

        imu_reading_.header.frame_id = frameid_;
        mag_reading_.header.frame_id = frameid_;
        tem_reading_.header.frame_id = frameid_;

        angular_velocity_covariance_ = angular_velocity_stdev_ * angular_velocity_stdev_;
        orientation_covariance_ = orientation_stdev_ * orientation_stdev_;
        linear_acceleration_covariance_ = linear_acceleration_stdev_ * linear_acceleration_stdev_;
        magnetic_field_covariance_ = magnetic_field_stddev_ * magnetic_field_stddev_;

        imu_reading_.linear_acceleration_covariance[0] = linear_acceleration_covariance_;
        imu_reading_.linear_acceleration_covariance[4] = linear_acceleration_covariance_;
        imu_reading_.linear_acceleration_covariance[8] = linear_acceleration_covariance_;

        imu_reading_.angular_velocity_covariance[0] = angular_velocity_covariance_;
        imu_reading_.angular_velocity_covariance[4] = angular_velocity_covariance_;
        imu_reading_.angular_velocity_covariance[8] = angular_velocity_covariance_;

        imu_reading_.orientation_covariance[0] = -1;
        imu_reading_.orientation_covariance[4] = -1;
        imu_reading_.orientation_covariance[8] = -1;

        mag_reading_.magnetic_field_covariance[0] = magnetic_field_covariance_;
        mag_reading_.magnetic_field_covariance[4] = magnetic_field_covariance_;
        mag_reading_.magnetic_field_covariance[8] = magnetic_field_covariance_;

        // self_test_.add("Close Test", this, &ImuNode::pretest);
        // self_test_.add("Interruption Test", this, &ImuNode::InterruptionTest);
        // self_test_.add("Connect Test", this, &ImuNode::ConnectTest);
        // self_test_.add("Read ID Test", this, &ImuNode::ReadIDTest);
        // self_test_.add("Gyro Bias Test", this, &ImuNode::GyroBiasTest);
        // self_test_.add("Streamed Data Test", this, &ImuNode::StreamedDataTest);
        // self_test_.add("Gravity Test", this, &ImuNode::GravityTest);
        // self_test_.add("Disconnect Test", this, &ImuNode::DisconnectTest);
        // self_test_.add("Resume Test", this, &ImuNode::ResumeTest);

        // diagnostic_.add( freq_diag_ );
        // diagnostic_.add( "Calibration Status", this, &ImuNode::calibrationStatus );
        // diagnostic_.add( "IMU Status", this, &ImuNode::deviceStatus );
    }

    void start()
    {
        try
        {
            imu.init(port);

            if (autocalibrate_ || calibrate_requested_)
            {
                doCalibrate();
                calibrate_requested_ = false;
                autocalibrate_ = false; // No need to do this each time we reopen the device.
            }
            else
            {
                ROS_INFO("Not calibrating the IMU sensor. Use the calibrate service to calibrate it before use.");
            }

            ROS_INFO("IMU sensor initialized.");
            running = true;
        }
        catch(const std::exception& e)
        {
            ROS_ERROR_STREAM("Exception thrown while starting IMU. This sometimes happens if you are not connected " <<
                             "to an IMU or if another process is trying to access the IMU port. You may try 'lsof|grep "
                             << port.c_str() <<
                             "' to see if other processes have the port open."<< std::endl << e.what());
            ROS_WARN("Could not start IMU!");
            std::cerr << e.what() << std::endl;
        }

    }

    bool spin()
    {
        start();
        while(ros::ok())
        {
            publishData();
            ros::spinOnce();
        }
        stop();
    }

    void publishData()
    {
        getData(imu_reading_, mag_reading_, tem_reading_);
        imu_data_pub_.publish(imu_reading_);
        mag_data_pub_.publish(mag_reading_);
        tem_data_pub_.publish(tem_reading_);
    }

    void getData(sensor_msgs::Imu& data, sensor_msgs::MagneticField& mag, sensor_msgs::Temperature& temp)
    {
        imu.receiveDataFromImu(data, mag, temp);
        data.angular_velocity.x = data.angular_velocity.x + bias_x_;
        data.angular_velocity.y = data.angular_velocity.y + bias_y_;
        data.angular_velocity.z = data.angular_velocity.z + bias_z_;
        data.linear_acceleration.x = data.linear_acceleration.x + abias_x_;
        data.linear_acceleration.y = data.linear_acceleration.y + abias_y_;
        data.linear_acceleration.z = data.linear_acceleration.z + abias_z_;
        
        data.header.stamp = ros::Time::now();
        mag.header.stamp = ros::Time::now();
        temp.header.stamp = ros::Time::now();
    }

    void doCalibrate()
    {
        ROS_INFO("Calibrating IMU gyros.");
        getData(imu_reading_, mag_reading_, tem_reading_);
        bias_x_ = -imu_reading_.angular_velocity.x;
        bias_y_ = -imu_reading_.angular_velocity.y;
        bias_z_ = -imu_reading_.angular_velocity.z;
        ROS_INFO("Gyro bias x = %f, y = %f e z = %f", bias_x_, bias_y_, bias_z_);

        ROS_INFO("Calibrating IMU accel.");
        abias_x_ = -imu_reading_.linear_acceleration.x;
        abias_y_ = -imu_reading_.linear_acceleration.y;
        abias_z_ = imu.GRAVITY - imu_reading_.linear_acceleration.z;
        ROS_INFO("Accel bias x = %f, y = %f e z = %f", abias_x_, abias_y_, abias_z_);

        ROS_INFO("Calculating angular drift.");
        double x_rate = 0.0;
        double y_rate = 0.0;
        double z_rate = 0.0;
        size_t count = 0;

        getData(imu_reading_, mag_reading_, tem_reading_);
        ros::Time start_time = imu_reading_.header.stamp;
        while(imu_reading_.header.stamp - start_time < ros::Duration(2.0))
        {
            getData(imu_reading_, mag_reading_, tem_reading_);
            x_rate += imu_reading_.angular_velocity.x;
            y_rate += imu_reading_.angular_velocity.y;
            z_rate += imu_reading_.angular_velocity.z;
            ++count;
        }

        if (count < 200)
            ROS_WARN("Imu: calibration check acquired fewer than 200 samples.");

        double average_rate = sqrt(x_rate*x_rate + y_rate*y_rate + z_rate*z_rate) / count;
        // calibration succeeded
        if (average_rate < max_drift_rate_)
        {
            ROS_INFO("Imu: calibration check succeeded: average angular drift %f rad/sec < %f rad/sec",
                      average_rate/**180*1000/M_PI*/, max_drift_rate_/**180*1000/M_PI*/);
            calibrated_ = true;
            ROS_INFO("IMU gyro calibration completed.");
            // freq_diag_.clear();
        }
        // calibration failed
        else
        {
            calibrated_ = false;
            ROS_ERROR("Imu: calibration check failed: average angular drift = %f rad/sec > %f rad/sec",
                      average_rate/**180*1000/M_PI*/, max_drift_rate_/**180*1000/M_PI*/);
        }
    }

    bool calibrate(std_srvs::Empty::Request &req, std_srvs::Empty::Response &resp)
    {
        bool old_running = running;

        try
        {
            calibrate_requested_ = true;
            if (old_running)
            {
                stop();
                start(); // Start will do the calibration.
            }
            else
            {
                imu.init(port);
                doCalibrate();
                imu.close();
            }
        }
        catch (const std::exception& e) {
          error_count_++;
          calibrated_ = false;
          ROS_ERROR("Exception thrown while calibrating IMU %s", e.what());
          stop();
          if (old_running)
            start(); // Might throw, but we have nothing to lose... Needs restructuring.
          return false;
        }

        return true;
    }
    void stop()
    {
        try
        {
            imu.close();
            ROS_INFO("IMU closed.");
        }
        catch(const std::exception& e)
        {
            ROS_WARN("Could not close IMU!");
            std::cerr << e.what() << std::endl;
        }

        ROS_INFO("Goodbye!");
    }

    ~ImuNode()
    {
        stop();
    }
};

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "memsense_nano_imu");
    ros::NodeHandle n;

    ImuNode in(n);
    in.spin();

    return 0;
}
