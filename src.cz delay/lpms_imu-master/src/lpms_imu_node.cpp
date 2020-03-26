/**
 * @file
 * @brief ROS driver for the LP IMU sensor
 *
 * @par Advertises
 *
 * - @b data Calibrated IMU data
 *
 * @par Parameters
 *
 * - @b ~sensor_model LP sensor model identifier (the node has so far been tested with DEVICE_LPMS_U2)
 * - @b ~port The port that the IMU is connected to (default /dev/ttyUSB0)
 * - @b ~frame_id Frame identifier if IMU reference frame for message header (default imu_global)
 * - @b ~rate Update rate, in Hz (default 50)
 */

#include <string>
#include <map>

#include "ros/ros.h"
#include "sensor_msgs/Imu.h"
#include "sensor_msgs/MagneticField.h"

#include "lpsensor/LpmsSensorI.h"
#include "lpsensor/LpmsSensorManagerI.h"

#include "timesync/TimestampSynchronizer.h"

//! Manages connection with the sensor, publishes data
/*!
  \TODO: Make noncopyable!
 */
class LpImuProxy
{
 public:
    LpImuProxy() : private_nh("~")
    {
        // Initialize mapping of LPMS sensor types
        device_map = {{"DEVICE_LPMS_B", DEVICE_LPMS_B},
                      {"DEVICE_LPMS_Upp", DEVICE_LPMS_U},
                      {"DEVICE_LPMS_C", DEVICE_LPMS_C},
                      {"DEVICE_LPMS_BLE", DEVICE_LPMS_BLE},
                      {"DEVICE_LPMS_RS232", DEVICE_LPMS_RS232},
                      {"DEVICE_LPMS_B2", DEVICE_LPMS_B2},
                      {"DEVICE_LPMS_U2", DEVICE_LPMS_U2},
                      {"DEVICE_LPMS_C2", DEVICE_LPMS_C2}};

        // Get node parameters
        private_nh.param<std::string>("sensor_model", sensor_model, "DEVICE_LPMS_U2");
        private_nh.param<std::string>("port", port, "/dev/ttyUSB0");
        private_nh.param<std::string>("frame_id", frame_id, "imu");
        private_nh.param("rate", rate, 100);

        private_nh.param("angular_velocity_offset_x", angular_velocity_offset_x, -0.064);
		private_nh.param("angular_velocity_offset_y", angular_velocity_offset_y, 0.015);
		private_nh.param("angular_velocity_offset_z", angular_velocity_offset_z, 0.025);
        // Timestamp synchronization
        private_nh.param("enable_time_sync", enable_Tsync, true);

        // Connect to the LP IMU device
        manager = LpmsSensorManagerFactory();
        imu = manager->addSensor(device_map[sensor_model], port.c_str());

        imu_pub = nh.advertise<sensor_msgs::Imu>("imu",1);
        mag_pub = nh.advertise<sensor_msgs::MagneticField>("mag",1);

        TimestampSynchronizer::Options defaultSyncOptions;
        defaultSyncOptions.useMedianFilter = true;
        defaultSyncOptions.medianFilterWindow = 1500;
        defaultSyncOptions.useHoltWinters = true;
        defaultSyncOptions.alfa_HoltWinters = 4e-4;
        defaultSyncOptions.beta_HoltWinters = 3e-4;
        defaultSyncOptions.alfa_HoltWinters_early = 5e-2;
        defaultSyncOptions.beta_HoltWinters_early = 1e-3;
        defaultSyncOptions.earlyClamp = true;
        defaultSyncOptions.earlyClampWindow = 120*200;
        defaultSyncOptions.timeOffset = 0.0;
        defaultSyncOptions.initialB_HoltWinters = -3.4e-7;
        pstampSynchronizer = std::make_unique<TimestampSynchronizer>(defaultSyncOptions);

//        imu->startCalibrateGyro();
//        ros::Duration delay= ros::Duration(5);
//        delay.sleep();
//        imu->stopGyrMisalignCal();
//        imu->saveCalibrationData();

        imu->setSensorStatus(0);
        imu->setOrientationOffset(0);
        then = ros::Time::now();
        t_delta = ros::Duration(1/rate);
        t_next = ros::Time::now() + t_delta;
        th = 0.0;
    }

    ~LpImuProxy(void)
    {
        manager->removeSensor(imu);
        delete manager;
    }

    ros::Time now;
    ros::Time then;
    ros::Time t_next;
    double elapsed;
    ros::Duration t_delta;
    double th;

    double normalize_angle(double angle){
         float res = angle;
         while (res > M_PI)
             res -= 2.0 * M_PI;
         while (res < -M_PI)
             res += 2.0 * M_PI;
         return res;
    }

    void update(const ros::TimerEvent& te)
    {
        if (imu->getConnectionStatus() == SENSOR_CONNECTION_CONNECTED &&
                imu->hasImuData())
        {
            data = imu->getCurrentData();

            /* Fill the IMU message */

            // Fill the header
            imu_msg.header.stamp = enable_Tsync ? ros::Time(pstampSynchronizer->sync(data.timeStamp, ros::Time::now().toSec(), data.frameCount)) : ros::Time::now();
            imu_msg.header.frame_id = frame_id;

            // Fill angular velocity data
            // - scale from deg/s to rad/s
            imu_msg.angular_velocity.x = data.g[0]*3.1415926/180;
            imu_msg.angular_velocity.x = imu_msg.angular_velocity.x + angular_velocity_offset_x;
            imu_msg.angular_velocity.y = data.g[1]*3.1415926/180;
            imu_msg.angular_velocity.y = imu_msg.angular_velocity.y + angular_velocity_offset_y;
            imu_msg.angular_velocity.z = data.g[2]*3.1415926/180;
            imu_msg.angular_velocity.z = imu_msg.angular_velocity.z + angular_velocity_offset_z;
            imu_msg.angular_velocity.z = -imu_msg.angular_velocity.z;

            if(fabs(imu_msg.angular_velocity.x) < 0.02){
            	imu_msg.angular_velocity.x = 0.0;
            }
            if(fabs(imu_msg.angular_velocity.y) < 0.02){
				imu_msg.angular_velocity.y = 0.0;
			}
            if(fabs(imu_msg.angular_velocity.z) < 0.02){
				imu_msg.angular_velocity.z = 0.0;
			}

            now = ros::Time::now();

			if (now > t_next){
				elapsed = (now - then).toSec();
				then = now;
				th += imu_msg.angular_velocity.z *elapsed;
				th = normalize_angle(th);

				// Fill orientation quaternion
				imu_msg.orientation.w = cos(th/2);//data.q[0];
				imu_msg.orientation.x = 0;//-data.q[1];
				imu_msg.orientation.y = 0;//-data.q[2];
				imu_msg.orientation.z = sin(th/2);//-data.q[3];

				// Fill linear acceleration data
				imu_msg.linear_acceleration.x = -data.a[0]*9.81;
				imu_msg.linear_acceleration.y = -data.a[1]*9.81;
				imu_msg.linear_acceleration.z = -data.a[2]*9.81;

				// \TODO: Fill covariance matrices
				// msg.orientation_covariance = ...
				// msg.angular_velocity_covariance = ...
				// msg linear_acceleration_covariance = ...
				imu_msg.orientation_covariance = {1e6, 0, 0, 0, 1e6, 0, 0, 0, 1e-6};
				imu_msg.angular_velocity_covariance = {1e6, 0, 0, 0, 1e6, 0, 0, 0, 1e-6};
				imu_msg.linear_acceleration_covariance = {-1,0,0,0,0,0,0,0,0};

				/* Fill the magnetometer message */
				mag_msg.header.stamp = imu_msg.header.stamp;
				mag_msg.header.frame_id = frame_id;

				// Units are microTesla in the LPMS library, Tesla in ROS.
				mag_msg.magnetic_field.x = data.b[0]*1e-6;
				mag_msg.magnetic_field.y = data.b[1]*1e-6;
				mag_msg.magnetic_field.z = data.b[2]*1e-6;

				// Publish the messages
				imu_pub.publish(imu_msg);
				mag_pub.publish(mag_msg);

				t_next = now + t_delta;
			}
        }
    }

    void run(void)
    {
        // The timer ensures periodic data publishing
        updateTimer = ros::Timer(nh.createTimer(ros::Duration(0.1/rate),
                                                &LpImuProxy::update,
                                                this));
    }

 private:

    // Access to LPMS data
    LpmsSensorManagerI* manager;
    LpmsSensorI* imu;
    ImuData data;
    std::map<std::string,int> device_map;

    // Access to ROS node
    ros::NodeHandle nh, private_nh;
    ros::Timer updateTimer;
    ros::Publisher imu_pub, mag_pub;
    sensor_msgs::Imu imu_msg;
    sensor_msgs::MagneticField mag_msg;

    // Parameters
    std::string sensor_model;
    std::string port;
    std::string frame_id;
    int rate;

    double angular_velocity_offset_x;
    double angular_velocity_offset_y;
    double angular_velocity_offset_z;
    // Timestamp syncronization
    bool enable_Tsync;

    std::unique_ptr<TimestampSynchronizer> pstampSynchronizer;
};

int main(int argc, char *argv[])
{

    ros::init(argc, argv, "lpms_imu");
    ros::NodeHandle nh, private_nh;

    LpImuProxy lpImu;

    lpImu.run();

    ros::spin();

    return 0;
}
