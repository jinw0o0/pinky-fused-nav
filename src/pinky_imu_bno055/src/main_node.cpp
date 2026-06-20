#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "realtime_tools/realtime_publisher.hpp"
#include "angles/angles.h"

#include "wiringPiI2C.h"

using realtime_tools::RealtimePublisher;

class PinkyIMUBNO055 : public rclcpp::Node
{
    public:
        PinkyIMUBNO055() : Node("pinky_imu_bno055")
        {
            this->declare_parameter<std::string>("interface", "/dev/i2c-0");
            this->declare_parameter<std::string>("frame_id", "imu_link");
            this->declare_parameter<double>("rate", 100.0);

            frame_id_ = this->get_parameter("frame_id").get_parameter_value().get<std::string>();

            auto pub_imu = this->create_publisher<sensor_msgs::msg::Imu>("imu_raw", rclcpp::SystemDefaultsQoS());
            rt_pub_imu_ = std::make_shared<RealtimePublisher<sensor_msgs::msg::Imu>>(pub_imu);

            auto interface = this->get_parameter("interface").get_parameter_value().get<std::string>();
            fd_ = wiringPiI2CSetupInterface(interface.c_str(), 0x28);
            if (fd_ == -1) {
                RCLCPP_FATAL(this->get_logger(), "Failed to init I2C communication.");
                assert(false);
            }

            auto chipid = wiringPiI2CReadReg8(fd_, 0x00);
            RCLCPP_INFO(this->get_logger(), "Initialize IMU device [0x%x] on %s", chipid, interface.c_str());

            // NOTE (2026-06-20): do NOT issue SYS_TRIGGER RST_SYS (0x3F<-0x20) here. On this
            // board the BNO055 intermittently fails to re-appear on i2c after a soft reset
            // (0x28 vanishes, this init loop then spins forever -> node hangs before
            // "Start measurement"). A power-on (cold boot) already POR-resets the chip into
            // CONFIG mode, so just ensure CONFIGMODE before configuring instead of resetting.
            // 0x3E == PWR_MODE: Normal => 0x00.  0x3D == OPR_MODE: IMU fusion => 0x08.
            // Mirror a known-good i2cset sequence (CONFIG -> IMU with a solid settle), and
            // VERIFY by reading OPR_MODE/SYS_STATUS back. Without the verify the chip can
            // silently stay in CONFIG (0x00) and the node then publishes all-zero data.
            wiringPiI2CWriteReg8(fd_, 0x3E, 0x00); // PWR_MODE = Normal
            rclcpp::sleep_for(std::chrono::milliseconds(20));
            bool fusion_ok = false;
            for (int attempt = 0; attempt < 10 && rclcpp::ok(); ++attempt)
            {
                wiringPiI2CWriteReg8(fd_, 0x3D, 0x00); // OPR_MODE = CONFIG
                rclcpp::sleep_for(std::chrono::milliseconds(25)); // CONFIG switch ~19ms
                wiringPiI2CWriteReg8(fd_, 0x3D, 0x08); // OPR_MODE = IMU fusion
                rclcpp::sleep_for(std::chrono::milliseconds(400)); // fusion startup
                int opr = wiringPiI2CReadReg8(fd_, 0x3D);
                int sys = wiringPiI2CReadReg8(fd_, 0x39);
                if (opr == 0x08 && sys == 0x05) { fusion_ok = true; break; }
                RCLCPP_WARN(this->get_logger(),
                    "BNO055 not in IMU mode (OPR=0x%x SYS=0x%x), retry %d", opr, sys, attempt);
            }
            if (!fusion_ok)
                RCLCPP_ERROR(this->get_logger(), "BNO055 failed to enter IMU fusion mode");

            // Timer Loop (100Hz)
            auto period = std::chrono::duration<double>(1.0 / this->get_parameter("rate").as_double());
            timer_ = this->create_wall_timer(period, std::bind(&PinkyIMUBNO055::timer_callback, this));

            RCLCPP_INFO(this->get_logger(), "Start measurement...");
        }
        ~PinkyIMUBNO055() {}

    private:
        void timer_callback()
        {
            uint8_t data[32] = {0, };
            wiringPiI2CReadBlockData(fd_, 0x08,  data, 32);

            double acc_data_x = (int16_t)((data[1] << 8) + data[0]) / 100.0;
            double acc_data_y = (int16_t)((data[3] << 8) + data[2]) / 100.0;
            double acc_data_z = (int16_t)((data[5] << 8) + data[4]) / 100.0;

            double gyro_data_x = (int16_t)((data[13] << 8) + data[12]) / 16.0;
            double gyro_data_y = (int16_t)((data[15] << 8) + data[14]) / 16.0;
            double gyro_data_z = (int16_t)((data[17] << 8) + data[16]) / 16.0;

            // double eur_data_yaw = angles::normalize_angle(angles::from_degrees((int16_t)((data[19] << 8) + data[18]) / 16.0));
            // double eur_data_roll = angles::normalize_angle(angles::from_degrees((int16_t)((data[21] << 8) + data[20]) / 16.0));
            // double eur_data_pitch = angles::normalize_angle(angles::from_degrees((int16_t)((data[23] << 8) + data[22]) / 16.0));

            double quat_w = (int16_t)((data[25] << 8) + data[24]) / 16384.0;
            double quat_x = (int16_t)((data[27] << 8) + data[26]) / 16384.0;
            double quat_y = (int16_t)((data[29] << 8) + data[28]) / 16384.0;
            double quat_z = (int16_t)((data[31] << 8) + data[30]) / 16384.0;

            // RCLCPP_INFO(this->get_logger(), "%f %f %f | %f %f %f | %f %f %f", acc_data_x, acc_data_y, acc_data_z, gyro_data_x, gyro_data_y, gyro_data_z, eur_data_roll, eur_data_pitch, eur_data_yaw);
            // RCLCPP_INFO(this->get_logger(), "%f %f %f %f", quat_x, quat_y, quat_z, quat_w);

            rt_pub_imu_->trylock();

            rt_pub_imu_->msg_.header.stamp = this->now();
            rt_pub_imu_->msg_.header.frame_id = frame_id_;

            rt_pub_imu_->msg_.orientation.x = quat_x;
            rt_pub_imu_->msg_.orientation.y = quat_y;
            rt_pub_imu_->msg_.orientation.z = quat_z;
            rt_pub_imu_->msg_.orientation.w = quat_w;

            rt_pub_imu_->msg_.orientation_covariance = {0.01, 0.0, 0.0, 0.0, 0.01, 0.0, 0.0, 0.0, 0.01};

            // BNO055 gyro registers are deg/s (16 LSB/dps); sensor_msgs/Imu requires rad/s.
            rt_pub_imu_->msg_.angular_velocity.x = angles::from_degrees(gyro_data_x);
            rt_pub_imu_->msg_.angular_velocity.y = angles::from_degrees(gyro_data_y);
            rt_pub_imu_->msg_.angular_velocity.z = angles::from_degrees(gyro_data_z);

            rt_pub_imu_->msg_.angular_velocity_covariance = {0.01, 0.0, 0.0, 0.0, 0.01, 0.0, 0.0, 0.0, 0.01};

            rt_pub_imu_->msg_.linear_acceleration.x = acc_data_x;
            rt_pub_imu_->msg_.linear_acceleration.y = acc_data_y;
            rt_pub_imu_->msg_.linear_acceleration.z = acc_data_z;

            rt_pub_imu_->msg_.linear_acceleration_covariance = {0.01, 0.0, 0.0, 0.0, 0.01, 0.0, 0.0, 0.0, 0.01};

            rt_pub_imu_->unlockAndPublish();
        }

    private:
        int fd_;
        std::string frame_id_;
        rclcpp::TimerBase::SharedPtr timer_;
        std::shared_ptr<RealtimePublisher<sensor_msgs::msg::Imu>> rt_pub_imu_;

};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PinkyIMUBNO055>());

    rclcpp::shutdown();
    return 0;
}