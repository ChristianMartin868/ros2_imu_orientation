/**
 * ROS 2 node that calculates the orientation in 3 DoF from IMU data.
 * Ported from the ROS 1 node by Stefan May (10.5.2020).
 *
 * Estimates orientation in two independent ways:
 *   1) using only accelerometer data  -> topic "poseAcc"
 *   2) using only gyroscope data      -> topic "poseGyro"
 */
#include <functional>
#include <memory>

#include "rclcpp/rclcpp.hpp"
// Type of input topic
#include "sensor_msgs/msg/imu.hpp"
// Type of output topics
#include "geometry_msgs/msg/pose_stamped.hpp"
// Helper classes to calculate orientation from 3D vector data
// (provided externally, namespace aut4 -- see README).
#include "vector3.h"
#include "quaternion.h"

class ImuPoseNode : public rclcpp::Node
{
public:
  ImuPoseNode()
  : Node("st_imu_pose_node")
  {
    sub_ = create_subscription<sensor_msgs::msg::Imu>(
      "imu/data_raw", 1,
      std::bind(&ImuPoseNode::callbackIMU, this, std::placeholders::_1));

    pub_pose_gyro_ = create_publisher<geometry_msgs::msg::PoseStamped>("poseGyro", 1);
    pub_pose_acc_ = create_publisher<geometry_msgs::msg::PoseStamped>("poseAcc", 1);
  }

private:
  // IMU message callback. Receives raw IMU measurements and computes the
  // orientation for the accelerometer-only and gyroscope-only cases.
  void callbackIMU(const sensor_msgs::msg::Imu::SharedPtr msg)
  {
    //------ Calculate sampling time ---------------------------------------
    const rclcpp::Time time(msg->header.stamp);
    if (!have_time_prev_) {
      time_prev_ = time;
      have_time_prev_ = true;
    }
    const double dT = (time - time_prev_).seconds();
    time_prev_ = time;

    //------ Convert and reference received measurement data ----------------

    // Accelerometer
    aut4::Vector3 vAcc(
      msg->linear_acceleration.x,
      msg->linear_acceleration.y,
      msg->linear_acceleration.z);
    vAcc.normalize();
    if (!have_acc_reference_) {
      vAccReference_ = vAcc;
      have_acc_reference_ = true;
    }

    // Gyroscope
    aut4::Vector3 vGyro;
    vGyro[0] = msg->angular_velocity.x;
    vGyro[1] = msg->angular_velocity.y;
    vGyro[2] = msg->angular_velocity.z;
    if (!have_gyro_offset_) {
      vGyroOffset_ = vGyro;
      have_gyro_offset_ = true;
    }
    vGyro -= vGyroOffset_;

    //------ Publish quaternion from accelerometer measurement -------------
    const aut4::Quaternion qAcc =
      aut4::Quaternion::quaternionFromTwoVectors(vAccReference_, vAcc);

    geometry_msgs::msg::PoseStamped msgPoseAcc;
    msgPoseAcc.header.frame_id = "map";
    msgPoseAcc.header.stamp = time;
    msgPoseAcc.pose.position.x = 4;
    msgPoseAcc.pose.position.y = 0;
    msgPoseAcc.pose.position.z = 0;
    msgPoseAcc.pose.orientation.x = qAcc.x;
    msgPoseAcc.pose.orientation.y = qAcc.y;
    msgPoseAcc.pose.orientation.z = qAcc.z;
    msgPoseAcc.pose.orientation.w = qAcc.w;
    pub_pose_acc_->publish(msgPoseAcc);

    //------ Publish quaternion from gyro measurement ---------------------
    if (!have_gyro_quat_) {
      qGyro_ = qAcc;
      have_gyro_quat_ = true;
    }
    // The gyroscope delivers incremental spin that is integrated over time.
    const aut4::Quaternion qGyroDiff =
      aut4::Quaternion::quaternionFromAngularVelocity(vGyro, dT);
    qGyro_ *= qGyroDiff;

    geometry_msgs::msg::PoseStamped msgPoseGyro;
    msgPoseGyro.header.frame_id = "map";
    msgPoseGyro.header.stamp = time;
    msgPoseGyro.pose.position.x = 2;
    msgPoseGyro.pose.position.y = 0;
    msgPoseGyro.pose.position.z = 0;
    msgPoseGyro.pose.orientation.x = qGyro_.x;
    msgPoseGyro.pose.orientation.y = qGyro_.y;
    msgPoseGyro.pose.orientation.z = qGyro_.z;
    msgPoseGyro.pose.orientation.w = qGyro_.w;
    pub_pose_gyro_->publish(msgPoseGyro);
  }

  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr sub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pub_pose_acc_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pub_pose_gyro_;

  // State carried across callbacks (was function-local `static` in ROS 1).
  rclcpp::Time time_prev_;
  aut4::Vector3 vAccReference_;
  aut4::Vector3 vGyroOffset_;
  aut4::Quaternion qGyro_;
  bool have_time_prev_ = false;
  bool have_acc_reference_ = false;
  bool have_gyro_offset_ = false;
  bool have_gyro_quat_ = false;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ImuPoseNode>());
  rclcpp::shutdown();
  return 0;
}
