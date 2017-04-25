#ifndef FIRST_HARDWARE_FIRST_ROBOT_HARDWARE_H
#define FIRST_HARDWARE_FIRST_ROBOT_HARDWARE_H

#include <atomic>
#include <ros/ros.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>
#include "first_msgs/JointState.h"

namespace first_hardware
{

class FirstRobotHW : public hardware_interface::RobotHW
{
public:
  FirstRobotHW();
  virtual ~FirstRobotHW();
  virtual bool init(ros::NodeHandle& root_nh);
  void read(const ros::Time& time, const ros::Duration& period);
  void write(const ros::Time& time, const ros::Duration& period);

private:
  void jointStateCallback(const first_msgs::JointState::ConstPtr& message);

  hardware_interface::JointStateInterface joint_state_interface;
  hardware_interface::EffortJointInterface joint_effort_interface;

  ros::Subscriber motor_joint_state;
  ros::Publisher publish_effort;

  double command;
  double position;
  double velocity;
  double effort;

  std::atomic<double> hardware_motor_position;
  std::atomic<double> hardware_motor_velocity;
  std::atomic<double> hardware_motor_effort;
};

}  // first_hardware

#endif  // FIRST_HARDWARE_FIRST_ROBOT_HARDWARE_H