#ifndef FIRST_HARDWARE_FIRST_ROBOT_HARDWARE_H
#define FIRST_HARDWARE_FIRST_ROBOT_HARDWARE_H

#include <ros/ros.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>

namespace first_hardware
{

class FirstRobotHW : public hardware_interface::RobotHW
{
public:
  FirstRobotHW();
  virtual ~FirstRobotHW();
  virtual bool init(ros::NodeHandle& root_nh, ros::NodeHandle &robot_hw_nh);
  void read(const ros::Time& time, const ros::Duration& period);
  void write(const ros::Time& time, const ros::Duration& period);

private:
  static const int JOINTS_COUNT = 1;

  hardware_interface::JointStateInterface joint_state_interface;
  hardware_interface::EffortJointInterface joint_effort_interface;

  ros::ServiceClient motor_speed_service;
  ros::Publisher publish_effort;

  double command[JOINTS_COUNT];
  double position[JOINTS_COUNT];
  double velocity[JOINTS_COUNT];
  double effort[JOINTS_COUNT];
};

}  // first_hardware

#endif  // FIRST_HARDWARE_FIRST_ROBOT_HARDWARE_H