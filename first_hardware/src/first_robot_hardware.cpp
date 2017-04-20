#include "first_hardware/first_robot_hardware.h"
#include "first_msgs/GetMotorSpeed.h"
#include <std_msgs/Float32.h>
#include <hardware_interface/robot_hw.h>
#include <string>
#include <vector>

namespace first_hardware
{

  FirstRobotHW::FirstRobotHW()
  {
  }

  FirstRobotHW::~FirstRobotHW()
  {
  }

  bool FirstRobotHW::init(ros::NodeHandle& root_nh, ros::NodeHandle &robot_hw_nh)
  {
    ros::service::waitForService("get_motor_velocity");
    motor_speed_service = root_nh.serviceClient<first_msgs::GetMotorSpeed>("get_motor_velocity");
    publish_effort = root_nh.advertise<std_msgs::Float32>("send_motor_1_effort", 1000);

    hardware_interface::JointStateHandle wheel_joint_state_handle("wheel_joint", &position[0], &velocity[0], &effort[0]);
    joint_state_interface.registerHandle(wheel_joint_state_handle);

    registerInterface(&joint_state_interface);

    hardware_interface::JointHandle wheel_joint_effort_handler(joint_state_interface.getHandle("wheel_joint"),
      &command[0]);
    joint_effort_interface.registerHandle(wheel_joint_effort_handler);

    registerInterface(&joint_effort_interface);
  }

  void FirstRobotHW::read(const ros::Time& time, const ros::Duration& period)
  {
    first_msgs::GetMotorSpeed message;
    message.request.motor_index = 0;
    if (motor_speed_service.call(message))
    {
      velocity[0] = message.response.velocity;
    }
  }

  void FirstRobotHW::write(const ros::Time& time, const ros::Duration& period)
  {
    std_msgs::Float32 message;
    message.data = command[0];
    publish_effort.publish(message);
  }

}  // first_hardware
