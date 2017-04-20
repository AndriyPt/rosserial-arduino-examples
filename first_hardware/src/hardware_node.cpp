#include "first_hardware/first_robot_hardware.h"
#include <controller_manager/controller_manager.h>
#include <hardware_interface/actuator_state_interface.h>
#include <ros/callback_queue.h>

using first_hardware::FirstRobotHW;

int main(int argc, char** argv)
{
  ros::init(argc, argv, "first_hardware_node");
  ros::NodeHandle nh;
  ros::NodeHandle private_node_handle("~");
  ros::CallbackQueue queue;
  nh.setCallbackQueue(&queue);

  FirstRobotHW robot;
  robot.init(nh, private_node_handle);
  controller_manager::ControllerManager cm(&robot, nh);

  ros::AsyncSpinner spinner(4, &queue);
  spinner.start();

  ros::Time ts = ros::Time::now();

  ros::Rate rate(50);
  while (ros::ok())
  {
     ros::Duration d = ros::Time::now() - ts;
     ts = ros::Time::now();
     robot.read(ts, d);
     cm.update(ts, d);
     robot.write(ts, d);
     rate.sleep();
  }

  spinner.stop();

  return 0;
}