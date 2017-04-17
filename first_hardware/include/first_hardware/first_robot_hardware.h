#ifndef FIRST_HARDWARE_FIRST_ROBOT_HARDWARE_H
#define FIRST_HARDWARE_FIRST_ROBOT_HARDWARE_H

#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>

namespace first_hardware
{

class FirstRobot : public hardware_interface::RobotHW
{
public:
  FirstRobot();

private:
  hardware_interface::JointStateInterface jnt_state_interface;
  hardware_interface::PositionJointInterface jnt_pos_interface;
  double cmd[2];
  double pos[2];
  double vel[2];
  double eff[2];
};

}  // first_hardware

#endif  // FIRST_HARDWARE_FIRST_ROBOT_HARDWARE_H