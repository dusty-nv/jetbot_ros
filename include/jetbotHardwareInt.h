#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>

class jetbotHardwareInt : public hardware_interface::RobotHW
{
public:
  jetbotHardwareInt() {


   // connect and register the joint state interface
   hardware_interface::JointStateHandle state_handle_left("robot_left_wheel_hinge", &pos[0], &vel[0], &eff[0]);
   jnt_state_interface.registerHandle(state_handle_left);

   hardware_interface::JointStateHandle state_handle_right("robot_right_wheel_hinge", &pos[1], &vel[1], &eff[1]);
   jnt_state_interface.registerHandle(state_handle_right);

   registerInterface(&jnt_state_interface);

   // connect and register the joint position interface
   hardware_interface::JointHandle vel_handle_left(jnt_state_interface.getHandle("robot_left_wheel_hinge"), &cmd[0]);
   jnt_vel_interface.registerHandle(vel_handle_left);

   hardware_interface::JointHandle vel_handle_right(jnt_state_interface.getHandle("robot_right_wheel_hinge"), &cmd[1]);
   jnt_vel_interface.registerHandle(vel_handle_right);

   registerInterface(&jnt_vel_interface);
  }

  void write() {
	  ROS_INFO("update motors");
		// Send commands to I2C controlling motors
  }

  void read() {
	  ROS_INFO("update odom");
		// Read odometry from extern interface
		pos[0] += cmd[0];
		pos[1] += cmd[1];
  }

private:
  hardware_interface::JointStateInterface jnt_state_interface;
  hardware_interface::VelocityJointInterface jnt_vel_interface;
  double cmd[2];
  double pos[2];
  double vel[2];
  double eff[2];
};
