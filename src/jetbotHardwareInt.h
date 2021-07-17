/**
 *  jetbotHardwareInt.h
 *
 *  MIT License
 *
 *  Copyright (c) 2018, Gavin Kane
 *
 *  Permission is hereby granted, free of charge, to any person obtaining a copy
 *  of this software and associated documentation files (the "Software"), to deal
 *  in the Software without restriction, including without limitation the rights
 *  to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 *  copies of the Software, and to permit persons to whom the Software is
 *  furnished to do so, subject to the following conditions:
 *
 *  The above copyright notice and this permission notice shall be included in all
 *  copies or substantial portions of the Software.
 *
 *  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 *  IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 *  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 *  AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 *  LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 *  OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 *  SOFTWARE.
 */

#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>
#include "adafruitmotorhat.h"
#include <inttypes.h>
#include <math.h>
#include <stdio.h>
#include <time.h>


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
   if (ros::param::get("/mobile_base_controller/v_multiplier", v_multiplier))
   {
	ROS_INFO("v_multiplier found: %f", v_multiplier);
   }
   if (ros::param::get("/mobile_base_controller/w_multiplier", w_multiplier))
   {
	ROS_INFO("w_multiplier found: %f", w_multiplier);
   }
  }

  void write() {
	  gettimeofday(&tv, NULL);
	  long long u_seconds = tv.tv_sec*1000000LL + tv.tv_usec;
          long diff = u_seconds - last_u_seconds;
	  last_u_seconds = u_seconds;
          //ROS_INFO("update rate %d us", diff);
	  //ROS_INFO("update motors %f %f", cmd[0], cmd[1]);
	  if (cmd[0] > 0)
	  {
	  	  motorDriver.getMotor(motor_left_ID)->run(AdafruitDCMotor::Command::kForward);
	  	  motorDriver.getMotor(motor_left_ID)->setSpeed(cmd[0]);
	  }
	  else
	  {
	  	  motorDriver.getMotor(motor_left_ID)->run(AdafruitDCMotor::Command::kBackward);
	  	  motorDriver.getMotor(motor_left_ID)->setSpeed(-cmd[0]);
	  }

	  if (cmd[1] > 0)
	  {
	  	  motorDriver.getMotor(motor_right_ID)->run(AdafruitDCMotor::Command::kForward);
	  	  motorDriver.getMotor(motor_right_ID)->setSpeed(cmd[1]);
	  }
	  else
	  {
	  	  motorDriver.getMotor(motor_right_ID)->run(AdafruitDCMotor::Command::kBackward);
	  	  motorDriver.getMotor(motor_right_ID)->setSpeed(-cmd[1]);
	  }
  }

  void read() {
		// Read odometry from extern interface

		// No external sensor exists
		// Use basic dead reconing with movement model

		// Motor has deadzone < 44.
		// This needs to be modified to include a hysterisis.
		// If motor is already turning, can maintain down to....
		// Scaling is also incorrect at present!!
		if (abs(cmd[0]) < 44)		
			temp_cmd[0] = 0;
		else 
			temp_cmd[0] = cmd[0];
		if (abs(cmd[1]) < 44)		
			temp_cmd[1] = 0;
		else 
			temp_cmd[1] = cmd[1];

		mean_v = (temp_cmd[0] + temp_cmd[1])/2;
		mean_w = (temp_cmd[0] - temp_cmd[1])/2;
		mean_v = mean_v * v_multiplier;
		mean_w = mean_w * w_multiplier;
		

		pos[0] += mean_v+mean_w;

		pos[1] += mean_v-mean_w;
  }

private:
  hardware_interface::JointStateInterface jnt_state_interface;
  hardware_interface::VelocityJointInterface jnt_vel_interface;
  double cmd[2];
  double pos[2];
  double vel[2];
  double eff[2];
  ros::NodeHandle nh;
  double mean_v;
  double mean_w;
  double temp_cmd[2];
  double v_multiplier = 0.0125;  
  double w_multiplier = 0.125;
  AdafruitMotorHAT motorDriver;
  int motor_left_ID = 1;
  int motor_right_ID = 2;


  struct timeval  tv;
  long long last_u_seconds;
};
