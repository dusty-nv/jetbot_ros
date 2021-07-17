/**
 *  motor2927_controller.cpp
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

#include <ros/ros.h>
#include "jetbotHardwareInt.h"
#include <controller_manager/controller_manager.h>

int main(int argc, char ** argv)
{
	ros::init(argc, argv, "motor2927_controller");

	jetbotHardwareInt jetbot;
	controller_manager::ControllerManager cm(&jetbot);

	ros::AsyncSpinner spinner(1);
	spinner.start();

	// Control Loop
	ros::Time prev_time = ros::Time::now();
	ros::Rate rate(50.0);

	while (ros::ok())
	{
		const ros::Time  time = ros::Time::now();
		const ros::Duration period = time - prev_time;

		jetbot.read();
		cm.update(time, period);
		jetbot.write();

		rate.sleep();
	}
}
