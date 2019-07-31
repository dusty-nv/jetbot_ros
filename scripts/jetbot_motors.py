#!/usr/bin/env python
import rospy
import time

import qwiic_scmd
from std_msgs.msg import String


# sets motor speed between [-1.0, 1.0]
def set_speed(motor_ID, value):
	max_pwm = 115.0
	#speed = int(min(max(abs(value * max_pwm), 0), max_pwm))
	speed = int(value * max_pwm)

	if motor_ID == 1 or 2:
		motor_driver.set_drive(motor_ID - 1, 0, speed)
	else:
		rospy.logerror('set_speed(%d, %f) -> invalid motor_ID=%d', motor_ID, value, motor_ID)
		return


# stops all motors
def all_stop():
	motor_driver.set_drive(0,0,0)
	motor_driver.set_drive(1,0,0)

	motor_driver.disable()


# directional commands (degree, speed)
def on_cmd_dir(msg):
	rospy.loginfo(rospy.get_caller_id() + ' cmd_dir=%s', msg.data)

# raw L/R motor commands (speed, speed)
def on_cmd_raw(msg):
	rospy.loginfo(rospy.get_caller_id() + ' cmd_raw=%s', msg.data)

# simple string commands (left/right/forward/backward/stop)
def on_cmd_str(msg):
	rospy.loginfo(rospy.get_caller_id() + ' cmd_str=%s', msg.data)


	if msg.data.lower() == "left":
		set_speed(motor_left_ID,  -1.0)
		set_speed(motor_right_ID,  1.0)
		motor_driver.enable()
	elif msg.data.lower() == "right":
		set_speed(motor_left_ID,   1.0)
		set_speed(motor_right_ID, -1.0)
		motor_driver.enable()
	elif msg.data.lower() == "forward":
		set_speed(motor_left_ID,   1.0)
		set_speed(motor_right_ID,  1.0)
		motor_driver.enable()
	elif msg.data.lower() == "backward":
		set_speed(motor_left_ID,  -1.0)
		set_speed(motor_right_ID, -1.0)  
		motor_driver.enable()
	elif msg.data.lower() == "stop":
		all_stop()
	else:
		rospy.logerror(rospy.get_caller_id() + ' invalid cmd_str=%s', msg.data)


# initialization
if __name__ == '__main__':

	# setup motor controller
	motor_driver = qwiic_scmd.QwiicScmd()

	motor_left_ID = 1
	motor_right_ID = 2

	print(motor_left_ID)	

	#motor_left = motor_driver.getMotor(motor_left_ID)
	#motor_right = motor_driver.getMotor(motor_right_ID)
	
	# stop the motors as precaution
	all_stop()

	# setup ros node
	rospy.init_node('jetbot_motors')
	
	rospy.Subscriber('~cmd_dir', String, on_cmd_dir)
	rospy.Subscriber('~cmd_raw', String, on_cmd_raw)
	rospy.Subscriber('~cmd_str', String, on_cmd_str)

	# start running
	rospy.spin()

	# stop motors before exiting
	all_stop()
