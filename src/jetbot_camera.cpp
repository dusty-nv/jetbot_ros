/*
 * Copyright (c) 2019, NVIDIA CORPORATION. All rights reserved.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a
 * copy of this software and associated documentation files (the "Software"),
 * to deal in the Software without restriction, including without limitation
 * the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.  IN NO EVENT SHALL
 * THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
 * DEALINGS IN THE SOFTWARE.
 */

#include <ros/ros.h>

#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>

#include <jetson-utils/gstCamera.h>

#include "image_converter.h"



// globals
#define DEFAULT_CAMERA -1	// -1 for onboard camera, or change to index of /dev/video V4L2 camera (>=0)	
		
gstCamera* camera = NULL;

imageConverter* camera_cvt = NULL;
ros::Publisher* camera_pub = NULL;


// aquire and publish camera frame
bool aquireFrame()
{
	void* imgCPU  = NULL;
	void* imgCUDA = NULL;
	void* imgRGBA = NULL;

	// get the latest frame
	if( !camera->Capture(&imgCPU, &imgCUDA, 1000) )
	{
		ROS_ERROR("failed to capture camera frame");
		return false;
	}
//	for(int i = 0 ; i < 640*480;i++)
//		if(((unsigned char*)(imgCPU))[i]!=0)
//			ROS_INFO("%d %d",i,((unsigned char*)(imgCPU))[i]);

	// convert from YUV to RGBA
	if( !camera->ConvertBGR8(imgCUDA, &imgRGBA,true) )
	{
		ROS_ERROR("failed to convert from NV12 to RGBA");
		return false;
	}


	// assure correct image size
	if( !camera_cvt->Resize(camera->GetWidth(), camera->GetHeight()) )
	{
		ROS_ERROR("failed to resize camera image converter");
		return false;
	}

	// populate the message
	sensor_msgs::Image msg;
/*	if( !camera_cvt->Convert(msg, sensor_msgs::image_encodings::BGR8) )
	{
		ROS_ERROR("failed to convert camera frame to sensor_msgs::Image");
		return false;

}*/
	msg.width = camera->GetWidth();
	msg.height = camera->GetHeight();
	msg.encoding = "bgr8";
	msg.step = sizeof(unsigned char)*camera->GetWidth()*3;

	msg.data.push_back(0);
	msg.data.resize(sizeof(unsigned char)*msg.width*3*msg.height);
memcpy(msg.data.data(),(unsigned char*)imgRGBA,sizeof(unsigned char)*msg.width*msg.height*3);

	// publish the message
	camera_pub->publish(msg);
	ROS_INFO("published camera frame");
	return true;
}


// node main loop
int main(int argc, char **argv)
{
	ros::init(argc, argv, "jetbot_camera");
 
	ros::NodeHandle nh;
	ros::NodeHandle private_nh("~");

	/*
	 * retrieve parameters
	 */
	int camera_index = -1;

	private_nh.param<int>("camera_index", camera_index, camera_index);
	
	ROS_INFO("opening camera device %i", camera_index);

	
	/*
	 * open camera device
	 */
	camera = gstCamera::Create(640,480,camera_index);

	if( !camera )
	{
		ROS_ERROR("failed to open camera device %i", camera_index);
		return 0;
	}


	/*
	 * create image converter
	 */
	camera_cvt = new imageConverter();

	if( !camera_cvt )
	{
		ROS_ERROR("failed to create imageConverter");
		return 0;
	}


	/*
	 * advertise publisher topics
	 */
	ros::Publisher camera_publisher = private_nh.advertise<sensor_msgs::Image>("raw", 2);
	camera_pub = &camera_publisher;


	/*
	 * start the camera streaming
	 */
	if( !camera->Open() )
	{
		ROS_ERROR("failed to start camera streaming");
		return 0;
	}


	/*
	 * start publishing video frames
	 */
	while( ros::ok() )
	{
		//if( raw_pub->getNumSubscribers() > 0 )
			aquireFrame();

		ros::spinOnce();
	}

	delete camera;
	return 0;
}

