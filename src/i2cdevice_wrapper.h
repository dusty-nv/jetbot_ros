/**
 *  i2cdevice_wrapper.h
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

#pragma once

#include "linux/i2c-dev.h"
#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>
#include <errno.h>
#include <cstring>
#include <unistd.h>
#include "ros/ros.h"

class i2cdevice_wrapper
{
public:
    i2cdevice_wrapper (int bus, char addr)
    {
	this->bus = bus;
	this->addr = addr;
	char filename[20];
	snprintf(filename, 19, "/dev/i2c-%d", this->bus);
	this->file = open(filename, O_RDWR);
	if (this->file < 0) {
		ROS_INFO("Oh dear, something went wrong with open()! %s\n", std::strerror(errno));
		exit(EXIT_FAILURE);
	}

	this->addr = addr;

	if (ioctl(this->file, I2C_SLAVE, this->addr) < 0) {
		ROS_INFO("Oh dear, something went wrong with ioctl()! %s\n", std::strerror(errno));
		exit(EXIT_FAILURE);
	}
        valid = true;
    };

    /** 
     */
    void write8 (char reg, char value)
    {
	// Using SMBus commands
	const __s32 result = i2c_smbus_write_byte_data(this->file, reg, value);
	if (result < 0) {
		// ERROR HANDLING: i2c transaction failed
		ROS_INFO("Oh dear, something went wrong with i2c_smbus_write_byte_data()>i2c_smbus_access()>ioctl()! %s\n", strerror(errno));
		this->valid = false;
		exit(EXIT_FAILURE);
	} else {
		// res contains the read word
		//printf("0x%02x\n", result);
	}
    }

    /** 
     */
    int read8 (char reg)
    {
	// Using SMBus commands
	const __s32 result = i2c_smbus_read_byte_data(this->file, reg);
	if (result < 0) {
		// ERROR HANDLING: i2c transaction failed
		ROS_INFO("Oh dear, something went wrong with i2c_smbus_read_byte_data()>i2c_smbus_access()>ioctl()! %s\n", strerror(errno));
		this->valid = false;
		exit(EXIT_FAILURE);
	} else {
		// res contains the read word
		//printf("0x%02x\n", result);
	}
   };
   bool isValid()
   {
	return valid;
   };
   ~i2cdevice_wrapper()
   {
	close(file);
   };
private:
   int bus = 1;
   char addr = 0x60;
   int file;
   bool valid = false;
};
