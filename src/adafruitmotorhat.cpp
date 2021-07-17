/**
 *  adafruitmotorhat.cpp
 *
 *  MIT License
 *
 *  Copyright (c) 2018, Tom Clarke
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

#include "adafruitmotorhat.h"

AdafruitMotorHAT::AdafruitMotorHAT (int bus, char address)
    : controller (bus, address)
{
    // add the dc motors
    for (int i = 0; i < 4; ++i)
    {
        dcMotors.push_back (std::make_shared<AdafruitDCMotor> (controller, i));
    }

    // set the frequency
    controller.setFrequency (frequency);
}

std::shared_ptr<AdafruitDCMotor> AdafruitMotorHAT::getMotor (unsigned int number)
{
    if (number > 0 && number <= dcMotors.size())
        return std::shared_ptr<AdafruitDCMotor> (dcMotors[number - 1]);
    else
        return std::shared_ptr<AdafruitDCMotor>();
}
