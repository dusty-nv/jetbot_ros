/**
 *  adafruitmotorhat.h
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

#pragma once

#include "pwm.h"
#include "adafruitdcmotor.h"

#include <memory>
#include <vector>

class AdafruitMotorHAT
{
public:
    AdafruitMotorHAT (int bus = 1, char address = 0x60);

    /** Get one of the DC motors controlled by the HAT.
     *  Expects a value between 1 and 4 inclusive.
     *  If the number is out-of-range, the shared pointer
     *  returned from the method will be empty.
     */
    std::shared_ptr<AdafruitDCMotor> getMotor (unsigned int number);

private:
    PWM controller;
    int frequency = 1600;
    std::vector<std::shared_ptr<AdafruitDCMotor>> dcMotors;
};
