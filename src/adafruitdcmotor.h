/**
 *  adafruitdcmotor.h
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

class AdafruitDCMotor
{
public:
    enum Command
    {
        kForward = 1,
        kBackward = 2,
        kBrake = 3,
        kRelease = 4,
    };

    AdafruitDCMotor (PWM& pwm, int index);

    /** Makes the motor perform an action.
     *  @see Commands
     */
    void run (Command command);

    /** Sets the speed of the motor.
     *  Expects a value between 0 and 255 inclusive.
     */
    void setSpeed (int speed);

private:
    void setPin (int pin, bool enabled);

    PWM& controller;
    int pwmPin = 0, in1Pin = 0, in2Pin = 0;
};
