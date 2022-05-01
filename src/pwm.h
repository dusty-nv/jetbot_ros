/**
 *  pwm.h
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

//#include "i2cdevice.h"
#include "i2cdevice_wrapper.h"

class PWM
{
public:
    // Registers/etc.
    enum Registers
    {
        kMode1       = 0x00,
        kMode2       = 0x01,
        kSubAddress1 = 0x02,
        kSubAddress2 = 0x03,
        kSubAddress3 = 0x04,
        kPreScale    = 0xFE,
        kLed0OnL     = 0x06,
        kLed0OnH     = 0x07,
        kLed0OffL    = 0x08,
        kLed0OffH    = 0x09,
        kAllLedOnL   = 0xFA,
        kAllLedOnH   = 0xFB,
        kAllLedOffL  = 0xFC,
        kAllLedOffH  = 0xFD,
    };

    // Bits
    enum Bits
    {
        kRestart     = 0x80,
        kSleep       = 0x10,
        kAllCall     = 0x01,
        kInvert      = 0x10,
        kOutDrive    = 0x04,
    };

    PWM (int busAddress, int deviceAddress);

    /** Sets the PWM frequency */
    void setFrequency (double frequency);

    /** Sets a single PWM channel */
    void setChannel (int channel, int on, int off);

    /** Sets all the PWM channels */
    void setAll (int on, int off);

private:
    i2cdevice_wrapper device;
};
