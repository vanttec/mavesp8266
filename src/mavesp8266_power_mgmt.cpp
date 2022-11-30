/****************************************************************************
 *
 * Copyright (c) 2022, CollMot Robotics Ltd. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file mavesp8266_power_mgmt.cpp
 * ESP8266 Wifi AP, MavLink UART/UDP Bridge
 *
 * @author Tamas Nepusz <tamas@collmot.com>
 */

#include <Arduino.h>
#include "mavesp8266_power_mgmt.h"

#define POWER_ON_LEVEL (_is_active_high ? HIGH : LOW)
#define POWER_OFF_LEVEL (_is_active_high ? LOW : HIGH)

//---------------------------------------------------------------------------------
MavESP8266PowerMgmt::MavESP8266PowerMgmt()
    : _pin_index(0), _is_active_high(false)
{
}

//---------------------------------------------------------------------------------
void
MavESP8266PowerMgmt::begin()
{
    pinMode(_pin_index, OUTPUT);
    digitalWrite(_pin_index, _is_active_high ? HIGH : LOW);
}

//---------------------------------------------------------------------------------
bool
MavESP8266PowerMgmt::isPowerOn(bool default_value) const
{
    if (supportsReadingPowerState()) {
        return digitalRead(_pin_index) == POWER_ON_LEVEL;
    } else {
        return default_value;
    }
}

//---------------------------------------------------------------------------------
void 
MavESP8266PowerMgmt::setPinIsActiveHigh(bool value)
{
    _is_active_high = value;
}

//---------------------------------------------------------------------------------
void
MavESP8266PowerMgmt::setPinIndex(uint8_t index)
{
    _pin_index = index;
}

//---------------------------------------------------------------------------------
bool
MavESP8266PowerMgmt::supportsReadingPowerState() const
{
    return _pin_index > 0;    
}


//---------------------------------------------------------------------------------
//-- Turn off power to the UAS flight controller
//-- Returns true if successful, false otherwise.
bool
MavESP8266PowerMgmt::requestPowerOff()
{
    if (_pin_index == 0) {
        return false;
    } else {
        digitalWrite(_pin_index, POWER_OFF_LEVEL);
        return true;
    }
}

//---------------------------------------------------------------------------------
//-- Turn on power to the UAS flight controller
//-- Returns true if successful, false otherwise.
bool
MavESP8266PowerMgmt::requestPowerOn()
{
    if (_pin_index == 0) {
        return false;
    } else {
        digitalWrite(_pin_index, POWER_ON_LEVEL);
        return true;
    }
}
