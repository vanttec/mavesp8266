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

#define POWER_ON_LEVEL (_control_pin_is_active_high ? HIGH : LOW)
#define POWER_OFF_LEVEL (_control_pin_is_active_high ? LOW : HIGH)
#define ACTIVE_LEVEL (_control_pin_is_active_high ? HIGH : LOW)
#define PASSIVE_LEVEL (_control_pin_is_active_high ? LOW : HIGH)

//---------------------------------------------------------------------------------
MavESP8266PowerMgmt::MavESP8266PowerMgmt()
    : _control_pin_index(0), _control_pin_is_active_high(false),
    _query_pin_index(0), _pulse_length_msec(0),
    _sending_pulse(false), _pulse_ends_at(0)
{
}

//---------------------------------------------------------------------------------
void
MavESP8266PowerMgmt::begin()
{
    if (_control_pin_index) {
        pinMode(_control_pin_index, OUTPUT);
    }

    if (_query_pin_index && _query_pin_index != _control_pin_index) {
        pinMode(_query_pin_index, INPUT);
    }

    writeControlPin(shouldSendPulse() ? PASSIVE_LEVEL : POWER_ON_LEVEL, /* log = */ false);
}

//---------------------------------------------------------------------------------
void
MavESP8266PowerMgmt::loop()
{
    if (_sending_pulse) {
        uint32_t now = millis();
        if (now > _pulse_ends_at) {
            finishPulse();
        }
    }
}

//---------------------------------------------------------------------------------
bool
MavESP8266PowerMgmt::isPowerOn(bool default_value) const
{
    if (supportsReadingPowerState()) {
        if (_query_pin_index > 0) {
            return digitalRead(_query_pin_index);
        } else {
            return digitalRead(_control_pin_index) == POWER_ON_LEVEL;
        }
    } else {
        return default_value;
    }
}

//---------------------------------------------------------------------------------
void 
MavESP8266PowerMgmt::setControlPinIsActiveHigh(bool value)
{
    _control_pin_is_active_high = value;
}

//---------------------------------------------------------------------------------
void
MavESP8266PowerMgmt::setPulseLengthMsec(uint16_t value)
{
    _pulse_length_msec = value;
}

//---------------------------------------------------------------------------------
void
MavESP8266PowerMgmt::setControlPinIndex(uint8_t index)
{
    _control_pin_index = index;
}

//---------------------------------------------------------------------------------
void
MavESP8266PowerMgmt::setQueryPinIndex(uint8_t index)
{
    _query_pin_index = index;
}

//---------------------------------------------------------------------------------
bool
MavESP8266PowerMgmt::supportsReadingPowerState() const
{
    return _query_pin_index > 0 || (_control_pin_index > 0 && _pulse_length_msec == 0);
}


//---------------------------------------------------------------------------------
//-- Turn off power to the UAS flight controller
//-- Returns true if successful, false otherwise.
bool
MavESP8266PowerMgmt::requestPowerOff()
{
    if (_control_pin_index == 0) {
        return false;
    } else if (shouldSendPulse()) {
        if (isPowerOn(/* default = */ true)) {
            startPulse();
        }
        return true;
    } else {
        writeControlPin(POWER_OFF_LEVEL);
        return true;
    }
}

//---------------------------------------------------------------------------------
//-- Turn on power to the UAS flight controller
//-- Returns true if successful, false otherwise.
bool
MavESP8266PowerMgmt::requestPowerOn()
{
    if (_control_pin_index == 0) {
        return false;
    } else if (shouldSendPulse()) {
        if (!isPowerOn(/* default = */ false)) {
            startPulse();
        }
        return true;
    } else {
        writeControlPin(POWER_ON_LEVEL);
        return true;
    }
}

//---------------------------------------------------------------------------------
//-- Returns whether the power management module responds to pulses instead of
//-- simply setting the GPIO pin to low or high
bool
MavESP8266PowerMgmt::shouldSendPulse() const
{
    return _control_pin_index > 0 && _pulse_length_msec > 0;
}

//---------------------------------------------------------------------------------
//-- Starts sending a new pulse on the GPIO pin
void
MavESP8266PowerMgmt::startPulse()
{
    // TODO(ntamas): how shall we handle if another pulse is already in progress?
    if (_control_pin_index == 0) {
        return;
    }

    writeControlPin(ACTIVE_LEVEL);
    _pulse_ends_at = millis() + static_cast<uint32_t>(_pulse_length_msec);
    _sending_pulse = true;
}

//---------------------------------------------------------------------------------
//-- Finishes the current pulse on the GPIO pin
void
MavESP8266PowerMgmt::finishPulse()
{
    if (!_sending_pulse) {
        return;
    }

    writeControlPin(PASSIVE_LEVEL);
    _pulse_ends_at = 0;
    _sending_pulse = false;
}

void
MavESP8266PowerMgmt::writeControlPin(bool value, bool log)
{
    if (_control_pin_index == 0) {
        return;
    }

    digitalWrite(_control_pin_index, value);

    if (log) {
        getWorld()->getLogger()->log("Power pin %s at %d\n", value ? "high": "low", millis());
    }
}
