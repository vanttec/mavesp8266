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
 * @file mavesp8266_power_mgmt.h
 * ESP8266 Wifi AP, MavLink UART/UDP Bridge
 *
 * @author Tamas Nepusz <tamas@collmot.com>
 */

#ifndef MAVESP8266_POWER_MGMT_H
#define MAVESP8266_POWER_MGMT_H

#include "mavesp8266.h"

class MavESP8266PowerMgmt {
public:
    MavESP8266PowerMgmt();

public:
    void begin();
    void loop();
    bool isPowerOn(bool default_value = true) const;
    bool requestPowerOff();
    bool requestPowerOn();
    void setControlPinIndex(uint8_t index);
    void setControlPinIsActiveHigh(bool value);
    void setPulseLengthMsec(uint16_t value);
    void setQueryPinIndex(uint8_t index);
    bool supportsReadingPowerState() const;

private:
    uint8_t _control_pin_index;
    bool _control_pin_is_active_high;
    uint8_t _query_pin_index;
    uint16_t _pulse_length_msec;
    bool _sending_pulse;
    uint32_t _pulse_ends_at;

    void startPulse();
    void finishPulse();
    bool shouldSendPulse() const;

    void writeControlPin(bool value, bool log = true);
};

#endif
