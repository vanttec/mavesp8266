/****************************************************************************
 *
 * Copyright (c) 2015, 2016 Gus Grubba. All rights reserved.
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
 * @file mavesp8266_vehicle.cpp
 * ESP8266 Wifi AP, MavLink UART/UDP Bridge
 *
 * @author Gus Grubba <mavlink@grubba.com>
 */

#include "mavesp8266.h"
#include "mavesp8266_vehicle.h"
#include "mavesp8266_parameters.h"
#include "mavesp8266_component.h"

//---------------------------------------------------------------------------------
MavESP8266Vehicle::MavESP8266Vehicle()
{
    _recv_chan = MAVLINK_COMM_0;
    _send_chan = MAVLINK_COMM_1;
}

//---------------------------------------------------------------------------------
//-- Initialize
void
MavESP8266Vehicle::begin(MavESP8266Bridge* forwardTo)
{
    MavESP8266Bridge::begin(forwardTo);
    //-- Start UART connected to UAS
    Serial.begin(getWorld()->getParameters()->getUartBaudRate());
    //-- Swap to TXD2/RXD2 (GPIO015/GPIO013) For ESP12 Only
#ifdef ENABLE_DEBUG
#ifdef ARDUINO_ESP8266_ESP12
    Serial.swap();
#endif
#endif
    // raise serial buffer size (default is 256)
    Serial.setRxBufferSize(4096);
}


//---------------------------------------------------------------------------------
//-- Read MavLink message from UAS
void
MavESP8266Vehicle::readMessage()
{
    if (_readMessage()) {
        _forwardTo->sendMessage(&_msg);
    }
    //-- Update radio status (1Hz)
    if(_heard_from && (millis() - _last_status_time > 1000)) {
        delay(0);
        _last_status_time = millis();
    }
}

void
MavESP8266Vehicle::readMessageRaw() {
    static uint8_t buf[1024];
    int buf_index = 0;

    while(Serial.available() && buf_index < 300)
    {
        int result = Serial.read();
        if (result >= 0)
        {
            buf[buf_index] = result;
            buf_index++;
        }
    }

    if (buf_index > 0) {
        _forwardTo->sendMessageRaw(buf, buf_index);
    }
}

//---------------------------------------------------------------------------------
//-- Send MavLink message to UAS
int
MavESP8266Vehicle::sendMessage(mavlink_message_t* message) {
    // Translate message to buffer
    char buf[300];
    unsigned len = mavlink_msg_to_send_buffer((uint8_t*)buf, message);
    // Send it
    while (Serial.availableForWrite() < 32) {
        // don't spin in the send loop, wait for 25% of the FIFO to be free
        delay(1);
    }
    Serial.write((uint8_t*)(void*)buf, len);
    _status.packets_sent++;
    return 1;
}

int
MavESP8266Vehicle::sendMessageRaw(uint8_t *buffer, int len) {
    Serial.write(buffer, len);
    //Serial.flush();
    return len;
}

//---------------------------------------------------------------------------------
//-- We have some special status to capture when asked for
linkStatus*
MavESP8266Vehicle::getStatus()
{
    _status.queue_status = 0;
    return &_status;
}

//---------------------------------------------------------------------------------
//-- Returns whether the UAS is likely to be armed
bool
MavESP8266Vehicle::isArmed()
{
    return _armed;
}

//---------------------------------------------------------------------------------
//-- Returns whether the UAS is likely to be powered on
bool
MavESP8266Vehicle::isPoweredOn()
{
#ifdef FC_POWER_PIN
    // We have a pin that can be used to turn the flight controller on or off
    // so use that
    return digitalRead(FC_POWER_PIN);
#else
    // We don't have such a pin so just track whether we've seen MAVLink
    // heartbeats from the FC
    return _heard_from;
#endif
}

//---------------------------------------------------------------------------------
//-- Read MavLink message from UAS
bool
MavESP8266Vehicle::_readMessage()
{
    bool msgReceived = false;
    int16_t avail = Serial.available();
    if (avail <= 0 && _non_mavlink_len != 0 && _rxstatus.parse_state <= MAVLINK_PARSE_STATE_IDLE) {
        // flush out the non-mavlink buffer when there is nothing pending. This
        // allows us to gather non-mavlink msgs into a single write
        _forwardTo->sendMessageRaw(_non_mavlink_buffer, _non_mavlink_len);
        _non_mavlink_len = 0;
    }
    while (avail--)
    {
        int result = Serial.read();
        if (result >= 0)
        {
            // Parsing
            uint8_t last_parse_error = _rxstatus.parse_error;
            msgReceived = mavlink_frame_char_buffer(&_rxmsg,
                                                    &_rxstatus,
                                                    result,
                                                    &_msg,
                                                    &_mav_status);
            handle_non_mavlink(result, msgReceived);
            if (last_parse_error != _rxstatus.parse_error) {
                _status.parse_errors++;
            }
            if(msgReceived) {
                _status.packets_received++;
                //-- Is this the first packet we got?
                if(!_heard_from) {
                    if(_msg.msgid == MAVLINK_MSG_ID_HEARTBEAT) {
                        _heard_from     = true;
                        _component_id   = _msg.compid;
                        _system_id      = _msg.sysid;
                        _seq_expected   = _msg.seq + 1;
                        _last_heartbeat = millis();
                    }
                } else {
                    if(_msg.msgid == MAVLINK_MSG_ID_HEARTBEAT)
                        _last_heartbeat = millis();
                    _checkLinkErrors(&_msg);
                }

                //-- Update the status of the 'armed' flag if we have a heartbeat
                if (_msg.msgid == MAVLINK_MSG_ID_HEARTBEAT) {
                    uint8_t system_status = mavlink_msg_heartbeat_get_system_status(&_msg);
                    uint8_t base_mode = mavlink_msg_heartbeat_get_base_mode(&_msg);
                    _armed = (
                        (
                            system_status == MAV_STATE_ACTIVE &&
                            (base_mode & MAV_MODE_FLAG_SAFETY_ARMED) != 0
                        ) ||
                        system_status == MAV_STATE_CRITICAL ||
                        system_status == MAV_STATE_EMERGENCY
                    );
                }

                if (msgReceived == MAVLINK_FRAMING_BAD_CRC) {
                    // we don't process messages locally with bad CRC,
                    // but we do forward them, so when new messages
                    // are added we can bridge them
                    break;
                }

#ifdef MAVLINK_FRAMING_BAD_SIGNATURE
                if (msgReceived == MAVLINK_FRAMING_BAD_SIGNATURE) {
                    break;
                }
#endif
                
                //-- Check for message we might be interested
                if(getWorld()->getComponent()->handleMessage(this, &_msg)){
                    //-- Eat message (don't send it to GCS)
                    msgReceived = false;
                    continue;
                }

                break;
            }
        }
    }
    if(!msgReceived) {
        if(_heard_from && (millis() - _last_heartbeat) > HEARTBEAT_TIMEOUT) {
            _heard_from = false;
            getWorld()->getLogger()->log("Heartbeat timeout from Vehicle\n");
        }
    }
    return msgReceived;
}

//---------------------------------------------------------------------------------
//-- Turn off power to the UAS flight controller
//-- Returns true if successful, false otherwise.
bool
MavESP8266Vehicle::requestPowerOff()
{
#ifdef FC_POWER_PIN
    digitalWrite(FC_POWER_PIN, LOW);
    return true;
#else
    return false;
#endif
}

//---------------------------------------------------------------------------------
//-- Turn on power to the UAS flight controller
//-- Returns true if successful, false otherwise.
bool
MavESP8266Vehicle::requestPowerOn()
{
#ifdef FC_POWER_PIN
    digitalWrite(FC_POWER_PIN, HIGH);
    return true;
#else
    return false;
#endif
}

