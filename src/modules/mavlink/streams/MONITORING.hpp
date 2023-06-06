/****************************************************************************
 *
 *   Copyright (c) 2020 PX4 Development Team. All rights reserved.
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
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
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

#ifndef MONITORING_HPP
#define MONITORING_HPP

#include <uORB/topics/px4io_status.h>
#include <uORB/topics/actuator_armed.h>
#include <uORB/topics/battery_status.h>

class MavlinkStreamMonitoring : public MavlinkStream
{
public:
    static MavlinkStream *new_instance(Mavlink *mavlink) { return new MavlinkStreamMonitoring(mavlink); }

    static constexpr const char *get_name_static() { return "MONITORING"; }
    static constexpr uint16_t get_id_static() { return MAVLINK_MSG_ID_MONITORING; }

    const char *get_name() const override { return get_name_static(); }
    uint16_t get_id() override { return get_id_static(); }

    unsigned get_size() override
    {
        return _battery_status_sub.advertised() ? MAVLINK_MSG_ID_MONITORING_LEN +
                                                  MAVLINK_NUM_NON_PAYLOAD_BYTES : 0;
    }

private:
    explicit MavlinkStreamMonitoring(Mavlink *mavlink) : MavlinkStream(mavlink) {}

    uORB::Subscription _px4io_status_sub{ORB_ID(px4io_status)};
    uORB::Subscription _actuator_armed_sub{ORB_ID(actuator_armed)};
    uORB::Subscription _battery_status_sub{ORB_ID(battery_status)};

    bool send() override
    {
        battery_status_s battery_status{};

        if (_battery_status_sub.update(&battery_status)) {
            _battery_status_sub.copy(&battery_status);

            px4io_status_s px4io_status{};
            _px4io_status_sub.copy(&px4io_status);
            actuator_armed_s actuator_armed{};
            _actuator_armed_sub.copy(&actuator_armed);

            mavlink_monitoring_t msg{};

            msg.ready = actuator_armed.ready_to_arm;
            msg.battery = roundf(battery_status.remaining * 100.f);
            msg.safety = px4io_status.status_safety_button_event;

            mavlink_msg_monitoring_send_struct(_mavlink->get_channel(), &msg);

            return true;
        }

        return false;
    }
};

#endif // MONITORING_HPP