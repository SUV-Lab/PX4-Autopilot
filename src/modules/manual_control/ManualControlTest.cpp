/****************************************************************************
 *
 *   Copyright (c) 2023 PX4 Development Team. All rights reserved.
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

#define MODULE_NAME "manual_control"

#include <gtest/gtest.h>
#include "ManualControl.hpp"

static constexpr uint8_t ACTION_KILL = action_request_s::ACTION_KILL;
static constexpr uint8_t ACTION_UNKILL = action_request_s::ACTION_UNKILL;

class TestManualControl : public ManualControl
{
public:
	void processInput() { ManualControl::processInput(); }
};

class SwitchTest : public ::testing::Test
{
public:
	void SetUp() override
	{
		// Disable autosaving parameters to avoid busy loop in param_set()
		param_control_autosave(false);

		// Set stick input timeout to half a second
		const float com_rc_loss_t = .5f;
		param_set(param_find("COM_RC_LOSS_T"), &com_rc_loss_t);
	}

	uORB::PublicationData<manual_control_switches_s> _manual_control_switches_pub{ORB_ID(manual_control_switches)};
	uORB::PublicationData<manual_control_setpoint_s> _manual_control_input_pub{ORB_ID(manual_control_input)};
	uORB::SubscriptionData<manual_control_setpoint_s> _manual_control_setpoint_sub{ORB_ID(manual_control_setpoint)};
	uORB::SubscriptionData<action_request_s> _action_request_sub{ORB_ID(action_request)};

	TestManualControl _manual_control;
};


TEST_F(SwitchTest, KillSwitch)
{
	// GIVEN: valid stick input from the RC
	_manual_control_input_pub.get().data_source = manual_control_setpoint_s::SOURCE_RC;
	_manual_control_input_pub.get().valid = true;
	_manual_control_input_pub.get().timestamp_sample = hrt_absolute_time();
	_manual_control_input_pub.update();

	// GIVEN: a kill switch in off position
	_manual_control_switches_pub.get().kill_switch = manual_control_switches_s::SWITCH_POS_OFF;
	_manual_control_switches_pub.get().timestamp_sample = hrt_absolute_time();
	_manual_control_switches_pub.update();

	_manual_control.processInput();

	// WHEN: the kill switch is switched on
	_manual_control_switches_pub.get().kill_switch = manual_control_switches_s::SWITCH_POS_ON;
	_manual_control_switches_pub.update();
	_manual_control.processInput();

	// THEN: the stick input is published for use
	EXPECT_TRUE(_manual_control_setpoint_sub.update());
	EXPECT_TRUE(_manual_control_setpoint_sub.get().valid);

	// THEN: a kill action request is published
	EXPECT_TRUE(_action_request_sub.update());
	EXPECT_EQ(_action_request_sub.get().action, ACTION_KILL);

	// WHEN: the kill switch is switched off again
	_manual_control_switches_pub.get().kill_switch = manual_control_switches_s::SWITCH_POS_OFF;
	_manual_control_switches_pub.update();
	_manual_control.processInput();

	// THEN: an unkill action request is published
	EXPECT_TRUE(_action_request_sub.update());
	EXPECT_EQ(_action_request_sub.get().action, ACTION_UNKILL);
}
