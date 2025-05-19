// SPDX-License-Identifier: Apache-2.0

#include "evse_board_supportImpl.hpp"

namespace module {
namespace board_support {

void evse_board_supportImpl::init() {
    EVLOG_error << "init";
    caps.min_current_A_import = 1;
    caps.max_current_A_import = 25;
    caps.min_phase_count_import = 1;
    caps.max_phase_count_import = 3;
    caps.supports_changing_phases_during_charging = false;
    caps.connector_type = types::evse_board_support::Connector_type::IEC62196Type2Cable;
    caps.min_current_A_export = 1;
    caps.max_current_A_export = 25;
    caps.min_phase_count_export = 1;
    caps.max_phase_count_export = 3;
}

void evse_board_supportImpl::ready() {
    EVLOG_error << "ready";
    publish_capabilities(caps);
}

void evse_board_supportImpl::handle_enable(bool& value) {
    EVLOG_error << "handle_enable: " << value;
    mod->enable(value);
}

void evse_board_supportImpl::handle_pwm_on(double& value) {
    EVLOG_error << "handle_pwm_on: " << value;
    mod->setDutyCycle(value);
}

void evse_board_supportImpl::handle_pwm_off() {
    EVLOG_error << "handle_pwm_off";
    mod->setDutyCycle(100);
}

void evse_board_supportImpl::handle_pwm_F() {
    EVLOG_error << "handle_pwm_F";
    mod->setDutyCycle(0);
}

void evse_board_supportImpl::handle_allow_power_on(types::evse_board_support::PowerOnOff& value) {
    EVLOG_error << "handle_allow_power_on: " << value;
    mod->setPowerPath(value.allow_power_on);
    types::board_support_common::BspEvent event;

    // signal contactor state
    if (value.allow_power_on) {
        event.event = types::board_support_common::Event::PowerOn;
    } else {
        event.event = types::board_support_common::Event::PowerOff;
    }
    publish_event(event);
}

void evse_board_supportImpl::handle_ac_switch_three_phases_while_charging(bool& value) {
    EVLOG_error << "handle_ac_switch_three_phases_while_charging: " << value;
}

void evse_board_supportImpl::handle_evse_replug(int& value) {
    EVLOG_error << "handle_evse_replug: " << value;
}

types::board_support_common::ProximityPilot evse_board_supportImpl::handle_ac_read_pp_ampacity() {
    EVLOG_error << "handle_ac_read_pp_ampacity";
    return mod->last_pp;
}

void evse_board_supportImpl::handle_ac_set_overcurrent_limit_A(double& value) {
    EVLOG_error << "handle_ac_set_overcurrent_limit_A: " << value;
}

} // namespace board_support
} // namespace module
