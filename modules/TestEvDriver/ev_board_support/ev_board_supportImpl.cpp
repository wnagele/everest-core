// SPDX-License-Identifier: Apache-2.0

#include "ev_board_supportImpl.hpp"

namespace module {
namespace ev_board_support {

void ev_board_supportImpl::init() {
    EVLOG_error << "init";
}

void ev_board_supportImpl::ready() {
    EVLOG_error << "ready";
}

void ev_board_supportImpl::handle_enable(bool& value) {
    EVLOG_error << "handle_enable: " << value;
}

void ev_board_supportImpl::handle_set_cp_state(types::ev_board_support::EvCpState& cp_state) {
    EVLOG_error << "handle_set_cp_state: " << cp_state;
}

void ev_board_supportImpl::handle_allow_power_on(bool& value) {
    EVLOG_error << "handle_allow_power_on: " << value;
}

void ev_board_supportImpl::handle_diode_fail(bool& value) {
    EVLOG_error << "handle_diode_fail: " << value;
}

void ev_board_supportImpl::handle_set_ac_max_current(double& current) {
    EVLOG_error << "handle_set_ac_max_current: " << current;
}

void ev_board_supportImpl::handle_set_three_phases(bool& three_phases) {
    EVLOG_error << "handle_set_three_phases: " << three_phases;
}

void ev_board_supportImpl::handle_set_rcd_error(double& rcd_current_mA) {
    EVLOG_error << "handle_set_rcd_error: " << rcd_current_mA;
}

} // namespace ev_board_support
} // namespace module
