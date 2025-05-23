// SPDX-License-Identifier: Apache-2.0
// Copyright Pionix GmbH and Contributors to EVerest
#ifndef OCPP_GENERIC_OCPP_IMPL_HPP
#define OCPP_GENERIC_OCPP_IMPL_HPP

//
// AUTO GENERATED - MARKED REGIONS WILL BE KEPT
// template version 3
//

#include <generated/interfaces/ocpp/Implementation.hpp>

#include "../OCPP.hpp"

// ev@75ac1216-19eb-4182-a85c-820f1fc2c091:v1
// insert your custom include headers here
// ev@75ac1216-19eb-4182-a85c-820f1fc2c091:v1

namespace module {
namespace ocpp_generic {

struct Conf {};

class ocppImpl : public ocppImplBase {
public:
    ocppImpl() = delete;
    ocppImpl(Everest::ModuleAdapter* ev, const Everest::PtrContainer<OCPP>& mod, Conf& config) :
        ocppImplBase(ev, "ocpp_generic"), mod(mod), config(config) {
    }

    // ev@8ea32d28-373f-4c90-ae5e-b4fcc74e2a61:v1
    // insert your public definitions here
    // ev@8ea32d28-373f-4c90-ae5e-b4fcc74e2a61:v1

protected:
    // command handler functions (virtual)
    virtual bool handle_stop() override;
    virtual bool handle_restart() override;
    virtual void handle_security_event(types::ocpp::SecurityEvent& event) override;
    virtual std::vector<types::ocpp::GetVariableResult>
    handle_get_variables(std::vector<types::ocpp::GetVariableRequest>& requests) override;
    virtual std::vector<types::ocpp::SetVariableResult>
    handle_set_variables(std::vector<types::ocpp::SetVariableRequest>& requests, std::string& source) override;
    virtual types::ocpp::ChangeAvailabilityResponse
    handle_change_availability(types::ocpp::ChangeAvailabilityRequest& request) override;
    virtual void handle_monitor_variables(std::vector<types::ocpp::ComponentVariable>& component_variables) override;

    // ev@d2d1847a-7b88-41dd-ad07-92785f06f5c4:v1
    // insert your protected definitions here
    // ev@d2d1847a-7b88-41dd-ad07-92785f06f5c4:v1

private:
    const Everest::PtrContainer<OCPP>& mod;
    const Conf& config;

    virtual void init() override;
    virtual void ready() override;

    // ev@3370e4dd-95f4-47a9-aaec-ea76f34a66c9:v1
    // insert your private definitions here
    std::mutex chargepoint_state_mutex; // mutex used for start/stop operations
    // ev@3370e4dd-95f4-47a9-aaec-ea76f34a66c9:v1
};

// ev@3d7da0ad-02c2-493d-9920-0bbbd56b9876:v1
// insert other definitions here
// ev@3d7da0ad-02c2-493d-9920-0bbbd56b9876:v1

} // namespace ocpp_generic
} // namespace module

#endif // OCPP_GENERIC_OCPP_IMPL_HPP
