// SPDX-License-Identifier: Apache-2.0
#ifndef TEST_EV_DRIVER_HPP
#define TEST_EV_DRIVER_HPP

//
// AUTO GENERATED - MARKED REGIONS WILL BE KEPT
// template version 2
//

#include "ld-ev.hpp"

// headers for provided interface implementations
#include <generated/interfaces/ev_board_support/Implementation.hpp>

// ev@4bf81b14-a215-475c-a1d3-0a484ae48918:v1
#include <termios.h>
#include <utils/thread.hpp>
// ev@4bf81b14-a215-475c-a1d3-0a484ae48918:v1

namespace module {

struct Conf {
    std::string serial_port;
    int baud_rate;
};

class TestEvDriver : public Everest::ModuleBase {
public:
    TestEvDriver() = delete;
    TestEvDriver(const ModuleInfo& info, Everest::TelemetryProvider& telemetry,
                 std::unique_ptr<ev_board_supportImplBase> p_ev_board_support, Conf& config) :
        ModuleBase(info), telemetry(telemetry), p_ev_board_support(std::move(p_ev_board_support)), config(config){};
    ~TestEvDriver();

    Everest::TelemetryProvider& telemetry;
    const std::unique_ptr<ev_board_supportImplBase> p_ev_board_support;
    const Conf& config;

    // ev@1fce4c5e-0ab8-41bb-90f7-14277703d2ac:v1
    void readThread();
    bool openDevice(const char* device, int baud);
    bool setConnect(bool value);
    bool setCharge(bool value);
    bool setDiodeFault(bool value);
    bool setPowerPath(bool value);
    // ev@1fce4c5e-0ab8-41bb-90f7-14277703d2ac:v1

protected:
    // ev@4714b2ab-a24f-4b95-ab81-36439e1478de:v1
    // insert your protected definitions here
    // ev@4714b2ab-a24f-4b95-ab81-36439e1478de:v1

private:
    friend class LdEverest;
    void init();
    void ready();

    // ev@211cfdbe-f69a-4cd6-a4ec-f8aaa3d1b6c8:v1
    bool setSerialAttributes();
    int fd;
    int baud;
    Everest::Thread readThreadHandle;
    // ev@211cfdbe-f69a-4cd6-a4ec-f8aaa3d1b6c8:v1
};

// ev@087e516b-124c-48df-94fb-109508c7cda9:v1
// insert other definitions here
// ev@087e516b-124c-48df-94fb-109508c7cda9:v1

} // namespace module

#endif // TEST_EV_DRIVER_HPP
