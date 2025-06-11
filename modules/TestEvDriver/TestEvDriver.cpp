// SPDX-License-Identifier: Apache-2.0
#include "TestEvDriver.hpp"

#include <fcntl.h>

namespace module {

static types::board_support_common::BspEvent cast_cp_type(std::string cp_state) {
    types::board_support_common::BspEvent event;
    if ("A" == cp_state) {
        event.event = types::board_support_common::Event::A;
    } else if ("B" == cp_state) {
        event.event = types::board_support_common::Event::B;
    } else if ("C" == cp_state) {
        event.event = types::board_support_common::Event::C;
    } else if ("D" == cp_state) {
        event.event = types::board_support_common::Event::D;
    } else if ("E" == cp_state) {
        event.event = types::board_support_common::Event::E;
    } else if ("F" == cp_state) {
        event.event = types::board_support_common::Event::F;
    }
    return event;
}

static types::board_support_common::ProximityPilot cast_pp_type(std::string pp_state) {
    types::board_support_common::ProximityPilot pp;
    if ("13A" == pp_state) {
        pp.ampacity = types::board_support_common::Ampacity::A_13;
    } else if ("20A" == pp_state) {
        pp.ampacity = types::board_support_common::Ampacity::A_20;
    } else if ("32A" == pp_state) {
        pp.ampacity = types::board_support_common::Ampacity::A_32;
    } else if ("63A-3P_70A-1P" == pp_state) {
        pp.ampacity = types::board_support_common::Ampacity::A_63_3ph_70_1ph;
    } else {
        pp.ampacity = types::board_support_common::Ampacity::None;
    }
    return pp;
}

void TestEvDriver::init() {
    invoke_init(*p_ev_board_support);
    fd = 0;
    baud = 0;
    if (!openDevice(config.serial_port.c_str(), config.baud_rate)) {
        EVLOG_error << "Could not open serial port " << config.serial_port << " with baud rate " << config.baud_rate;
        return;
    }
    setConnect(false);
    setCharge(false);
    setDiodeFault(false);
    setPowerPath(false);
    readThreadHandle = std::thread(&TestEvDriver::readThread, this);
}

void TestEvDriver::ready() {
    invoke_ready(*p_ev_board_support);
}

TestEvDriver::~TestEvDriver() {
    if (fd)
        close(fd);
}

bool TestEvDriver::openDevice(const char* device, int _baud) {
    fd = open(device, O_RDWR | O_NOCTTY | O_SYNC);
    if (fd < 0) {
        printf("Serial: error %d opening %s: %s\n", errno, device, strerror(errno));
        return false;
    } else {
        printf ("Serial: opened %s as %i\n", device, fd);
    }

    switch (_baud) {
        case 9600:
            baud = B9600;
            break;
        case 19200:
            baud = B19200;
            break;
        case 38400:
            baud = B38400;
            break;
        case 57600:
            baud = B57600;
            break;
        case 115200:
            baud = B115200;
            break;
        case 230400:
            baud = B230400;
            break;
        default:
            baud = 0;
            return false;
    }

    return setSerialAttributes();
}

bool TestEvDriver::setSerialAttributes() {
    struct termios tty;
    if (tcgetattr(fd, &tty) != 0) {
        printf("Serial: error %d from tcgetattr\n", errno);
        return false;
    }

    cfsetospeed(&tty, baud);
    cfsetispeed(&tty, baud);

    tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8; // 8-bit chars
    // disable IGNBRK for mismatched speed tests; otherwise receive break
    // as \000 chars
    tty.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL | IXON | IXOFF | IXANY);
    tty.c_lflag = 0;     // no signaling chars, no echo,
                         // no canonical processing
    tty.c_oflag = 0;     // no remapping, no delays
    tty.c_cc[VMIN] = 0;  // read blocks
    tty.c_cc[VTIME] = 5; // 0.5 seconds read timeout

    tty.c_cflag |= (CLOCAL | CREAD);   // ignore modem controls,
                                       // enable reading
    tty.c_cflag &= ~(PARENB | PARODD); // shut off parity
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag &= ~CRTSCTS;

    if (tcsetattr(fd, TCSANOW, &tty) != 0) {
        printf("Serial: error %d from tcsetattr\n", errno);
        return false;
    }
    return true;
}

bool TestEvDriver::setConnect(bool value) {
    static char buf[32];
    int len = std::snprintf(buf, sizeof buf, "CMD,CONNECT=%u\n", value ? 1 : 0);
    write(fd, buf, len);
    return true;
}

bool TestEvDriver::setCharge(bool value) {
    static char buf[32];
    int len = std::snprintf(buf, sizeof buf, "CMD,CHARGE=%u\n", value ? 1 : 0);
    write(fd, buf, len);
    return true;
}

bool TestEvDriver::setDiodeFault(bool value) {
    static char buf[32];
    int len = std::snprintf(buf, sizeof buf, "CMD,DIODE_FAULT=%u\n", value ? 1 : 0);
    write(fd, buf, len);
    return true;
}

bool TestEvDriver::setPowerPath(bool value) {
    static char buf[32];
    int len = std::snprintf(buf, sizeof buf, "CMD,POWER_PATH=%u\n", value ? 1 : 0);
    write(fd, buf, len);
    return true;
}

void TestEvDriver::readThread() {
    uint8_t buf[1024];
    size_t bufLen = 0;

    std::regex pattern_CP(R"(STATE,CP=(A|B|C|D|E|F))");
    std::regex pattern_PP(R"(STATE,PP=(DISCONNECTED|LATCHED|UNLATCHED|13A|20A|32A|63A-3P_70A-1P))");
    std::regex pattern_DUTY_CYCLE(R"(STATE,DUTY_CYCLE=((100(?:\.0{1,2})?|(?:[0-9]{1,2})(?:\.\d{1,2})?)))");

    while (true) {
        if (readThreadHandle.shouldExit())
            break;

        if (fd > 0 && bufLen < sizeof(buf)) {
            int n = read(fd, buf + bufLen, sizeof(buf) - bufLen);
            if (n > 0) {
                bufLen += n;

                size_t lineStart = 0;
                for (size_t lineEnd = 0; lineEnd < bufLen; ++lineEnd) {
                    if (buf[lineEnd] == '\n') {
                        std::string line(reinterpret_cast<const char*>(buf + lineStart), lineEnd - lineStart);

                        std::cmatch match;
                        if (std::regex_match(line.c_str(), match, pattern_CP)) {
                            std::string value = match[1];
                            last_cp = cast_cp_type(value);
                            p_ev_board_support->publish_bsp_event(last_cp);

                            types::board_support_common::BspMeasurement bspm;
                            bspm.cp_pwm_duty_cycle = last_duty_cycle;
                            bspm.proximity_pilot = last_pp;
                            p_ev_board_support->publish_bsp_measurement(bspm);
                        } else if (std::regex_match(line.c_str(), match, pattern_PP)) {
                            std::string value = match[1];
                            last_pp = cast_pp_type(value);
                        } else if (std::regex_match(line.c_str(), match, pattern_DUTY_CYCLE)) {
                            std::string value = match[1];
                            last_duty_cycle = std::stod(value);
                        } else {
                            EVLOG_error << "Invalid message format: " << line;
                        }

                        lineStart = lineEnd + 1; // advance to next line and skip \n
                    }
                }

                if (lineStart < bufLen) {
                    size_t remaining = bufLen - lineStart;
                    std::memmove(buf, buf + lineStart, remaining);
                    bufLen = remaining;
                } else {
                    bufLen = 0;
                }
            }
        }
    }
}

} // namespace module
