// SPDX-License-Identifier: Apache-2.0
#include "TestEvDriver.hpp"

#include <fcntl.h>

namespace module {

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
    while (true) {
        if (readThreadHandle.shouldExit())
            break;
    }
}

} // namespace module
