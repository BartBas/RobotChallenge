/**
 * @file MotorController.cpp
 * @brief Implementation of serial initialisation and packet encoding for the
 *        motor controller.
 *
 * @details
 * Handles the low-level UART setup and binary packet construction. Key
 * implementation notes:
 *
 * ### Serial Initialisation
 * The port is opened in raw, non-blocking mode (`O_NDELAY`) and immediately
 * reconfigured to 115 200 8N1 via `tcsetattr()`. A 1-second read timeout is
 * set through `VTIME` so that `readResponse()` does not block indefinitely.
 * The port file descriptor is stored in `serial_port`; a value of -1 indicates
 * that the port failed to open and all subsequent operations will be no-ops
 * with an error logged to stderr.
 *
 * ### Packet Encoding (`sendPacket`)
 * Fields are packed into a 20-bit value and split across three payload bytes
 * (bytes 1–3 of the 4-byte frame; byte 0 is the `0xFF` sync byte):
 * - `cmd_enable` is placed at bit 19.
 * - `direction` occupies bits 18–10 (9-bit field, pre-masked with `0x1FF`).
 * - `turn` occupies bits 9–8.
 * - `speed` occupies bits 7–1 (7-bit field, clamped to 0–100 before encoding).
 * - `pickup` occupies bit 0.
 *
 * The encoded packet and all field values are logged to stdout before each
 * `write()` call to aid debugging.
 *
 * ### Flush Behaviour
 * `tcflush(TCIOFLUSH)` is called immediately before each `write()` to discard
 * any stale bytes in the kernel's transmit and receive buffers, preventing
 * command queuing on slow or intermittently-connected serial links.
 */

#include "MotorController.h"
#include <iomanip>

MotorController::MotorController(const std::string& portName) {
    serial_port = open(portName.c_str(), O_RDWR | O_NOCTTY | O_NDELAY);

    if (serial_port < 0) {
        std::cerr << "[MotorController] Error " << errno
                  << " opening " << portName << ": " << strerror(errno) << std::endl;
        return;
    }

    if (tcgetattr(serial_port, &tty) != 0) {
        std::cerr << "[MotorController] tcgetattr error" << std::endl;
    }

    cfsetospeed(&tty, B115200);
    cfsetispeed(&tty, B115200);

    tty.c_cflag &= ~PARENB;
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag &= ~CSIZE;
    tty.c_cflag |= CS8;
    tty.c_cflag |= CREAD | CLOCAL;
    tty.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
    tty.c_iflag &= ~(IXON | IXOFF | IXANY);
    tty.c_oflag &= ~OPOST;
    tty.c_cc[VMIN]  = 0;
    tty.c_cc[VTIME] = 10;

    if (tcsetattr(serial_port, TCSANOW, &tty) != 0) {
        std::cerr << "[MotorController] tcsetattr error" << std::endl;
    }

    std::cout << "[MotorController] Opened " << portName << std::endl;
}

MotorController::~MotorController() {
    if (serial_port >= 0) {
        eStop();
        close(serial_port);
    }
}

bool MotorController::drive(int direction, TurnDirection turn, int speed, bool pickup) {
    direction = ((direction % 360) + 360) % 360;
    speed = std::max(0, std::min(100, speed));
    return sendPacket(true, direction, turn, speed, pickup);
}

bool MotorController::eStop() {
    return sendPacket(false, 0, TurnDirection::NONE, 0, false);
}

bool MotorController::sendPacket(bool cmd_enable, int direction,
                                  TurnDirection turn, int speed, bool pickup) {
    if (serial_port < 0) {
        std::cerr << "[MotorController] Port not open" << std::endl;
        return false;
    }

    uint32_t packet = 0;
    if (cmd_enable)          packet |= (1u << 19);
    packet |= ((uint32_t)(direction & 0x1FF)) << 10;
    packet |= ((uint32_t)((int)turn  & 0x03)) << 8;
    packet |= ((uint32_t)(speed      & 0x7F)) << 1;
    if (pickup)              packet |= 0x01u;

    uint8_t bytes[4] = {
        0xFF,
        (uint8_t)((packet >> 16) & 0xFF),
        (uint8_t)((packet >>  8) & 0xFF),
        (uint8_t)( packet        & 0xFF)
    };

    const char* turnStr = (turn == TurnDirection::LEFT)  ? "LEFT"  :
                          (turn == TurnDirection::RIGHT) ? "RIGHT" : "NONE";
    std::cout << "[MotorController] TX"
              << "  cmd="  << (cmd_enable ? 1 : 0)
              << "  dir="  << direction   << "deg"
              << "  turn=" << turnStr
              << "  spd="  << speed       << "%"
              << "  pick=" << (pickup ? 1 : 0)
              << "  raw=0x" << std::hex << std::uppercase
              << std::setw(2) << std::setfill('0') << (int)bytes[0]
              << std::setw(2) << std::setfill('0') << (int)bytes[1]
              << std::setw(2) << std::setfill('0') << (int)bytes[2]
              << std::setw(2) << std::setfill('0') << (int)bytes[3]
              << std::dec << std::endl;

    tcflush(serial_port, TCIOFLUSH);
    bool ok = write(serial_port, bytes, 4) == 4;
    if (!ok) std::cerr << "[MotorController] write() failed: " << strerror(errno) << std::endl;
    return ok;
}

bool MotorController::sendRaw(const std::string& cmd) {
    if (serial_port < 0) return false;
    std::string s = cmd + "\n";
    return write(serial_port, s.c_str(), s.size()) > 0;
}

std::string MotorController::readResponse() {
    if (serial_port < 0) return "";
    char buf[256];
    memset(buf, '\0', sizeof(buf));
    int n = read(serial_port, buf, sizeof(buf) - 1);
    return (n > 0) ? std::string(buf) : "";
}