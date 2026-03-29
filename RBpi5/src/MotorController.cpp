#include "MotorController.h"

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

    // 8N1, raw mode
    tty.c_cflag &= ~PARENB;
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag &= ~CSIZE;
    tty.c_cflag |= CS8;
    tty.c_cflag |= CREAD | CLOCAL;
    tty.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
    tty.c_iflag &= ~(IXON | IXOFF | IXANY);
    tty.c_oflag &= ~OPOST;
    tty.c_cc[VMIN]  = 0;
    tty.c_cc[VTIME] = 10; // 1 s read timeout

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

    uint8_t bytes[3] = {
        (uint8_t)((packet >> 16) & 0xFF),
        (uint8_t)((packet >>  8) & 0xFF),
        (uint8_t)( packet        & 0xFF)
    };

    return write(serial_port, bytes, 3) == 3;
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
