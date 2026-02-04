#include "MotorController.h"


MotorController::MotorController(const std::string& portName) {
    // Open the serial port
    serial_port = open(portName.c_str(), O_RDWR | O_NOCTTY);

    if (serial_port < 0) {
        std::cerr << "Error " << errno << " opening " << portName << ": " << strerror(errno) << std::endl;
        return;
    }

    // Read existing settings
    if(tcgetattr(serial_port, &tty) != 0) {
        std::cerr << "Error from tcgetattr" << std::endl;
    }

    // Set Baud Rate (15200  matches Arduino)
    cfsetospeed(&tty, B115200);
    cfsetispeed(&tty, B115200);

    // Basic 8N1 settings (8 bits, no parity, 1 stop bit)
    tty.c_cflag &= ~PARENB; 
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag &= ~CSIZE;
    tty.c_cflag |= CS8;
    tty.c_cflag |= CREAD | CLOCAL; // Turn on READ & ignore ctrl lines

    // Apply settings
    tcsetattr(serial_port, TCSANOW, &tty);
}

MotorController::~MotorController() {
    if (serial_port >= 0) {
        close(serial_port);
    }
}

bool MotorController::sendCommand(const std::string& cmd) {
    std::string formatted = cmd + "\n";
    int bytes_written = write(serial_port, formatted.c_str(), formatted.size());
    return (bytes_written > 0);
}

std::string MotorController::readResponse() {
    char buf[256];
    memset(&buf, '\0', sizeof(buf));
    int n = read(serial_port, buf, sizeof(buf));
    return (n > 0) ? std::string(buf) : "";
}

void MotorController::test() {
    std::cout << "Starting Motor Test..." << std::endl;
    if (sendCommand("TEST_RUN")) {
        std::cout << "Command sent successfully." << std::endl;
    } else {
        std::cerr << "Command failed!" << std::endl;
    }
}
