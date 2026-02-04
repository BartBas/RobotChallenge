#ifndef MOTOR_CONTROLLER_H
#define MOTOR_CONTROLLER_H

#include <string>
#include <termios.h>
#include <iostream>
#include <fcntl.h>   // File control definitions
#include <errno.h>   // Error number definitions
#include <unistd.h>  // UNIX standard function definitions
#include <cstring>

class MotorController {
private:
    int serial_port;
    struct termios tty;

public:
    // Constructor takes the device path (e.g., "/dev/ttyACM0")
    MotorController(const std::string& portName);
    
    // Destructor to ensure the port is closed safely
    ~MotorController();

    // The test function you requested
    void test();

    // Core communication methods
    bool sendCommand(const std::string& cmd);
    std::string readResponse();
};

#endif
