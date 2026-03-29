#ifndef MOTOR_CONTROLLER_H
#define MOTOR_CONTROLLER_H

#include <string>
#include <termios.h>
#include <iostream>
#include <fcntl.h>
#include <errno.h>
#include <unistd.h>
#include <cstring>
#include <cstdint>
#include <algorithm>

class MotorController {
public:
    enum class TurnDirection {
        NONE  = 0,
        LEFT  = 1,
        RIGHT = 2
    };

    // portName: e.g. "/dev/ttyUSB1"
    explicit MotorController(const std::string& portName);
    ~MotorController();

    // High-level drive command.
    // direction : compass heading in degrees (0–359)
    // turn      : TurnDirection::NONE / LEFT / RIGHT
    // speed     : 0–100 (%)
    // pickup    : true = run pickup servo
    bool drive(int direction, TurnDirection turn, int speed, bool pickup = false);

    // Emergency-stop — sets cmd_enable=0, Arduino calls stopAllMotors()
    bool eStop();

    bool isOpen() const { return serial_port >= 0; }

    // Raw helpers (legacy / debug use)
    bool sendRaw(const std::string& cmd);
    std::string readResponse();

private:
    int serial_port = -1;
    struct termios tty{};

    // Packs and sends the 3-byte binary packet the Arduino firmware expects.
    //
    // Packet layout (24 bits, MSB first):
    //   Bit 23-20 : unused (0)
    //   Bit 19    : cmd_enable   – 0 = E-stop, 1 = normal
    //   Bit 18-10 : direction    – 9 bits, 0-359°
    //   Bit  9-8  : turn_val     – 0=none, 1=left, 2=right
    //   Bit  7-1  : speed_val    – 7 bits, 0-100
    //   Bit  0    : pickup       – 1=run servo, 0=stop
    bool sendPacket(bool cmd_enable, int direction, TurnDirection turn,
                    int speed, bool pickup);
};

#endif // MOTOR_CONTROLLER_H
