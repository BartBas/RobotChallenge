#ifndef MOTOR_CONTROLLER_H
#define MOTOR_CONTROLLER_H

/**
 * @file MotorController.h
 * @brief Serial interface for sending drive commands to the motor firmware.
 *
 * @details
 * Opens a UART connection to the Arduino-based motor controller and encodes
 * drive commands into the 4-byte binary packet format expected by the firmware.
 *
 * ### Packet Format
 * Commands are transmitted as a 4-byte frame. The first byte is always `0xFF`
 * (sync/start byte). The remaining 3 bytes carry a 24-bit field packed as
 * follows (MSB first):
 *
 * | Bits  | Width | Field       | Description                              |
 * |-------|-------|-------------|------------------------------------------|
 * | 23–20 | 4     | (unused)    | Always 0                                 |
 * | 19    | 1     | cmd_enable  | 0 = E-stop, 1 = normal operation         |
 * | 18–10 | 9     | direction   | Heading in degrees, 0–359                |
 * | 9–8   | 2     | turn_val    | 0 = none, 1 = left, 2 = right            |
 * | 7–1   | 7     | speed_val   | Speed percentage, 0–100                  |
 * | 0     | 1     | pickup      | 1 = run pickup servo, 0 = stop           |
 *
 * ### Direction Convention
 * Matches the convention used throughout the codebase:
 * | Value | Direction |
 * |-------|-----------|
 * | 0 / 360 | Forward (360 preferred — 0 is a firmware dead-zone) |
 * | 90    | Right     |
 * | 180   | Backward  |
 * | 270   | Left      |
 *
 * ### Serial Configuration
 * - Baud rate: 115 200
 * - Format: 8N1, raw (no flow control, no canonical mode)
 * - Read timeout: 1 s (`VTIME = 10` × 100 ms)
 *
 * ### Usage
 * @code
 * MotorController motors("/dev/ttyUSB1");
 * motors.drive(360, MotorController::TurnDirection::NONE, 45, false);
 * motors.eStop();
 * @endcode
 */

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

    explicit MotorController(const std::string& portName);
    ~MotorController();

    bool drive(int direction, TurnDirection turn, int speed, bool pickup = false);
    bool eStop();

    bool isOpen() const { return serial_port >= 0; }

    bool sendRaw(const std::string& cmd);
    std::string readResponse();

private:
    int serial_port = -1;
    struct termios tty{};

    bool sendPacket(bool cmd_enable, int direction, TurnDirection turn,
                    int speed, bool pickup);
};

#endif // MOTOR_CONTROLLER_H