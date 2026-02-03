# Robot Challenge: Cup Retriever

A robotics project designed to autonomously (or semi-autonomously) identify, pick up, and return "used" cups to a designated collection point. This repository contains the control logic for the Raspberry Pi 5 and the motor driver firmware for the Arduino.

---

## Repository Structure
The project is split into two main hardware layers:

### /ArduinoMotorController
Contains the firmware for the Arduino board equipped with a Motor Controller Hat. It handles the drive controllers and motor drivers.

### /RBpi5
The "brain" of the robot. This directory contains the high-level logic, sensor integration (e.g., Camera, LiDAR), and communication protocols to send commands to the Arduino.

---

## Hardware Stack

| Component | Specification |
| :--- | :--- |
| **Controller** | Raspberry Pi 5 |
| **Camara** | Raspberry Pi Camera Module 3 |
| **LIDAR** | LiDAR: X4-PRO (EAI YDLIDAR) |
| **Actuator Driver** | Arduino (with Ps2x & motor shield v5.6) |
| **Chassis** | 4WD Custom Build |
| **Power** | ParkSide 12v Battery |