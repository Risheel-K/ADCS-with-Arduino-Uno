# ADCS-with-Arduino-Uno
This project implements a real-time Attitude Determination and Control System (ADCS) using an Arduino Uno and the Adafruit BNO055 absolute orientation sensor. The system reads precise pitch, yaw, and roll values and is designed for applications like spacecraft orientation, drone stabilization, and robotics balance control.

The BNO055 sensor provides fused 9-DOF orientation data, combining accelerometer, gyroscope, and magnetometer readings internally, and outputs stable Euler angles (Pitch, Yaw, Roll) over the I²C interface. The Arduino Uno processes these orientation values and sends them over serial for monitoring or control.
Hardware Components Used:
Component	Function
Arduino Uno	Microcontroller for sensor interfacing and data processing
Adafruit BNO055 Sensor	9-DOF IMU providing real-time orientation (Euler angles)
I²C Communication (SDA/SCL)	Interface for data exchange between Arduino and BNO055
USB Cable	Powers Arduino and enables serial communication to a PC
Jumper Wires + Breadboard	Prototyping connections
5V Power Supply (optional)	For standalone operation beyond USB power
Software Modules:
Module	Description
Wire.h	Enables I2C communication between Arduino and BNO055 sensor
Adafruit_Sensor.h	Unified sensor access for consistent reading formats
Adafruit_BNO055.h	Device driver library for interfacing with the BNO055 sensor
imumaths.h	Provides quaternion and vector math utilities used by the sensor backend
Functional Workflow:

    Initialization (setup()):

        Starts the Serial communication at 115200 baud.

        Initializes the BNO055 sensor over I2C at address 0x28.

        Verifies sensor connection; stops execution if the sensor is not detected.

    Real-Time Loop (loop()):

        Continuously reads Euler angle orientation data from the sensor:

            Pitch → X-axis rotation

            Yaw → Y-axis rotation

            Roll → Z-axis rotation

        Displays values in the Serial Monitor every 100 milliseconds.

Key Features:

    Full 3-axis attitude data (±360°) with minimal drift

    Uses BNO055’s internal sensor fusion, no need for external algorithms

    Modular and lightweight code, ideal for embedded real-time systems

    Scalable for integration with reaction wheels, servos, or motor drivers

    Can be extended with relay/H-bridge circuits for full ADCS actuation

Use Cases / Applications:

    CubeSat and picosatellite ADCS prototyping

    Robotics orientation and stabilization systems

    Quadcopter or drone IMU integration

    Educational demos for aerospace engineering

    Mechatronics and control systems labs

Future Integration Potential:

This module can be integrated into a complete ADCS system with:

    Reaction wheels controlled via DC motors

    Redundant LDR-based sun sensors for eclipse fallback

    PID-based motor control for precise orientation adjustment

    Closed-loop attitude correction logic for space or aerial platforms
