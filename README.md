# ADCS-with-Arduino-Uno
An affordable and accurate Attitude Determination and Control System (ADCS) that uses sensor fusion and real-time control algorithms using an Arduino Uno, DC motors, and LDR sensors to orient and steady spacecraft

The Attitude Determination and Control System (ADCS) is essential for spacecraft that require precise orientation control throughout their mission. It stabilizes the vehicle and maintains its desired direction despite external disturbances such as atmospheric drag, gravity gradients, and solar radiation. The system relies on sensors to determine the spacecraft’s attitude and actuators to make necessary adjustments for stability.

Our ADCS implementation uses the Adafruit BNO055 sensor, which integrates an accelerometer, gyroscope, and magnetometer for accurate orientation sensing. For attitude control, reaction wheels powered by DC motors are employed, with relays managing motor operation. In case of sensor failure, Light Dependent Resistors (LDRs) act as a redundancy mechanism by detecting sunlight intensity and estimating orientation.

An Arduino Uno microcontroller processes real-time sensor data and executes control algorithms to adjust the satellite’s pitch, yaw, and roll. The system maintains stability within ±20° of the target orientation, applying corrective actions when predefined thresholds are exceeded. To ensure reliability, simulation software is used to test the ADCS under various mission conditions.
