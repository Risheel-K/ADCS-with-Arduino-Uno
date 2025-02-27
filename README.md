# ADCS-with-Arduino-Uno
An affordable and accurate Attitude Determination and Control System (ADCS) that uses sensor fusion and real-time control algorithms using an Arduino Uno, DC motors, and LDR sensors to orient and steady spacecraft

The Attitude Determination and Control System (ADCS) is essential for spacecraft that require precise orientation control throughout their mission. It stabilizes the vehicle and maintains its desired direction despite external disturbances such as atmospheric drag, gravity gradients, and solar radiation. The system relies on sensors to determine the spacecraft’s attitude and actuators to make necessary adjustments for stability.  

Our ADCS implementation uses the **Adafruit BNO055 sensor**, which integrates an accelerometer, gyroscope, and magnetometer for accurate orientation sensing. When the **BNO055 operates in its normal mode**, it provides precise attitude data, allowing the system to adjust orientation using **reaction wheels powered by DC motors**, controlled via **relays**. However, if **the system enters eclipse mode** (where BNO055 fails due to low power or other constraints), the ADCS switches to **Light Dependent Resistors (LDRs)** as a redundancy mechanism. The LDRs detect sunlight intensity and direction, enabling the system to approximate orientation and stabilize the spacecraft.  

An **Arduino Uno microcontroller** processes real-time sensor data and executes control algorithms to adjust the satellite’s pitch, yaw, and roll. The system maintains stability within **±20° of the target orientation**, applying corrective actions when predefined thresholds are exceeded. To ensure reliability, **simulation software** is used to test the ADCS under various mission conditions.
To ensure the reliability and accuracy of the Attitude Determination and Control System (ADCS), multiple test codes have been developed and validated individually before integration. The testing process includes:

1. LDR Testing: Verifying the functionality of Light Dependent Resistors (LDRs) for orientation approximation in eclipse mode.
2. BNO055 Testing: Ensuring accurate attitude determination using the Adafruit BNO055 sensor.
3. Pitch Axis Testing: Evaluating the control and stability of the satellite’s pitch axis.
4. Roll Axis Testing: Testing the roll axis control and response to orientation changes.
5. Yaw Axis Testing: Checking the accuracy and response of yaw axis adjustments.
6. Full System Integration: Combining all tested components to validate overall system performance and stability.
