#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

// Define relay pins for each motor
#define RELAY_YAW1 6   // Motor 1 - Yaw
#define RELAY_YAW2 7   
#define RELAY_PITCH1 8  // Motor 2 - Pitch
#define RELAY_PITCH2 9  
#define RELAY_ROLL1 10  // Motor 3 - Roll
#define RELAY_ROLL2 11  

// Define LDR pins
#define LDR_YAW A0
#define LDR_PITCH A1
#define LDR_ROLL A2

// LDR Threshold
#define LDR_THRESHOLD 100

// Initialize BNO055 sensor
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28, &Wire);

// Kalman filter variables
float x_kalman = 0, P_kalman = 1, Q = 0.01, R = 0.1;

// Kalman filter function
float kalmanFilter(float value) {
  float K = P_kalman / (P_kalman + R);  // Kalman Gain
  x_kalman = x_kalman + K * (value - x_kalman);
  P_kalman = (1 - K) * P_kalman + Q;
  return x_kalman;
}

void setup() {
  Serial.begin(115200);
  while (!Serial) delay(10);

  Serial.println("BNO055 & LDR Motor Control with Kalman Filter\n");

  // Initialize relay pins as OUTPUT
  pinMode(RELAY_YAW1, OUTPUT);
  pinMode(RELAY_YAW2, OUTPUT);
  pinMode(RELAY_PITCH1, OUTPUT);
  pinMode(RELAY_PITCH2, OUTPUT);
  pinMode(RELAY_ROLL1, OUTPUT);
  pinMode(RELAY_ROLL2, OUTPUT);

  pinMode(LDR_YAW, INPUT);
  pinMode(LDR_PITCH, INPUT);
  pinMode(LDR_ROLL, INPUT);

  stopMotors();

  // Initialize BNO055 sensor
  if (!bno.begin()) {
    Serial.println("No BNO055 detected ... Check wiring or I2C address!");
    while (1);
  }
  delay(1000);
}

void loop() {
  sensors_event_t orientationData;
  bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);

  // Read sensor values
  double yaw = orientationData.orientation.y;
  double pitch = orientationData.orientation.x;
  double roll = orientationData.orientation.z;

  int ldrYaw = analogRead(LDR_YAW);
  int ldrPitch = analogRead(LDR_PITCH);
  int ldrRoll = analogRead(LDR_ROLL);

  // Apply Kalman filter
  double yawFiltered = kalmanFilter(yaw);
  double pitchFiltered = kalmanFilter(pitch);
  double rollFiltered = kalmanFilter(roll);

  // Display values with delay
  Serial.print("Raw Yaw: "); Serial.print(yaw);
  Serial.print("\tFiltered Yaw: "); Serial.print(yawFiltered);
  delay(500);

  Serial.print("\tRaw Pitch: "); Serial.print(pitch);
  Serial.print("\tFiltered Pitch: "); Serial.print(pitchFiltered);
  delay(500);

  Serial.print("\tRaw Roll: "); Serial.print(roll);
  Serial.print("\tFiltered Roll: "); Serial.print(rollFiltered);
  delay(500);

  Serial.print("\tLDR Yaw Intensity: "); Serial.print(ldrYaw);
  Serial.print("\tLDR Pitch Intensity: "); Serial.print(ldrPitch);
  Serial.print("\tLDR Roll Intensity: "); Serial.println(ldrRoll);
  delay(500);

  // Print for graph plotting
  Serial.print(yaw); Serial.print(",");
  Serial.print(yawFiltered); Serial.print(",");
  Serial.print(pitch); Serial.print(",");
  Serial.print(pitchFiltered); Serial.print(",");
  Serial.print(roll); Serial.print(",");
  Serial.print(rollFiltered); Serial.print(",");
  Serial.print(ldrYaw); Serial.print(",");
  Serial.print(ldrPitch); Serial.print(",");
  Serial.println(ldrRoll);
  delay(500);

  // Motor control logic
  controlMotor(RELAY_YAW1, RELAY_YAW2, yawFiltered, ldrYaw);
  controlMotor(RELAY_PITCH1, RELAY_PITCH2, pitchFiltered, ldrPitch);
  controlMotor(RELAY_ROLL1, RELAY_ROLL2, rollFiltered, ldrRoll);
}

// Function to control motor based on LDR and BNO055 sensor
void controlMotor(int relay1, int relay2, double sensorValue, int ldrValue) {
  if (ldrValue > LDR_THRESHOLD) {
    digitalWrite(relay1, HIGH);
    digitalWrite(relay2, LOW);
  } 
  else if (ldrValue < LDR_THRESHOLD) {
    digitalWrite(relay1, LOW);
    digitalWrite(relay2, HIGH);
  } 
  else {
    if (sensorValue > 0) {
      digitalWrite(relay1, HIGH);
      digitalWrite(relay2, LOW);
    } else if (sensorValue < 0) {
      digitalWrite(relay1, LOW);
      digitalWrite(relay2, HIGH);
    } else {
      digitalWrite(relay1, LOW);
      digitalWrite(relay2, LOW);
    }
  }
}

// Function to stop all motors
void stopMotors() {
  digitalWrite(RELAY_YAW1, LOW);
  digitalWrite(RELAY_YAW2, LOW);
  digitalWrite(RELAY_PITCH1, LOW);
  digitalWrite(RELAY_PITCH2, LOW);
  digitalWrite(RELAY_ROLL1, LOW);
  digitalWrite(RELAY_ROLL2, LOW);
}