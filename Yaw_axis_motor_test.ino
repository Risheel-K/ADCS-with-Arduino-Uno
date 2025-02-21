#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

// Define the relay pins
#define RELAY_PIN1 6
#define RELAY_PIN2 7

// Initialize the BNO055 sensor
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28, &Wire);

void setup() {
  Serial.begin(115200);
  while (!Serial) delay(10);

  // Initialize relay pins as OUTPUT
  pinMode(RELAY_PIN1, OUTPUT);
  pinMode(RELAY_PIN2, OUTPUT);

  // Initialize the BNO055 sensor
  if (!bno.begin()) {
    Serial.println("No BNO055 detected ... Check wiring or I2C address!");
    while (1);
  }
  delay(1000); // Allow sensor to stabilize
}

void loop() {
  sensors_event_t orientationData;
  bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);

  double yaw = orientationData.orientation.y; // Get yaw value

  Serial.print("Yaw: ");
  Serial.println(yaw);

  if (yaw > 0) {
    // Yaw is positive → Rotate *anticlockwise*
    digitalWrite(RELAY_PIN1, HIGH);
    digitalWrite(RELAY_PIN2, LOW);
  } 
  else if (yaw < 0) {
    // Yaw is negative → Rotate *clockwise*
    digitalWrite(RELAY_PIN1, LOW);
    digitalWrite(RELAY_PIN2, HIGH);
  } 
  else {
    // If yaw is zero → Stop the motor
    digitalWrite(RELAY_PIN1, LOW);
    digitalWrite(RELAY_PIN2, LOW);
  }

  delay(100); // Small delay to stabilize readings
}