#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

// Define relay pins
#define RELAY_PIN1 6
#define RELAY_PIN2 7

// Define BNO055 sensor object
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28, &Wire);

void setup() {
  Serial.begin(115200);
  while (!Serial) delay(10);

  Serial.println("BNO055 Sensor Test\n");

  // Initialize relay pins as OUTPUT
  pinMode(RELAY_PIN1, OUTPUT);
  pinMode(RELAY_PIN2, OUTPUT);

  // Initialize BNO055 sensor
  if (!bno.begin()) {
    Serial.println("No BNO055 detected ... Check wiring or I2C address!");
    while (1);
  }
  delay(1000); // Allow sensor to stabilize
}

void loop() {
  sensors_event_t orientationData;
  bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);

  double pitch = orientationData.orientation.x; // Get pitch value

  Serial.print("Pitch: ");
  Serial.println(pitch);

  if (pitch < 180) {
    // Pitch less than 180 → Relay OFF → Motor rotates *anticlockwise*
    digitalWrite(RELAY_PIN1, LOW);
    digitalWrite(RELAY_PIN2, LOW);
  } 
  else if (pitch >= 180) {
    // Pitch 180 or more → Relay ON → Motor rotates *clockwise*
    digitalWrite(RELAY_PIN1, HIGH);
    digitalWrite(RELAY_PIN2, HIGH);
  } 

  delay(100); // Small delay to stabilize readings
}