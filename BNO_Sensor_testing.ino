#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

uint16_t BNO055_SAMPLERATE_DELAY_MS = 100;
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28, &Wire);

void setup() {
  Serial.begin(115200);
  while (!Serial) delay(10);

  Serial.println("BNO055 Sensor Test\n");

  if (!bno.begin()) {
    Serial.println("No BNO055 detected ... Check wiring or I2C address!");
    while (1);
  }
  delay(1000);
}

void loop() {
  sensors_event_t orientationData;
  bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);

  double pitch = orientationData.orientation.x;
  double yaw = orientationData.orientation.y;
  double roll = orientationData.orientation.z;

  Serial.print("Pitch: "); Serial.print(pitch);
  Serial.print("\tYaw: "); Serial.print(yaw);
  Serial.print("\tRoll: "); Serial.println(roll);

  delay(BNO055_SAMPLERATE_DELAY_MS);
}