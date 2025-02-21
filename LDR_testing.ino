const int ldrPins[6] = {A0, A1, A2, A3, A4, A5};  // LDR sensor pins

void setup() {
  Serial.begin(115200);
  while (!Serial) delay(10);
  Serial.println("LDR Light Intensity Test\n");
}

void loop() {
  for (int i = 0; i < 6; i++) {
    int ldrValue = analogRead(ldrPins[i]);  // Read LDR value
    
    // Correct intensity mapping (higher value = more light, lower value = less light)
    int intensity = map(ldrValue, 0, 1023, 100, 0); 
    intensity = constrain(intensity, 0, 100); // Ensure valid range

    Serial.print("LDR ");
    Serial.print(i + 1);
    Serial.print(" Intensity: ");
    Serial.print(intensity);
    Serial.println("%");
  }
  
  Serial.println(); // New line for readability
  delay(500); // Delay before next reading
}