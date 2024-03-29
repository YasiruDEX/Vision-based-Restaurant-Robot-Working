#include <Wire.h>

void setup() {
  Wire.begin();               
  Serial.begin(9600);          
}

int readings[3] = {0}; // Array to store readings from each sensor

void loop() {
  for (int sensor = 0; sensor < 3; sensor++) {
    // Send command to sensor to trigger measurement
    Wire.beginTransmission(112 + sensor); // Increment address for each sensor
    Wire.write(byte(0x00)); 
    Wire.write(byte(0x51)); 
    Wire.endTransmission();    

    delay(70);  // Wait for measurement to complete

    // Request reading from sensor
    Wire.beginTransmission(112 + sensor);
    Wire.write(byte(0x02)); 
    Wire.endTransmission(); 

    // Receive and store reading from sensor
    Wire.requestFrom(112 + sensor, 2); 
    if (2 <= Wire.available()) { 
      readings[sensor] = Wire.read() << 8; 
      readings[sensor] |= Wire.read(); 
    }
  }

  // Print readings from each sensor
  for (int sensor = 0; sensor < 3; sensor++) {
    Serial.print("Sensor ");
    Serial.print(sensor + 1); // Print sensor number
    Serial.print(": ");
    Serial.print(readings[sensor]); 
    Serial.println("cm");
  }

  delay(250); 
}
