#include <WiFi.h>
#include <Encoder.h>

// Define PID parameters
double Kp = 1.0;  // Proportional gain
double Ki = 0.0;  // Integral gain
double Kd = 0.0;  // Derivative gain
\
// Define variables
double setpoint = 100.0;  // Desired speed for both motors
double motorSpeed1 = 0.0; // Current speed of motor 1
double motorSpeed2 = 0.0; // Current speed of motor 2

double errorSum = 0.0;    // Integral of error
double lastError = 0.0;   // Previous error

// Variables for tracking counts and time
volatile long prevCount1 = 0;
volatile long prevCount2 = 0;
unsigned long prevTime = 0;

//Motor speed Controll pins
const int leftMotorChannel = 0;
const int rightMotorChannel = 1;

// Function to read distance from a single ultrasonic sensor
int readDistance(int address) {
  Wire.beginTransmission(address);
  Wire.write(byte(0x00));
  Wire.write(byte(0x51));
  Wire.endTransmission();

  delay(70);

  Wire.beginTransmission(address);
  Wire.write(byte(0x02));
  Wire.endTransmission();

  Wire.requestFrom(address, 2);

  if (2 <= Wire.available()) {
    int reading = Wire.read() << 8;
    reading |= Wire.read();
    return reading;
  }

  return -1; // Return -1 if reading was unsuccessful
}

// PID control function
void pidControl() {
    // Measure actual speeds of motors (encoder feedback or other methods)
    double actualSpeed1 = getActualSpeed1();
    double actualSpeed2 = getActualSpeed2();

    // Calculate error
    double error1 = setpoint - actualSpeed1;
    double error2 = setpoint - actualSpeed2;

    // Integral term
    errorSum += error1 + error2;

    // Derivative term
    double dError = (error1 + error2) - lastError;

    // Calculate PID output
    double output1 = Kp * error1 + Ki * errorSum + Kd * dError;
    double output2 = Kp * error2 + Ki * errorSum + Kd * dError;

    // Apply PID output to adjust motor speeds
    adjustMotorSpeed(output1, output2);

    // Update last error
    lastError = error1 + error2;
}

//Set the motor speed
void speed(int leftSpeed, int rightSpeed) {
  // Set speed for left motor
  ledcWrite(leftMotorChannel, leftSpeed);

  // Set speed for right motor
  ledcWrite(rightMotorChannel, rightSpeed);
}

// Function to adjust motor speeds based on PID output
void adjustMotorSpeed(double output1, double output2) {
    // Adjust motor speeds using PID output
    // Example code: Set motor speeds based on PID output

    int offset = 0;
    motorSpeed1 = setpoint + output1 +offset;
    motorSpeed2 = setpoint + output2 +offset;
}

// Function to get actual speed of motor 1 (replace with your own implementation)
double getActualSpeed1() {
    // Get current time
    unsigned long currentTime = millis();

    // Calculate time interval
    unsigned long timeInterval = currentTime - prevTime;

    // Read encoder counts
    long currentCount1 = encoder1.read();

    // Calculate speed for motor 1
    double speed1 = calculateSpeed(prevCount1, currentCount1, timeInterval);

    // Update previous count and time
    prevCount1 = currentCount1;
    prevTime = currentTime;
    // Measure actual speed of motor 1 using encoders or other feedback mechanisms
    return speed1;
}

double getActualSpeed2() {
    // Get current time
    unsigned long currentTime = millis();

    // Calculate time interval
    unsigned long timeInterval = currentTime - prevTime;

    // Read encoder counts
    long currentCount2 = encoder2.read();

    // Calculate speed for motor 1
    double speed2 = calculateSpeed(prevCount2, currentCount2, timeInterval);

    // Update previous count and time
    prevCount2 = currentCount2;
    prevTime = currentTime;
    // Measure actual speed of motor 1 using encoders or other feedback mechanisms
    return speed2;
}


// Function to calculate actual speed
double calculateSpeed(long prevCount, long currentCount, unsigned long timeInterval) {
    long deltaCount = currentCount - prevCount;
    double revolutions = deltaCount / encoderResolution; // Convert counts to revolutions
    double timeSeconds = timeInterval / 1000.0; // Convert milliseconds to seconds
    double speed = (revolutions / timeSeconds) * 60.0; // Convert to RPM
    return speed;
}

// Main loop
void loop() {
  //Get Distances from Ultrasonic sensors 
  int addresses[] = {112, 113, 114}; // Addresses of the three sensors
  int distances[3]; // Array to store distances from each sensor

  for (int i = 0; i < 3; i++) {
    distances[i] = readDistance(addresses[i]);
  }

  if (distances[1]<=distace_to_Obstacle || distances[2]<=distace_to_Obstacle || distances[3]<=distace_to_Obstacle){
    //Break Code here
  }
  else{
    // Perform PID control
    pidControl();

    // Other tasks...
  }

}
