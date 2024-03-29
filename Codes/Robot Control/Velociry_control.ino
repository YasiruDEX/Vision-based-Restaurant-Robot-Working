#include <Arduino.h>

//Encoder values
volatile int lastEncoded1 = 0;
volatile long encoderValue1 = 0;
int lastMSB1 = 0;
int lastLSB1 = 0;

volatile int lastEncoded2 = 0;
volatile long encoderValue2 = 0;
int lastMSB2 = 0;
int lastLSB2 = 0;

// Define encoder pins
const int encoder1APin = 2;
const int encoder1BPin = 3;
const int encoder2APin = 4;
const int encoder2BPin = 5;

//Encoder Details
int encoderResolution=3000;

// Define PID parameters
double Kp = 1.0;  // Proportional gain
double Ki = 0.0;  // Integral gain
double Kd = 0.0;  // Derivative gain

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
unsigned long currentTime;

//Motor speed control pins
const int leftMotorChannel = 0;
const int rightMotorChannel = 1;


// PID control function for constant speed
void pidControl_speed() {
    // Measure actual speeds of motors
    double actualSpeed1 = getActualSpeed1(); //This is in RPM
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
  analogWrite(leftMotorChannel, leftSpeed);

  // Set speed for right motor
  analogWrite(rightMotorChannel, rightSpeed);
}

// Function to adjust motor speeds based on PID output
void adjustMotorSpeed(double output1, double output2) {
    // Adjust motor speeds using PID output
    // Example code: Set motor speeds based on PID output

    int offset = 0;
    motorSpeed1 = setpoint + output1 +offset;
    motorSpeed2 = setpoint + output2 +offset;
}

// Function to get actual speed of motor 1 
double getActualSpeed1() {

    // Calculate time interval
    unsigned long timeInterval = currentTime - prevTime;

    // Read encoder counts

    long currentCount1 = lastEncoded1;

    // Calculate speed for motor 1
    double speed1 = calculateSpeed(prevCount1, currentCount1, timeInterval);

    // Update previous count and time
    prevCount1 = currentCount1;
    prevTime = currentTime;
    // Measure actual speed of motor 1 using encoders or other feedback mechanisms
    return speed1;
}

double getActualSpeed2() {

    // Calculate time interval
    unsigned long timeInterval = currentTime - prevTime;

    // Read encoder counts
    long currentCount2 = lastEncoded2;

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

void updateEncoder()
{

  //Encoder Update for Encoder 1
  int MSB1 = digitalRead(encoder1APin); //MSB = most significant bit
  int LSB1 = digitalRead(encoder1BPin); //LSB = least significant bit

  int encoded1 = (MSB1 << 1) |LSB1; //converting the 2 pin value to single number
  int sum1  = (lastEncoded1 << 2) | encoded1; //adding it to the previous encoded value

  if(sum1 == 0b1101 || sum1 == 0b0100 || sum1 == 0b0010 || sum1 == 0b1011) encoderValue1 ++;
  if(sum1 == 0b1110 || sum1 == 0b0111 || sum1 == 0b0001 || sum1 == 0b1000) encoderValue1 --;

  lastEncoded1 = encoded1; //store this value for next time

  //Encoder Update for Encoder 2
  int MSB2 = digitalRead(encoder2APin); //MSB = most significant bit
  int LSB2 = digitalRead(encoder2BPin); //LSB = least significant bit

  int encoded2 = (MSB2 << 1) |LSB2; //converting the 2 pin value to single number
  int sum2  = (lastEncoded2 << 2) | encoded2; //adding it to the previous encoded value

  if(sum2 == 0b1101 || sum2 == 0b0100 || sum2 == 0b0010 || sum2 == 0b1011) encoderValue2 ++;
  if(sum2 == 0b1110 || sum2 == 0b0111 || sum2 == 0b0001 || sum2 == 0b1000) encoderValue2 --;

  lastEncoded2 = encoded2; //store this value for next time
}


void setup(){

  //Encoder Setup
  Serial.begin (9600);

  pinMode(encoder1APin, INPUT); 
  pinMode(encoder1BPin, INPUT);

  digitalWrite(encoder1APin, HIGH); //turn pullup resistor on
  digitalWrite(encoder1BPin, HIGH); //turn pullup resistor on

  //call updateEncoder() when any high/low changed seen
  //on interrupt 0 (pin 2), or interrupt 1 (pin 3) 
  attachInterrupt(0, updateEncoder, CHANGE); 
  attachInterrupt(1, updateEncoder, CHANGE);
  
}

// Main loop
void loop() {
    currentTime = millis();
    // Perform PID control
    pidControl_speed();

    // Other tasks...
}
