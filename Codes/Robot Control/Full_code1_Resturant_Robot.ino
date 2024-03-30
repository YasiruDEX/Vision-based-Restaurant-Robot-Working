#include <Arduino.h>
#include <Wire.h>


// Motor A pins
const int motorA1 = 2;
const int motorA2 = 3;
const int motorAEn = 4; //PWM pin for Speed control
// Motor B pins
const int motorB1 = 5;
const int motorB2 = 6;
const int motorBEn = 7; //PWM pin for Speed control

// Define Speeed values
const int MAX_SPEED = 200;
const int MIN_SPEED = 100;
const int BASE_SPEED = 150;

//Encoder values
volatile int lastEncoded1 = 0;
volatile long encoderValue1 = 0;  //This is the current encoder value
int lastMSB1 = 0;
int lastLSB1 = 0;

volatile int lastEncoded2 = 0;
volatile long encoderValue2 = 0;
int lastMSB2 = 0;
int lastLSB2 = 0;

bool encoderFlag= true;

// Define encoder pins
const int encoder1APin = 2;
const int encoder1BPin = 3;
const int encoder2APin = 4;
const int encoder2BPin = 5;

//Encoder Details
int encoderResolution=3000;  //Not sure

//Radius values
const double R =50 ;
const double r =10 ;

// Define PID parameters
double Kp = 1.0;  // Proportional gain
double Ki = 0.0;  // Integral gain
double Kd = 0.0;  // Derivative gain

// Define variables
//double setpoint1;  // Desired speed for both motors
//double setpoint2 ;  // Desired speed for both motors
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

// Ultrasonic minimum distance
int distace_to_Obstacle=30;

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

//Find the absolute value
double absvalue(double value){
  if (value<0){
    return -value;
  }
  else{
    return value;
  }
}

// PID control function
void pidControl_speed(double setpoint1,double setpoint2) {
    // Measure actual speeds of motors (encoder feedback or other methods)
    double actualSpeed1 = absvalue(getActualSpeed1());
    double actualSpeed2 = absvalue(getActualSpeed2());

    // Calculate error
    double error1 = setpoint1 - actualSpeed1;
    double error2 = setpoint2 - actualSpeed2;

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

    int offset1 = 0;
    int offset2 = 0;
    motorSpeed1 = BASE_SPEED + output1 +offset1;
    motorSpeed2 = BASE_SPEED + output2 +offset2;

    speed(motorSpeed1,motorSpeed2);
}

// Function to get actual speed of motor 1 (replace with your own implementation)
double getActualSpeed1() {

    // Calculate time interval
    unsigned long timeInterval = currentTime - prevTime;

    // Read encoder counts
    long currentCount1 = encoderValue1;

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
    long currentCount2 = encoderValue2;

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
  currentTime=millis();
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


void controlMotors(int pinA1, int pinA2, int pinB1, int pinB2, double A_SPEED = 150, double B_SPEED = 150) {
  //Motor 1 Control
  digitalWrite( motorA1, pinA1);
  digitalWrite( motorA2, pinA2);

  //Motor 2 Control
  digitalWrite( motorB1, pinB1);
  digitalWrite( motorB2, pinB2);

  //Control the speeds of the both motors
  pidControl_speed(A_SPEED, B_SPEED);
}

// Function to stop the motors
void stopMotors() {
  controlMotors(0, 0, 0, 0);
}

void Forward(int A_Speed, int B_Speed){
    controlMotors(1, 0, 1, 0, A_Speed, B_Speed);
}

void Right(int A_Speed, int B_Speed){
    controlMotors(0, 0, 1, 0, A_Speed, B_Speed);
}

void Left(int A_Speed, int B_Speed){
    controlMotors(1, 0, 0, 0, A_Speed, B_Speed);
}

void Rotate(int direction){
    int startEncoderValue1;
    int startEncoderValue2;
  if ( encoderFlag){
    startEncoderValue1= encoderValue1;
    startEncoderValue2= encoderValue2;
    encoderFlag = false;
  }
  
  int expectedRotationValue = (R/(4*r))*encoderResolution;

  if (direction == 0){ // 0 for right 90 rotation
    if ((absvalue(encoderValue1 - startEncoderValue1) <= expectedRotationValue) && (absvalue(encoderValue2 - startEncoderValue2) <= expectedRotationValue)){
      controlMotors(1,0, 0, 1, MAX_SPEED, MAX_SPEED);
    }
    else if (absvalue(encoderValue1 - startEncoderValue1) < expectedRotationValue){
      controlMotors(1, 0, 0, 0, MAX_SPEED, MAX_SPEED);
    }
    else if (absvalue(encoderValue2 - startEncoderValue2) < expectedRotationValue){
      controlMotors(0, 0, 0, 1, MAX_SPEED, MAX_SPEED);
    }
    else{
      encoderFlag = true;
    }
  }
  else if (direction == 1){ // 1 for left 90 rotation
    if (((absvalue(encoderValue1 - startEncoderValue1) <= expectedRotationValue)) && (absvalue(encoderValue2 - startEncoderValue2) <= expectedRotationValue)){
      controlMotors(1,0, 0, 1, MAX_SPEED, MAX_SPEED);
    }
    else if (absvalue(encoderValue1 - startEncoderValue1) < expectedRotationValue){
      controlMotors(1, 0, 0, 0, MAX_SPEED, MAX_SPEED);
    }
    else if (absvalue(encoderValue2 - startEncoderValue2) < expectedRotationValue){
      controlMotors(0, 0, 0, 1, MAX_SPEED, MAX_SPEED);
    }
    else{
      encoderFlag = true;
    }
  }
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
    pidControl_speed(BASE_SPEED,BASE_SPEED);

    // Other tasks...
  }

}
