#include <Arduino.h>
#include <Wire.h>


// Motor A pins
const int RPWM1 = 6;
const int LPWM1 = 7;
const int R_EN1 = 8; //PWM pin for Speed control
const int L_EN1 = 8; //PWM pin for Speed control

// Motor B pins
const int RPWM2 = 6;
const int LPWM2 = 7;
const int R_EN2 = 8; //PWM pin for Speed control
const int L_EN2 = 8; //PWM pin for Speed control

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
const double R = 50 ;
const double r = 10 ;

// Define PID parameters
double Kp = 1.0;  // Proportional gain
double Ki = 0.0;  // Integral gain
double Kd = 0.0;  // Derivative gain

// Define variables
//double setpoint1;  // Desired speed for both motors
//double setpoint2 ;  // Desired speed for both motors
double actualMotorSpeed1 = 0.0; // Current speed of motor 1
double actualMotorSpeed2 = 0.0; // Current speed of motor 2

double errorSum = 0.0;    // Integral of error
double lastError = 0.0;   // Previous error

//Angle errors
double errorSum1;
double errorSum2;

double lastError1;
double lastError2;

// Variables for tracking counts and time
volatile long prevCount1 = 0;
volatile long prevCount2 = 0;
unsigned long prevTime = 0;
unsigned long currentTime;

//Motor speed control pins
//const int leftMotorChannel = 0;
//const int rightMotorChannel = 1;

// Ultrasonic minimum distance
int distance_to_Obstacle=30;

// Define UART values
const int STOP = 0;
const int SLOW_FORWARD = 1;
const int MEDIUM_FORWARD = 2;
const int FAST_FORWARD = 3;
const int SLOW_RIGHT = 4;
const int SLOW_LEFT = 5;
const int RIGHT_90 = 6;
const int LEFT_90 = 7;

int receivedInt = 0;

int expectedRotationValue;


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

// PID control function for angle
void pidControl_angleR(int startEncoderValue1,int startEncoderValue2) {

    // Calculate error
    double error1 = (encoderValue1-startEncoderValue1)-expectedRotationValue;
    double error2 = (encoderValue2-startEncoderValue2)+expectedRotationValue;

    // Integral term
    errorSum1 += error1;
    errorSum2 += error2;

    // Derivative term
    double dError1 = error1 - lastError1;
    double dError2 = error2 - lastError2;

    // Calculate PID output
    double output1 = Kp * error1 + Ki * errorSum1 + Kd * dError1;
    double output2 = Kp * error2 + Ki * errorSum2 + Kd * dError2;

    // Apply PID output to adjust motor speeds
    int offset1 = 0;
    int offset2 = 0;
    actualMotorSpeed1 =  output1 +offset1;
    actualMotorSpeed2 =  output2 +offset2;

    // Update last error
    lastError1 = error1;
    lastError2 = error2;
}

// PID control function for angle
void pidControl_angleL(int startEncoderValue1,int startEncoderValue2) {

    // Calculate error
    double error1 = (encoderValue1-startEncoderValue1)+expectedRotationValue;
    double error2 = (encoderValue2-startEncoderValue2)-expectedRotationValue;

    // Integral term
    errorSum1 += error1;
    errorSum2 += error2;

    // Derivative term
    double dError1 = error1 - lastError1;
    double dError2 = error2 - lastError2;

    // Calculate PID output
    double output1 = Kp * error1 + Ki * errorSum1 + Kd * dError1;
    double output2 = Kp * error2 + Ki * errorSum2 + Kd * dError2;

    // Apply PID output to adjust motor speeds
    int offset1 = 0;
    int offset2 = 0;
    actualMotorSpeed1 =  output1 +offset1;
    actualMotorSpeed2 =  output2 +offset2;

    // Update last error
    lastError1 = error1;
    lastError2 = error2;
}


//Set the motor speed
/*void speed(int leftSpeed, int rightSpeed) {
  // Set speed for left motor
  analogWrite(leftMotorChannel, leftSpeed);

  // Set speed for right motor
  analogWrite(rightMotorChannel, rightSpeed);
}*/


// Function to adjust motor speeds based on PID output
void adjustMotorSpeed(double output1, double output2) {
    // Adjust motor speeds using PID output

    int offset1 = 0;
    int offset2 = 0;
    actualMotorSpeed1 = BASE_SPEED + output1 +offset1;
    actualMotorSpeed2 = BASE_SPEED + output2 +offset2;
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




/*void controlMotors(int pinA1, int pinA2, int pinB1, int pinB2, double A_SPEED = 150, double B_SPEED = 150) {
  //Motor 1 Control
  digitalWrite( motorA1, pinA1);
  digitalWrite( motorA2, pinA2);

  //Motor 2 Control
  digitalWrite( motorB1, pinB1);
  digitalWrite( motorB2, pinB2);

  //Control the speeds of the both motors
  pidControl_speed(A_SPEED, B_SPEED);
}*/

// Function to stop the motors
void stopMotors() {
  //controlMotors(0, 0, 0, 0);
  analogWrite(LPWM1, 0);
  analogWrite(RPWM1, 0);


  analogWrite(LPWM2, 0);
  analogWrite(RPWM2, 0);

}

void Forward(int A_Speed, int B_Speed){
    //controlMotors(1, 0, 1, 0, A_Speed, B_Speed);
  pidControl_speed(A_SPEED, B_SPEED);
  analogWrite(LPWM1, actualMotorSpeed1);
  analogWrite(RPWM1, 0);


  analogWrite(LPWM2, actualMotorSpeed2);
  analogWrite(RPWM2, 0);
}

void Motor1L(int Speed){
  analogWrite(LPWM1, Speed);
  analogWrite(RPWM1, 0);
}

void Motor1R(int Speed){
  analogWrite(LPWM1, 0);
  analogWrite(RPWM1, Speed);
}

void Motor2R(int Speed){
  analogWrite(LPWM2, 0);
  analogWrite(RPWM2, Speed);
}

void Motor2L(int Speed){
  analogWrite(LPWM2, Speed);
  analogWrite(RPWM2, 0);
}

/*void Rotate_Slow(int direction){
  controlMotors(1, 0, 0, 1, BASE_SPEED, BASE_SPEED);
}*/

void Rotate_90(int direction){
    int startEncoderValue1;
    int startEncoderValue2;
    if ( encoderFlag){
      startEncoderValue1= encoderValue1;
      startEncoderValue2= encoderValue2;
      encoderFlag = false;
    }
  
    
    if (direction == 0){ // 0 for right 90 rotation
      pidControl_angleR(startEncoderValue1,startEncoderValue2);
      if lastError1>=0 && lastError2>=0{
        Motor1L(actualMotorSpeed1);
        Motor2R(actualMotorSpeed2);
      }

      else if lastError1<0 && lastError2<0{
        Motor1R(actualMotorSpeed1);
        Motor2L(actualMotorSpeed2);
      }
      else if lastError1<0{
        Motor1R(actualMotorSpeed1);
      }

      else if lastError2<0{
        Motor2L(actualMotorSpeed2);
      }
    }

    if (direction == 1){ // 0 for Left 90 rotation
      pidControl_angleL(startEncoderValue1,startEncoderValue2);
      if lastError1>=0 && lastError2>=0{
        Motor1L(actualMotorSpeed1);
        Motor2R(actualMotorSpeed2);
      }

      else if lastError1<0 && lastError2<0{
        Motor1R(actualMotorSpeed1);
        Motor2L(actualMotorSpeed2);
      }
      else if lastError1<0{
        Motor1R(actualMotorSpeed1);
      }

      else if lastError2<0{
        Motor2L(actualMotorSpeed2);
      }
    }

/*
        if ((absvalue(encoderValue1 - startEncoderValue1) <= expectedRotationValue) && (absvalue(encoderValue2 - startEncoderValue2) <= expectedRotationValue)){
            pidControl_speed(A_SPEED, B_SPEED);
            analogWrite(LPWM1, actualMotorSpeed1);
            analogWrite(RPWM1, 0);


            analogWrite(LPWM2, 0);
            analogWrite(RPWM2, actualMotorSpeed2);
        }
        else if (absvalue(encoderValue1 - startEncoderValue1) < expectedRotationValue){
          controlMotors(1, 0, 0, 0, MAX_SPEED, MAX_SPEED);
        }
        else if (absvalue(encoderValue2 - startEncoderValue2) < expectedRotationValue){
          controlMotors(0, 0, 0, 1, MAX_SPEED, MAX_SPEED);
        
        else{
          encoderFlag = true;
          receivedInt = 0;
        }
    }
    else if (direction == 1){ // 1 for left 90 rotation
        
        if (((absvalue(encoderValue1 - startEncoderValue1) <= expectedRotationValue)) && (absvalue(encoderValue2 - startEncoderValue2) <= expectedRotationValue)){
            pidControl_speed(A_SPEED, B_SPEED);
            analogWrite(LPWM1, 0);
            analogWrite(RPWM1, actualMotorSpeed1);


            analogWrite(LPWM2, actualMotorSpeed2);
            analogWrite(RPWM2, 0);
        }
        /*else if (absvalue(encoderValue1 - startEncoderValue1) < expectedRotationValue){
          controlMotors(1, 0, 0, 0, MAX_SPEED, MAX_SPEED);
        }
        else if (absvalue(encoderValue2 - startEncoderValue2) < expectedRotationValue){
          controlMotors(0, 0, 0, 1, MAX_SPEED, MAX_SPEED);
        }
        else{
          encoderFlag = true;
          receivedInt = 0;
        }
    }*/
}

void setup(){
  currentTime=millis();
  //Encoder Setup
  Serial.begin (9600);

  //Motor1 pin define
  pinMode(RPWM1,OUTPUT);
  pinMode(LPWM1,OUTPUT);
  pinMode(R_EN1,OUTPUT);
  pinMode(L_EN1,OUTPUT);

  ////Motor1 pin define
  pinMode(RPWM2,OUTPUT);
  pinMode(LPWM2,OUTPUT);
  pinMode(R_EN2,OUTPUT);
  pinMode(L_EN2,OUTPUT);

 //Pull up the EN pins of the motor driver
  digitalWrite(R_EN1, HIGH);
  digitalWrite(L_EN1, HIGH);

  digitalWrite(R_EN2, HIGH);
  digitalWrite(L_EN2, HIGH);

  pinMode(encoder1APin, INPUT); 
  pinMode(encoder1BPin, INPUT);

  digitalWrite(encoder1APin, HIGH); //turn pullup resistor on
  digitalWrite(encoder1BPin, HIGH); //turn pullup resistor on

  //call updateEncoder() when any high/low changed seen
  //on interrupt 0 (pin 2), or interrupt 1 (pin 3) 
  attachInterrupt(0, updateEncoder, CHANGE); 
  attachInterrupt(1, updateEncoder, CHANGE);

  expectedRotationValue = (R/(4*r))*encoderResolution;
  
}

void loop() {

    if (Serial.available() > 0) {
        // Read the incoming byte
        receivedInt = Serial.parseInt();
    }
        
      // Check the received command and control motors accordingly
      switch (receivedInt) {
          case STOP:
              stopMotors();
              break;
          case SLOW_FORWARD:
              Forward(MIN_SPEED, MIN_SPEED);
              break;
          case MEDIUM_FORWARD:
              Forward(BASE_SPEED, BASE_SPEED);
              break;
          case FAST_FORWARD:
              Forward(MAX_SPEED, MAX_SPEED);
              break;
          case SLOW_RIGHT:
              Rotate_Slow(0);
              break;
          case SLOW_LEFT:
              Rotate_Slow(1);
              break;
          case RIGHT_90:
              Rotate_90(0);
          case LEFT_90:
              Rotate_90(1);
              break;
          default:
              // Stop the motors if invalid command received
              stopMotors();
              break;
  }
    
}

/*
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
 */
