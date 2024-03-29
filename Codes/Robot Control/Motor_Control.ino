// Motor A pins
const int motorA1 = 2;
const int motorA2 = 3;
const int motorAEn = 4; //PWM pin for Speed control
// Motor B pins
const int motorB1 = 5;
const int motorB2 = 6;
const int motorBEn = 7; //PWM pin for Speed control

// Encoders A,B feedback pins
const int encoderA = 8;
const int encoderB = 9;

// Define UART values
const int FORWARD = 0;
const int RIGHT = 1;
const int LEFT = 2;
const int STOP = 3;
const int RIGHT90 = 4;
const int LEFT90 = 5;
const int MAX_SPEED = 6;
const int MIN_SPEED = 7;

// Define Speeed values
const int MAX_SPEED = 200;
const int MIN_SPEED = 100;
const int BASE_SPEED = 150;
int A_SPEED; // 0-255 range
int B_SPEED; // 0-255 range

// Function to control the motors
void controlMotors(int A1, int A2, int A_SPEED = BASE_SPEED, int B1, int B2, int B_SPEED = BASE_SPEED) {
    digitalWrite(motorAEn, A_SPEED);
    digitalWrite(motorA1, A1);
    digitalWrite(motorA2, A2);

    digitalWrite(motorBEn, B_SPEED);
    digitalWrite(motorB1, B1);
    digitalWrite(motorB2, B2);
}

// Function to stop the motors
void stopMotors() {
  controlMotors(0, 0, 0, 0, 0, 0);
}

void Forward(int A_Speed, int B_Speed){
    controlMotors(1, 0, 1, 0, A_Speed, B_Speed);
}

void Right(int A_SPEED, int B_SPEED){
    controlMotors(0, 0, 1, 0, A_Speed, B_Speed);
}

void Left(int A_SPEED, int B_SPEED){
    controlMotors(1, 0, 0, 0, A_Speed, B_Speed);
}

void Right90(int A_SPEED, int B_SPEED){
    controlMotors(0, 0, 1, 0, A_Speed, B_Speed);
}

void Left90(int A_SPEED, int B_SPEED){
    controlMotors(1, 0, 0, 0, A_Speed, B_Speed);
}




void setup() {
  Serial.begin(9600);

  // Initialize motor pins
  pinMode(motorA1, OUTPUT);
  pinMode(motorA2, OUTPUT);
  pinMode(motorB1, OUTPUT);
  pinMode(motorB2, OUTPUT);

  // Initialize encoder pins
  pinMode(encoderA, INPUT);
  pinMode(encoderB, INPUT);
}

void loop() {
    // Read encoder values and perform necessary actions
    int encoderAValue = digitalRead(encoderA);
    int encoderBValue = digitalRead(encoderB);

    if (Serial.available() > 0) {
        // Read the incoming byte
        int receivedInt = Serial.parseInt();
        
        // Check the received command and control motors accordingly
        switch (receivedInt) {
            case FORWARD:
                forward(A_SPEED, B_SPEED);
                break;
            case RIGHT:
                Right(MIN_SPEED, MIN_SPEED);
                break;
            case LEFT:
                Left(A_SPEED, B_SPEED);
                break;
            case STOP:
                stopMotors();
                break;
            case RIGHT90:
                Right(BASE_SPEED, BASE_SPEED);
                break;
            case LEFT90:
                Left(BASE_SPEED, BASE_SPEED);
                break;
            case MAX_SPEED:
                A_SPEED = MAX_SPEED;
                B_SPEED = MAX_SPEED;
            case MIN_SPEED:
                A_SPEED = MIN_SPEED;
                B_SPEED = MIN_SPEED;
                break;
            default:
                // Stop the motors if invalid command received
                stopMotors();
                break;
    }
    
  }
}
// ENCODER FEEDBACK HAVEN'T IMPLEMENTED YET


