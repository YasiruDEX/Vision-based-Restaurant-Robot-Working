#include <AccelStepper.h>

// Define pins for stepper motor connections
#define STEP_PIN 2
#define DIR_PIN 3
#define ENABLE_PIN 4

// Define the maximum and minimum angles for the tray
int MAX_ANGLE = 45;
int MIN_ANGLE = -45;

// Define PID constants
float Kp = 2.0; // Proportional gain we should tune
float Ki = 0.0; // Integral gain 
float Kd = 0.0; // Derivative gain 

// Define PID variables
float setpoint_roll = 0; // Desired roll angle(final)
float setpoint_pitch = 0; // Desired pitch angle(final)
float input_roll, input_pitch; // Predicted roll and pitch angles(from kalman filter)
float last_input_roll, last_input_pitch; // Previous roll and pitch angles(from kalman filter)
float iTerm_roll = 0, iTerm_pitch = 0; // Integral term
float output_roll, output_pitch; // PID controller output angles
int step_roll, step_pitch; // PID contoller outputs in steps
unsigned long lastTime;
float deltaTime;

// Define PID limits
int MAX_OUTPUT= 50; // Maximum output value (let me consider stepper has 200 steps per revolution)
int MIN_OUTPUT = -50; // Minimum output value

// Initialize AccelStepper object
AccelStepper stepper(AccelStepper::DRIVER, STEP_PIN, DIR_PIN);

// PID calculation function
int computePID(float input, float setpoint, float &lastInput, float &iTerm, float Kp, float Ki, float Kd) {
  unsigned long now = millis();
  deltaTime = (float)(now - lastTime) / 1000; // Time since last PID calculation
  lastTime = now;

  // Compute error terms
  float error = setpoint - input;
  iTerm += (Ki * error * deltaTime);

  // Apply limits to integral term
  if (iTerm > MAX_OUTPUT) iTerm = MAX_OUTPUT;
  else if (iTerm < MIN_OUTPUT) iTerm = MIN_OUTPUT;

  // Compute PID output
  float dInput = (input - lastInput) / deltaTime;
  float output = Kp * error + iTerm - Kd * dInput;

  // Apply limits to PID output
  if (output > MAX_OUTPUT) output = MAX_OUTPUT;
  else if (output < MIN_OUTPUT) output = MIN_OUTPUT;

  // Save current input for next iteration
  lastInput = input;
  int outputStep = output; // our solved angle and step relation equation
  
  return outputStep;
}

void setup() {
  // Initialize serial communication
  Serial.begin(9600);

  // Set stepper motor pins as outputs
  pinMode(STEP_PIN, OUTPUT);
  pinMode(DIR_PIN, OUTPUT);
  pinMode(ENABLE_PIN, OUTPUT);

  // Enable stepper motor driver
  digitalWrite(ENABLE_PIN, LOW);

  // Set initial position of stepper motor
  stepper.setMaxSpeed(1000); // Set maximum speed in steps per second
  stepper.setAcceleration(500); // Set acceleration in steps per second per second
}

void loop() {
  // Read predicted roll and pitch angles from sensors
  // Replace the following lines with your actual sensor readings
  input_roll = 0; // Example: replace 0 with actual roll angle reading
  input_pitch = 0; // Example: replace 0 with actual pitch angle reading

  // Compute PID output for roll angle
  step_roll = computePID(input_roll, setpoint_roll, last_input_roll, iTerm_roll, Kp, Ki, Kd);

  // Compute PID output for pitch angle
  step_pitch = computePID(input_pitch, setpoint_pitch, last_input_pitch, iTerm_pitch, Kp, Ki, Kd);

  // Drive stepper motor based on PID outputs
  // Adjust direction and step size based on your motor and driver configuration
  if (output_roll > 0) {
    digitalWrite(DIR_PIN, HIGH); // Set direction for positive rotation
    stepper.moveTo(output_roll); // Move stepper motor
  } else if (output_roll < 0) {
    digitalWrite(DIR_PIN, LOW); // Set direction for negative rotation
    stepper.moveTo(-output_roll); // Move stepper motor
  }

  // Update stepper motor position
  stepper.run();

  // Print debug information
  Serial.print("Roll Step Output: ");
  Serial.println(step_roll);
  Serial.print("Pitch Step Output: ");
  Serial.println(step_pitch);
  
  delay(100); // Delay for stability
}

