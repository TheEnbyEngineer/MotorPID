// Arduino motor position control using PID
// External libraries
// https://www.arduino.cc/reference/en/libraries/pid/
// https://www.arduino.cc/reference/en/libraries/rotaryencoder/

#include <Arduino.h>
#include <RotaryEncoder.h>
#include <PID_v1.h>

// Variables that does not need to be tuned
double motorSetpoint = 0, motorFeedback = 0, motorOutput = 0;
bool done = true;
RotaryEncoder *encoder = nullptr;

// Specify the PID object and initial tuning parameters (motor)
double motorProportional = 10, motorIntegral = 0.1, motorDerivative = 0.2, tolerance = 1;
PID motorPID(&motorFeedback, &motorOutput, &motorSetpoint, motorProportional, motorIntegral, motorDerivative, DIRECT);


void setup() {
  // Start PID
  motorPID.SetMode(AUTOMATIC);
  motorPID.SetOutputLimits(-255, 255);
  
  // Start encoder
  encoder = new RotaryEncoder(28, 29, RotaryEncoder::LatchMode::TWO03);
  attachInterrupt(digitalPinToInterrupt(29), checkPosition, CHANGE);
  attachInterrupt(digitalPinToInterrupt(28), checkPosition, CHANGE);

  // Start motor
  pinMode(27, OUTPUT);
  pinMode(26, OUTPUT);
  
  // Start serial
  Serial.begin(115200);
}

 // Check the encoder state
void checkPosition() {
  encoder->tick();
}

void loop() {

  // Setpoint
  if (done) {
    Serial.print("Setpoint (in encoder units): ");
    while (!Serial.available()) {} // Wait for user input
    motorSetpoint = Serial.readStringUntil('\n').toInt();
    done = false;
  }


  // Feedback
  motorFeedback = encoder->getPosition();

  // Compute PID
  motorPID.Compute();

  // Output
  if (motorOutput > 1) {
    analogWrite(27, map(motorOutput, 0, 255, 100, 255));
    analogWrite(26, 0);
  }
  else if (motorOutput < -1) {
    analogWrite(26, map(motorOutput*-1, 0, 255, 100, 255));
    analogWrite(27, 0);
  }
  else {
    analogWrite(26, 0);
    analogWrite(27, 0);
  }

  // Shutdown when target is reached
  if (((motorFeedback - motorSetpoint) < tolerance) and ((motorFeedback - motorSetpoint) > tolerance*-1)) {
    motorSetpoint = motorFeedback; // Once the position is within tolerance, the setpoint would be set to the same value as the feedback position, even if it is not the same, to effective "shutdown" the controller
    done = true;
    analogWrite(26, 0);
    analogWrite(27, 0);
  }
  
  // Debug
  Serial.print("Target-position: ");
  Serial.print(motorSetpoint);
  Serial.print("    Current-position: ");
  Serial.print(motorFeedback);
  Serial.print("    Throttle: ");
  Serial.print(motorOutput);
  Serial.print("    Error: ");
  Serial.print(motorFeedback - motorSetpoint);
  Serial.print("Done: ");
  Serial.println(done);
  
}
