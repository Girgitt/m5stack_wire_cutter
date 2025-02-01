#include <Arduino.h>
#include <M5CoreS3.h>
#include <Wire.h>
#include "Module_Stepmotor.h"

// change to CORE2 if using Core2
#define CORE_S3

#ifdef CORE_S3
    static const int X_STEP_PIN = 18;
    static const int X_DIR_PIN = 17;
    static const int Y_STEP_PIN = 6;
    static const int Y_DIR_PIN = 7;
    static const int Z_STEP_PIN = 13;
    static const int Z_DIR_PIN = 0;
    
    static const int SDA_PIN = 12;
    static const int SCL_PIN = 11;
#elif defined CORE2
    static const int X_STEP_PIN = 13;
    static const int X_DIR_PIN = 14;
    static const int Y_STEP_PIN = 27;
    static const int Y_DIR_PIN = 19;
    static const int Z_STEP_PIN = 2; // Not sure about this one
    static const int Z_DIR_PIN = 0;

    static const int SDA_PIN = 21;
    static const int SCL_PIN = 22;
#endif

static Module_Stepmotor driver;

const unsigned int stepPulseDelayMicroseconds = 600; // 500 µs high, 500 µs low

void setup() {
    // Initialize serial communication for debugging.
    Serial.begin(115200);
    while (!Serial) {
      ; // Wait for serial port if needed.
    }
    Serial.println("M5Stack CoreS3 initialized.");
    Wire.begin(SDA_PIN, SCL_PIN, 400000UL);

    driver.init(Wire, 0x27);
    driver.resetMotor(0, 0);
    driver.resetMotor(1, 0);
    driver.resetMotor(2, 0);
    driver.enableMotor(1);

    // // PWM, sets the speed of the stepper motor, adjust accordingly
    // ledcSetup(0, 800, 8);

    // // XYZ step
    // ledcAttachPin(X_STEP_PIN, 0);
    // ledcAttachPin(Y_STEP_PIN, 0);
    // ledcAttachPin(Z_STEP_PIN, 0);

    // ledcWrite(0, 127);

    // // XYZ dir
    pinMode(X_DIR_PIN, OUTPUT);
    digitalWrite(X_DIR_PIN, 1);
    pinMode(Y_DIR_PIN, OUTPUT);
    digitalWrite(Y_DIR_PIN, 1);
    pinMode(Z_DIR_PIN, OUTPUT);
    digitalWrite(Z_DIR_PIN, 1);

    pinMode(X_STEP_PIN, OUTPUT);
    pinMode(X_STEP_PIN, OUTPUT);
    pinMode(X_STEP_PIN, OUTPUT);
}

void moveStepper(int steps) {
  if (steps == 0) return; // Nothing to do if step count is zero.

  // Set motor direction:
  //   HIGH on the DIR pin is assumed to move "right".
  //   LOW moves "left".
  if (steps > 0) {
    digitalWrite(X_DIR_PIN, 1);
    Serial.println(">> moving right");
  } else {
    digitalWrite(X_DIR_PIN, 0);
    Serial.println(">> moving left");
    steps = -steps;  // Convert negative steps to positive.
  }

  Serial.println("  > move");
  // Generate the required number of step pulses.
  for (int i = 0; i < steps; i++) {
    digitalWrite(X_STEP_PIN, 1);
    delayMicroseconds(stepPulseDelayMicroseconds);
    digitalWrite(X_STEP_PIN, 0);
    delayMicroseconds(stepPulseDelayMicroseconds);
  }
  Serial.println("  < move");

}


/*
  moveStepperDynamic(steps, minDelay, maxDelay, rampSteps):

  - steps: Total steps to move. A positive value moves "right" (X_DIR_PIN = HIGH)
           and a negative value moves "left" (X_DIR_PIN = LOW).
  - minDelay: Minimum delay (in microseconds) when at full speed.
  - maxDelay: Maximum delay (in microseconds) during acceleration and deceleration.
  - rampSteps: The number of steps to use for acceleration at the beginning and
               deceleration at the end.

  The function linearly interpolates the step delay from maxDelay to minDelay over
  the acceleration region, uses minDelay for the constant-speed region, and then
  linearly increases the delay from minDelay back to maxDelay during deceleration.
*/
void moveStepperDynamic(int steps, unsigned int minDelay, unsigned int maxDelay, int rampSteps) {
  if (steps == 0) return; // Nothing to do if step count is zero.

  // Set the direction:
  //   HIGH on the DIR pin moves "right" and LOW moves "left".
  if (steps > 0) {
    digitalWrite(X_DIR_PIN, HIGH);
    Serial.println(">> moving right");
  } else {
    digitalWrite(X_DIR_PIN, LOW);
    Serial.println(">> moving left");
    steps = -steps;  // Convert negative steps to positive.
  }

  // Adjust the ramp steps if the total steps are too few.
  int effectiveRamp = rampSteps;
  if (steps < 2 * rampSteps) {
    effectiveRamp = steps / 2;
  }

  Serial.println("  > move");
  // Loop over each step and adjust the delay to implement acceleration/deceleration.
  for (int i = 0; i < steps; i++) {
    unsigned int currentDelay;

    if (i < effectiveRamp) {
      // Acceleration phase: delay decreases linearly from maxDelay to minDelay.
      currentDelay = maxDelay - ((maxDelay - minDelay) * i) / effectiveRamp;
    } else if (i >= steps - effectiveRamp) {
      // Deceleration phase: delay increases linearly from minDelay back to maxDelay.
      int decelStep = i - (steps - effectiveRamp);
      currentDelay = minDelay + ((maxDelay - minDelay) * decelStep) / effectiveRamp;
    } else {
      // Constant speed region.
      currentDelay = minDelay;
    }

    // Generate the step pulse using the current delay.
    digitalWrite(X_STEP_PIN, HIGH);
    delayMicroseconds(currentDelay);
    digitalWrite(X_STEP_PIN, LOW);
    delayMicroseconds(currentDelay);
  }
  Serial.println("  < move");
}



void loop() {
    // digitalWrite(X_DIR_PIN, 0);
    // delay(1000);
    
    // digitalWrite(X_DIR_PIN, 1);
    // delay(1000);

  // 300 steps == 48,5mm; 1step = 0,16167mm
  // Example 1: Move the X-axis 100 steps to the right.
  Serial.println("Moving X axis 300 steps to the right.");
  //moveStepper(300);
  moveStepperDynamic(300, 400, 1800, 50);
  delay(2000);  

  // Example 2: Move the X-axis 50 steps to the left.
  Serial.println("Moving X axis 300 steps to the left.");
  //moveStepper(-300);
  moveStepperDynamic(-300, 400, 1800, 50);
  delay(2000); 
}