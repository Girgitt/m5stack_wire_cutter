#include <Arduino.h>
//#include <M5CoreS3.h>
#include <M5Unified.h>
#include <Wire.h>
#include "Module_Stepmotor.h"
#include <SCServo.h>

SCSCL st;

// change to CORE2 if using Core2
#define CORE_2

#ifdef CORE_S3
    static const int X_STEP_PIN = 18;
    static const int X_DIR_PIN = 17;
    static const int Y_STEP_PIN = 6;
    static const int Y_DIR_PIN = 7;
    static const int Z_STEP_PIN = 13;
    static const int Z_DIR_PIN = 0;
    
    static const int SDA_PIN = 12;
    static const int SCL_PIN = 11;

    static const int SERVO_UART_TX = 17;  // G17 (TX)
    static const int SERVO_UART_RX = 18;  // G18 (RX)

    static const int PB_0 = 9; 
    static const int PB_1 = 8;

#elif defined CORE_2
    //static const int X_STEP_PIN = 13;
    //static const int X_DIR_PIN = 14;
    static const int Y_STEP_PIN = 27;
    static const int Y_DIR_PIN = 19;
    static const int Z_STEP_PIN = 2; // Not sure about this one
    static const int Z_DIR_PIN = 0;

    static const int SDA_PIN = 21;
    static const int SCL_PIN = 22;

    static const int SERVO_UART_TX = 13; 
    static const int SERVO_UART_RX = 14; 

    static const int PB_0 = 26; 
    static const int PB_1 = 36; 
#endif

static Module_Stepmotor driver;

HardwareSerial SerialServo(2);

const unsigned int stepPulseDelayMicroseconds = 600; // 500 µs high, 500 µs low


void sendServoCommand(uint8_t id, uint16_t position, uint16_t time) {
    uint8_t cmd[10];
    
    // Protocol: Header (0x55 0x55), ID, Length, Cmd, Params, Checksum
    cmd[0] = 0xFF;  // Header
    cmd[1] = 0xFF;  // Header
    cmd[2] = id;    // Servo ID
    cmd[3] = 7;     // Length of the command
    cmd[4] = 1;     // Command (Move)

    // Position (low byte, high byte)
    cmd[5] = position & 0xFF;
    cmd[6] = (position >> 8) & 0xFF;

    // Time (low byte, high byte)
    cmd[7] = time & 0xFF;
    cmd[8] = (time >> 8) & 0xFF;

    // Checksum
    uint8_t checksum = 0;
    for (int i = 2; i < 9; i++) {
        checksum += cmd[i];
    }
    cmd[9] = ~checksum;  // Inverted checksum

    // Enable TX mode
    pinMode(SERVO_UART_TX, OUTPUT);
    
    // Send command
    SerialServo.write(cmd, sizeof(cmd));
    SerialServo.flush();

    // Switch back to RX mode (release TX line)
    pinMode(SERVO_UART_TX, INPUT);
}

void sendServoCommand2(uint8_t id, uint16_t position, uint16_t speed) {
    uint8_t cmd[11];
    uint8_t checksum = 0;

    cmd[0] = 0xFF;  // Header
    cmd[1] = 0xFF;  // Header
    cmd[2] = id;    // Servo ID
    cmd[3] = 7;     // Length
    cmd[4] = 0x03;  // WRITE DATA instruction
    cmd[5] = 0x1E;  // Starting address for Goal Position
    cmd[6] = position & 0xFF;        // Goal Position Low Byte
    cmd[7] = (position >> 8) & 0xFF; // Goal Position High Byte
    cmd[8] = speed & 0xFF;           // Moving Speed Low Byte
    cmd[9] = (speed >> 8) & 0xFF;    // Moving Speed High Byte

    for (int i = 2; i < 10; i++) {
        checksum += cmd[i];
    }
    cmd[10] = ~checksum;  // Checksum

    SerialServo.write(cmd, sizeof(cmd));
    SerialServo.flush();
}


void setup() {
    M5.begin();
    // Initialize serial communication for debugging.
    Serial.begin(115200, SERIAL_8N1);
    while (!Serial) {
      ; // Wait for serial port if needed.
    }
    Serial.println("M5Stack CoreS3 - Serial initialized.");

    //SerialServo.begin(1000000, SERIAL_8N1, SERVO_UART_RX, SERVO_UART_TX);
    SerialServo.begin(1000000, SERIAL_8N1, SERVO_UART_TX, SERVO_UART_RX);
    st.pSerial = &SerialServo;
    //while (!SerialServo) {
    //  ; // Wait for serial port if needed.
    //  
    //}
    Serial.println("M5Stack CoreS3 - SerialServo initialized.");

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
    // Core cannot use dirX together with external serial servos
    #ifdef CORE_S3
      pinMode(X_DIR_PIN, OUTPUT);
      digitalWrite(X_DIR_PIN, 1);
      pinMode(X_STEP_PIN, OUTPUT);
    #endif
    
    pinMode(Y_DIR_PIN, OUTPUT);
    digitalWrite(Y_DIR_PIN, 1);
    pinMode(Y_STEP_PIN, OUTPUT);

    pinMode(Z_DIR_PIN, OUTPUT);
    digitalWrite(Z_DIR_PIN, 1);
    pinMode(Z_STEP_PIN, OUTPUT);

    // temporary 3V source for pullup required by servo one-wire serial line 
    pinMode(PB_0, OUTPUT);
    pinMode(PB_1, OUTPUT);
    digitalWrite(PB_0, 1);
    digitalWrite(PB_1, 1);

    M5.Lcd.printf("Started");
}

void moveStepper(int steps) {
  if (steps == 0) return; // Nothing to do if step count is zero.

  // Set motor direction:
  //   HIGH on the DIR pin is assumed to move "right".
  //   LOW moves "left".
  if (steps > 0) {
    digitalWrite(Y_DIR_PIN, 1);
    Serial.println(">> moving right");
  } else {
    digitalWrite(Y_DIR_PIN, 0);
    Serial.println(">> moving left");
    steps = -steps;  // Convert negative steps to positive.
  }

  Serial.println("  > move");
  // Generate the required number of step pulses.
  for (int i = 0; i < steps; i++) {
    digitalWrite(Y_STEP_PIN, 1);
    delayMicroseconds(stepPulseDelayMicroseconds);
    digitalWrite(Y_STEP_PIN, 0);
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
    digitalWrite(Y_DIR_PIN, HIGH);
    Serial.println(">> moving right");
  } else {
    digitalWrite(Y_DIR_PIN, LOW);
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
    digitalWrite(Y_STEP_PIN, HIGH);
    delayMicroseconds(currentDelay);
    digitalWrite(Y_STEP_PIN, LOW);
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
  moveStepperDynamic(100, 400, 1800, 50);
  //sendServoCommand2(1, 500, 20);
  st.WritePos(254, 1000, 1500, 50);
  delay(2000);  
  
  //SerialServo.println("test test test");
  
  //st.WritePosEx(254, 1000, 1500, 50);

  // Example 2: Move the X-axis 50 steps to the left.
  Serial.println("Moving X axis 300 steps to the left.");
  //moveStepper(-300);
  moveStepperDynamic(-100, 400, 1800, 50);
  st.WritePos(254, 100, 1500, 50);
  //sendServoCommand2(1, 100, 20);
  delay(2000);

  //sendServoCommand(254, 100, 1000); 
  // //st.WritePosEx(254, 100, 1500, 50);
  //delay(200);
}