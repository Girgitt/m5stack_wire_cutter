#include <Arduino.h>
//#include <M5CoreS3.h>
#include <M5Unified.h>
#include <Wire.h>
#include "Module_Stepmotor.h"
#include <SCServo.h>

#include <WiFi.h>
#include <PubSubClient.h>

#include "config.h"

SCSCL sc;
SMS_STS st;
// change to CORE2 if using Core2
#define CORE_2

#ifdef CORE_S3
    //static const int X_STEP_PIN = 18;  // used by UART2 on Port.C
    //static const int X_DIR_PIN = 17;   // used by UART2 on Port.C
    static const int Y_STEP_PIN = 6;
    static const int Y_DIR_PIN = 7;
    static const int Z_STEP_PIN = 13;
    static const int Z_DIR_PIN = 0;
    
    static const int SDA_PIN = 12;
    static const int SCL_PIN = 11;

    static const int SERVO_UART_TX = 18;  // G17 (TX)
    static const int SERVO_UART_RX = 17;  // G18 (RX)

    static const int PB_0 = 9; 
    static const int PB_1 = 8;

#elif defined CORE_2
    //static const int X_STEP_PIN = 13;  // used by UART2 on Port.C
    //static const int X_DIR_PIN = 14;   // used by UART2 on Port.C
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


// WiFi credentials
const char* ssid = WIFI_SSID;
const char* password = WIFI_PASSWORD;

// MQTT broker details
const char* mqtt_client_id = MQTT_CLIENT_ID; // Unique ID
const char* mqtt_server = MQTT_SERVER;
const int mqtt_port = MQTT_PORT;
const char* mqtt_username = MQTT_USERNAME;  // Optional, if required
const char* mqtt_password = MQTT_PASSWORD;  // Optional, if required
static unsigned long lastPublish = 0;

WiFiClient espClient;
PubSubClient client(espClient);

// MQTT topic definitions
String global_status_topic = "cutters/" + String(mqtt_client_id) + "/status";
String servo_state_topic = "cutters/" + String(mqtt_client_id) + "/servo/0";
String servo_command_topic = "cutters/" + String(mqtt_client_id) + "/servo/0/command";
String stepper_state_topic = "cutters/" + String(mqtt_client_id) + "/stepper/0";
String stepper_command_topic = "cutters/" + String(mqtt_client_id) + "/stepper/0/command";

int servo_pos_init = 200;      //adjust to your blade length so the blade does not block wire in open posiotion
int servo_pos_cut_1 = 1120;    //adjust to your blade length so the close position does not destroy servo or its mounting
int servo_pos_retract_1 = servo_pos_cut_1 - 300;
int servo_pos_cut_2 = servo_pos_cut_1;

int servo_delay_ms_long = 600;  // give loong enouth time for the cut travel and initial movement
int servo_delay_ms_short = 300; // give loong enouth time for the first cut and retraction move

int servo_speed_move = 0;
int servo_accel_move = 0;

int servo_speed_cut = 1000;
int servo_accel_cut = 0;



static Module_Stepmotor driver;

HardwareSerial SerialServo(2);

const unsigned int stepPulseDelayMicroseconds = 600; // 500 µs high, 500 µs low


String current_state = "IDLE"; // IDLE, FEEDING, CUTTING
String requested_state = "IDLE";
int cutter_state_report_timeout_ms = 1000;
int cutter_state_report_age_ms = 0;

int requested_length_mm = 0;

// Function to connect to WiFi
void setup_wifi() {
  delay(10);
  // Connect to WiFi
  Serial.println();
  Serial.print("Connecting to WiFi...");
  WiFi.begin(ssid, password);
  
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.print(".");
  }
  Serial.println(" connected!");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());
}


void publishServoState(){
  //String state = relayState ? "on" : "off";
  //client.publish(servo_state_topic.c_str(), state.c_str(), 1);
}

void publishStepperState(){
  //String state = relayState ? "on" : "off";
  //client.publish(servo_state_topic.c_str(), state.c_str(), 1);
}

void publishGlobalSatus(){
  String status_node = global_status_topic + "/state";
  client.publish(status_node.c_str(), current_state.c_str(), 1);
}

// Callback function to handle received messages
void callback(char* topic, byte* payload, unsigned int length) {
  // Convert payload to string
  String message = "";
  for (int i = 0; i < length; i++) {
    message += (char)payload[i];
  }
  Serial.println("handling command message: "+message);
  // Check if the received message is for servo
  if (String(topic) == String(servo_command_topic)) {
    
    Serial.println("handling servo command: "+message);
    if (message == "cut") {
      if (current_state == "IDLE"){
        requested_state = "CUTTING";
      }
      else{
        Serial.println("ERROR: requested qutting while state is not IDLE");
      }

    }
  }
  else if (String(topic) == String(stepper_command_topic)) {
    Serial.println("handling stepper command: "+message);

    // we are expecting something like "feed:300"

    int separatorIndex = message.indexOf(":");
    
    // If ":" is found, proceed with splitting
    if (separatorIndex != -1) {
        String command = message.substring(0, separatorIndex);  // Extract "feed"
        String valueStr = message.substring(separatorIndex + 1); // Extract "length"
        
        // Convert valueStr to integer
        int value = valueStr.toInt();
        
        // Validate input
        if (command == "feed" && value > 0 && value < 1000) {
            Serial.println("Valid command received: " + command);
            Serial.println("Valid value received: " + String(value));
            
            requested_length_mm = value;
            requested_state = "FEEDING";


        } else {
            Serial.println("Invalid command or value out of range.");
        }
    } else {
        Serial.println("Invalid format: ':' missing.");
    }


  }

}

// Reconnect to MQTT server
void reconnect() {
  // Loop until we are connected
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    
    // Attempt to connect
    //if (client.connect("M5StackClient", mqtt_username, mqtt_password)) {
    if (client.connect(mqtt_client_id, mqtt_username, mqtt_password)) {
      Serial.println("connected to mqtt broker");
      
      // Subscribe to the command topic
      client.subscribe(servo_command_topic.c_str()); // Convert String to const char* using c_str()
      client.subscribe(stepper_command_topic.c_str()); // Convert String to const char* using c_str()

    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      delay(5000);
    }
  }
}



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

void move_servo_to_initial_position(){
  st.WritePosEx(1, servo_pos_init, servo_speed_move, servo_accel_move);
  delay(800);
}

void make_cut_sequence(){
  st.WritePosEx(1, servo_pos_cut_1, servo_speed_move, servo_accel_move);
  delay(servo_delay_ms_long);
  
  st.WritePosEx(1, servo_pos_cut_1, servo_speed_cut, servo_accel_cut);
  delay(servo_delay_ms_short);
  
  st.WritePosEx(1, servo_pos_retract_1, servo_speed_move, servo_accel_move);
  delay(servo_delay_ms_short);
  
  st.WritePosEx(1, servo_pos_cut_2, servo_speed_cut, servo_accel_cut);
  delay(servo_delay_ms_long);
  
  move_servo_to_initial_position();
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
    //sc.pSerial = &SerialServo;
    st.pSerial = &SerialServo;
    while (!SerialServo) {
     ; // Wait for serial port if needed.
     
    }
    Serial.println("M5Stack CoreS3 - SerialServo initialized.");

    Wire.begin(SDA_PIN, SCL_PIN, 400000UL);

    driver.init(Wire, 0x27);
    driver.resetMotor(0, 0);
    driver.resetMotor(1, 0);
    driver.resetMotor(2, 0);
    
    driver.enableMotor(1); // we are using Y axis only

    // // PWM, sets the speed of the stepper motor, adjust accordingly
    // ledcSetup(0, 800, 8);

    // // XYZ step
    // ledcAttachPin(X_STEP_PIN, 0);
    // ledcAttachPin(Y_STEP_PIN, 0);
    // ledcAttachPin(Z_STEP_PIN, 0);

    // ledcWrite(0, 127);

    // // XYZ dir
    // Core cannot use dirX together with external serial servos
    // #ifdef CORE_S3
    //   pinMode(X_DIR_PIN, OUTPUT);
    //   digitalWrite(X_DIR_PIN, 1);
    //   pinMode(X_STEP_PIN, OUTPUT);
    // #endif
    
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
    // Connect to WiFi
    setup_wifi();
    M5.Lcd.printf("Wifi");
    
    // Set up MQTT client
    client.setServer(mqtt_server, mqtt_port);
    client.setCallback(callback);
    
    // Connect to MQTT server
    reconnect();
    if (client.connected()){
      M5.Lcd.printf("MQTT");
      publishGlobalSatus();
    }
    
    client.setKeepAlive(60);

    move_servo_to_initial_position();
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

void test_moves(){
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
  
  
  //uint8_t reslt = sc.WritePos(1, 1000, 1500, 50);
  //Serial.print("servo result:");
  //Serial.println(reslt);
  
  //st.WritePosEx(1, 200, 4095, 255); 
  //delay(100); 
  
  st.WritePosEx(1, 1120, 0, 0);
  //delay(500);
  delay(500);
  st.WritePosEx(1, 1120, 1000, 0);
  delay(300);
  st.WritePosEx(1, 900, 0, 0);
  delay(300);
  st.WritePosEx(1, 1140, 1000, 255);
  delay(600);
  // for (int i = 0; i<30; i++){
  //   int force = st.ReadLoad(1);
  //   Serial.println(force);
  //   if(force > 800){
  //     st.WritePosEx(1, 100, 4095, 255);
  //   }
  //   delay(100);
  // }

  // delay(1500); 
  // st.WritePosEx(1, 1330, 100, 10);
  // delay(500);
  // st.WritePosEx(1, 1360, 100, 10);
  // delay(500);
  // st.WritePosEx(1, 1390, 100, 10);
  // delay(500);

  st.WritePosEx(1,200, 0, 0);
  //SerialServo.println("test test test");
  
  //st.WritePosEx(254, 1000, 1500, 50);

  // Example 2: Move the X-axis 50 steps to the left.
  Serial.println("Moving X axis 300 steps to the left.");
  //moveStepper(-300);
  moveStepperDynamic(-100, 400, 1800, 50);
  
  //sc.WritePos(1, 100, 1500, 50);
  ////st.WritePosEx(1, 2000, 1500, 50);
  //sendServoCommand2(1, 100, 20);
  //st.WritePosEx(1, 2000, 3400, 100);
}



void loop(){

  if (current_state != requested_state){
    Serial.println("requested new state: "+requested_state);
    
    if (requested_state == "FEEDING"){
      
      current_state = "FEEDING";
      // 800 steps == 127mm
      moveStepperDynamic(int(float(requested_length_mm) / (127.0 / 800.0)), 600, 1800, 50);

    }
    else if (requested_state == "CUTTING"){
      current_state = "CUTTING";
      make_cut_sequence();
    }
    
    current_state = "IDLE";
    requested_state = "IDLE";
  }

  if (client.connected()){
    client.loop(); // Process incoming messages
  }
  else{
    reconnect();
  }

  delay(20);
}