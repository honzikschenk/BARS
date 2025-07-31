/*
 * BARS Robot - ROS2 Arduino Integration Example
 * 
 * This example demonstrates how the existing Arduino code can be extended
 * to communicate with the ROS2 framework through the Arduino bridge node.
 * 
 * Features:
 * - JSON-based serial communication
 * - Servo command parsing
 * - Status reporting
 * - Compatible with existing Arduino servo control
 */

#include <ArduinoJson.h>
#include <Servo.h>

// Servo configuration
const int NUM_SERVOS = 12;
Servo servos[NUM_SERVOS];
int servo_pins[NUM_SERVOS] = {2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13};

// Servo state tracking
struct ServoState {
  int id;
  float current_angle;
  float target_angle;
  float speed;
  bool enabled;
  bool moving;
  unsigned long last_update;
};

ServoState servo_states[NUM_SERVOS];

// Communication
const size_t JSON_BUFFER_SIZE = 512;
StaticJsonDocument<JSON_BUFFER_SIZE> doc;
unsigned long last_status_report = 0;
const unsigned long STATUS_INTERVAL = 100; // 10Hz status reporting

void setup() {
  Serial.begin(115200);
  
  // Initialize servos
  for (int i = 0; i < NUM_SERVOS; i++) {
    servos[i].attach(servo_pins[i]);
    servo_states[i].id = i;
    servo_states[i].current_angle = 0.0;
    servo_states[i].target_angle = 0.0;
    servo_states[i].speed = 0.5;
    servo_states[i].enabled = false;
    servo_states[i].moving = false;
    servo_states[i].last_update = millis();
    
    // Set to neutral position
    servos[i].write(90);
  }
  
  // Send initialization message
  doc.clear();
  doc["type"] = "status";
  doc["message"] = "BARS Arduino initialized";
  doc["servos"] = NUM_SERVOS;
  serializeJson(doc, Serial);
  Serial.println();
  
  delay(1000);
}

void loop() {
  // Process incoming commands
  processSerialCommands();
  
  // Update servo positions
  updateServos();
  
  // Send status reports
  if (millis() - last_status_report > STATUS_INTERVAL) {
    sendStatusReport();
    last_status_report = millis();
  }
  
  delay(10); // Small delay for stability
}

void processSerialCommands() {
  if (Serial.available() > 0) {
    String input = Serial.readStringUntil('\n');
    input.trim();
    
    if (input.length() > 0) {
      deserializeJson(doc, input);
      
      String command_type = doc["type"];
      
      if (command_type == "servo_cmd") {
        handleServoCommand();
      } else if (command_type == "status_request") {
        sendDetailedStatus();
      } else if (command_type == "emergency_stop") {
        emergencyStop();
      }
    }
  }
}

void handleServoCommand() {
  int servo_id = doc["servo_id"];
  float angle = doc["angle"];
  float speed = doc["speed"];
  bool enable = doc["enable"];
  
  if (servo_id >= 0 && servo_id < NUM_SERVOS) {
    servo_states[servo_id].target_angle = angle;
    servo_states[servo_id].speed = constrain(speed, 0.1, 1.0);
    servo_states[servo_id].enabled = enable;
    
    if (!enable) {
      servos[servo_id].detach();
    } else if (!servos[servo_id].attached()) {
      servos[servo_id].attach(servo_pins[servo_id]);
    }
  }
}

void updateServos() {
  unsigned long current_time = millis();
  
  for (int i = 0; i < NUM_SERVOS; i++) {
    if (!servo_states[i].enabled) continue;
    
    float dt = (current_time - servo_states[i].last_update) / 1000.0;
    servo_states[i].last_update = current_time;
    
    // Calculate movement step based on speed
    float angle_diff = servo_states[i].target_angle - servo_states[i].current_angle;
    float max_step = servo_states[i].speed * 180.0 * dt; // degrees per second
    
    if (abs(angle_diff) > 0.01) { // Still moving
      servo_states[i].moving = true;
      
      if (abs(angle_diff) <= max_step) {
        servo_states[i].current_angle = servo_states[i].target_angle;
        servo_states[i].moving = false;
      } else {
        servo_states[i].current_angle += (angle_diff > 0) ? max_step : -max_step;
      }
      
      // Convert radians to servo degrees (0-180)
      int servo_angle = constrain(90 + servo_states[i].current_angle * 180.0 / PI, 0, 180);
      servos[i].write(servo_angle);
    } else {
      servo_states[i].moving = false;
    }
  }
}

void sendStatusReport() {
  // Send basic status message
  doc.clear();
  doc["type"] = "status";
  doc["timestamp"] = millis();
  doc["active_servos"] = countActiveServos();
  doc["battery_voltage"] = analogRead(A0) * 5.0 / 1023.0; // Example voltage reading
  
  serializeJson(doc, Serial);
  Serial.println();
}

void sendDetailedStatus() {
  // Send detailed servo states
  for (int i = 0; i < NUM_SERVOS; i++) {
    doc.clear();
    doc["type"] = "servo_state";
    doc["servo_id"] = servo_states[i].id;
    doc["angle"] = servo_states[i].current_angle;
    doc["speed"] = servo_states[i].speed;
    doc["moving"] = servo_states[i].moving;
    doc["enabled"] = servo_states[i].enabled;
    doc["error"] = false; // Placeholder for error detection
    doc["temp"] = 25; // Placeholder temperature
    
    serializeJson(doc, Serial);
    Serial.println();
    
    delay(5); // Small delay between messages
  }
}

void emergencyStop() {
  for (int i = 0; i < NUM_SERVOS; i++) {
    servo_states[i].enabled = false;
    servo_states[i].moving = false;
    servos[i].detach();
  }
  
  doc.clear();
  doc["type"] = "status";
  doc["message"] = "Emergency stop activated";
  serializeJson(doc, Serial);
  Serial.println();
}

int countActiveServos() {
  int count = 0;
  for (int i = 0; i < NUM_SERVOS; i++) {
    if (servo_states[i].enabled) count++;
  }
  return count;
}