/*
  USAD - Urban Smart Adaptive Dispatcher
  Traffic Signal Controller for Arduino
  
  This sketch controls traffic lights based on commands received from the main Python application
  via serial communication.
  
  Hardware Requirements:
  - Arduino Uno/Mega
  - 4 Traffic light sets (8-12 digital pins for LEDs or relays)
  - 1 Emergency light (1-2 digital pins)
  
  Serial Protocol:
  Command format: DIRECTION,GREEN_TIME,YELLOW_TIME,RED_TIME
  Example: N,30,5,40   (North: 30s green, 5s yellow, 40s red)
*/

#include <EEPROM.h>

// ==================== PIN CONFIGURATION ====================
// Traffic Lights: North, South, East, West
const int NORTH_RED = 2;
const int NORTH_YELLOW = 3;
const int NORTH_GREEN = 4;

const int SOUTH_RED = 5;
const int SOUTH_YELLOW = 6;
const int SOUTH_GREEN = 7;

const int EAST_RED = 8;
const int EAST_YELLOW = 9;
const int EAST_GREEN = 10;

const int WEST_RED = 11;
const int WEST_YELLOW = 12;
const int WEST_GREEN = 13;

// Emergency/Alert Light
const int EMERGENCY_RED = A0;

// Buzzer (optional)
const int BUZZER = A1;

// ==================== DATA STRUCTURES ====================
struct SignalTiming {
  int green_time;   // in seconds
  int yellow_time;  // in seconds
  int red_time;     // in seconds
};

struct TrafficState {
  char direction;   // 'N', 'S', 'E', 'W'
  SignalTiming timing;
  unsigned long start_time;
  int current_phase; // 0=green, 1=yellow, 2=red
};

// ==================== GLOBAL VARIABLES ====================
TrafficState traffic_state = {'N', {30, 5, 40}, 0, 0};
bool emergency_mode = false;
unsigned long last_serial_time = 0;
const unsigned long SERIAL_TIMEOUT = 5000; // 5 seconds

// Pin arrays for easier control
int red_pins[] = {NORTH_RED, SOUTH_RED, EAST_RED, WEST_RED};
int yellow_pins[] = {NORTH_YELLOW, SOUTH_YELLOW, EAST_YELLOW, WEST_YELLOW};
int green_pins[] = {NORTH_GREEN, SOUTH_GREEN, EAST_GREEN, WEST_GREEN};

// ==================== SETUP ====================
void setup() {
  // Initialize Serial Communication
  Serial.begin(9600);
  
  // Initialize all traffic light pins as OUTPUT
  for (int i = 0; i < 4; i++) {
    pinMode(red_pins[i], OUTPUT);
    pinMode(yellow_pins[i], OUTPUT);
    pinMode(green_pins[i], OUTPUT);
  }
  
  // Initialize emergency and buzzer pins
  pinMode(EMERGENCY_RED, OUTPUT);
  pinMode(BUZZER, OUTPUT);
  
  // Turn off all lights initially
  all_lights_off();
  
  // Print startup message
  Serial.println("USAD Traffic Signal Controller Ready");
  Serial.println("Format: DIRECTION,GREEN_TIME,YELLOW_TIME,RED_TIME");
  Serial.println("Example: N,30,5,40");
}

// ==================== MAIN LOOP ====================
void loop() {
  // Check for incoming serial data
  if (Serial.available() > 0) {
    process_serial_command();
    last_serial_time = millis();
  }
  
  // Check for serial timeout (failsafe to all red)
  if (millis() - last_serial_time > SERIAL_TIMEOUT) {
    if (!emergency_mode) {
      enter_failsafe_mode();
    }
  }
  
  // Update traffic signal state
  if (emergency_mode) {
    update_emergency_signal();
  } else {
    update_traffic_signal();
  }
}

// ==================== SERIAL COMMUNICATION ====================
void process_serial_command() {
  String command = Serial.readStringUntil('\n');
  command.trim();
  
  // Check for emergency signal
  if (command == "EMERGENCY") {
    emergency_mode = true;
    Serial.println("EMERGENCY MODE ACTIVATED");
    return;
  }
  
  // Check for emergency clear
  if (command == "CLEAR_EMERGENCY") {
    emergency_mode = false;
    Serial.println("Emergency mode cleared");
    return;
  }
  
  // Check for status request
  if (command == "STATUS") {
    send_status();
    return;
  }
  
  // Parse normal signal command
  parse_signal_command(command);
}

void parse_signal_command(String command) {
  /*
    Format: DIRECTION,GREEN_TIME,YELLOW_TIME,RED_TIME
    Example: N,30,5,40
  */
  
  int comma_count = 0;
  String parts[4] = {"", "", "", ""};
  int part_index = 0;
  
  for (int i = 0; i < command.length(); i++) {
    if (command[i] == ',') {
      part_index++;
      if (part_index > 3) break;
    } else {
      parts[part_index] += command[i];
    }
  }
  
  // Validate format
  if (part_index != 3) {
    Serial.println("ERROR: Invalid command format");
    return;
  }
  
  char direction = parts[0][0];
  int green_time = parts[1].toInt();
  int yellow_time = parts[2].toInt();
  int red_time = parts[3].toInt();
  
  // Validate values
  if (green_time <= 0 || yellow_time <= 0 || red_time <= 0) {
    Serial.println("ERROR: Invalid timing values (must be positive)");
    return;
  }
  
  if (green_time > 120 || yellow_time > 30 || red_time > 120) {
    Serial.println("ERROR: Timing values exceed limits");
    return;
  }
  
  // Update state
  traffic_state.direction = direction;
  traffic_state.timing.green_time = green_time;
  traffic_state.timing.yellow_time = yellow_time;
  traffic_state.timing.red_time = red_time;
  traffic_state.start_time = millis();
  traffic_state.current_phase = 0; // Start with green
  
  // Acknowledge
  Serial.print("SIGNAL_SET: ");
  Serial.print(direction);
  Serial.print(" - G:");
  Serial.print(green_time);
  Serial.print("s Y:");
  Serial.print(yellow_time);
  Serial.print("s R:");
  Serial.println(red_time);
}

void send_status() {
  Serial.print("STATUS: Direction=");
  Serial.print(traffic_state.direction);
  Serial.print(" Phase=");
  Serial.print(traffic_state.current_phase);
  Serial.print(" Emergency=");
  Serial.println(emergency_mode ? "ON" : "OFF");
}

// ==================== TRAFFIC SIGNAL CONTROL ====================
void update_traffic_signal() {
  unsigned long elapsed = millis() - traffic_state.start_time;
  unsigned long green_ms = traffic_state.timing.green_time * 1000;
  unsigned long yellow_ms = traffic_state.timing.yellow_time * 1000;
  unsigned long red_ms = traffic_state.timing.red_time * 1000;
  
  int new_phase;
  
  if (elapsed < green_ms) {
    new_phase = 0; // GREEN
  } else if (elapsed < (green_ms + yellow_ms)) {
    new_phase = 1; // YELLOW
  } else if (elapsed < (green_ms + yellow_ms + red_ms)) {
    new_phase = 2; // RED
  } else {
    // Cycle complete, reset
    traffic_state.start_time = millis();
    new_phase = 0;
  }
  
  // Update only if phase changed
  if (new_phase != traffic_state.current_phase) {
    traffic_state.current_phase = new_phase;
    update_signal_display();
  }
}

void update_signal_display() {
  all_lights_off();
  
  int direction_index;
  switch (traffic_state.direction) {
    case 'N': direction_index = 0; break;
    case 'S': direction_index = 1; break;
    case 'E': direction_index = 2; break;
    case 'W': direction_index = 3; break;
    default: return;
  }
  
  // Turn on appropriate light for active direction
  switch (traffic_state.current_phase) {
    case 0: // GREEN
      digitalWrite(green_pins[direction_index], HIGH);
      break;
    case 1: // YELLOW
      digitalWrite(yellow_pins[direction_index], HIGH);
      beep_buzzer(1);
      break;
    case 2: // RED
      digitalWrite(red_pins[direction_index], HIGH);
      break;
  }
  
  // Turn on red for all other directions
  for (int i = 0; i < 4; i++) {
    if (i != direction_index) {
      digitalWrite(red_pins[i], HIGH);
    }
  }
}

// ==================== EMERGENCY MODE ====================
void update_emergency_signal() {
  // Blink red light rapidly and sound buzzer
  static unsigned long last_blink = 0;
  static bool emergency_light_on = false;
  
  if (millis() - last_blink > 300) { // 300ms blink interval
    emergency_light_on = !emergency_light_on;
    digitalWrite(EMERGENCY_RED, emergency_light_on ? HIGH : LOW);
    
    if (emergency_light_on) {
      beep_buzzer(2);
    }
    
    last_blink = millis();
  }
  
  // All other lights off
  all_lights_off();
}

void enter_failsafe_mode() {
  // All lights red (safest state)
  all_lights_off();
  for (int i = 0; i < 4; i++) {
    digitalWrite(red_pins[i], HIGH);
  }
  
  // Slow blink emergency light
  static unsigned long last_blink = 0;
  static bool failsafe_light_on = false;
  
  if (millis() - last_blink > 1000) {
    failsafe_light_on = !failsafe_light_on;
    digitalWrite(EMERGENCY_RED, failsafe_light_on ? HIGH : LOW);
    last_blink = millis();
  }
}

// ==================== UTILITY FUNCTIONS ====================
void all_lights_off() {
  for (int i = 0; i < 4; i++) {
    digitalWrite(red_pins[i], LOW);
    digitalWrite(yellow_pins[i], LOW);
    digitalWrite(green_pins[i], LOW);
  }
  digitalWrite(EMERGENCY_RED, LOW);
}

void beep_buzzer(int beeps) {
  for (int i = 0; i < beeps; i++) {
    digitalWrite(BUZZER, HIGH);
    delay(100);
    digitalWrite(BUZZER, LOW);
    delay(100);
  }
}

// ==================== DEBUG FUNCTIONS ====================
void test_all_lights() {
  // Test sequence: all greens, then yellows, then reds
  for (int i = 0; i < 4; i++) {
    digitalWrite(green_pins[i], HIGH);
  }
  delay(2000);
  
  for (int i = 0; i < 4; i++) {
    digitalWrite(green_pins[i], LOW);
    digitalWrite(yellow_pins[i], HIGH);
  }
  delay(2000);
  
  for (int i = 0; i < 4; i++) {
    digitalWrite(yellow_pins[i], LOW);
    digitalWrite(red_pins[i], HIGH);
  }
  delay(2000);
  
  all_lights_off();
}
