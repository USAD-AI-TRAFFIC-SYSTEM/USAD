/*
  USAD - Urban Smart Adaptive Dispatcher
  Traffic Signal Controller
 
  Controls 4-lane traffic lights with AI model integration
  - Green: 25 seconds
  - Yellow: 4 seconds
  - Red: automatic (when other lanes are active)
 
  Pin Configuration:
  - Lane 1 (North): Green=2, Yellow=3, Red=4
  - Lane 2 (South): Green=5, Yellow=6, Red=7
  - Lane 3 (East): Green=8, Yellow=9, Red=10
  - Lane 4 (West): Green=11, Yellow=12, Red=13
 
  Serial Commands (9600 baud, COM4):
  - LANE1, LANE2, LANE3, LANE4: Activate specific lane
  - AUTO: Return to automatic cycling mode
*/


// ==================== PIN CONFIGURATION ====================
// Lane 1 (North)
const int LANE1_GREEN = 2;
const int LANE1_YELLOW = 3;
const int LANE1_RED = 4;


// Lane 2 (South)
const int LANE2_GREEN = 5;
const int LANE2_YELLOW = 6;
const int LANE2_RED = 7;


// Lane 3 (East)
const int LANE3_GREEN = 8;
const int LANE3_YELLOW = 9;
const int LANE3_RED = 10;


// Lane 4 (West)
const int LANE4_GREEN = 11;
const int LANE4_YELLOW = 12;
const int LANE4_RED = 13;


// ==================== TIMING CONSTANTS ====================
const unsigned long GREEN_TIME = 25000;  // 25 seconds
const unsigned long YELLOW_TIME = 4000;  // 4 seconds


// ==================== STATE VARIABLES ====================
int current_lane = 0;           // 0=Lane1, 1=Lane2, 2=Lane3, 3=Lane4
int current_phase = 0;          // 0=green, 1=yellow, 2=transitioning
unsigned long phase_start_time = 0;
bool auto_mode = true;          // true=automatic cycling, false=model control
int model_commanded_lane = -1;  // Lane commanded by model


// Pin arrays for easier control
int green_pins[] = {LANE1_GREEN, LANE2_GREEN, LANE3_GREEN, LANE4_GREEN};
int yellow_pins[] = {LANE1_YELLOW, LANE2_YELLOW, LANE3_YELLOW, LANE4_YELLOW};
int red_pins[] = {LANE1_RED, LANE2_RED, LANE3_RED, LANE4_RED};


// ==================== SETUP ====================
void setup() {
  // Initialize Serial Communication (COM4)
  Serial.begin(9600);
 
  // Initialize all traffic light pins as OUTPUT
  for (int i = 0; i < 4; i++) {
    pinMode(green_pins[i], OUTPUT);
    pinMode(yellow_pins[i], OUTPUT);
    pinMode(red_pins[i], OUTPUT);
  }
 
  // Turn off all lights initially
  all_lights_off();
 
  // Initialize timing
  phase_start_time = millis();
  current_phase = 0;
  current_lane = 0;
 
  // Display initial state (Lane 1 green, others red)
  update_display();
 
  // Print startup message
  Serial.println("USAD Traffic Controller Ready");
  Serial.println("Commands: LANE1, LANE2, LANE3, LANE4, AUTO");
  Serial.println("Auto mode: ON");
}


// ==================== MAIN LOOP ====================
void loop() {
  // Check for serial commands from Python model
  if (Serial.available() > 0) {
    process_command();
  }
 
  // Update traffic signal state
  update_traffic_signal();
}


// ==================== COMMAND PROCESSING ====================
void process_command() {
  String command = Serial.readStringUntil('\n');
  command.trim();
  command.toUpperCase();
 
  if (command == "LANE1") {
    activate_lane(0);
    Serial.println("Lane 1 activated by model");
  }
  else if (command == "LANE2") {
    activate_lane(1);
    Serial.println("Lane 2 activated by model");
  }
  else if (command == "LANE3") {
    activate_lane(2);
    Serial.println("Lane 3 activated by model");
  }
  else if (command == "LANE4") {
    activate_lane(3);
    Serial.println("Lane 4 activated by model");
  }
  else if (command == "AUTO") {
    auto_mode = true;
    model_commanded_lane = -1;
    Serial.println("Auto mode activated");
  }
  else {
    Serial.print("Unknown command: ");
    Serial.println(command);
  }
}


// ==================== LANE ACTIVATION ====================
void activate_lane(int lane_number) {
  auto_mode = false;
  model_commanded_lane = lane_number;
  current_lane = lane_number;
  current_phase = 0;  // Start with green
  phase_start_time = millis();
  update_display();
}


// ==================== TRAFFIC SIGNAL UPDATE ====================
void update_traffic_signal() {
  unsigned long elapsed = millis() - phase_start_time;
 
  if (current_phase == 0) {
    // GREEN phase
    if (elapsed >= GREEN_TIME) {
      // Switch to YELLOW
      current_phase = 1;
      phase_start_time = millis();
      update_display();
    }
  }
  else if (current_phase == 1) {
    // YELLOW phase
    if (elapsed >= YELLOW_TIME) {
      // Yellow done, transition to next lane
      current_phase = 2;
     
      if (auto_mode) {
        // Auto cycling: move to next lane
        current_lane = (current_lane + 1) % 4;
      } else {
        // Model control: stay on commanded lane or wait for new command
        if (model_commanded_lane >= 0) {
          current_lane = model_commanded_lane;
        }
        // If no new command, keep cycling the same lane
      }
     
      // Start green phase for new/same lane
      current_phase = 0;
      phase_start_time = millis();
      update_display();
    }
  }
}


// ==================== DISPLAY UPDATE ====================
void update_display() {
  // Turn off all lights
  all_lights_off();
 
  // Set active lane based on current phase
  if (current_phase == 0) {
    // GREEN for active lane
    digitalWrite(green_pins[current_lane], HIGH);
  }
  else if (current_phase == 1) {
    // YELLOW for active lane
    digitalWrite(yellow_pins[current_lane], HIGH);
  }
 
  // RED for all other lanes
  for (int i = 0; i < 4; i++) {
    if (i != current_lane) {
      digitalWrite(red_pins[i], HIGH);
    }
  }
}


// ==================== UTILITY FUNCTIONS ====================
void all_lights_off() {
  for (int i = 0; i < 4; i++) {
    digitalWrite(red_pins[i], LOW);
    digitalWrite(yellow_pins[i], LOW);
    digitalWrite(green_pins[i], LOW);
  }
}



