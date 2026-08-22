/*
  USAD - Urban Smart Adaptive Dispatcher
  Traffic Signal Controller (Slave Mode)
 
  The Arduino acts as a SLAVE — it only changes lights when commanded
  by the Python program via serial. No internal timing or auto-cycling.
  Python is the single source of truth for signal state.
  
  RULE: Only ONE lane can be green/yellow at a time. All others are ALWAYS red.
 
  Pin Configuration:
  - Lane 1 (North): Green=2, Yellow=3, Red=4
  - Lane 2 (South): Green=5, Yellow=6, Red=7
  - Lane 3 (East): Green=8, Yellow=9, Red=10
  - Lane 4 (West): Green=11, Yellow=12, Red=13
 
  Serial Commands (9600 baud):
  - LANE1_GREEN, LANE2_GREEN, LANE3_GREEN, LANE4_GREEN
  - LANE1_YELLOW, LANE2_YELLOW, LANE3_YELLOW, LANE4_YELLOW
  - ALL_RED
  - PING → responds "PONG"
  - LANE1, LANE2, LANE3, LANE4 (legacy, same as GREEN)
*/

// ==================== PIN CONFIGURATION ====================
const int GREEN_PINS[]  = {2, 5, 8, 11};
const int YELLOW_PINS[] = {3, 6, 9, 12};
const int RED_PINS[]    = {4, 7, 10, 13};

// Current state tracking
int active_lane = -1;    // -1 = none, 0-3 = lane index
int active_phase = 0;    // 0 = RED, 1 = GREEN, 2 = YELLOW

// Serial buffer
String inputBuffer = "";

// ==================== SETUP ====================
void setup() {
  Serial.begin(9600);
  Serial.setTimeout(100);
  
  for (int i = 0; i < 4; i++) {
    pinMode(GREEN_PINS[i], OUTPUT);
    pinMode(YELLOW_PINS[i], OUTPUT);
    pinMode(RED_PINS[i], OUTPUT);
  }
  
  // Safe state: all red
  applyState();
  
  Serial.println("USAD Traffic Controller Ready");
}

// ==================== MAIN LOOP ====================
void loop() {
  // Read serial character by character to avoid buffer issues
  while (Serial.available() > 0) {
    char c = Serial.read();
    if (c == '\n' || c == '\r') {
      if (inputBuffer.length() > 0) {
        inputBuffer.trim();
        inputBuffer.toUpperCase();
        handleCommand(inputBuffer);
        inputBuffer = "";
      }
    } else {
      inputBuffer += c;
      // Prevent buffer overflow
      if (inputBuffer.length() > 20) {
        inputBuffer = "";
      }
    }
  }
}

// ==================== COMMAND HANDLER ====================
void handleCommand(String cmd) {
  // Parse lane number and phase from command
  if (cmd == "LANE1_GREEN" || cmd == "LANE1") {
    active_lane = 0;
    active_phase = 1;
  }
  else if (cmd == "LANE2_GREEN" || cmd == "LANE2") {
    active_lane = 1;
    active_phase = 1;
  }
  else if (cmd == "LANE3_GREEN" || cmd == "LANE3") {
    active_lane = 2;
    active_phase = 1;
  }
  else if (cmd == "LANE4_GREEN" || cmd == "LANE4") {
    active_lane = 3;
    active_phase = 1;
  }
  else if (cmd == "LANE1_YELLOW") {
    active_lane = 0;
    active_phase = 2;
  }
  else if (cmd == "LANE2_YELLOW") {
    active_lane = 1;
    active_phase = 2;
  }
  else if (cmd == "LANE3_YELLOW") {
    active_lane = 2;
    active_phase = 2;
  }
  else if (cmd == "LANE4_YELLOW") {
    active_lane = 3;
    active_phase = 2;
  }
  else if (cmd == "ALL_RED") {
    active_lane = -1;
    active_phase = 0;
  }
  else if (cmd == "PING") {
    Serial.println("PONG");
    return;
  }
  else if (cmd == "AUTO") {
    Serial.println("OK");
    return;
  }
  else {
    Serial.print("ERR:");
    Serial.println(cmd);
    return;
  }
  
  // Apply the new state to hardware
  applyState();
  Serial.println("OK");
}

// ==================== APPLY STATE TO HARDWARE ====================
// This is the ONLY function that touches the pins.
// It guarantees: at most ONE lane is green or yellow, all others are red.
void applyState() {
  // Step 1: Turn EVERYTHING off first
  for (int i = 0; i < 4; i++) {
    digitalWrite(GREEN_PINS[i], LOW);
    digitalWrite(YELLOW_PINS[i], LOW);
    digitalWrite(RED_PINS[i], LOW);
  }
  
  // Step 2: Set all lanes to RED
  for (int i = 0; i < 4; i++) {
    digitalWrite(RED_PINS[i], HIGH);
  }
  
  // Step 3: If a lane is active, override its red with green or yellow
  if (active_lane >= 0 && active_lane < 4) {
    // Turn OFF this lane's red first
    digitalWrite(RED_PINS[active_lane], LOW);
    
    if (active_phase == 1) {
      // GREEN
      digitalWrite(GREEN_PINS[active_lane], HIGH);
    }
    else if (active_phase == 2) {
      // YELLOW
      digitalWrite(YELLOW_PINS[active_lane], HIGH);
    }
  }
}
