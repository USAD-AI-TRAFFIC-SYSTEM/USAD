/*
  USAD - Traffic Light Diagnostic Test
  
  Turns ON all traffic light LEDs (pins 2-13) simultaneously.
  Use this to check which bulbs are broken or need replacement.
  
  Pin Configuration (same as traffic_controller.ino):
  - Lane 1 (North): Green=2, Yellow=3, Red=4
  - Lane 2 (South): Green=5, Yellow=6, Red=7
  - Lane 3 (East):  Green=8, Yellow=9, Red=10
  - Lane 4 (West):  Green=11, Yellow=12, Red=13
  
  Upload this sketch, and every LED should light up.
  Any LED that stays OFF is broken or has a wiring issue.
*/

const int FIRST_PIN = 2;
const int LAST_PIN = 13;

void setup() {
  Serial.begin(9600);
  
  // Set all traffic light pins as OUTPUT and turn them ON
  for (int pin = FIRST_PIN; pin <= LAST_PIN; pin++) {
    pinMode(pin, OUTPUT);
    digitalWrite(pin, HIGH);
  }
  
  Serial.println("=== USAD TRAFFIC LIGHT DIAGNOSTIC ===");
  Serial.println("All LEDs (pins 2-13) should be ON.");
  Serial.println("");
  Serial.println("If any LED is OFF, check:");
  Serial.println("  - Bulb/LED is not burned out");
  Serial.println("  - Wiring is secure");
  Serial.println("  - Resistor is connected");
  Serial.println("");
  Serial.println("Pin mapping:");
  Serial.println("  Pin 2  = Lane 1 (North) GREEN");
  Serial.println("  Pin 3  = Lane 1 (North) YELLOW");
  Serial.println("  Pin 4  = Lane 1 (North) RED");
  Serial.println("  Pin 5  = Lane 2 (South) GREEN");
  Serial.println("  Pin 6  = Lane 2 (South) YELLOW");
  Serial.println("  Pin 7  = Lane 2 (South) RED");
  Serial.println("  Pin 8  = Lane 3 (East)  GREEN");
  Serial.println("  Pin 9  = Lane 3 (East)  YELLOW");
  Serial.println("  Pin 10 = Lane 3 (East)  RED");
  Serial.println("  Pin 11 = Lane 4 (West)  GREEN");
  Serial.println("  Pin 12 = Lane 4 (West)  YELLOW");
  Serial.println("  Pin 13 = Lane 4 (West)  RED");
  Serial.println("");
  Serial.println("Upload traffic_controller.ino when done testing.");
}

void loop() {
  // Nothing to do — all lights stay on
}
