#include <Stepper.h>

const int stepsPerRevolution = 2048;  // number of steps per revolution for 28BYJ-48
const int rolePerMinute = 17;         // motor speed (0–17 RPM for 28BYJ-48)
const float revolutionsToMove = 0.63

;      // how many full turns to open/close

Stepper myStepper(stepsPerRevolution, 8, 10, 9, 11);

void setup() {
  myStepper.setSpeed(rolePerMinute);
  Serial.begin(9600);
  Serial.println("Type 'o' to open, 'c' to close.");
}

void loop() {
  if (Serial.available() > 0) {
    char inputVariable = Serial.read();

    if (inputVariable == 'o') {
      Serial.println("Opening...");
      // Move forward X full revolutions
      myStepper.step(stepsPerRevolution * revolutionsToMove);
      Serial.println("Open.");
    }

    if (inputVariable == 'c') {
      Serial.println("Closing...");
      // Move backward X full revolutions
      myStepper.step(-stepsPerRevolution * revolutionsToMove);
      Serial.println("Closed.");
    }
  }
}