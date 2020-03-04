// Define stepper motor connections and steps per revolution:
#define dirPinRight 3
#define stepPinRight 2
#define dirPinLeft 6
#define stepPinLeft 5
#define stepsPerRevolution 1675
void setup() {
  // Declare pins as output:
  pinMode(stepPinRight, OUTPUT);
  pinMode(dirPinRight, OUTPUT);
  pinMode(stepPinLeft, OUTPUT);
  pinMode(dirPinLeft, OUTPUT);
}
void loop() {
  // Set the spinning direction clockwise:
  digitalWrite(dirPinRight, HIGH);
  digitalWrite(dirPinLeft, HIGH);
  // Spin the stepper motor 1 revolution slowly:
  for (int i = 0; i < stepsPerRevolution; i++) {
    // These four lines result in 1 step:
    digitalWrite(stepPinRight, HIGH);
    digitalWrite(stepPinLeft, HIGH);
    delayMicroseconds(500);
    digitalWrite(stepPinRight, LOW);
    digitalWrite(stepPinLeft, LOW);
    delayMicroseconds(500);
   
  }

  // Set the spinning direction counterclockwise:
  digitalWrite(dirPinRight, LOW);
  digitalWrite(dirPinLeft, LOW);
  // Spin the stepper motor 1 revolution quickly:
  for (int i = 0; i < stepsPerRevolution; i++) {
    // These four lines result in 1 step:
    digitalWrite(stepPinRight, HIGH);
    digitalWrite(stepPinLeft, HIGH);
    delayMicroseconds(500);
    digitalWrite(stepPinRight, LOW);
    digitalWrite(stepPinLeft, LOW);
    delayMicroseconds(500);
  }
  
  
  /*
  // Set the spinning direction clockwise:
  digitalWrite(dirPinRight, HIGH);
  digitalWrite(dirPinLeft, HIGH);
  // Spin the stepper motor 5 revolutions fast:
  for (int i = 0; i < 5 * stepsPerRevolution; i++) {
    // These four lines result in 1 step:
    digitalWrite(stepPinRight, HIGH);
    digitalWrite(stepPinLeft, HIGH);
    delayMicroseconds(500);
    digitalWrite(stepPinRight, LOW);
    digitalWrite(stepPinLeft, LOW);
    delayMicroseconds(500);
  }
  delay(1000);
  // Set the spinning direction counterclockwise:
  digitalWrite(dirPinRight, LOW);
  digitalWrite(dirPinLeft, LOW);
  //Spin the stepper motor 5 revolutions fast:
  for (int i = 0; i < 5 * stepsPerRevolution; i++) {
    // These four lines result in 1 step:
    digitalWrite(stepPinRight, HIGH);
    digitalWrite(stepPinLeft, HIGH);
    delayMicroseconds(500);
    digitalWrite(stepPinRight, LOW);
    digitalWrite(stepPinLeft, LOW);
    delayMicroseconds(500);
  }
  delay(1000);
  */
}
