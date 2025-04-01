#include <AFMotor.h>
#include <QTRSensors.h>

// Motor Connections
AF_DCMotor motor1(1, MOTOR12_1KHZ); // Left Motor on M1
AF_DCMotor motor2(4, MOTOR12_1KHZ); // Right Motor on M4

// PID and Speed Constants
#define KP 2
#define KD 5
#define Ki 0.01
#define M1_MIN_SPEED 100
#define M4_MIN_SPEED 100
#define M1_MAX_SPEED 150
#define M4_MAX_SPEED 150

// Sensor Configuration
#define NUM_SENSORS 8
#define TIMEOUT 2500
#define EMITTER_PIN 7

// Junction Detection Parameters
#define JUNCTION_THRESHOLD 600  // Adjust based on your sensor readings
#define JUNCTION_TIMEOUT 1000   // Time to wait at junction
// Pin Definitions
#define BUZZER_PIN 13
#define CALIBRATION_BUTTON A5

// Junction Types
enum JunctionType {
  NO_JUNCTION,
  T_JUNCTION,
  CROSS_JUNCTION,
  LEFT_JUNCTION,
  RIGHT_JUNCTION
};

// Global Variables
QTRSensors qtr;
unsigned int sensorValues[NUM_SENSORS];
int lastError = 0;
unsigned long lastJunctionTime = 0;
JunctionType currentJunction = NO_JUNCTION;

void setup() {
  Serial.begin(9600);
  pinMode(BUZZER_PIN, OUTPUT);
  pinMode(CALIBRATION_BUTTON, INPUT);

  // Initialize QTR Sensors
  qtr.setTypeAnalog();
  qtr.setSensorPins((const uint8_t[]){A0, A1, A2, A3, A4, 6, 3, 2}, NUM_SENSORS);
  qtr.setEmitterPin(EMITTER_PIN);

  // Calibration Process
  calibrateSensors();
}
void calibrateSensors() {
  Serial.println("Press Button to Calibrate...");
  while (analogRead(CALIBRATION_BUTTON) > 500);
  
  Serial.println("Calibrating...");
  for (int i = 0; i < 400; i++) {
    qtr.calibrate();
    delay(5);
  }
  Serial.println("Calibration Done!");
}

void loop() {
  unsigned int sensors[NUM_SENSORS];
  int position = qtr.readLineBlack(sensors);
  
  // Detect junction type
  JunctionType junction = detectJunction(sensors);
  
  // Handle junction
  if (junction != NO_JUNCTION) {
    handleJunction(junction, sensors);
  }
  else {
    // Normal line following
    followLine(sensors, position);
  }
}

JunctionType detectJunction(unsigned int sensors[]) {
  // Count sensors detecting the line
  int lineDetectedCount = 0;
  for (int i = 0; i < NUM_SENSORS; i++) {
    if (sensors[i] > JUNCTION_THRESHOLD) {
      lineDetectedCount++;
    }
  }
// Junction detection logic
  if (lineDetectedCount >= 6) {
    // Check left and right sensor groups
    int leftSensorsDetected = 0;
    int rightSensorsDetected = 0;
    
    for (int i = 0; i < 3; i++) {
      if (sensors[i] > JUNCTION_THRESHOLD) leftSensorsDetected++;
      if (sensors[NUM_SENSORS - 1 - i] > JUNCTION_THRESHOLD) rightSensorsDetected++;
    }

    // Determine junction type
    if (lineDetectedCount == 8) return CROSS_JUNCTION;
    if (leftSensorsDetected >= 3 && rightSensorsDetected < 3) return LEFT_JUNCTION;
    if (rightSensorsDetected >= 3 && leftSensorsDetected < 3) return RIGHT_JUNCTION;
    return T_JUNCTION;
  }
  
  return NO_JUNCTION;
}

void handleJunction(JunctionType junction, unsigned int sensors[]) {
  // Avoid multiple rapid junction detections
  if (millis() - lastJunctionTime < JUNCTION_TIMEOUT) return;
  
  // Buzz to indicate junction
  digitalWrite(BUZZER_PIN, HIGH);
  delay(100);
  digitalWrite(BUZZER_PIN, LOW);

  // Junction handling strategy
  switch(junction) {
    case CROSS_JUNCTION:
      // Go straight through cross junction
      setMotors(60, 60);
      delay(200);
      break;
    
    case T_JUNCTION:
 // Choose a default path (could be modified based on preference)
      setMotors(60, 40);  // Slight right turn
      delay(300);
      break;
    
    case LEFT_JUNCTION:
      // Turn left
      setMotors(40, 60);
      delay(300);
      break;
    
    case RIGHT_JUNCTION:
      // Turn right
      setMotors(60, 40);
      delay(200);
      break;
  }

  // Update last junction time
  lastJunctionTime = millis();
}

void followLine(unsigned int sensors[], int position) {
  int error = position - 3500;  // Center position for 8 sensors
  
  // Advanced PID Control
  int motorSpeed = KP * error + KD * (error - lastError);
  lastError = error;

  int leftMotorSpeed = constrain(M1_MIN_SPEED + motorSpeed, 0, M1_MAX_SPEED);
  int rightMotorSpeed = constrain(M4_MIN_SPEED - motorSpeed, 0, M4_MAX_SPEED);

  setMotors(leftMotorSpeed, rightMotorSpeed);
}

void setMotors(int motor1speed, int motor2speed) {
motor1.setSpeed(motor1speed);
  motor2.setSpeed(motor2speed);
  motor1.run(FORWARD);
  motor2.run(FORWARD);
}

void stopMotors() {
  motor1.setSpeed(0);
  motor2.setSpeed(0);
  motor1.run(RELEASE);
  motor2.run(RELEASE);
}
