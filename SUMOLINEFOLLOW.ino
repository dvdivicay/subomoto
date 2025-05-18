#include <QTRSensors.h>
#include <SparkFun_TB6612.h>

#define PWMA 10
#define PWMB 11
#define STBY 13

#define AIN1 6 //RIGHT
#define BIN1 8 //LEFT
#define AIN2 7 //RIGHT
#define BIN2 9 //LEFT

#define SENSOR_SAMPLES 3

#define TRIG_PIN 3 //COMMON NA TRIGGER
#define ECHO_LEFT 4 //ECHO 1
#define ECHO_MID 5 // ECHO 2
#define ECHO_RIGHT 2 //ECHO 3

#define MODE_PIN2 A1  
#define MODE_PIN1 A0 
#define RUSH_PIN 12

const int offsetA = 1;
const int offsetB = -1;

Motor motor1 = Motor(AIN1, AIN2, PWMA, offsetA, STBY); // RIGHT
Motor motor2 = Motor(BIN1, BIN2, PWMB, offsetB, STBY); // LEFT

QTRSensorsRC qtr((unsigned char[]){A2, A3, A4, A5}, 4, 1000, QTR_NO_EMITTER_PIN); // RC-type sensors
unsigned int sensorBuffer[4][SENSOR_SAMPLES];
unsigned int sensorValues[4];

float Kp = 3.899;
float Ki = 0.0;
float Kd = 5.988;

int position;
int lastError = 0;
int baseSpeed = 245;
int maxSpeed = 255;
bool allBlack = true;

int P, D, I, previousError, PIDvalue, error;

int leftLineDetectedCount = 0;
int rightLineDetectedCount = 0;
int maxDetectedCount = 6;  // PILA KA READINGS MAKA TRIGGER SA SHARP TURN
int wobbleThreshold = 400; // A value above which we don't consider black (it’s not black)

long distanceR, distanceL, distanceM;

unsigned long lastSeenOpponent = 0;
unsigned long backStartTime = 0;
bool isBackingUp = false;
int lastMode = -1;

void setup() {
  Serial.begin(9600);
  pinMode(STBY, OUTPUT);
  pinMode(TRIG_PIN, OUTPUT);
  digitalWrite(STBY, HIGH);

  pinMode(RUSH_PIN, INPUT_PULLUP);
  pinMode(MODE_PIN1, INPUT_PULLUP); 
  pinMode(MODE_PIN2, INPUT_PULLUP);  
}

bool calibrated = false;

void loop() {

  int m1 = digitalRead(MODE_PIN1);
  int m2 = digitalRead(MODE_PIN2);

  int mode = (m2 << 1) | m1;

  if (mode != lastMode) {
    waitForStartButton();  // Only block on change
    lastMode = mode;
  } 

  switch (mode) {
  case 0: // 00
      if (!calibrated) {
        calibrateSensors();
        calibrated = true;
      }
      driveMotors(0, 0);
    break;
  case 1: // 01
    if (lastMode != 1) {
      I = 0;
      lastError = 0;
      baseSpeed = 245;
    }
    lineFollowing();
    break;
  case 2: // 10
    // Sumo Mode
    sumoBot();
    break;
  case 3: // 11
    driveMotors(0, 0);
    break;
  }

}

void lineFollowing(){

  position = qtr.readLine(sensorValues); // Value between 0 to 5000
  smoothSensorValues();
  error = (1780 - position) / 15;

  for (int i = 0; i < 4; i++) {
    Serial.print(sensorValues[i]);
    Serial.print("\t");
    Serial.println("");
    if (sensorValues[i] > wobbleThreshold) {
      allBlack = false;
      break;
    }
  }

  if (allBlack) {
    driveMotors(0, 0); // or spin in place, etc.
    delay(500);
  }

  P = error;
  I += error;
  I = constrain(I, -900, 900); // Move this here
  D = error - lastError;

  D = constrain(error - lastError, -200, 200);

  lastError = error;

  PIDvalue = (Kp * P) + (Ki * I) + (Kd * D);

  if (abs(error) < 2) {
    PIDvalue = 0;
    I = 0;
    baseSpeed = min(baseSpeed + 5, maxSpeed);
  }

  PIDvalue = constrain(PIDvalue, -baseSpeed, baseSpeed);

  int leftSpeed = baseSpeed + PIDvalue;
  int rightSpeed = baseSpeed - PIDvalue;

  leftSpeed = constrain(leftSpeed, 0, maxSpeed);
  rightSpeed = constrain(rightSpeed, 0, maxSpeed);

  if (sensorValues[0] >= wobbleThreshold) {
      leftLineDetectedCount++;
      if (leftLineDetectedCount >= maxDetectedCount) {
          I = 0; // reset integral to avoid windup
          driveMotors(baseSpeed, 10); // pivot right
      }
  }
  else {
      leftLineDetectedCount = 0;
  }

  if (sensorValues[3] >= wobbleThreshold) {
      rightLineDetectedCount++;
      if (rightLineDetectedCount >= maxDetectedCount) {
          I = 0;
          driveMotors(10, baseSpeed); // pivot left
      }
  } 
  else {
      rightLineDetectedCount = 0;
  }

  driveMotors(leftSpeed, rightSpeed);

}

void sumoBot(){

  if (isBackingUp) {
    if (millis() - backStartTime < 400) {
      motor1.drive(-150);
      motor2.drive(-150);
      return;
    }else {
      motor1.drive(-150);
      motor2.drive(150);   
      delay(350);          
      isBackingUp = false;
    }
  }

  qtr.read(sensorValues);

  for (int i = 0; i < 4; i++) {
    if (sensorValues[i] < 200) {
      isBackingUp = true; // start backing up
      backStartTime = millis(); // record the time
      motor1.drive(-150);
      motor2.drive(-150);
      return; // skip rest
    }
  }

  distanceM = getDistance(ECHO_MID);
  delay(10);
  distanceR = getDistance(ECHO_RIGHT);
  delay(10);
  distanceL = getDistance(ECHO_LEFT);
  delay(10);

  if (distanceR > 0 && distanceM < 20) {
    lastSeenOpponent = millis();
    motor1.drive(255);
    motor2.drive(255);
  } else if (millis() - lastSeenOpponent < 700) {
    motor1.drive(255);
    motor2.drive(255);
  } else if (distanceR > 0 && distanceR < 20) {
    motor1.drive(-150);
    motor2.drive(255);
    delay(550);
  } else if (distanceL > 0 && distanceL < 20) {
    motor1.drive(255);
    motor2.drive(-150);
    delay(550);
  } else {
    motor1.drive(0);
    motor2.drive(0);
  }
}

long getDistance(int echoPin) {

  pinMode(echoPin, INPUT);
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);
  
  long duration = pulseIn(echoPin, HIGH, 30000); // 30ms timeout

  // Return -1 if nothing was sensed
  if (duration == 0) return -1;

  long distance = duration * 0.034 / 2;

  return (distance >= 1 && distance <= 100) ? distance : -1;
}

void driveMotors(int leftSpeed, int rightSpeed) {
    motor2.drive(leftSpeed);
    motor1.drive(rightSpeed);
}

void calibrateSensors() {
  for (int i = 0; i < 400; i++) {
    qtr.calibrate();
    driveMotors(-170, 170);
    delay(5);
  }
  driveMotors(0, 0);
}

void smoothSensorValues() {
  for (int i = 0; i < 4; i++) {
    // Shift old values
    for (int j = SENSOR_SAMPLES - 1; j > 0; j--) {
      sensorBuffer[i][j] = sensorBuffer[i][j - 1];
    }
    sensorBuffer[i][0] = sensorValues[i]; // Insert new reading

    // Average
    int sum = 0;
    for (int j = 0; j < SENSOR_SAMPLES; j++) {
      sum += sensorBuffer[i][j];
    }
    sensorValues[i] = sum / SENSOR_SAMPLES;
  }
}

void waitForStartButton() {
  while (digitalRead(RUSH_PIN) == HIGH) {
    delay(10);
  }
  delay(300); // Simple debounce
}