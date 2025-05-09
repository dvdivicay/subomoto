#include <QTRSensors.h>
#include <SparkFun_TB6612.h>

//COMMON PINS
#define PWMA 10
#define PWMB 11
#define STBY 12

// #define first_AIN1 2 //RIGHT
// #define first_BIN1 4 //LEFT
// #define first_AIN2 3 //RIGHT
// #define first_BIN2 5 //LEFT

//WHEELS
#define first_AIN1 6 //RIGHT
#define first_BIN1 8 //LEFT
#define first_AIN2 7 //RIGHT
#define first_BIN2 9 //LEFT
#define CTRL_QTR 2
#define SENSOR_SAMPLES 3

unsigned int sensorBuffer[6][SENSOR_SAMPLES];

const int offsetA = 1;
const int offsetB = -1;

Motor motor1 = Motor(first_AIN1, first_AIN2, PWMA, offsetA, STBY); // RIGHT
Motor motor2 = Motor(first_BIN1, first_BIN2, PWMB, offsetB, STBY); // LEFT

// Motor motorB1 = Motor(second_AIN1, second_AIN2, PWMA, offsetA, STBY); // RIGHT REAR
// Motor motorB2 = Motor(second_BIN1, second_BIN2, PWMB, offsetB, STBY); // LEFT REAR

QTRSensorsRC qtr((unsigned char[]){A2, A3, A4, A5, A6, A7}, 6, 1000, CTRL_QTR); // RC-type sensors
unsigned int sensorValues[6];

int P, D, I, previousError, PIDvalue, error;

// float Kp = 0.568;
// float Ki = 0.01;
// float Kd = 6.678;

// float Kp = 0.998;
// float Ki = 0;
// float Kd = 6;

// float Kp = 1.2; //1.03
// float Ki = 0.01;//0
// float Kd = 21; //13.1

float Kp = 1.3; //1.03
float Ki = 0;//0
float Kd = 7.3; //13.1

int position;
int lastError = 0;
int baseSpeed = 230;
int maxSpeed = 255;

int leftLineDetectedCount = 0;
int rightLineDetectedCount = 0;
int maxDetectedCount = 6;  // How many consecutive readings we need to trigger a sharp turn
int wobbleThreshold = 100; // A value above which we don't consider black (it’s not black)

void setup() {

  Serial.begin(9600);
  pinMode(STBY, OUTPUT);
  pinMode(CTRL_QTR, OUTPUT);
  digitalWrite(CTRL_QTR, HIGH);
  digitalWrite(STBY, HIGH);  

  delay(3000);

  calibrateSensors();

  delay(5000);

}

void loop() {
  
  position = qtr.readLine(sensorValues); // Value between 0 to 5000
  smoothSensorValues();
  error = (3000 - position) / 10;

  Serial.println(position);

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
  }


  PIDvalue = constrain(PIDvalue, -baseSpeed, baseSpeed);

  int leftSpeed = baseSpeed + PIDvalue;
  int rightSpeed = baseSpeed - PIDvalue;

  leftSpeed = constrain(leftSpeed, 0, maxSpeed);
  rightSpeed = constrain(rightSpeed, 0, maxSpeed);

  // Serial.print("PIDVALUE: ");
  // Serial.print(PIDvalue);
  // Serial.print("\t");
  // Serial.print("LeftSpeed: ");
  // Serial.print(leftSpeed);
  // Serial.print("\t");
  // Serial.print("RightSpeed: ");
  // Serial.print(rightSpeed);
  // Serial.print("\n");
  Serial.print("Position: ");
  Serial.print(position);
  Serial.print("\n");
  // if (sensorValues[0] <= wobbleThreshold && sensorValues[1] <= wobbleThreshold && sensorValues[2] <= wobbleThreshold) {
  //   leftLineDetectedCount++;
  //   if (leftLineDetectedCount >= maxDetectedCount) {
  //     // If we detect black line for `maxDetectedCount` consecutive times, make a sharp turn
  //     driveMotors(baseSpeed, 10); // drive to the right (sharp turn)
  //   }
  // } else {
  //   leftLineDetectedCount = 0;  // Reset the count if the sensor is no longer on the black line
  // }

  // // Check for rightmost sensor (sensorValues[5])
  // if (sensorValues[5] <= wobbleThreshold && sensorValues[4] <= wobbleThreshold && sensorValues[3] <= wobbleThreshold) {
  //   rightLineDetectedCount++;
  //   if (rightLineDetectedCount >= maxDetectedCount) {
  //     // If we detect black line for `maxDetectedCount` consecutive times, make a sharp turn
  //     driveMotors(10, baseSpeed); // drive to the left (sharp turn)
  //   }
  // } else {
  //   rightLineDetectedCount = 0;  // Reset the count if the sensor is no longer on the black line
  // }

  driveMotors(leftSpeed, rightSpeed);

}

void calibrateSensors() {
  Serial.println("Calibrating sensors...");
  for (int i = 0; i < 400; i++) {
    qtr.calibrate();
    driveMotors(-170, 170);
    delay(5);
  }
  driveMotors(0, 0);
  Serial.println("Calibration done.");
}

void smoothSensorValues() {
  for (int i = 0; i < 6; i++) {
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

void driveMotors(int leftSpeed, int rightSpeed) {
    motor2.drive(leftSpeed);
    motor1.drive(rightSpeed);
}
