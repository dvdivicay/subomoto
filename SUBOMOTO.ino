#include <QTRSensors.h>
#include <SparkFun_TB6612.h>
#include <Ultrasonic.h>

//speed variable
int maxspeed = 255;
int spinspeed = 250;
int skipSpeed = 30;
int leftSpeed;
int rightSpeed;

volatile unsigned long startTime = 0;  
volatile unsigned long endTime = 0;    
volatile long duration = 0;
volatile bool measuring = false;  

#define PWMA 10
#define PWMB 11
#define STBY 12

//REAR WHEELS
#define first_AIN1 6 //RIGHT
#define first_BIN1 8 //LEFT
#define first_AIN2 7 //RIGHT
#define first_BIN2 9 //LEFT

const int offsetA = 1;
const int offsetB = -1;

Ultrasonic ultrasonicL(2, 3);
Ultrasonic ultrasonicM(4, 5);
Ultrasonic ultrasonicR(A4, A5);

long distanceR, distanceL, distanceM;

// QTRSensorsRC qtr((unsigned char[]){A2, A3, A4}, 3, 2500);

Motor motor1 = Motor(first_AIN1, first_AIN2, PWMA, offsetA, STBY); // RIGHT FRONT
Motor motor2 = Motor(first_BIN1, first_BIN2, PWMB, offsetB, STBY); // LEFT FRONT

void setup() {

  Serial.begin(9600);
  pinMode(STBY, OUTPUT);
  // pinMode(CTRL_QTR, OUTPUT);
  // digitalWrite(CTRL_QTR, HIGH);
  digitalWrite(STBY, HIGH);  
  // delay(3000);
  // calibrateSensors();

  delay(4700); 
}

// void calibrateSensors() {
//   Serial.println("Calibrating sensors...");
//   for (int i = 0; i < 400; i++) {
//     qtr.calibrate();
//     driveMotors(-170, 170);
//     delay(5);
//   }
//   driveMotors(0, 0);
//   Serial.println("Calibration done.");
// }

void loop() {

  scanForOpponent();
  delay(50);
}

void scanForOpponent() {

  distanceM = ultrasonicM.read();
  distanceR = ultrasonicR.read();
  distanceL = ultrasonicL.read();

  Serial.print("\n");
  Serial.print("Leftsensor: ");
  Serial.print(distanceL);
  Serial.print("\t");
  Serial.print("Middle sensor: ");
  Serial.print(distanceM);
  Serial.print("\t");
  Serial.print("Rightsensor: ");
  Serial.print(distanceR);

  if (distanceM < 27) {
    motor1.drive(255);
    motor2.drive(255);
  }else if(distanceR < 27){
    motor1.drive(-150);
    motor2.drive(255);
    delay(550);
  }
  else if (distanceL < 27){
    motor1.drive(255);
    motor2.drive(-150);
    delay(550);
  }
  else{
    motor1.drive(0);
    motor2.drive(0);
  }
   
}

void moveForward() {
  driveMotors(255, 255);
}

void turnRight() {
  driveMotors(255, 0);
}

void turnLeft() {
  driveMotors(0, 255);
}

void moveBackward() { 
  driveMotors(-255, -255);
}

void stopMotors() {
  driveMotors(0,0);
}

// bool detectBorder() {
//   unsigned int sensorValues[3];

//   int reading = qtr.readLine(sensorValues);
  
//   Serial.print(reading);
//   for (int i = 0; i < 3; i++) {
//     if (sensorValues[i] > 400) {
//       return true;
//     }
//   }

//   return false;
// }

void driveMotors(int leftSpeed, int rightSpeed) {
    motor2.drive(leftSpeed);
    motor1.drive(rightSpeed);
}

