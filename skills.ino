#include <Servo.h>

// Phototransistor object
struct Phototransistor {
  uint8_t pin;
  int val = analogRead(pin);
  
  Phototransistor(): pin(A0), val(0) {}
  Phototransistor(uint8_t p, int x): pin(p), val(x) {}
};

// DCMotor object
struct DCMotorPair {
  uint8_t pinL;
  uint8_t pinR;

  DCMotorPair(): pinL(9), pinR(10) {}
  DCMotorPair(uint8_t p1, uint8_t p2): pinL(p1), pinR(p2) {}

  void makeTurn(double angleRadians, int timeForHalfRotation) {
    double time = abs(angleRadians) / PI * timeForHalfRotation;
    if (angleRadians >= 0) {
      moveLeft(255);
      delay(time);
      stop();
    }
    else {
      moveRight(255);
      delay(time);
      stop();
    }
  }

  void stop() {
    analogWrite(pinL, 0);

    analogWrite(pinR, 0);
  }

  void moveForward(int speed) {
    analogWrite(pinL, speed);
    analogWrite(pinR, speed);
  }

  void moveLeft(int speed) {
    analogWrite(pinL, 0);
    analogWrite(pinR, speed);
  }

  void moveRight(int speed) {
    analogWrite(pinL, speed);
    analogWrite(pinR, 0);
  }
};

// models a sweeping motion (from 0 to 180) when called repeatedly
// writes the angle then changes the angle (and direction if applicable)
void sweep(Servo& servo, int& angle, bool& isIncrementing) {
  servo.write(angle);

  if (isIncrementing) {
    angle++;
    
    if (angle >= 180) {
    isIncrementing = false;
  }
  } 
  else {
    angle--;

    if (angle <= 0) {
      isIncrementing = true;
    }
  }
}

double cot(double theta) {
  return cos(theta) / sin(theta);
}  

// Global variables initialisation
Phototransistor PhotoL(A0, 0);
Phototransistor PhotoR(A1, 1);
DCMotorPair Motors(10, 9);
Servo servoL;
Servo servoR;

const int timeForHalfRotation = 250;
const int distanceBetweenPhototransistorCM = 6; // (cm)
int angle = 0;
double maxAngleL = 0;
double maxAngleR = 0;
int maxPhotoValL = 0;
int maxPhotoValR = 0;
bool isIncrementing = true;

void setup() {
  //Initialise pin modes and serial
  pinMode(A0, INPUT);
  pinMode(A1, INPUT);

  pinMode(9,OUTPUT);
  pinMode(10,OUTPUT);

  servoL.attach(10);
  servoR.attach(9);

  Serial.begin(9600);
}

void loop() {
  // STEP 1: work out the position of the light source by sweeping (standing still)
  for (int i = 0; i < 360; i++) {
    //Read both input voltages
    PhotoL.val = analogRead(A0);
    PhotoR.val = analogRead(A1);

    //Write ADC values to serial
    Serial.print(PhotoL.val);
    Serial.write(" ");
    Serial.println(PhotoR.val);

    sweep(servoL, angle, isIncrementing);
    sweep(servoR, angle, isIncrementing);

    if (PhotoL.val > maxPhotoValL) {
      maxPhotoValL = PhotoL.val;
      maxAngleL = angle;
    }
    if (PhotoR.val > maxPhotoValR) {
      maxPhotoValR = PhotoR.val;
      maxAngleR = angle;
    }

    //Pause before next calculation
    delay(100);
  }

  // STEP 2: WORK OUT OPTIMAL ANGLE AND MOVE FORWARD
  double maxAngleLRadians = maxAngleL * PI / 180;
  double maxAngleRRadians = maxAngleR * PI/ 180;
  double distance = 6/(cot(maxAngleLRadians) + cot(maxAngleRRadians));
  double optimalAngle = atan2(distance / abs(3 - (distance / atan2(maxAngleLRadians))));
  double deviationFromOptimalAngle = PI/2 - optimalAngle;

  DCMotorPair.makeTurn(deviationFromOptimalAngle, timeForHalfRotation);
  angle = 90;
  PhotoL.val = analogRead(A0);
  PhotoR.val = analogRead(A1);
  int lightLevel = PhotoL.val + PhotoR.val;
  int nextLightLevel = lightLevel;

  while (nextLightLevel >= lightLevel) {
    DCMotorPair.moveForward(255);
    delay(100);
    nextLightLevel = lightLevel;

    PhotoL.val = analogRead(A0);
    PhotoR.val = analogRead(A1);
    lightLevel = PhotoL.val + PhotoR.val;
  }
  DCMotorPair.stop();
  
}