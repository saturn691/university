#include <Servo.h>

// Phototransistor object
struct Phototransistor {
  const uint8_t pin;
  int val = analogRead(pin);
  
  Phototransistor(): pin(A0), val(0) {}
  Phototransistor(uint8_t p, int x): pin(p), val(x) {}
};

// DCMotor object
struct DCMotorPair {
  const uint8_t pinL;
  const uint8_t pinR;

  DCMotorPair(): pinL(9), pinR(10) {}
  DCMotorPair(uint8_t p1, uint8_t p2): pinL(p1), pinR(p2) {}
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

// Global variables initialisation
Phototransistor PhotoL(A0, 0);
Phototransistor PhotoR(A1, 1);
DCMotorPair Motors(5, 6);
Servo servoL;
Servo servoR;

const double SENSITIVITY = 0.6;
const int timeForHalfRotation = 250;
const int distanceBetweenPhototransistorCM = 6; // (cm)
int angle = 0;
double maxAngleL = 0;
double maxAngleR = 0;
int maxPhotoValL = 0;
int maxPhotoValR = 0;
bool isIncrementing = true;

int adjust(double x) {
  return int(pow(255,SENSITIVITY) * pow(x,1-SENSITIVITY));
}

void setup() {
  //Initialise pin modes and serial
  pinMode(A0, INPUT);
  pinMode(A1, INPUT);  

  pinMode(Motors.pinL, OUTPUT);
  pinMode(Motors.pinR, OUTPUT);

  servoR.attach(3);
  servoL.attach(11);

  Serial.begin(9600);
}

void loop() {
  for (int i = 0; i < 360; i++) {
    //Read both input voltages
    PhotoL.val = analogRead(A0);
    PhotoR.val = analogRead(A1);

    sweep(servoL, angle, isIncrementing);
    sweep(servoR, angle, isIncrementing);
    
    double speedL = adjust(PhotoL.val/4);
    double speedR = adjust(PhotoR.val/4);

    analogWrite(Motors.pinR, speedL);
    analogWrite(Motors.pinL, speedR);

    delay(5);
  }
 
}