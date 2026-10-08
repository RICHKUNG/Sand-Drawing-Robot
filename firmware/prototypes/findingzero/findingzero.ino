#include <AccelStepper.h>
#include <MultiStepper.h>
#include <math.h>
#define INNERSENSOR 15  //橘
#define OUTERSENSOR 4   //黃
#define DIAMETER 276.0  //兩手臂長度和
#define SPEED 1000.0
#define OUTERINITIAL 1000
#define OUTERERROR 25
AccelStepper inner(1, 22, 23);
AccelStepper outer(1, 18, 19);
MultiStepper steppers;
void findzero();
void testloop(int a);
void setup() {
  Serial.begin(115200);
  pinMode(INNERSENSOR, INPUT_PULLUP);
  pinMode(OUTERSENSOR, INPUT_PULLUP);
  inner.setMaxSpeed(1000.0);
  inner.setAcceleration(20000.0);
  inner.setMinPulseWidth(40);
  inner.setCurrentPosition(0);
  outer.setMaxSpeed(1000.0);
  outer.setAcceleration(20000.0);
  outer.setMinPulseWidth(40);
  outer.setCurrentPosition(0);
  findzero();
  //testloop(5);
  //findzero();
  steppers.addStepper(inner);
  steppers.addStepper(outer);
}
void loop() {
  /*inner.moveTo(500);
  inner.runToPosition();
  outer.moveTo(500);
  outer.runToPosition();
  delay(500);
  findzero();*/
}
void findzero() {
  inner.setSpeed(SPEED);
  while (digitalRead(INNERSENSOR)) {
    inner.runSpeed();
  }
  inner.setCurrentPosition(0);

  outer.setSpeed(SPEED);
  while (digitalRead(OUTERSENSOR)) {
    outer.runSpeed();
  }
  outer.move(OUTERERROR);
  outer.runToPosition();
  outer.setCurrentPosition(OUTERINITIAL);
  Serial.printf("findzero\n");
}
void testloop(int a) {
  for (int i = 0; i < a; i++) {
    inner.move(20);
    inner.runToPosition();
    inner.setSpeed(SPEED);
    while (digitalRead(INNERSENSOR)) {
      inner.runSpeed();
    }
  }
  Serial.printf("inner averge=%d\n", inner.currentPosition() / a);
  for (int i = 0; i < a; i++) {
    outer.move(200);
    outer.runToPosition();
    outer.setSpeed(SPEED);
    while (digitalRead(OUTERSENSOR)) {
      outer.runSpeed();
    }
  }
  Serial.printf("outer averge=%d\n", (outer.currentPosition() - OUTERINITIAL + OUTERERROR) / a);
}