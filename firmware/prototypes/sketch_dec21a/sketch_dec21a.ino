#include <AccelStepper.h>
#include <MultiStepper.h>
#include <math.h>
#define INNERSENSOR 15  //橘
#define OUTERSENSOR 4   //黃
#define DIAMETER 276.0  //兩手臂長度和
#define SPEED 1000.0
#define OUTERERROR 15    //外圈修正
#define INNERSTEPS 1790  //平均內圈步數
#define OUTERSTEPS 1760  //平均外圈步數
AccelStepper inner(1, 22, 23);
AccelStepper outer(1, 18, 19);
MultiStepper steppers;
struct XY {
  float X;
  float Y;
};
struct Rtheta {
  float radi;
  float theta;
};
struct twoPLS {
  int inner;
  int outer;
};
Rtheta xy_to_rtheta(XY coor);       //直角到極座標
twoPLS rtheta_to_pls(Rtheta pole);  //極座標到步進座標
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
  struct XY test;
  char* space_add;
  int index = 0;
  int b0 = Serial.available();
  char c0[20] = "";
  char num1[10] = "";
  char num2[10] = "";
  if (b0 > 0) {
    Serial.readBytes(c0, b0);
    for (int i = 0; i <= strlen(c0); i++) {
      if (c0[i] != ' ' & c0[i] != ',') {
        if (index == 0) {
          num1[i] = c0[i];
        } else {
          num2[i - index - 1] = c0[i];
        }
      } else {
        index = i;
        test.X = atof(num1);
      }
    }
    test.Y = atof(num2);
    struct Rtheta outcome1 = xy_to_rtheta(test);
    struct twoPLS outcome = rtheta_to_pls(xy_to_rtheta(test));
    Serial.printf("(%f,%f)\n", test.X, test.Y);
    Serial.printf("[%f,%fπ]\n", outcome1.radi, outcome1.theta / PI);
    if (outcome1.radi > DIAMETER) Serial.printf("OUT OF RANGE\n\n");
    else {
      long pos[2] = { outcome.inner, outcome.outer };
      Serial.printf("innerPLS=%d\nouterPLS=%d\n\n", outcome.inner, outcome.outer);
      steppers.moveTo(pos);
      steppers.runSpeedToPosition();
    }
  }
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
  outer.setCurrentPosition(OUTERSTEPS / 2);
  Serial.printf("foundzero\n");
  // Serial.printf("innerpos=%d\n",inner.currentPosition());
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
  Serial.printf("outer averge=%d\n", (outer.currentPosition() - (OUTERSTEPS / 2)) / a);
}
Rtheta xy_to_rtheta(XY coor) {
  struct Rtheta temp;
  temp.radi = sqrt(sq(coor.X) + sq(coor.Y));
  //if (coor.X < 0)
  temp.theta = atan2(coor.Y, coor.X);
  //else temp.theta = atan(coor.Y / coor.X);
  //if (temp.theta < 0) temp.theta = temp.theta + 2 * PI;
  return temp;
}
twoPLS rtheta_to_pls(Rtheta pole) {
  struct twoPLS temp;
  float innerang = pole.theta - acos(pole.radi / DIAMETER);
  float outerang = 2.0 * pole.theta - innerang;
  temp.inner = round(innerang * INNERSTEPS / (2.0 * PI));
  if (temp.inner < 0) temp.inner = temp.inner + INNERSTEPS;  //恆正
  temp.outer = round(outerang * OUTERSTEPS / (2.0 * PI));
  if (temp.outer < 0) temp.outer = temp.outer + OUTERSTEPS;  //恆正
  return temp;
}