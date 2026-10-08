#include <AccelStepper.h>
#include <MultiStepper.h>
#include <math.h>
#define INNERSENSOR 15  //黃
#define OUTERSENSOR 4   //橘
#define DIAMETER 276.0
#define AV_LOOP_STEP 1800.0  //平均每圈步數
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

void setup() {
  Serial.begin(115200);
  pinMode(INNERSENSOR, INPUT_PULLUP);
  pinMode(OUTERSENSOR, INPUT_PULLUP);
  inner.setMaxSpeed(1000.0);
  inner.setMinPulseWidth(40);
  inner.setCurrentPosition(0);
  outer.setMaxSpeed(1000.0);
  outer.setMinPulseWidth(40);
  outer.setCurrentPosition(0);
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
    //Serial.printf("check\n");
    struct Rtheta outcome1 = xy_to_rtheta(test);
    struct twoPLS outcome = rtheta_to_pls(xy_to_rtheta(test));
    Serial.printf("R=%f\n", outcome1.radi);
    Serial.printf("theta=%f\n", outcome1.theta);
    Serial.printf("pls1=%d\n", outcome.inner);
    Serial.printf("pls2=%d\n", outcome.outer);
  }
}

Rtheta xy_to_rtheta(XY coor) {
  struct Rtheta temp;
  temp.radi = sqrt(sq(coor.X) + sq(coor.Y));
  if (coor.X < 0) temp.theta = atan(coor.Y / coor.X) + PI;
  else temp.theta = atan(coor.Y / coor.X);
  return temp;
}

twoPLS rtheta_to_pls(Rtheta pole) {
  struct twoPLS temp;
  float inner = (pole.theta - acos(pole.radi / DIAMETER)) * AV_LOOP_STEP / 2 / PI;
  float outer = (2.0 * pole.theta) * AV_LOOP_STEP / 2.0 / PI - temp.inner;
  temp.inner = round(inner);
  temp.outer = round(outer);
  return temp;
}