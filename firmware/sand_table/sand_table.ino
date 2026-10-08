//開發占比:皆自行完成，無使用AI，只在寫RthetatoPLS函數時有看過其他同學的code
#include "setupsand.h"
#include "gcode.h"
#include "endcode.h"
#define MAXSPACE 0.5  //插補標準
//繪圖函數
void rungcode(int a);                             //跑Gcode
void runfunc();                                   //跑極座標函數
void positiveMove(float innersp, float outersp);  //跑正向運動圖形
void movestraight(XY A, XY B);                    //兩點間直線插補
void handin();                                    //手動輸入座標

void setup() {
  Serial.begin(115200);
  pinMode(INNERSENSOR, INPUT_PULLUP);
  pinMode(OUTERSENSOR, INPUT_PULLUP);
  inner.setMaxSpeed(500.0);
  inner.setAcceleration(20000.0);
  inner.setMinPulseWidth(40);
  inner.setCurrentPosition(0);
  outer.setMaxSpeed(500.0);
  outer.setAcceleration(20000.0);
  outer.setMinPulseWidth(40);
  outer.setCurrentPosition(0);
  steppers.addStepper(inner);
  steppers.addStepper(outer);
  findzero(0);
  testloop(3);
  rungcode(1);
  runfunc();
  findzero(innersteps / 2);
  rungcode(2);
  findzero(innersteps / 2);
}
void loop() {
  handin();
}

void rungcode(int a) {
  if (a == 1) {
    for (int i = 0; i < sizeof(gcode) / sizeof(gcode[0]); i++) {
      struct XY init;
      init.X = gcode[i - 1][0] - 276.0;
      init.Y = gcode[i - 1][1] - 276.0;
      struct XY tar;
      tar.X = gcode[i][0] - 276.0;
      tar.Y = gcode[i][1] - 276.0;
      if (i == 0) {
        struct XY temp = currentXY();
        movestraight(temp, tar);
      } else movestraight(init, tar);
    }
  } else if (a == 2) {
    for (int i = 0; i < sizeof(endcode) / sizeof(endcode[0]); i++) {
      struct XY init;
      init.X = endcode[i - 1][0] - 276.0;
      init.Y = endcode[i - 1][1] - 276.0;
      struct XY tar;
      tar.X = endcode[i][0] - 276.0;
      tar.Y = endcode[i][1] - 276.0;
      if (i == 0) {
        struct XY temp = currentXY();
        movestraight(temp, tar);
      } else movestraight(init, tar);
    }
  } else {
    struct XY temp = currentXY();
    movestraight(temp, { 0.0, 0.0 });
  }
}

void runfunc() {
  static float t = 30.0;
  struct Rtheta fun2;
  struct XY temp = currentXY();
  movestraight(temp, { 0.0, 0.0 });
  while (fun2.radi < RADIUS) {
    fun2.radi = 1.3 * t + 20 * sinf(5 * t);
    fun2.theta = t;
    PLStoMove(rtheta_to_pls(fun2));
    t += 0.01;
  }
  Serial.printf("r=%f\n", fun2.radi);
  while (t > 0) {
    fun2.radi = 50 - 200 * cos(2 * t);
    fun2.theta = t;
    PLStoMove(rtheta_to_pls(fun2));
    t -= 0.001;
  }
}

void positiveMove(float innersp, float outersp) {
  inner.setSpeed(innersp);
  outer.setSpeed(outersp);
  inner.move(innersteps);
  outer.move(outersp * outersteps / innersp);
  inner.runSpeedToPosition();
  outer.runSpeedToPosition();
}

void movestraight(XY A, XY B) {  //如果相距太遠就切兩段遞迴
  if ((abs(B.X - A.X) > MAXSPACE) || (abs(B.Y - A.Y) > MAXSPACE)) {
    struct XY temp;  //找中點
    temp.X = (A.X + B.X) / 2;
    temp.Y = (A.Y + B.Y) / 2;
    movestraight(A, temp);  //分段處理
    movestraight(temp, B);
  } else {
    PLStoMove(rtheta_to_pls(xy_to_rtheta(B)));
  }
}
void handin() {
  struct XY test;
  char *space_add;
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
    struct Rtheta num = xy_to_rtheta(test);
    if (num.radi > RADIUS) Serial.printf("OUT OF RANGE\n\n");
    else {
      struct XY temp = currentXY();
      Serial.printf("from X=%f Y=%f\n", temp.X, temp.Y);
      movestraight(temp, test);
      struct XY temp1 = currentXY();
      Serial.printf("to X=%f Y=%f\n", temp1.X, temp1.Y);
    }
  }
}