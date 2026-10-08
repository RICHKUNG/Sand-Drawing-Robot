#include <cmath>
#include <AccelStepper.h>
#include <MultiStepper.h>
#include <math.h>
#define INNERSENSOR 15  //橘
#define OUTERSENSOR 4   //黃
#define RADIUS 276.0    //兩手臂長度和
#define SPEED 1000.0
#define OUTERERROR 25   //外圈修正
int innersteps = 1800;  //平均內圈步數
int outersteps = 1800;  //平均外圈步數
AccelStepper inner(1, 22, 23);
AccelStepper outer(1, 16, 17);
MultiStepper steppers;
//各種座標
struct XY {  //XY座標系統
  double X;
  double Y;
};
struct Rtheta {  //Rθ座標系統
  double radi;
  double theta;
};
struct twoPLS {  //兩個馬達的絕對步數位置
  int inner;
  int outer;
};

//基本函數
void findzero(int outerinitial);    //找原點
void testloop(int a);               //測步數
XY currentXY();                     //推算現在XY座標
Rtheta xy_to_rtheta(XY coor);       //直角到極座標
twoPLS rtheta_to_pls(Rtheta pole);  //極座標到步進座標
void PLStoMove(twoPLS);             //真正移動並確保最短路徑

void findzero(int outerinitial) {
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
  outer.setCurrentPosition(outerinitial);  //外圈步數
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
  innersteps = inner.currentPosition() / a;
  Serial.printf("inner averge=%d\n", innersteps);
  for (int i = 0; i < a; i++) {
    outer.move(200);
    outer.runToPosition();
    outer.setSpeed(SPEED);
    while (digitalRead(OUTERSENSOR)) {
      outer.runSpeed();
    }
  }
  outersteps = round((outer.currentPosition()) / a);
  Serial.printf("outer averge=%d\n", outersteps);
  findzero(outersteps / 2);
}

XY currentXY() {  //用目前馬達位置推算當前XY座標
  struct XY temp;
  temp.X = RADIUS * cos((inner.currentPosition() % innersteps) * 2.0 * PI / innersteps) / 2.0 + RADIUS * cos((outer.currentPosition() % outersteps) * 2.0 * PI / outersteps) / 2.0;
  temp.Y = RADIUS * sin((inner.currentPosition() % innersteps) * 2.0 * PI / innersteps) / 2.0 + RADIUS * sin((outer.currentPosition() % outersteps) * 2.0 * PI / outersteps) / 2.0;
  return temp;
}

Rtheta xy_to_rtheta(XY coor) {
  struct Rtheta temp;
  temp.radi = sqrt(sq(coor.X) + sq(coor.Y));
  temp.theta = atan2(coor.Y, coor.X);
  if (temp.theta < -PI) temp.theta = temp.theta + 2 * PI;  //輸出鎖定在-pi~pi
  if (temp.theta > PI) temp.theta = temp.theta - 2 * PI;
  return temp;
}
twoPLS rtheta_to_pls(Rtheta pole) {
  while (pole.theta < -PI) { pole.theta = pole.theta + 2 * PI; }  //輸入鎖定在-pi~pi
  while (pole.theta > PI) { pole.theta = pole.theta - 2 * PI; }
  double innerang = pole.theta - abs(acos(pole.radi / RADIUS));  //alpha恆正(右手姿態)
  double outerang = 2.0 * pole.theta - innerang;
  struct twoPLS temp;
  temp.inner = round(innerang * innersteps / (2.0 * PI));
  if (temp.inner < 0) temp.inner = temp.inner + innersteps;  //恆正
  temp.outer = round(outerang * outersteps / (2.0 * PI));
  if (temp.outer < 0) temp.outer = temp.outer + outersteps;  //恆正
  return temp;
}
void PLStoMove(twoPLS target) {
  while (inner.currentPosition() - target.inner > innersteps / 2) { target.inner += innersteps; }  //確保不跑半圈以上
  while (outer.currentPosition() - target.outer > outersteps / 2) { target.outer += outersteps; }
  while (target.inner - inner.currentPosition() > innersteps / 2) { target.inner -= innersteps; }  //確保不跑半圈以上
  while (target.outer - outer.currentPosition() > outersteps / 2) { target.outer -= outersteps; }
  inner.setSpeed(100);
  outer.setSpeed(100);
  long pos[2] = { target.inner, target.outer };
  steppers.moveTo(pos);
  steppers.runSpeedToPosition();
}