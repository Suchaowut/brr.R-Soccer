#include <POP32.h>
#include <POP32_Huskylens.h>
POP32_Huskylens huskylens;
#define degToRad 0.0174f
#define sin30 sin(30.f * degToRad)
#define cos30 cos(30.f * degToRad)

#define SensC A1
#define SensL A2
#define SensR A3
#define SenCRef 1680
#define SenLRef 1640
#define SenRRef 1945

unsigned long lastSeenBallTime = 0;
bool returningToGoal = false;

// main move atan //*/
#define Xaxis_Kp 1.2
#define Xaxis_Kd 0.7

#define Yaxis_Kp 1.5
#define Yaxis_Kd 0.25
////////    ////////*/
//
#define RotYel_Kp 0.5
#define RotYel_Ki 0.0
#define RotYel_Kd 0.5
#define RotYelErrorGap 15

#define RotBlu_Kp 0.5
#define RotBlu_Ki 0.0
#define RotBlu_Kd 0.5
#define RotBluErrorGap 15

uint8_t rxCnt = 0, rxBuf[8];
float pvYaw, lastYaw;
float realYaw;

#define head_Kp 1.0f
#define head_Ki 0.0f
#define head_Kd 0.0f
float head_error, head_pError, head_w, head_d, head_i;
unsigned long loopTimer;

#define rot_Kp 2.0
#define rot_Ki 0.0
#define rot_Kd 0.5
#define sp_rot 160      // ค่า setpoint ที่ลูกบอลอยู่ตรงกลางกล้องแกน x  320/2 = 160
#define rotErrorGap 10  // ค่า Error ที่ยอมให้หุ่นหยุดทำงาน
#define idleSpd 45      // ค่าความเร็วการหมุนเมื่อไม่เจอลูกบอล
float rot_error, rot_pError, rot_i, rot_d, rot_w;
int ballPosX;
// ค่าที่ใช้ปรับหุ่นให้เข้าใกล้ลูกบอล
#define fli_Kp 1.2  //0.65
#define fli_Ki 0.0
#define fli_Kd 0.0
#define flingErrorGap 10  // ค่า Error ที่ยอมให้หุ่นหยุดทำงาน
float fli_error, fli_pError, fli_i, fli_d, fli_spd;
int ballPosY;

float Xaxis_Error, Xaxis_PvEror, Xaxis_D, Xaxis_spd;
float Yaxis_Error, Yaxis_PvEror, Yaxis_D, Yaxis_spd;
float RotYel_Error, RotYel_PvEror, RotYel_D, RotYel_w;
float RotBlu_Error, RotBlu_PvEror, RotBlu_D, RotBlu_w;

#define limPin A0
#define reloadSpd 60

float thetaRad, vx, vy, spd1, spd2, spd3;
void wheel(int s1, int s2, int s3) {
  motor(1, s1);
  motor(2, s2);
  motor(3, s3);
}
void holonomic(float spd, float theta, float omega) {
  thetaRad = theta * degToRad;
  vx = spd * cos(thetaRad);
  vy = spd * sin(thetaRad);
  spd1 = vy * cos30 - vx * sin30 + omega;
  spd2 = -vy * cos30 - vx * sin30 + omega;
  spd3 = vx + omega;
  wheel(spd1, spd2, spd3);
}
void zeroYaw() {
  Serial1.begin(115200);
  delay(100);
  // Sets data rate to 115200 bps
  Serial1.write(0XA5);
  Serial1.write(0X54);
  delay(100);
  // pitch correction roll angle
  Serial1.write(0XA5);
  Serial1.write(0X55);
  delay(100);
  // zero degree heading
  Serial1.write(0XA5);
  Serial1.write(0X52);
  delay(100);
  // automatic mode
}
bool getIMU() {
  while (Serial1.available()) {
    rxBuf[rxCnt] = Serial1.read();
    if (rxCnt == 0 && rxBuf[0] != 0xAA) return false;
    rxCnt++;
    if (rxCnt == 8) {  // package is complete
      rxCnt = 0;
      if (rxBuf[0] == 0xAA && rxBuf[7] == 0x55) {  // data package is correct
        pvYaw = (int16_t)(rxBuf[1] << 8 | rxBuf[2]) / 100.f;
        // realYaw = pvYaw < 0 ? pvYaw+360 : pvYaw;
        return true;
      }
    }
  }
  return false;
}

void Auto_zero() {
  zeroYaw();
  getIMU();
  int timer = millis();
  oled.clear();
  oled.text(1, 2, "Setting zero");
  while (abs(pvYaw) > 0.02) {
    if (getIMU()) {
      oled.text(3, 6, "Yaw: %f ", pvYaw);
      oled.show();
      //beep();
      if (millis() - timer > 2000) {
        zeroYaw();
        timer = millis();
      }
    }
  }
  oled.clear();
  oled.show();
  beep();
}
void updateIMU() {
  for (int i = 0; i < 16; i++) {
    getIMU();
  }
}
// Shooting
void shoot() {
  sound(1000, 200);
  // motor(4, reloadSpd);
  // delay(150);
  // motor(4, 0);
  // delay(50);
}
void reload() {
  motor(4, reloadSpd);
  int timer = 0;
  for (int i = 0; i < 2000; i++) {
    timer++;
    if (analog(limPin) > 1000) break;
    delay(1);
  }
  if (timer == 2000) {     // ถ้าก้านยิงติด
    motor(4, -reloadSpd);  // เลื่อนก้านยิงไปข้างหน้า
    delay(500);            //ก่อน 0.5 วินาที
    motor(4, reloadSpd);
    timer = 0;
    for (int i = 0; i < 2000; i++) {
      timer++;
      if (analog(limPin) > 1000) break;
      delay(1);
    }
  }
  motor(4, 0);
}
float erYaw;
void heading(float spd, float theta, float spYaw) {
  // if(spYaw >= 0 && pvYaw < 0) {
  //   erYaw = pvYaw + 360;
  // }else if (spYaw < 0 && pvYaw >= 0){
  //   erYaw = pvYaw- 360;
  // }
  // else {
  //   erYaw = pvYaw;
  // }
  // head_error = spYaw - erYaw;
  head_error = spYaw - pvYaw;
  head_i = head_i + head_error;
  head_i = constrain(head_i, -50, 50);
  head_d = head_error - head_pError;
  head_w = (head_error * head_Kp) + (head_i * head_Ki) + (head_d * head_Kd);
  head_w = constrain(head_w, -50, 50);
  holonomic(spd, theta, head_w);
  head_pError = head_error;

  // Serial.println(head_error);
}

void SetYaw() {
  getIMU();
  if ((pvYaw >= 10 || pvYaw < -10)) {
    getIMU();
    loopTimer = millis();
    while (millis() - loopTimer <= 100) {
      getIMU();
      heading(0, 0, 0);
    }
    holonomic(0, 0, 0);
    getIMU();
  }
}

// void TrackXaxis() {
//   if (huskylens.updateBlocks() && huskylens.blockSize[1]) {
//     Xaxis_Error = huskylens.blockInfo[1][0].x - 160;
//     Xaxis_D = Xaxis_Error - Xaxis_PvEror;
//     Xaxis_spd = (Xaxis_Error * Xaxis_Kp) + (Xaxis_D * Xaxis_Kd);
//     Xaxis_spd = constrain(Xaxis_spd, -80, 80);
//     Xaxis_PvEror = Xaxis_Error;
//     getIMU();
//     if (abs(pvYaw) > 1 && abs(Xaxis_Error) >= 15) {
//       getIMU();
//       heading(Xaxis_spd, 0, 0);
//     }

//   }
// }

void TrackXaxis() {
  if (huskylens.updateBlocks() && huskylens.blockSize[1]) {
    Xaxis_Error = huskylens.blockInfo[1][0].x - 160;
    Xaxis_D = Xaxis_Error - Xaxis_PvEror;
    Xaxis_spd = (Xaxis_Error * Xaxis_Kp) + (Xaxis_D * Xaxis_Kd);
    if (abs(Xaxis_spd) < 15 && abs(Xaxis_Error) > 10) {
      Xaxis_spd = (Xaxis_spd > 0) ? 15 : -15;
    } else Xaxis_spd = constrain(Xaxis_spd, -80, 80);
    Xaxis_PvEror = Xaxis_Error;
    getIMU();
    if (abs(pvYaw) > 1 && abs(Xaxis_Error) >= 15) {
      getIMU();
      heading(Xaxis_spd, 0, 0);
    }
  }
}

float ReadSeTha() {
  if (huskylens.updateBlocks() && huskylens.blockSize[1]) {
    float ballPosX = huskylens.blockInfo[1][0].x;
    float ballPosY = huskylens.blockInfo[1][0].y;
    float QuaDrantX = huskylens.blockInfo[1][0].x - 150;
    float QuaDrantY = 180 - huskylens.blockInfo[1][0].y;
    float TanTheta = QuaDrantY / QuaDrantX;
    float Setha = atan(TanTheta) * (180 / PI);
    float SethaPos;
    if (QuaDrantX >= 0 && QuaDrantY >= 0) {  //QuaDrant1
      SethaPos = Setha;
    } else if (QuaDrantX < 0 && QuaDrantY >= 0) {  //QuaDrant2
      SethaPos = 180 + Setha;
    } else if (QuaDrantX < 0 && QuaDrantY < 0) {  //QuaDrant3
      SethaPos = 180 + abs(Setha);
    } else if (QuaDrantX >= 0 && QuaDrantY < 0) {  //QuaDrant4
      SethaPos = 360 + Setha;
    }
    return SethaPos;
  }
}

// void ReadgoalPos() {
//   while (1) {
//     if (huskylens.updateBlocks() && huskylens.blockSize[1] && huskylens.blockSize[3]) {
//       float goalPosX = huskylens.blockInfo[3][0].x;
//       float goalPosY = huskylens.blockInfo[3][0].y;
//       float QuaDrantX = huskylens.blockInfo[3][0].x - 150;
//       float QuaDrantY = 180 - huskylens.blockInfo[3][0].y;
//       float TanTheta = QuaDrantY / QuaDrantX;
//       float Setha = atan(TanTheta) * (180 / PI);
//       float SethaPos;
//       if (QuaDrantX >= 0 && QuaDrantY >= 0) {  //QuaDrant1
//         SethaPos = Setha;
//       } else if (QuaDrantX < 0 && QuaDrantY >= 0) {  //QuaDrant2
//         SethaPos = 180 + Setha;
//       }
//       Serial.print("goalPosX = ");
//       Serial.println(goalPosX);
//       Serial.print("goalPosY = ");
//       Serial.println(goalPosY);
//       Serial.print("Setha = ");
//       Serial.println(Setha);
//       Serial.println("///////////////////////////////////////////");
//     }
//   }
// }

void ShootAndReload() {
  shoot();
  delay(500);
  reload();
}
void CoordsBall() {
  oled.text(5, 0, "                ");
  while (1) {
    huskylens.updateBlocks();

    oled.text(2, 0, "ballX %d     ", huskylens.blockInfo[1][0].x);
    oled.text(3, 0, "ballY %d     ", huskylens.blockInfo[1][0].y);
    oled.show();
  }
}

void chksens() {
  while (1) {
    oled.text(2, 0, "C=%d     ", analog(1));
    oled.text(3, 0, "L=%d     ", analog(2));
    oled.text(4, 0, "R=%d     ", analog(3));
    oled.show();
  }
}

struct MenuItem {
  const char* name;
  void (*action)();
};

MenuItem menuItems[] = {
  { "TouchLine", TouchLine },
  { "SmartATK", SmartATK },
  { "Shoot&load", ShootAndReload },
  { "CoordsBall", CoordsBall },
  { "check Sens", chksens }
};

const int menuCount = sizeof(menuItems) / sizeof(MenuItem);

bool firstRun = true;  // ตัวแปรตรวจสอบว่ารอบแรกหรือไม่
int run_count = 0;

void menu() {
  int x = 1, lastX = 1;;
  long startTime;
  if (firstRun == false) startTime = millis();

  x = knob(1, menuCount);
  lastX = x;

  while (firstRun ? !SW_OK() : (millis() - startTime < 5000 && !SW_OK() && run_count < 2)) { /* firstRun == true ให้ใช้เงื่อนไข A → !SW_OK() (รอจนกว่าจะกดปุ่ม OK)
                                                                                                firstRun == false ให้ใช้เงื่อนไข B → (millis() - startTime < 5000 && !SW_OK()) ไม่มีการกด OK ภายใน 5 วิ จะวิ่งเอง*/
    x = knob(1, menuCount);

    if (x != lastX) {
      oled.fillScreen(BLACK);
      oled.show();
      lastX = x;
    }

    if (SW_A()) {
      Auto_zero();
    }

    if (SW_B()) {
      ShootAndReload();
    }

    // แสดงเมนูปัจจุบัน
    char modeText[32];
    snprintf(modeText, sizeof(modeText), "Mode: %s", menuItems[x - 1].name);
    oled.text(0, 0, modeText);

    // แสดงเมนูก่อนหน้าและถัดไป
    const char* prev = (x > 1) ? menuItems[x - 2].name : "-";
    const char* next = (x < menuCount) ? menuItems[x].name : "-";

    char nexText[48], preText[48];

    // snprintf(navText, sizeof(navText), "Back: %s      Next: %s     ", prev, next);
    // oled.text(2, 0, navText);
    snprintf(preText, sizeof(preText), "<: %s", prev);
    snprintf(nexText, sizeof(nexText), ">: %s", next);
    oled.text(2, 0, preText);
    oled.text(3, 0, nexText);

    oled.show();
  }
  
  oled.text(2, 0, "                   ");
  oled.text(3, 0, "                   ");
  firstRun = false;  // หลังจากรอบแรก จะไม่ต้องรอกดปุ่ม OK อีก
  run_count++;
  // sound(777, 100);
  oled.fillScreen(BLACK);
  oled.show();
  menuItems[x - 1].action();
}

void setup() {
  Serial.begin(9600);
  reload();
  while (!huskylens.begin(Wire)) {
    oled.text(1, 0, "Huskylens failed!");
    oled.show();
  }
  delay(1000);
  zeroYaw();
  // Auto_zero();
  // while(1){
  //   updateIMU();
  //   oled.text(3, 6, "Yaw: %f ", pvYaw);
  //   oled.show();
  // }
  // waitAnykey();
  menu();
}
void loop() {
  menu();
  // getIMU();
  // Serial.println(pvYaw);
  // heading(0, 0, 180);
  // readyShoot();
  // if ((huskylens.updateBlocks() && huskylens.blockSize[1])) {
  //   // lastGoalCoord = lastGoalPos();
    
  //   // if (ballPosY <= 60 && abs(ballPosX - goalEstX) < 15) {
  //   //   holonomic(0, 0, 0);
  //   //   beep();
  //   // } else holonomic(0, 0, 0);
  // }
}
