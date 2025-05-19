// void TrackXaxis2() {
//   if (huskylens.updateBlocks() && huskylens.blockSize[1]) {
//     Xaxis_Error = huskylens.blockInfo[1][0].x - 160;
//     Xaxis_D = Xaxis_Error - Xaxis_PvEror;
//     Xaxis_spd = (Xaxis_Error * Xaxis_Kp) + (Xaxis_D * Xaxis_Kd);
//     if (abs(Xaxis_spd) < 15 && abs(Xaxis_Error) > 6) {
//       Xaxis_spd = (Xaxis_spd > 0) ? 10 : -10;
//     } else Xaxis_spd = constrain(Xaxis_spd, -80, 80);
//     Xaxis_PvEror = Xaxis_Error;
//     getIMU();
//     heading(Xaxis_spd, 0, 0);
//   }
// }

// int vecCurve;
// int prevDynamicSpeed = 70;  // เพิ่มตัวแปร global ด้านบนไว้ด้วยนะ

// int calcBallSpeed(int ballY) {
//   int maxSpeed = 100;
//   int minSpeed = 40;
//   int speed = map(ballY, 20, 140, maxSpeed, minSpeed);
//   return constrain(speed, minSpeed, maxSpeed);
// }

// void AtanTrack2() {
//   if ((huskylens.updateBlocks() && huskylens.blockSize[1])) {
//     lastGoalCoord = lastGoalPos();
//     ballPosX = huskylens.blockInfo[1][0].x;
//     ballPosY = huskylens.blockInfo[1][0].y;
//     float QuaDrantX = huskylens.blockInfo[1][0].x - 160;
//     float QuaDrantY = 180 - huskylens.blockInfo[1][0].y;
//     float TanTheta = QuaDrantY / QuaDrantX;
//     float Setha = atan(TanTheta) * (180 / PI);
//     float SethaPos;
//     float DisTanT = sqrt(pow(abs(QuaDrantX), 2) + pow(abs(QuaDrantY), 2));
//     if (Setha >= 0) {
//       SethaPos = Setha;
//     } else if (Setha < 0) {
//       SethaPos = 180 + Setha;
//     }
//     getIMU();

//     Yaxis_Error = 160 - huskylens.blockInfo[1][0].y;
//     Yaxis_D = Yaxis_Error - Yaxis_PvEror;
//     Yaxis_spd = (Yaxis_Error * Yaxis_Kp) + (Yaxis_D * Yaxis_Kd);
//     Yaxis_spd = constrain(Yaxis_spd, -100, 100);
//     Yaxis_PvEror = Yaxis_Error;

//     // คำนวณความเร็วแบบปรับตามระยะ Y ของลูก
//     int rawSpeed = calcBallSpeed(ballPosY);
//     int dynamicSpeed = 0.8 * prevDynamicSpeed + 0.2 * rawSpeed;
//     prevDynamicSpeed = dynamicSpeed;

//     holonomic(dynamicSpeed, SethaPos, 0);

//     if (Yaxis_Error <= 20) {
//       getIMU();
//       lastYaw = pvYaw;
//       if (abs(pvYaw) <= 10) {
//         TrackXaxis2();
//         if (abs(Xaxis_Error) <= 5) {
//           wheel(0, 0, 0);
//           SetYaw();
//           getIMU();
//           // beep();
//           lastYaw = pvYaw;
//           Dribbling();
//         }
//       } else if (!(abs(pvYaw) <= 10)) {
//         while ((huskylens.updateBlocks() && huskylens.blockSize[1])) {
//           getIMU();
//           rot_error = 160 - huskylens.blockInfo[1][0].x;
//           rot_d = rot_d + rot_error;
//           rot_d = constrain(rot_d, -100, 100);
//           rot_d = rot_error - rot_pError;
//           rot_pError = rot_error;
//           rot_w = (rot_error * 0.6) + (rot_i * rot_Ki) + (rot_d * rot_Kd);
//           rot_w = constrain(rot_w, -100, 100);

//           int targetSpeed = (abs(pvYaw) > 50) ? 60 : 40;
//           int currentSpeed = 0.8 * currentSpeed + 0.2 * targetSpeed;  // smoothing speed
//           if (lastYaw < 0) {
//             vecCurve = 0;
//           } else {
//             vecCurve = 180;
//           }
//           holonomic(currentSpeed, vecCurve, rot_w);
//           if (abs(pvYaw) <= 5) break;
//         }
//       }
//     }
//   }
// }

// void Dribbling() {
//   float angleToGoal = 90, goalEstX, goalEstY, goalEstWidth;
//   while (huskylens.updateBlocks() && huskylens.blockSize[1]) {
//     if (huskylens.updateBlocks() && !(huskylens.blockSize[2] || huskylens.blockSize[3])) {
//       getIMU();
//       heading(100, angleToGoal, 0);
//     } else {
//       // track goal พร้อมลูก
//       getIMU();
//       ballPosX = huskylens.blockInfo[1][0].x;
//       ballPosY = huskylens.blockInfo[1][0].y;

//       if (huskylens.updateBlocks() && huskylens.blockSize[2]) {
//         goalEstX = huskylens.blockInfo[2][0].x;
//         goalEstY = huskylens.blockInfo[2][0].y;
//         goalEstWidth = huskylens.blockInfo[2][0].width;
//       } else if (huskylens.updateBlocks() && huskylens.blockSize[3]) {
//         goalEstX = huskylens.blockInfo[3][0].x;
//         goalEstY = huskylens.blockInfo[3][0].y;
//         goalEstWidth = huskylens.blockInfo[3][0].width;
//       }

//       float goalXLeft = goalEstX - goalEstWidth / 2.0;
//       float goalXRight = goalEstX + goalEstWidth / 2.0;

//       float distanceToEdge = min(abs(ballPosX - goalXLeft), abs(ballPosX - goalXRight));

//       float dx = goalEstX - ballPosX;
//       float dy = goalEstY - ballPosY;
//       float angleToGoal = atan2(dy, dx) * (180 / PI);
//       if (angleToGoal < 0) angleToGoal += 360;
//       if (angleToGoal > 180) angleToGoal = 180 - (angleToGoal - 180);
//       if (distanceToEdge < 10) {
//         angleToGoal = constrain(angleToGoal, 30, 150);
//       } else {
//         angleToGoal = constrain(angleToGoal, 5, 175);
//       }
//       if (ballPosY >= 210 && abs(ballPosX) <= 10) {
//         heading(100, angleToGoal, 0);
//       } else {
//         heading(100, 90, 0);
//       }
//       if (abs(ballPosX - goalEstWidth) < 10 && ballPosY > 220 && analogRead(SensC) > SenCRef) {
//         holonomic(0, 0, 0);
//         shoot();
//         delay(100);
//         holonomic(60, 270, 0);
//         delay(300);
//         holonomic(0, 0, 0);
//         reload();
//         holonomic(0, 0, 0);
//         delay(1000);
//       }
//     }
//   }
// }

// void TouchLine() {
//   while (1) {
//     if (huskylens.updateBlocks() && huskylens.blockSize[1]) {
//       lastYaw = pvYaw;
//       if (analogRead(SensC) > SenCRef) {
//         holonomic(0, 0, 0);
//         delay(100);
//         holonomic(50, 270, 0);
//         delay(100);
//         holonomic(80, 270, 0);
//         delay(200);
//       } else if (analogRead(SensL) > SenLRef) {
//         holonomic(0, 0, 0);
//         delay(100);
//         holonomic(50, 0, 0);
//         delay(100);
//         holonomic(80, 0, 0);
//         delay(200);
//       } else if (analogRead(SensR) > SenRRef) {
//         holonomic(0, 180, 0);
//         delay(100);
//         holonomic(50, 180, 0);
//         delay(100);
//         holonomic(80, 180, 0);
//         delay(200);
//       } else {
//         AtanTrack2();
//       }
//     } else {
//       getIMU();
//       wheel(0, 0, 0);
//       while (abs(pvYaw) <= 2) {
//         SetYaw();
//       }

//       // unsigned long StartTime = millis();
//       while (1) {
//         int sideRot = 160 - ballPosX;  //คำนวนทิศการหมุนหาลูกบอลเมื่อเจอล่าสุด
//         holonomic(0, 0, sideRot / abs(sideRot) * idleSpd);
//         if (huskylens.updateBlocks() && huskylens.blockSize[1]) { break; }
//         // SetYaw();
//       }
//       while (analogRead(SensL) < SenLRef && analogRead(SensR) < SenRRef) {
//         getIMU();
//         heading(100, 270, 0);
//         if (huskylens.updateBlocks() && huskylens.blockSize[1]) { break; }
//       }
//     }
//   }
// }
int lastError = 0;
long errorStarttime = 0;

void TrackXaxis2() {
  if (huskylens.updateBlocks() && huskylens.blockSize[1]) {
    Xaxis_Error = huskylens.blockInfo[1][0].x - 170;
    Xaxis_D = Xaxis_Error - Xaxis_PvEror;
    if (abs(Xaxis_Error - lastError) < 3) {
      if (errorStarttime == 0) errorStarttime = millis();
      else errorStarttime = 0;
    }
    Xaxis_spd = (Xaxis_Error * Xaxis_Kp) + (Xaxis_D * Xaxis_Kd);
    if (abs(Xaxis_spd) < 15 && abs(Xaxis_Error) >= 2) {
      Xaxis_spd = (Xaxis_spd > 0) ? 30 : -30;
    } else {
      if (errorStarttime > 0 && millis() - errorStarttime > 1000) {
        Xaxis_spd *= 2.5;
        if(Xaxis_spd <= 10) Xaxis_spd = 20;
      }
      Xaxis_spd = constrain(Xaxis_spd, -80, 80);
    }
    Xaxis_PvEror = Xaxis_Error;
    lastError = Xaxis_Error;
    getIMU();
    heading(Xaxis_spd, 0, 0);
  }
}

int prevDynamicSpeed = 70;  // เพิ่มตัวแปร global ด้านบนไว้ด้วยนะ

int calcBallSpeed(int ballY) {
  int maxSpeed = 100;
  int minSpeed = 40;
  int speed = map(ballY, 20, 160, maxSpeed, minSpeed);
  return constrain(speed, minSpeed, maxSpeed);
}

int vecCurve;
void AtanTrack2() {
  if ((huskylens.updateBlocks() && huskylens.blockSize[1])) {
    lastGoalCoord = lastGoalPos();
    ballPosX = huskylens.blockInfo[1][0].x;
    ballPosY = huskylens.blockInfo[1][0].y;
    float QuaDrantX = huskylens.blockInfo[1][0].x - 150;
    float QuaDrantY = 180 - huskylens.blockInfo[1][0].y;
    float TanTheta = QuaDrantY / QuaDrantX;
    float Setha = atan(TanTheta) * (180 / PI);
    float SethaPos;
    float DisTanT = sqrt(pow(abs(QuaDrantX), 2) + pow(abs(QuaDrantY), 2));
    if (Setha >= 0) {
      SethaPos = Setha;
    } else if (Setha < 0) {
      SethaPos = 180 + Setha;
    }
    getIMU();
    // SetYaw();
    Yaxis_Error = 160 - huskylens.blockInfo[1][0].y;
    Yaxis_D = Yaxis_Error - Yaxis_PvEror;
    Yaxis_spd = (Yaxis_Error * Yaxis_Kp) + (Yaxis_D * Yaxis_Kd);
    Yaxis_spd = constrain(Yaxis_spd, -100, 100);
    Yaxis_PvEror = Yaxis_Error;

    holonomic(Yaxis_spd, SethaPos, 0);
    // คำนวณความเร็วแบบปรับตามระยะ Y ของลูก
    // int rawSpeed = calcBallSpeed(ballPosY);
    // int dynamicSpeed = 0.8 * prevDynamicSpeed + 0.2 * rawSpeed;
    // prevDynamicSpeed = dynamicSpeed;

    // holonomic(dynamicSpeed, SethaPos, 0);
    if (Yaxis_Error <= 20) {
      getIMU();
      lastYaw = pvYaw;
      if (abs(pvYaw) <= 8) {
        TrackXaxis2();
        if (abs(Xaxis_Error) <= 8) {
          wheel(0, 0, 0);
          // SetYaw();
          getIMU();
          // beep();
          lastYaw = pvYaw;
          // beep();
          Dribbling();
        }
      } else if (abs(pvYaw) > 8) {
        while ((huskylens.updateBlocks() && huskylens.blockSize[1]) && Yaxis_Error <= 20) {
          getIMU();
          rot_error = 170 - huskylens.blockInfo[1][0].x;
          rot_d = rot_error - rot_pError;
          rot_pError = rot_error;
          rot_w = (rot_error * 0.3) + (rot_i * rot_Ki) + (rot_d * rot_Kd);
          rot_w = constrain(rot_w, -60, 60);

          Yaxis_Error = 160 - huskylens.blockInfo[1][0].y;
          Yaxis_D = Yaxis_Error - Yaxis_PvEror;
          Yaxis_spd = (Yaxis_Error * Yaxis_Kp) + (Yaxis_D * Yaxis_Kd);
          Yaxis_spd = constrain(Yaxis_spd, -100, 100);
          Yaxis_PvEror = Yaxis_Error;

          if (Yaxis_Error > 15) holonomic(Yaxis_spd, 90, rot_w);

          int targetSpeed = (abs(pvYaw) > 50) ? 90 : 70;
          int currentSpeed = 0.8 * currentSpeed + 0.2 * targetSpeed;  // smoothing speed
          if (pvYaw < 0) {
            vecCurve = 0;
          } else {
            vecCurve = 180;
          }
          holonomic(currentSpeed, vecCurve, rot_w);
          if (abs(pvYaw) <= 8) break;
        }
      }
    }
  }
}

float angleToGoal = 90;
float previousAngle = 90;
int lastSeenGoal = 1;  // front
// int count, bypassLR, bypassC, CountC;     // sensor

void Dribbling() {
  bypassLR = 0;
  count = 0;
  bypassC = 0;
  countC = 0;
  float goalEstX = 160, goalEstY = 120, goalEstWidth = 50;

  while (huskylens.updateBlocks() && huskylens.blockSize[1]) {
    if (analogRead(SensC) > SenCRef && (huskylens.updateBlocks() && !(huskylens.blockSize[2] || huskylens.blockSize[3]))) {
      holonomic(0, 0, 0);
      delay(100);
      holonomic(50, 270, 0);
      delay(100);
      holonomic(80, 270, 0);
      delay(200);
      break;
    }
    getIMU();
    ballPosX = huskylens.blockInfo[1][0].x;
    ballPosY = huskylens.blockInfo[1][0].y;

    // ตรวจสอบโกลซ้ายหรือขวา
    if (huskylens.blockSize[2]) {
      goalEstX = huskylens.blockInfo[2][0].x;
      goalEstY = huskylens.blockInfo[2][0].y;
      goalEstWidth = huskylens.blockInfo[2][0].width;
    } else if (huskylens.blockSize[3]) {
      goalEstX = huskylens.blockInfo[3][0].x;
      goalEstY = huskylens.blockInfo[3][0].y;
      goalEstWidth = huskylens.blockInfo[3][0].width;
    } else {
      heading(100, angleToGoal, 0);  // ใช้มุมล่าสุดไปก่อน
      continue;
    }

    if (goalEstX - 150 < 0) lastSeenGoal = 1;
    else if (goalEstX - 150 > 0) lastSeenGoal = 2;
    else lastSeenGoal = 0;

    float goalLeft = goalEstX - goalEstWidth / 2.0;
    float goalRight = goalEstX + goalEstWidth / 2.0;
    float centerGoalX = goalEstX;
    float diffFromCenter = ballPosX - centerGoalX;

    float targetAngle = 90;
    if (abs(diffFromCenter) >= 15) {
      float maxAngleOffset = 20;
      float maxOffset = goalEstWidth / 2.0;
      float offsetRatio = constrain(diffFromCenter / maxOffset, -1.0, 1.0);
      targetAngle = 90 + offsetRatio * maxAngleOffset;
    }

    if (ballPosY < 220) {
      targetAngle = 90;
    }

    angleToGoal = previousAngle * 0.7 + targetAngle * 0.3;
    previousAngle = angleToGoal;
    int lastangleToGoal = angleToGoal;


    if (ballPosY < 225) {
      angleToGoal = 90;
    }

    bool ballInGoalArea = false;
    if (goalEstWidth > 0) {
      ballInGoalArea = (ballPosX >= goalLeft && ballPosX <= goalRight);
    }

    float error = lastangleToGoal - 90;
    // if(huskylens.updateBlocks() && !(huskylens.blockSize[2] && huskylens.blockSize[3])) {
    //   if(){
    //     error = angleToGoal - 90;
    //   }
    // }
    if (error > 180) error -= 360;
    if (error < -180) error += 360;

    int rott = 0;

    // หากโกลอยู่ที่ขอบซ้ายหรือขวา หนุนให้หมุนแรงขึ้น
    if (goalEstX < 110 || goalEstX > 210) {
      rott = constrain(error * 2, -30, 30);  // หมุนเร็วขึ้นเมื่อโกลอยู่ขอบ
    } else if (goalLeft <= 160 || 180 <= goalRight) {
      // ศูนย์กลางภาพ (170) อยู่ในเขตโกล — หมุนช้าลง
      rott = constrain(error, -10, 10);
    }
    // float oppositeAngle = 180 - angleToGoal;

    // int speed = map(ballPosY, 20, 225, 40, 100);
    // speed = constrain(speed, 40, 100);
    // if (abs(170 - ballPosX) < 15 && ballPosY > 225 && abs(goalEstX - 170) > 0) holonomic(100, angleToGoal, rott);
    if (abs(170 - ballPosX) < 10 && ballPosY > 215 && abs(goalEstX - 170) > 10 && !ballInGoalArea) holonomic(100, 180 - angleToGoal, rott);
    else if ((ballPosX > 150 && ballPosX < 200) && ballInGoalArea) holonomic(100, angleToGoal, 0);
    else if ((ballPosX > 150 && ballPosX < 200) && !ballInGoalArea) heading(100, angleToGoal, 0);
    // else holonomic(100, angleToGoal, 0);
    // if(ballPosY < 210) heading(100, angleToGoal, 0);

    // if (/*goalEstY > 30 && */ballPosY > 230 && ((analogRead(SensC) > SenCRef && ballInGoalArea) /*|| (ballPosX >= goalLeft + 10 && ballPosX <= goalRight - 10)*/)) {
    if (ballPosY > 220 && (analogRead(SensC) > SenCRef && (ballPosX >= goalLeft + 10 && ballPosX <= goalRight - 10))) /*ballInGoalArea*/ {
      holonomic(0, 0, 0);
      shoot();
      delay(100);
      holonomic(60, 270, 0);
      delay(300);
      holonomic(0, 0, 0);
      reload();
      delay(100);
      holonomic(0, 0, 0);
      break;
      // delay(1000);
    }
  }
}


void TouchLine() {
  while (1) {
    if (analogRead(limPin) < 700) reload();
    int FoundLeft = 0, FoundRight = 0, FoundCent = 0;
    if (huskylens.updateBlocks() && huskylens.blockSize[1]) {  // ball found
      // lastYaw = pvYaw;


      if (count >= 3) bypassLR = 1;

      /*if (analogRead(SensC) > SenCRef) {
        holonomic(0, 0, 0);
        delay(100);
        holonomic(50, 270, 0);
        delay(100);
        holonomic(80, 270, 0);
        delay(200);
        
      } else*/
      if (analogRead(SensL) > SenLRef && bypassLR == 0) {
        count++;
        holonomic(0, 20, 0);
        delay(100);
        holonomic(50, 20, 0);
        delay(100);
        holonomic(80, 20, 0);
        delay(100);
      } else if (analogRead(SensR) > SenRRef && bypassLR == 0) {
        count++;
        holonomic(0, 160, 0);
        delay(100);
        holonomic(50, 160, 0);
        delay(100);
        holonomic(80, 160, 0);
        delay(100);
      } else {
        AtanTrack2();
      }
    } else {
      bypassLR = 0;
      count = 0;
      // getIMU();
      // wheel(0, 0, 0);
      // getIMU();
      // while (analogRead(SensL) < SenLRef && analogRead(SensR) < SenRRef) {
      //   getIMU();
      //   holonomic(80, 270, 0);
      //   SetYaw();
      //   if (huskylens.updateBlocks() && huskylens.blockSize[1]) {
      //     break;
      //   }
      //   getIMU();
      // }
      int nubL, nubR, vecCurveV;
      loopTimer = millis();
      // while (millis() - loopTimer <= 3000) {
      while (!(huskylens.updateBlocks() && huskylens.blockSize[1])) {
        reload();
        if (analogRead(SensR) > SenRRef) {
          FoundRight = 1;
        }
        if (analogRead(SensL) > SenLRef) {
          FoundLeft = 1;
        }
        if (analogRead(SensC) > SenCRef) {
          FoundCent = 1;
        }
        // FoundLeft = (analogRead(SensL) > SenLRef) ? 1 : 0;
        // FoundRight = (analogRead(SensR) > SenRRef) ? 1 : 0;
        if (FoundRight == 1 && FoundLeft == 1) {
          // holonomic(0, 0, 0);
          // delay(50);
          FoundLeft = 0;
          FoundRight = 0;
          loopTimer = millis();
          while (millis() - loopTimer <= 800) {
            // vecCurveV = (huskylens.updateBlocks() && (huskylens.blockSize[2] || huskylens.blockSize[3])) ? 270 : 90;
            holonomic(80, 90, 0);
            if (analogRead(SensC) > SenCRef) {
              FoundCent = 1;
              // vecCurveV = 270;
              break;
            }
            if ((huskylens.updateBlocks() && huskylens.blockSize[1]) /*|| FoundCent == 1*/) {
              break;
            }
          }
          loopTimer = millis();
          vecCurveV = 90;
          while (millis() - loopTimer <= 1500) {
            getIMU();
            if (analogRead(SensL) > SenLRef) vecCurveV = 45;
            else if (analogRead(SensR) > SenRRef) vecCurveV = 135;
            // if (analogRead(SensL) > SenLRef) FoundCent == 1;

            heading(80, vecCurveV, 0);
            if ((huskylens.updateBlocks() && huskylens.blockSize[1]) /*|| FoundCent == 1*/) {
              break;
            }
          }
        }
        // else if (FoundCent == 1 && FoundRight == 1 && FoundLeft == 1) {
        //   // holonomic(0, 0, 0);
        //   // delay(50);
        //   FoundCent = 0;
        //   FoundLeft = 0;
        //   FoundRight = 0;
        //   loopTimer = millis();
        //   vecCurveV = 270;
        //   while (millis() - loopTimer <= 1000) {
        //     getIMU();
        //     if (analogRead(SensL) > SenLRef) vecCurveV = 315;
        //     else if (analogRead(SensR) > SenRRef) vecCurveV = 225;

        //     heading(80, vecCurveV, 0);
        //     if (huskylens.updateBlocks() && huskylens.blockSize[1]) {
        //       break;
        //     }
        //   }
        //   // holonomic(60, 90, 0);
        //   // delay(1000);
        // }
        else if (analogRead(SensC) > SenCRef || (FoundRight == 0 && FoundLeft == 0 && FoundCent == 1)) {
          // holonomic(0, 0, 0);
          // delay(50);
          FoundLeft = 0;
          FoundRight = 0;
          FoundCent = 0;
          getIMU();
          heading(80, 270, 0);
          // delay(500
        } else if (FoundLeft == 0 && analogRead(SensR) > SenRRef) {
          // holonomic(0, 0, 0);
          // delay(50);
          FoundRight = 1;
          // heading(80, 240, 0);
          holonomic(80, 240, 0);
          nubR = millis();
          while (millis() - nubR <= 80) {
            if (analogRead(SensL) > SenLRef) {
              heading(0, 0, 0);
              FoundLeft = 1;
              // beep();
            }
            if (huskylens.updateBlocks() && huskylens.blockSize[1]) {
              break;
            }
          }
          // delay(500);
        } else if (FoundRight == 0 && analogRead(SensL) > SenLRef) {
          // holonomic(0, 0, 0);
          // delay(50);
          FoundLeft = 1;
          // getIMU();
          // heading(80, 300, 0);
          holonomic(80, 300, 0);
          nubL = millis();
          while (millis() - nubL <= 80) {
            if (analogRead(SensR) > SenRRef) {
              heading(0, 0, 0);
              FoundRight = 1;
              // beep();
            }
            if (huskylens.updateBlocks() && huskylens.blockSize[1]) {
              break;
            }
          }
          // delay(500);
        } else if (analogRead(SensR) < SenRRef && analogRead(SensL) < SenLRef && analogRead(SensC) < SenCRef) {
          FoundLeft = 0;
          FoundRight = 0;
          FoundCent = 0;
          getIMU();
          heading(80, 270, 0);
        }
        if (abs(pvYaw) > 10) {
          SetYaw();
        }
        if (huskylens.updateBlocks() && huskylens.blockSize[1]) {
          break;
        }
        getIMU();
      }
    }
  }
}

// void TouchLine() {                   BUGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGG
//   while (1) {
//     reload();
//     int FoundLeft = 0, FoundRight = 0, FoundCent = 0;
//     if (huskylens.updateBlocks() && huskylens.blockSize[1]) {  // ball found
//       // lastYaw = pvYaw;
//       /*if (analogRead(SensC) > SenCRef) {
//         holonomic(0, 0, 0);
//         delay(100);
//         holonomic(50, 270, 0);
//         delay(100);
//         holonomic(80, 270, 0);
//         delay(200);
//       } else */
//       if (analogRead(SensL) > SenLRef) {
//         holonomic(0, 20, 0);
//         delay(100);
//         holonomic(50, 20, 0);
//         delay(100);
//         holonomic(80, 20, 0);
//         delay(100);
//       } else if (analogRead(SensR) > SenRRef) {
//         holonomic(0, 160, 0);
//         delay(100);
//         holonomic(50, 160, 0);
//         delay(100);
//         holonomic(80, 160, 0);
//         delay(100);
//       } else {
//         AtanTrack2();
//       }
//     } else {
//       // getIMU();
//       // wheel(0, 0, 0);
//       // getIMU();
//       // while (analogRead(SensL) < SenLRef && analogRead(SensR) < SenRRef) {
//       //   getIMU();
//       //   holonomic(80, 270, 0);
//       //   SetYaw();
//       //   if (huskylens.updateBlocks() && huskylens.blockSize[1]) {
//       //     break;
//       //   }
//       //   getIMU();
//       // }
//       int nubL, nubR, vecCurveV, countTo0_180;
//       loopTimer = millis();
//       // while (millis() - loopTimer <= 3000) {
//       while (!(huskylens.updateBlocks() && huskylens.blockSize[1])) {
//         reload();
//         if (analogRead(SensR) > SenRRef) {
//           FoundRight = 1;
//         }
//         if (analogRead(SensL) > SenLRef) {
//           FoundLeft = 1;
//         }
//         if (analogRead(SensC) > SenCRef) {
//           FoundCent = 1;
//         }
//         // FoundLeft = (analogRead(SensL) > SenLRef) ? 1 : 0;
//         // FoundRight = (analogRead(SensR) > SenRRef) ? 1 : 0;
//         if (FoundRight == 1 && FoundLeft == 1) {
//           // holonomic(0, 0, 0);
//           // delay(50);
//           FoundLeft = 0;
//           FoundRight = 0;
//           if (countTo0_180 == 2) {
//             countTo0_180++;
//             loopTimer = millis();
//             // delay(500);
//             while (millis() - loopTimer <= 500) {
//               holonomic(80, 90, 0);
//               if ((huskylens.updateBlocks() && huskylens.blockSize[1]) /*|| FoundCent == 1*/) {
//                 countTo0_180 = 0;
//                 break;
//               }
//               // if (analogRead(SensC) > SenCRef) {
//               //   FoundCent = 1;
//               //   vecCurveV = 270;
//               //   break;
//               // }
//             }
//             // while (FoundCent == 1 && millis() - loopTimer <= 500){
//             //   holonomic(80, 270, 0);
//             // }
//             if (lastGoalPos() == 1) vecCurveV = 0;
//             else vecCurveV = 180;
//             loopTimer = millis();
//             while (FoundCent == 0 && (millis() - loopTimer <= 3000)) {
//               if (analogRead(SensC) > SenCRef) {
//                 FoundCent = 1;
//                 vecCurveV = 270;
//                 break;
//               }
//               getIMU();
//               if (analogRead(SensL) > SenLRef) vecCurveV = 0;
//               else if (analogRead(SensR) > SenRRef) vecCurveV = 180;
//               // if (analogRead(SensL) > SenLRef) FoundCent == 1;

//               heading(80, vecCurveV, 0);
//               if ((huskylens.updateBlocks() && huskylens.blockSize[1]) /*|| FoundCent == 1*/) {
//                 countTo0_180 = 0;
//                 break;
//               }
//             }
//             if (countTo0_180 == 10) /*|| FoundCent == 1*/ {
//               countTo0_180 = 0;
//               // break;
//             }
//             FoundCent = 0;
//             // countTo0_180 = 0;
//           } else {
//             loopTimer = millis();
//             vecCurveV = 90;
//             // delay(500);
//             while (millis() - loopTimer <= 350) {
//               holonomic(80, 90, 0);
//               if (analogRead(SensC) > SenCRef) {
//                 FoundCent = 1;
//                 vecCurveV = 270;
//                 break;
//               }
//               if ((huskylens.updateBlocks() && huskylens.blockSize[1]) /*|| FoundCent == 1*/) {
//                 countTo0_180 = 0;
//                 break;
//               }
//             }
//             vecCurveV = 270;
//             while (FoundCent == 1 && millis() - loopTimer <= 350){
//               holonomic(80, vecCurveV, 0);

//               if ((huskylens.updateBlocks() && huskylens.blockSize[1]) /*|| FoundCent == 1*/) {
//                 countTo0_180 = 0;
//                 break;
//               }
//               if(analogRead(SensL) > SenLRef || analogRead(SensR) > SenRRef){
//                 vecCurveV = 90;
//               }
//               if(vecCurveV == 90 && analogRead(SensC) > SenCRef){
//                 loopTimer = millis();
//                 vecCurveV = 270;
//               }
//             }
//             loopTimer = millis();
//             while (FoundCent == 0 && (millis() - loopTimer <= 1000)) {
//               if (analogRead(SensC) > SenCRef) {
//                 FoundCent = 1;
//                 vecCurveV = 270;
//                 break;
//               }
//               getIMU();
//               if (analogRead(SensL) > SenLRef) vecCurveV = 45;
//               else if (analogRead(SensR) > SenRRef) vecCurveV = 135;
//               // if (analogRead(SensL) > SenLRef) FoundCent == 1;

//               heading(80, vecCurveV, 0);
//               if ((huskylens.updateBlocks() && huskylens.blockSize[1]) /*|| FoundCent == 1*/) {
//                 break;
//               }
//             }
//             countTo0_180++;
//             while (FoundCent == 1 && (millis() - loopTimer <= 500)) {
//               countTo0_180 = 0;
//               getIMU();
//               if (analogRead(SensL) > SenLRef) vecCurveV = 315;
//               else if (analogRead(SensR) > SenRRef) vecCurveV = 225;
//               // if (analogRead(SensL) > SenLRef) FoundCent == 1;

//               heading(80, vecCurveV, 0);
//               if ((huskylens.updateBlocks() && huskylens.blockSize[1]) /*|| FoundCent == 1*/) {
//                 break;
//               }
//             }
//             FoundCent = 0;
//           }
//           // holonomic(60, 90, 0);
//           // delay(1000);
//         }
//         // else if (FoundCent == 1 && FoundRight == 1 && FoundLeft == 1) {
//         //   // holonomic(0, 0, 0);
//         //   // delay(50);
//         //   FoundCent = 0;
//         //   FoundLeft = 0;
//         //   FoundRight = 0;
//         //   loopTimer = millis();
//         //   vecCurveV = 270;
//         //   while (millis() - loopTimer <= 1000) {
//         //     getIMU();
//         //     if (analogRead(SensL) > SenLRef) vecCurveV = 315;
//         //     else if (analogRead(SensR) > SenRRef) vecCurveV = 225;

//         //     heading(80, vecCurveV, 0);
//         //     if (huskylens.updateBlocks() && huskylens.blockSize[1]) {
//         //       break;
//         //     }
//         //   }
//         //   // holonomic(60, 90, 0);
//         //   // delay(1000);
//         // }
//         else if (analogRead(SensC) > SenCRef || (FoundRight == 0 && FoundLeft == 0 && FoundCent == 1)) {
//           // holonomic(0, 0, 0);
//           // delay(50);
//           FoundLeft = 0;
//           FoundRight = 0;
//           FoundCent = 0;
//           getIMU();
//           heading(80, 270, 0);
//           // delay(500
//         } else if (FoundLeft == 0 && analogRead(SensR) > SenRRef) {
//           // holonomic(0, 0, 0);
//           // delay(50);
//           FoundRight = 1;
//           // heading(80, 240, 0);
//           holonomic(80, 240, 0);
//           nubR = millis();
//           while (millis() - nubR <= 80) {
//             if (analogRead(SensL) > SenLRef) {
//               heading(0, 0, 0);
//               FoundLeft = 1;
//               // beep();
//             }
//             if (huskylens.updateBlocks() && huskylens.blockSize[1]) {
//               break;
//             }
//           }
//           // delay(500);
//         } else if (FoundRight == 0 && analogRead(SensL) > SenLRef) {
//           // holonomic(0, 0, 0);
//           // delay(50);
//           FoundLeft = 1;
//           // getIMU();
//           // heading(80, 300, 0);
//           holonomic(80, 300, 0);
//           nubL = millis();
//           while (millis() - nubL <= 80) {
//             if (analogRead(SensR) > SenRRef) {
//               heading(0, 0, 0);
//               FoundRight = 1;
//               // beep();
//             }
//             if (huskylens.updateBlocks() && huskylens.blockSize[1]) {
//               break;
//             }
//           }
//           // delay(500);
//         } else if (analogRead(SensR) < SenRRef && analogRead(SensL) < SenLRef && analogRead(SensC) < SenCRef) {
//           FoundLeft = 0;
//           FoundRight = 0;
//           FoundCent = 0;
//           getIMU();
//           heading(80, 270, 0);
//         }
//         if (abs(pvYaw) > 10) {
//           SetYaw();
//         }
//         if (huskylens.updateBlocks() && huskylens.blockSize[1]) {
//           break;
//         }
//         getIMU();
//       }
//     }
//   }
// }