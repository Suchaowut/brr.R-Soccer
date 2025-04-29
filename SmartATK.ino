bool goalDetected = false;
float goalPosX, goalPosY, lastGoalAngle = 90;

// อ่านตำแหน่ง Goal แล้วเก็บในประวัติ
// void ReadgoalPos() {
//   if (huskylens.updateBlocks() && huskylens.blockSize[1] && huskylens.blockSize[3]) {
//     float goalPosX, goalPosY;

//     if (huskylens.blockSize[2]) {
//       goalPosX = huskylens.blockInfo[2][0].x;
//       goalPosY = huskylens.blockInfo[2][0].y;
//     } else if (huskylens.blockSize[3]) {
//       goalPosX = huskylens.blockInfo[3][0].x;
//       goalPosY = huskylens.blockInfo[3][0].y;
//     }
//     float QuaDrantX = goalPosX - 150;
//     float QuaDrantY = 180 - goalPosY;
//     float TanTheta = QuaDrantY / QuaDrantX;
//     float SethaPos;

//     if (QuaDrantX >= 0 && QuaDrantY >= 0) {
//       SethaPos = atan(TanTheta) * (180.0 / PI);
//     } else if (QuaDrantX < 0 && QuaDrantY >= 0) {
//       SethaPos = 180 + atan(TanTheta) * (180.0 / PI);
//     } else {
//       SethaPos = 90;
//     }

//     goalXHistory[goalHistoryIndex] = goalPosX;
//     goalYHistory[goalHistoryIndex] = goalPosY;
//     goalAngleHistory[goalHistoryIndex] = SethaPos;
//     goalHistoryIndex = (goalHistoryIndex + 1) % GOAL_HISTORY_SIZE;

//     goalDetected = true;
//     lastGoalSeenTime = millis();
//   }
// }

void ReadgoalPos() {
  if (huskylens.updateBlocks() && huskylens.blockSize[1]) {
    // อ่านตำแหน่งโกล id 2 หรือ 3 ก็ได้
    if (huskylens.blockSize[2]) {
      goalPosX = huskylens.blockInfo[2][0].x;
      goalPosY = huskylens.blockInfo[2][0].y;
    } else if (huskylens.blockSize[3]) {
      goalPosX = huskylens.blockInfo[3][0].x;
      goalPosY = huskylens.blockInfo[3][0].y;
    }

    float QuaDrantX = goalPosX - 150;
    float QuaDrantY = 180 - goalPosY;
    float TanTheta = QuaDrantY / QuaDrantX;
    float SethaPos;

    if (QuaDrantX >= 0 && QuaDrantY >= 0) {
      SethaPos = atan(TanTheta) * (180.0 / PI);
    } else if (QuaDrantX < 0 && QuaDrantY >= 0) {
      SethaPos = 180 + atan(TanTheta) * (180.0 / PI);
    } else {
      SethaPos = 90;
    }

    // บันทึกค่า setha ที่ได้จากโกลเมื่อเจอล่าซู้ด
    lastGoalAngle = SethaPos;
  }
}

float getLatestGoalWidth() {
  if (huskylens.updateBlocks() && huskylens.blockSize[3]) {
    return huskylens.blockInfo[3][0].width;
  } else if (huskylens.updateBlocks() && huskylens.blockSize[2]) {
    return huskylens.blockInfo[2][0].width;
  }
  return 0;
}

float calculateSoftApproachSpeed(float currentValue, float setpoint, float maxSpeed, float minSpeed, float slowDownRange) {
  float error = abs(currentValue - setpoint);

  if (error > slowDownRange) {
    // ไกลจาก setpoint มาก → ใช้ความเร็วสูงสุด
    return maxSpeed;
  } else {
    // เข้าใกล้ setpoint → ค่อยๆลดความเร็ว
    float speed = minSpeed + (maxSpeed - minSpeed) * (error / slowDownRange);
    return constrain(speed, minSpeed, maxSpeed);
  }
}

// int vecCurve;
void Runto() {
  int vecCurve;
  ReadgoalPos();
  if (huskylens.updateBlocks() && huskylens.blockSize[1]) {
    ballPosX = huskylens.blockInfo[1][0].x;
    ballPosY = huskylens.blockInfo[1][0].y;
    float QuaDrantX = huskylens.blockInfo[1][0].x - 160;
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

    // ----------- ตรงนี้เริ่มควบคุมเลี้ยงบอล --------------
    Yaxis_Error = 160 - ballPosY;
    Yaxis_D = Yaxis_Error - Yaxis_PvEror;
    Yaxis_spd = (Yaxis_Error * Yaxis_Kp) + (Yaxis_D * Yaxis_Kd);
    Yaxis_spd = constrain(Yaxis_spd, -100, 100);
    Yaxis_PvEror = Yaxis_Error;

    holonomic(Yaxis_spd, SethaPos, 0);

    if (Yaxis_Error <= 20) {
      getIMU();
      if (abs(pvYaw) <= 8) {
        TrackXaxis2();
        if (abs(Xaxis_Error) <= 8) {
          Dribbler();
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

void Dribbler() {
  long NoGoalTime; bool reverse;
  if (!(huskylens.updateBlocks() && huskylens.blockSize[3] || huskylens.blockSize[2])) {
    NoGoalTime = millis();
    reverse = false;
  }
  while (huskylens.updateBlocks() && huskylens.blockSize[1]) {
    ReadgoalPos();
    ballPosX = huskylens.blockInfo[1][0].x;
    ballPosY = huskylens.blockInfo[1][0].y;
    rot_error = 160 - ballPosX;
    
    // Smooth error ด้วยค่าเฉลี่ยเคลื่อนที่
    float smooth_rot_error = (rot_error * 0.4 + rot_pError * 0.6);
    rot_d = smooth_rot_error - rot_pError;
    rot_pError = smooth_rot_error;
    
    if (abs(rot_error) < 5) {
      rot_w = 0;  // หยุดหมุนเมื่อใกล้ศูนย์
    } else if (abs(rot_error) < 15) {
      rot_w = constrain(rot_error * 0.3 + rot_d * 0.05, -40, 40);  // หมุนนิ่ม
    } else {
      rot_w = constrain(rot_error * rot_Kp + rot_d * rot_Kd, -100, 100);  // ปกติ
    }

    Yaxis_Error = 160 - ballPosY;
    Yaxis_D = Yaxis_Error - Yaxis_PvEror;
    Yaxis_spd = (Yaxis_Error * Yaxis_Kp) + (Yaxis_D * Yaxis_Kd);
    Yaxis_spd = constrain(Yaxis_spd, -100, 100);
    Yaxis_PvEror = Yaxis_Error;

    float goalX = goalPosX;
    float goalWidth = getLatestGoalWidth();

    float goalLeft = goalX - goalWidth / 2;
    float goalRight = goalX + goalWidth / 2;

    float theta = 90;

    if (lastGoalAngle > 105) {
      theta = 180;
    } else if (lastGoalAngle < 75) {
      theta = 0;
    } else {
      theta = 90;
    }

    if (!(huskylens.updateBlocks() && huskylens.blockSize[3] || huskylens.blockSize[2])) {
      if (millis() - NoGoalTime >= 600) {
        if (theta > 90 && reverse == false) {
          theta = 180;
          reverse = true;
        } else if (theta < 90 && reverse == false) {
          theta = 0;
          reverse = true;
        }
      }
      if (millis() - NoGoalTime >= 2000) break;
    }

    float goalHalfWidth = (goalRight - goalLeft) / 2.0;

    // เงื่อนไข: ถ้าบอลอยู่ภายในโกล และอยู่ใกล้ศูนย์กลางโกลพอสมควร
    if (abs(ballPosX - goalX) <= goalHalfWidth * 0.6) {  // 0.6 = เกินครึ่งนิดหน่อย (60%)
      holonomic(0, 0, 0);
      beep();
      shootlana();
      break;
    }

    float xSpeed = calculateSoftApproachSpeed(ballPosX, (goalLeft + goalRight) / 2, 80, 40, 15);

    // === ตัดสินใจเคลื่อนที่ ===
    if (Yaxis_Error > 15) {
      holonomic(Yaxis_spd, 90, rot_w);  // ยังหาบอลอยู่
    } else if (abs(rot_error) < 15) {
      holonomic(xSpeed, theta, 0);  // วิ่งเข้าโกลตรงๆ
    } else {
      holonomic(xSpeed, theta, rot_w);  // เลี้ยงลูกพร้อมหมุนเข้าโกล
    }
  }
}

void SmartATK() {
  while (1) {
    ReadgoalPos();
    int FoundLeft = 0, FoundRight = 0, FoundCent = 0;
    if (huskylens.updateBlocks() && huskylens.blockSize[1]) {
      /*if (analogRead(SensC) > SenCRef) {
        holonomic(0, 0, 0);
        delay(100);
        holonomic(50, 270, 0);
        delay(100);
        holonomic(80, 270, 0);
        delay(200);
      } else */
      if (analogRead(SensL) > SenLRef) {
        holonomic(0, 20, 0);
        delay(100);
        holonomic(50, 20, 0);
        delay(100);
        holonomic(80, 20, 0);
        delay(100);
      } else if (analogRead(SensR) > SenRRef) {
        holonomic(0, 160, 0);
        delay(100);
        holonomic(50, 160, 0);
        delay(100);
        holonomic(80, 160, 0);
        delay(100);
      } else {
        Runto();
      }
    } else {
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
      while (1) {
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
          holonomic(80, 90, 0);
          vecCurveV = 90;
          while (millis() - loopTimer <= 1500) {
            getIMU();
            if (analogRead(SensL) > SenLRef && analogRead(SensR) > SenRRef) vecCurveV = 90;
            else if (analogRead(SensL) > SenLRef) vecCurveV = 45;
            else if (analogRead(SensR) > SenRRef) vecCurveV = 135;

            heading(80, vecCurveV, 0);
            if ((huskylens.updateBlocks() && huskylens.blockSize[1]) ) {
              break;
            }
            if (analogRead(SensC) > SenCRef) {
              FoundCent = 1;
              break;
            }
          }
          // holonomic(60, 90, 0);
          // delay(1000);
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
        else if (analogRead(SensC) > SenCRef || FoundCent == 1) {
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
