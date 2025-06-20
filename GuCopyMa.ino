int vecCurve;
void AtanTrack3() {
  if ((huskylens.updateBlocks() && huskylens.blockSize[1])) {
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
    // SetYaw();
    Yaxis_Error = 160 - huskylens.blockInfo[1][0].y;
    Yaxis_D = Yaxis_Error - Yaxis_PvEror;
    Yaxis_spd = (Yaxis_Error * Yaxis_Kp) + (Yaxis_D * Yaxis_Kd);
    Yaxis_spd = constrain(Yaxis_spd, -100, 100);
    Yaxis_PvEror = Yaxis_Error;

    getIMU();
    holonomic(Yaxis_spd, SethaPos, 0);

    float goalEstX, goalEstY, goalEstWidth;
    if (huskylens.blockSize[2]) {
      goalEstX = huskylens.blockInfo[2][0].x;
      goalEstY = huskylens.blockInfo[2][0].y;
      goalEstWidth = huskylens.blockInfo[2][0].width;
    } else if (huskylens.blockSize[3]) {
      goalEstX = huskylens.blockInfo[3][0].x;
      goalEstY = huskylens.blockInfo[3][0].y;
      goalEstWidth = huskylens.blockInfo[3][0].width;
    }
    float goalLeft = goalEstX - goalEstWidth / 2.0;
    float goalRight = goalEstX + goalEstWidth / 2.0;
    bool ballInGoalArea = false;
    if (goalEstWidth > 0) {
      ballInGoalArea = (ballPosX >= goalLeft && ballPosX <= goalRight);
    }
    if (Yaxis_Error <= 20) {
      getIMU();
      lastYaw = pvYaw;
      if (abs(pvYaw) <= 10 || ballInGoalArea) {  //(huskylens.updateBlocks() && huskylens.blockSize[2] && huskylens.blockSize[3])) {
        TrackXaxis2();
        if (abs(Xaxis_Error) <= 10) {
          wheel(0, 0, 0);
          // SetYaw();
          getIMU();
          // beep();
          // lastYaw = pvYaw;
          // bypassYaw = 0;
          // beep();
          Bumping();
        }
      } else if ((abs(pvYaw) > 10) && (huskylens.updateBlocks() && huskylens.blockSize[1])) {
        // if (huskylens.updateBlocks() && huskylens.blockSize[2] && huskylens.blockSize[3]) bypassYaw = 1;
        while ((huskylens.updateBlocks() && huskylens.blockSize[1]) && Yaxis_Error <= 20) {
          getIMU();
          rot_error = 155 - huskylens.blockInfo[1][0].x;
          rot_d = rot_error - rot_pError;
          rot_pError = rot_error;
          rot_w = (rot_error * 0.45) + (rot_i * 0.05) + (rot_d * 0.8);
          rot_w = constrain(rot_w, -70, 70);

          Yaxis_Error = 150 - huskylens.blockInfo[1][0].y;
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
          if (abs(pvYaw) <= 10) {
            // bypassYaw = 1;
            break;
          }
        }
      }
    }
  }
}

// front
// int count, bypassLR, bypassC, CountC;     // sensor

void Bumping() {
  float angleToGoal = 90;
  int previousAngle = 90;
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
    float QuaDrantX = huskylens.blockInfo[1][0].x - 150;
    float QuaDrantY = 180 - huskylens.blockInfo[1][0].y;
    float TanTheta = QuaDrantY / QuaDrantX;
    float Setha = atan(TanTheta) * (180 / PI);
    float SethaPos;
    if (Setha >= 0) {
      SethaPos = Setha;
    } else if (Setha < 0) {
      SethaPos = 180 + Setha;
    }

    // ตรวจสอบโกลซ้ายหรือขวา
    if (huskylens.blockSize[2]) {
      goalEstX = huskylens.blockInfo[2][0].x;
      goalEstY = huskylens.blockInfo[2][0].y;
      goalEstWidth = huskylens.blockInfo[2][0].width;
    } else if (huskylens.blockSize[3]) {
      goalEstX = huskylens.blockInfo[3][0].x;
      goalEstY = huskylens.blockInfo[3][0].y;
      goalEstWidth = huskylens.blockInfo[3][0].width;
    }

    float goalLeft = goalEstX - goalEstWidth / 2.0;
    float goalRight = goalEstX + goalEstWidth / 2.0;
    float centerGoalX = goalEstX;
    float diffFromCenter = ballPosX - centerGoalX;

    float targetAngle = 90;
    if (abs(diffFromCenter) >= 10) {
      float maxAngleOffset = 25;
      float maxOffset = goalEstWidth / 2.0;
      float offsetRatio = constrain(diffFromCenter / maxOffset, -1.0, 1.0);
      targetAngle = 90 + offsetRatio * maxAngleOffset;
    }

    if (ballPosY < 200) {
      targetAngle = 90;
    }

    angleToGoal = previousAngle * 0.7 + targetAngle * 0.3;
    previousAngle = angleToGoal;
    int lastangleToGoal = angleToGoal;

    if (ballPosY < 205) {
      angleToGoal = 90;
    }
    if (analogRead(SensL) > SenLRef) angleToGoal = 70;
    else if (analogRead(SensR) > SenRRef) angleToGoal = 120;

    bool ballInGoalArea = false;
    if (goalEstWidth > 0) {
      ballInGoalArea = (ballPosX >= goalLeft && ballPosX <= goalRight);
    }

    if ((ballPosX > 120 && ballPosX < 180) && ballPosY > 205) heading(100, angleToGoal, 0);
    /*else if ((ballPosX > 150 && ballPosX < 200) && !ballInGoalArea) holonomic(100, angleToGoal, 0);*/
    else heading(100, 90, 0);

    // else holonomic(100, angleToGoal, 0);
    // if(ballPosY < 210) heading(100, angleToGoal, 0);

    // if (/*goalEstY > 30 && */ballPosY > 230 && ((analogRead(SensC) > SenCRef && ballInGoalArea) /*|| (ballPosX >= goalLeft + 10 && ballPosX <= goalRight - 10)*/)) {
    if (ballPosY > 205 && (analogRead(SensC) > SenCRef /*&& (ballPosX >= goalLeft + 10 && ballPosX <= goalRight - 10)*/) /*&& ballInGoalArea*/) {
      shoot();
      holonomic(0, 0, 0);
      delay(50);
      holonomic(100, 270, 0);
      delay(800);
      holonomic(0, 0, 0);
      reload();
      delay(50);
      holonomic(0, 0, 0);
      break;
      // delay(1000);
    }
  }
}

void Bigbump() {
  while (1) {
    if (analogRead(limPin) < 700) reload();
    int FoundLeft = 0, FoundRight = 0, FoundCent = 0;
    if (huskylens.updateBlocks() && huskylens.blockSize[1]) {  // ball found
      // lastYaw = pvYaw;


      if (count >= 2) bypassLR = 1;
      if (countC >= 1) bypassC = 1;

      // if(count == 2) AtanTrack3();

      if (analogRead(SensC) > SenCRef && (huskylens.updateBlocks() && !(huskylens.blockSize[2] || huskylens.blockSize[3])) && bypassC == 0) {
        holonomic(0, 0, 0);
        delay(100);
        holonomic(50, 270, 0);
        delay(100);
        holonomic(80, 270, 0);
        delay(100);
        countC++;

      } else if (analogRead(SensL) > SenLRef && bypassLR == 0) {
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
        AtanTrack3();
      }
    } else {
      bypassLR = 0;
      count = 0;
      bypassC = 0;
      countC = 0;

      // state 0 main   state 1
      int goalEstX, goalEstY, goalEstWidth;
      int nubL, nubR, vecCurveV;
      int FoundLeft = 0, FoundRight = 0, FoundCent = 0;
      int state = 0;
      int state3Count = 0;
      int lastVecCurveV = 5;
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
        if (FoundRight == 1 && FoundLeft == 1) {

          FoundLeft = 0;
          FoundRight = 0;
          // count++;
          loopTimer = millis();
          while (millis() - loopTimer <= 550) {
            getIMU();
            // if (analogRead(SensL) > SenLRef) FoundCent == 1;
            heading(80, 90, 0);
            if ((huskylens.updateBlocks() && huskylens.blockSize[1]) /*|| FoundCent == 1*/) {
              break;
            }
          }
          // if (count == 2) {
          //   count = 0;
          //   state = 6;
          // }
          // if (count > 2) {
          //   loopTimer = millis();
          //   while (millis() - loopTimer <= 550) {
          //     getIMU();
          //     // if (analogRead(SensL) > SenLRef) FoundCent == 1;
          //     heading(100, 90, 0);
          //     if ((huskylens.updateBlocks() && huskylens.blockSize[1]) /*|| FoundCent == 1*/) {
          //       break;
          //     }
          //   }
          // }
          // else state = 1;
        } else if (analogRead(SensL) > SenLRef && analogRead(SensR) < SenRRef && FoundRight == 0) {
          FoundLeft = 1;
          // getIMU();
          holonomic(100, 315, 0);
          nubL = millis();
          while (millis() - nubL <= 120) {
            if (analogRead(SensR) > SenRRef) {
              holonomic(0, 0, 0);
              FoundRight = 1;
              break;
            }
          }
          // state = 1;
          //break;
        } else if (analogRead(SensL) < SenLRef && analogRead(SensR) > SenRRef && FoundLeft == 0) {
          FoundRight = 1;
          // getIMU();
          holonomic(100, 225, 0);
          nubR = millis();
          while (millis() - nubR <= 150) {
            if (analogRead(SensL) > SenLRef) {
              // getIMU();
              holonomic(0, 0, 0);
              FoundLeft = 1;
              break;
            }
            if (huskylens.updateBlocks() && huskylens.blockSize[1]) {
              break;
            }
          }
          // state = 1;
          //break;
        } else if (analogRead(SensR) < SenRRef && analogRead(SensL) < SenLRef && analogRead(SensC) < SenCRef) {
          FoundLeft = 0;
          FoundRight = 0;
          FoundCent = 0;
          if ((huskylens.blockSize[2] && huskylens.blockSize[3]) || !(huskylens.blockSize[2] || huskylens.blockSize[3])) {
            goalEstX = 150;
          } else if (huskylens.blockSize[2]) {
            goalEstX = huskylens.blockInfo[2][0].x;
            goalEstY = huskylens.blockInfo[2][0].y;
            goalEstWidth = huskylens.blockInfo[2][0].width;
          } else if (huskylens.blockSize[3]) {
            goalEstX = huskylens.blockInfo[3][0].x;
            goalEstY = huskylens.blockInfo[3][0].y;
            goalEstWidth = huskylens.blockInfo[3][0].width;
          }
          if (!(huskylens.blockSize[2] || huskylens.blockSize[3])) {
            vecCurveV = 270;
          } else if (goalEstX >= 150 + 60) {
            vecCurveV = 315;
          } else if (goalEstX <= 150 - 60) {
            vecCurveV = 225;
          } else {
            vecCurveV = 270;
          }
          getIMU();
          heading(100, vecCurveV, 0);
        }
        if (huskylens.updateBlocks() && huskylens.blockSize[1]) {
          break;
        }

        // if (abs(pvYaw) > 15) {
        //   SetYaw();
        // }


        if (state == 1) {
          vecCurveV = 90;
          getIMU();
          heading(100, 240, 0);

          if (huskylens.updateBlocks() && huskylens.blockSize[1]) {
            break;
          }
          loopTimer = millis();
          while (millis() - loopTimer <= 1100) {
            if (analogRead(SensL) > SenLRef && (millis() - loopTimer >= 350)) vecCurveV = 25;
            else if (analogRead(SensR) > SenRRef && (millis() - loopTimer >= 350)) vecCurveV = 155;
            // if (analogRead(SensL) > SenLRef) FoundCent == 1;
            getIMU();
            heading(100, vecCurveV, 0);
            if ((huskylens.updateBlocks() && huskylens.blockSize[1]) /*|| FoundCent == 1*/) {
              break;
            }
          }
          if (vecCurveV == 90) count++;
          else count = 0;
          state = 0;
        } else if (state == 2) {  // ส่ายหน้า
          int Yor = 0;
          int targetyor = 60;
          count = 0;
          // beep();
          holonomic(0, 0, 0);
          // if (analogRead(SensL) > SenLRef || analogRead(SensR) > SenRRef || analogRead(SensC) > SenCRef) {
          //   holonomic(0, 0, 0);
          //   state = 0;
          //   break;
          // }

          loopTimer = millis();
          while (millis() - loopTimer <= 2000) {
            getIMU();
            if (!(huskylens.updateBlocks() && huskylens.blockSize[1])) {
              for (Yor = pvYaw; pvYaw <= targetyor; Yor++) {
                if (huskylens.updateBlocks() && huskylens.blockSize[1]) {
                  holonomic(0, 0, 0);
                  break;
                }
                getIMU();
                heading(0, 0, Yor);
              }
            }

            // holonomic(0, 0, 0);
            // while (abs(pvYaw) > 11 && huskylens.updateBlocks() && !(huskylens.blockSize[1])) heading(0, 0, 0);

            if (!(huskylens.updateBlocks() && huskylens.blockSize[1])) {
              for (Yor = pvYaw; pvYaw >= -targetyor; Yor--) {
                if (huskylens.updateBlocks() && huskylens.blockSize[1]) {
                  holonomic(0, 0, 0);
                  break;
                }
                getIMU();
                heading(0, 0, Yor);
              }
            }

            holonomic(0, 0, 0);
            // while (abs(pvYaw) > 11 && huskylens.updateBlocks() && !(huskylens.blockSize[1])) heading(0, 0, 0);
            // if (!(huskylens.updateBlocks() && huskylens.blockSize[1])) {
            //   for (Yor = pvYaw; pvYaw >= 0; Yor--) {
            //     if (huskylens.updateBlocks() && huskylens.blockSize[1]) {
            //       holonomic(0, 0, 0);
            //       break;
            //     }
            //     getIMU();
            //     heading(0, 0, Yor);
            //   }
            // }

            if (analogRead(SensL) > SenLRef || analogRead(SensR) > SenRRef || analogRead(SensC) > SenCRef || (huskylens.updateBlocks() && huskylens.blockSize[1])) {
              holonomic(0, 0, 0);
              state = 0;
              break;
            }
          }
          while (abs(pvYaw) > 11 && huskylens.updateBlocks() && !(huskylens.blockSize[1])) heading(0, 0, 0);
          state = 3;
        } else if (state == 4) {
          count = 0;
          // beep();
          // holonomic(0, 0, 0);
          // if (analogRead(SensL) > SenLRef || analogRead(SensR) > SenRRef || analogRead(SensC) > SenCRef) {
          //   holonomic(0, 0, 0);
          //   state = 0;
          //   break;
          // }
          // loopTimer = millis();
          // while (millis() - loopTimer <= 2000) {
          loopTimer = millis();
          while (millis() - loopTimer <= 150) {
            getIMU();
            heading(80, 180, 0);

            if (analogRead(SensL) > SenLRef || analogRead(SensR) > SenRRef || analogRead(SensC) > SenCRef || (huskylens.updateBlocks() && huskylens.blockSize[1])) {
              if (analogRead(SensL) > SenLRef) holonomic(80, 0, 0), delay(200);
              else if (analogRead(SensR) > SenRRef) holonomic(80, 180, 0), delay(200);
              else if (analogRead(SensC) > SenCRef) holonomic(80, 270, 0), delay(200);
              holonomic(0, 0, 0);
              state = 0;
              break;
            }
          }

          loopTimer = millis();
          while (state == 3 && millis() - loopTimer <= 900) {
            getIMU();
            heading(80, 0, 0);

            if (analogRead(SensL) > SenLRef || analogRead(SensR) > SenRRef || analogRead(SensC) > SenCRef) {
              if (analogRead(SensL) > SenLRef) holonomic(80, 0, 0), delay(200);
              else if (analogRead(SensR) > SenRRef) holonomic(80, 180, 0), delay(200);
              else if (analogRead(SensC) > SenCRef) holonomic(80, 270, 0), delay(200);
              holonomic(0, 0, 0);
              state = 0;
              break;
            }
          }
          loopTimer = millis();
          while (state == 3 && millis() - loopTimer <= 100) {
            getIMU();
            heading(80, 180, 0);

            if (analogRead(SensL) > SenLRef || analogRead(SensR) > SenRRef || analogRead(SensC) > SenCRef) {
              if (analogRead(SensL) > SenLRef) holonomic(80, 0, 0), delay(200);
              else if (analogRead(SensR) > SenRRef) holonomic(80, 180, 0), delay(200);
              else if (analogRead(SensC) > SenCRef) holonomic(80, 270, 0), delay(200);
              holonomic(0, 0, 0);
              state = 0;
              break;
            }
          }
        } else if (state == 3) {
          count = 0;
          long startTime = millis();
          loopTimer = millis();

          // เลือกทิศทาง vecCurveV ตามเงื่อนไขที่ผู้ใช้ต้องการ
          if (state3Count == 0) {
            vecCurveV = (random(0, 1000) < 500) ? 5 : 175;  // สุ่มครั้งแรก
          } else if (state3Count == 1) {
            vecCurveV = (lastVecCurveV == 5) ? 175 : 5;  // กลับทิศจากครั้งที่แล้ว
            state3Count = 0;
          } else {
            vecCurveV = (random(0, 1000) < 500) ? 5 : 175;
          }

          lastVecCurveV = vecCurveV;  // บันทึกค่าล่าสุด
          state3Count++;              // เพิ่มจำนวนครั้งที่เข้า state 3
          unsigned long changeDirDelay = random(800, 1000);
          unsigned long lastSwitchTime = millis();
          bool wentLeft = (vecCurveV == 5);
          int switchCount = 0;
          // เพิ่มตัวแปรเพื่อติดตามเวลาเจอเส้นล่าสุด
          unsigned long lastLeftDetected = 0;
          unsigned long lastRightDetected = 0;


          while (millis() - loopTimer <= 3000 && !(huskylens.updateBlocks() && huskylens.blockSize[1])) {
            getIMU();
            heading(100, vecCurveV, 0);

            int sensorL = analogRead(SensL);
            int sensorR = analogRead(SensR);
            unsigned long currentTime = millis();

            bool switched = false;

            if (sensorL > SenLRef && sensorR < SenRRef) {  // เจอเส้นซ้าย
              if (!wentLeft && millis() - lastRightDetected <= 400) {
                changeDirDelay += 250;  // เพิ่ม delay หากเพิ่งเจอเส้นขวาใน 100 ms ที่แล้ว
              }
              lastLeftDetected = millis();

              if (!wentLeft && currentTime - lastSwitchTime <= 500) {
                switchCount++;
              }
              vecCurveV = 5;
              wentLeft = true;
              lastSwitchTime = currentTime;
            }

            else if (sensorL < SenLRef && sensorR > SenRRef) {  // เจอเส้นขวา
              if (wentLeft && millis() - lastLeftDetected <= 400) {
                changeDirDelay += 250;  // เพิ่ม delay หากเพิ่งเจอเส้นซ้ายใน 100 ms ที่แล้ว
              }
              lastRightDetected = millis();

              if (wentLeft && currentTime - lastSwitchTime <= 500) {
                switchCount++;
              }
              vecCurveV = 175;
              wentLeft = false;
              lastSwitchTime = currentTime;
            }


            // หากสลับฝั่งเร็วไปมาหลายครั้ง → เด้งมุมสนาม → เดินหน้าหลบ
            if (switchCount >= 2) {
              unsigned long forwardStart = millis();
              while (millis() - forwardStart <= 500) {
                getIMU();
                heading(100, 90, 0);
              }

              // reset ทุกอย่างหลังหลบมุม
              switchCount = 0;
              vecCurveV = (random(0, 1000) <= 500) ? 5 : 175;
              wentLeft = (vecCurveV == 5);
              lastSwitchTime = millis();
              changeDirDelay = random(800, 1000);
            }


            if (currentTime - lastSwitchTime >= changeDirDelay) {
              vecCurveV = (vecCurveV == 5) ? 175 : 5;
              wentLeft = !wentLeft;
              lastSwitchTime = currentTime;
              changeDirDelay = random(800, 1000);
            }
          }

          state = 0;
        } else if (state == 5) {
          loopTimer = millis();
          while (!(huskylens.updateBlocks() && huskylens.blockSize[1]) && millis() - loopTimer <= 400) {
            getIMU();
            heading(80, 270, 0);
          }
        } /*else if (state == 6) {  // ส่ายหน้าใช้ความเร็ว
          int ballFounds = false;
          loopTimer = millis();
          if (huskylens.updateBlocks() && huskylens.blockSize[1]) ballFounds = true;
          while (millis() - loopTimer < 900) {
            holonomic(0, 90, 20);
            if (huskylens.updateBlocks() && huskylens.blockSize[1]) {
              ballFounds = true;
              break;
            }
          }
          getIMU();
          loopTimer = millis();
          while (ballFounds == false && abs(pvYaw) > 5 && millis() - loopTimer < 600) {
            getIMU();
            heading(0, 0, 0);
            if (huskylens.updateBlocks() && huskylens.blockSize[1]) {
              ballFounds = true;
              break;
            }
          }
          loopTimer = millis();
          while (ballFounds == false && millis() - loopTimer < 1700) {
            holonomic(0, 90, -20);
            if (huskylens.updateBlocks() && huskylens.blockSize[1]) {
              ballFounds = true;
              break;
            }
          }
          loopTimer = millis();
          while (ballFounds == false && millis() - loopTimer < 400) {
            holonomic(0, 90, -20);
            if (huskylens.updateBlocks() && huskylens.blockSize[1]) {
              ballFounds = true;
              break;
            }
          }
          loopTimer = millis();
          getIMU();
          while (ballFounds == false && abs(pvYaw) > 5 && millis() - loopTimer < 1000) {
            getIMU();
            heading(0, 0, 0);
            if (huskylens.updateBlocks() && huskylens.blockSize[1]) {
              ballFounds = true;
              break;
            }
          }
          // loopTimer = millis();
          // while (ballFounds == false && loopTimer - millis() <= 600) {
          //   getIMU();
          //   heading(100, 90, 0);
          //   if (huskylens.updateBlocks() && huskylens.blockSize[1]) {
          //     ballFounds = true;
          //     break;
          //   }
          // }
          state = 0;
        }*/
      }
    }
  }
}

void FindTurnBigbump() {
  while (1) {
    if (analogRead(limPin) < 700) reload();
    int speedState;
    int FoundLeft = 0, FoundRight = 0, FoundCent = 0;
    if (huskylens.updateBlocks() && huskylens.blockSize[1]) {  // ball found
      // lastYaw = pvYaw;
      count = 0;
      speedState = 100;


      if (count >= 2) bypassLR = 1;
      if (countC >= 1) bypassC = 1;

      // if(count == 2) AtanTrack3();

      if (analogRead(SensC) > SenCRef && (huskylens.updateBlocks() && !(huskylens.blockSize[2] || huskylens.blockSize[3])) && bypassC == 0) {
        holonomic(0, 0, 0);
        delay(100);
        holonomic(50, 270, 0);
        delay(100);
        holonomic(80, 270, 0);
        delay(100);
        countC++;

      } else if (analogRead(SensL) > SenLRef && bypassLR == 0) {
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
        AtanTrack3();
      }
    } else {
      bypassLR = 0;

      bypassC = 0;
      countC = 0;

      // state 0 main   state 1
      int goalEstX, goalEstY, goalEstWidth;
      int nubL, nubR, vecCurveV;
      int FoundLeft = 0, FoundRight = 0, FoundCent = 0;
      int state = 0;
      int state3Count = 0;
      int lastVecCurveV = 5;
      while (!(huskylens.updateBlocks() && huskylens.blockSize[1])) {
        reload();
        if (count == 0) {
          speedState = 100;
        }

        if (analogRead(SensR) > SenRRef) {
          FoundRight = 1;
        }
        if (analogRead(SensL) > SenLRef) {
          FoundLeft = 1;
        }
        if (analogRead(SensC) > SenCRef) {
          FoundCent = 1;
        }
        if (FoundRight == 1 && FoundLeft == 1) {

          FoundLeft = 0;
          FoundRight = 0;
          count++;
          loopTimer = millis();
          while (millis() - loopTimer <= 400) {
            getIMU();
            // if (analogRead(SensL) > SenLRef) FoundCent == 1;
            heading(80, 90, 0);
            if ((huskylens.updateBlocks() && huskylens.blockSize[1]) /*|| FoundCent == 1*/) {
              break;
            }
          }
          if (count >= 1) {
            speedState = 50;
          }
          if (count >= 2) {
            //count = 0;
            state = 6;
          }
          // if (count > 2) {
          //   loopTimer = millis();
          //   while (millis() - loopTimer <= 550) {
          //     getIMU();
          //     // if (analogRead(SensL) > SenLRef) FoundCent == 1;
          //     heading(100, 90, 0);
          //     if ((huskylens.updateBlocks() && huskylens.blockSize[1]) /*|| FoundCent == 1*/) {
          //       break;
          //     }
          //   }
          // }
          // else state = 1;
        } else if (analogRead(SensL) > SenLRef && analogRead(SensR) < SenRRef && FoundRight == 0) {
          FoundLeft = 1;
          // getIMU();
          holonomic(100, 315, 0);
          nubL = millis();
          while (millis() - nubL <= 120) {
            if (analogRead(SensR) > SenRRef) {
              holonomic(0, 0, 0);
              FoundRight = 1;
              break;
            }
          }
          // state = 1;
          //break;
        } else if (analogRead(SensL) < SenLRef && analogRead(SensR) > SenRRef && FoundLeft == 0) {
          FoundRight = 1;
          // getIMU();
          holonomic(100, 225, 0);
          nubR = millis();
          while (millis() - nubR <= 150) {
            if (analogRead(SensL) > SenLRef) {
              // getIMU();
              holonomic(0, 0, 0);
              FoundLeft = 1;
              break;
            }
            if (huskylens.updateBlocks() && huskylens.blockSize[1]) {
              break;
            }
          }
          // state = 1;
          //break;
        } else if (analogRead(SensR) < SenRRef && analogRead(SensL) < SenLRef && analogRead(SensC) < SenCRef) {
          FoundLeft = 0;
          FoundRight = 0;
          FoundCent = 0;
          if ((huskylens.blockSize[2] && huskylens.blockSize[3]) || !(huskylens.blockSize[2] || huskylens.blockSize[3])) {
            goalEstX = 150;
          } else if (huskylens.blockSize[2]) {
            goalEstX = huskylens.blockInfo[2][0].x;
            goalEstY = huskylens.blockInfo[2][0].y;
            goalEstWidth = huskylens.blockInfo[2][0].width;
          } else if (huskylens.blockSize[3]) {
            goalEstX = huskylens.blockInfo[3][0].x;
            goalEstY = huskylens.blockInfo[3][0].y;
            goalEstWidth = huskylens.blockInfo[3][0].width;
          }
          if (!(huskylens.blockSize[2] || huskylens.blockSize[3])) {
            vecCurveV = 270;
          } else if (goalEstX >= 150 + 60) {
            vecCurveV = 315;
          } else if (goalEstX <= 150 - 60) {
            vecCurveV = 225;
          } else {
            vecCurveV = 270;
          }
          getIMU();
          heading(speedState, vecCurveV, 0);
        }
        if (huskylens.updateBlocks() && huskylens.blockSize[1]) {
          break;
        }

        // if (abs(pvYaw) > 15) {
        //   SetYaw();
        // }


        if (state == 1) {
          vecCurveV = 90;
          getIMU();
          heading(100, 240, 0);

          if (huskylens.updateBlocks() && huskylens.blockSize[1]) {
            break;
          }
          loopTimer = millis();
          while (millis() - loopTimer <= 1100) {
            if (analogRead(SensL) > SenLRef && (millis() - loopTimer >= 350)) vecCurveV = 25;
            else if (analogRead(SensR) > SenRRef && (millis() - loopTimer >= 350)) vecCurveV = 155;
            // if (analogRead(SensL) > SenLRef) FoundCent == 1;
            getIMU();
            heading(100, vecCurveV, 0);
            if ((huskylens.updateBlocks() && huskylens.blockSize[1]) /*|| FoundCent == 1*/) {
              break;
            }
          }
          if (vecCurveV == 90) count++;
          else count = 0;
          state = 0;
        } else if (state == 2) {  // ส่ายหน้า
          int Yor = 0;
          int targetyor = 60;
          count = 0;
          // beep();
          holonomic(0, 0, 0);
          // if (analogRead(SensL) > SenLRef || analogRead(SensR) > SenRRef || analogRead(SensC) > SenCRef) {
          //   holonomic(0, 0, 0);
          //   state = 0;
          //   break;
          // }

          loopTimer = millis();
          while (millis() - loopTimer <= 2000) {
            getIMU();
            if (!(huskylens.updateBlocks() && huskylens.blockSize[1])) {
              for (Yor = pvYaw; pvYaw <= targetyor; Yor++) {
                if (huskylens.updateBlocks() && huskylens.blockSize[1]) {
                  holonomic(0, 0, 0);
                  break;
                }
                getIMU();
                heading(0, 0, Yor);
              }
            }

            // holonomic(0, 0, 0);
            // while (abs(pvYaw) > 11 && huskylens.updateBlocks() && !(huskylens.blockSize[1])) heading(0, 0, 0);

            if (!(huskylens.updateBlocks() && huskylens.blockSize[1])) {
              for (Yor = pvYaw; pvYaw >= -targetyor; Yor--) {
                if (huskylens.updateBlocks() && huskylens.blockSize[1]) {
                  holonomic(0, 0, 0);
                  break;
                }
                getIMU();
                heading(0, 0, Yor);
              }
            }

            holonomic(0, 0, 0);
            // while (abs(pvYaw) > 11 && huskylens.updateBlocks() && !(huskylens.blockSize[1])) heading(0, 0, 0);
            // if (!(huskylens.updateBlocks() && huskylens.blockSize[1])) {
            //   for (Yor = pvYaw; pvYaw >= 0; Yor--) {
            //     if (huskylens.updateBlocks() && huskylens.blockSize[1]) {
            //       holonomic(0, 0, 0);
            //       break;
            //     }
            //     getIMU();
            //     heading(0, 0, Yor);
            //   }
            // }

            if (analogRead(SensL) > SenLRef || analogRead(SensR) > SenRRef || analogRead(SensC) > SenCRef || (huskylens.updateBlocks() && huskylens.blockSize[1])) {
              holonomic(0, 0, 0);
              state = 0;
              break;
            }
          }
          while (abs(pvYaw) > 11 && huskylens.updateBlocks() && !(huskylens.blockSize[1])) heading(0, 0, 0);
          state = 3;
        } else if (state == 4) {
          count = 0;
          // beep();
          // holonomic(0, 0, 0);
          // if (analogRead(SensL) > SenLRef || analogRead(SensR) > SenRRef || analogRead(SensC) > SenCRef) {
          //   holonomic(0, 0, 0);
          //   state = 0;
          //   break;
          // }
          // loopTimer = millis();
          // while (millis() - loopTimer <= 2000) {
          loopTimer = millis();
          while (millis() - loopTimer <= 150) {
            getIMU();
            heading(80, 180, 0);

            if (analogRead(SensL) > SenLRef || analogRead(SensR) > SenRRef || analogRead(SensC) > SenCRef || (huskylens.updateBlocks() && huskylens.blockSize[1])) {
              if (analogRead(SensL) > SenLRef) holonomic(80, 0, 0), delay(200);
              else if (analogRead(SensR) > SenRRef) holonomic(80, 180, 0), delay(200);
              else if (analogRead(SensC) > SenCRef) holonomic(80, 270, 0), delay(200);
              holonomic(0, 0, 0);
              state = 0;
              break;
            }
          }

          loopTimer = millis();
          while (state == 3 && millis() - loopTimer <= 900) {
            getIMU();
            heading(80, 0, 0);

            if (analogRead(SensL) > SenLRef || analogRead(SensR) > SenRRef || analogRead(SensC) > SenCRef) {
              if (analogRead(SensL) > SenLRef) holonomic(80, 0, 0), delay(200);
              else if (analogRead(SensR) > SenRRef) holonomic(80, 180, 0), delay(200);
              else if (analogRead(SensC) > SenCRef) holonomic(80, 270, 0), delay(200);
              holonomic(0, 0, 0);
              state = 0;
              break;
            }
          }
          loopTimer = millis();
          while (state == 3 && millis() - loopTimer <= 100) {
            getIMU();
            heading(80, 180, 0);

            if (analogRead(SensL) > SenLRef || analogRead(SensR) > SenRRef || analogRead(SensC) > SenCRef) {
              if (analogRead(SensL) > SenLRef) holonomic(80, 0, 0), delay(200);
              else if (analogRead(SensR) > SenRRef) holonomic(80, 180, 0), delay(200);
              else if (analogRead(SensC) > SenCRef) holonomic(80, 270, 0), delay(200);
              holonomic(0, 0, 0);
              state = 0;
              break;
            }
          }
        } else if (state == 3) {
          count = 0;
          long startTime = millis();
          loopTimer = millis();

          // เลือกทิศทาง vecCurveV ตามเงื่อนไขที่ผู้ใช้ต้องการ
          if (state3Count == 0) {
            vecCurveV = (random(0, 1000) < 500) ? 5 : 175;  // สุ่มครั้งแรก
          } else if (state3Count == 1) {
            vecCurveV = (lastVecCurveV == 5) ? 175 : 5;  // กลับทิศจากครั้งที่แล้ว
            state3Count = 0;
          } else {
            vecCurveV = (random(0, 1000) < 500) ? 5 : 175;
          }

          lastVecCurveV = vecCurveV;  // บันทึกค่าล่าสุด
          state3Count++;              // เพิ่มจำนวนครั้งที่เข้า state 3
          unsigned long changeDirDelay = random(800, 1000);
          unsigned long lastSwitchTime = millis();
          bool wentLeft = (vecCurveV == 5);
          int switchCount = 0;
          // เพิ่มตัวแปรเพื่อติดตามเวลาเจอเส้นล่าสุด
          unsigned long lastLeftDetected = 0;
          unsigned long lastRightDetected = 0;


          while (millis() - loopTimer <= 3000 && !(huskylens.updateBlocks() && huskylens.blockSize[1])) {
            getIMU();
            heading(100, vecCurveV, 0);

            int sensorL = analogRead(SensL);
            int sensorR = analogRead(SensR);
            unsigned long currentTime = millis();

            bool switched = false;

            if (sensorL > SenLRef && sensorR < SenRRef) {  // เจอเส้นซ้าย
              if (!wentLeft && millis() - lastRightDetected <= 400) {
                changeDirDelay += 250;  // เพิ่ม delay หากเพิ่งเจอเส้นขวาใน 100 ms ที่แล้ว
              }
              lastLeftDetected = millis();

              if (!wentLeft && currentTime - lastSwitchTime <= 500) {
                switchCount++;
              }
              vecCurveV = 5;
              wentLeft = true;
              lastSwitchTime = currentTime;
            }

            else if (sensorL < SenLRef && sensorR > SenRRef) {  // เจอเส้นขวา
              if (wentLeft && millis() - lastLeftDetected <= 400) {
                changeDirDelay += 250;  // เพิ่ม delay หากเพิ่งเจอเส้นซ้ายใน 100 ms ที่แล้ว
              }
              lastRightDetected = millis();

              if (wentLeft && currentTime - lastSwitchTime <= 500) {
                switchCount++;
              }
              vecCurveV = 175;
              wentLeft = false;
              lastSwitchTime = currentTime;
            }


            // หากสลับฝั่งเร็วไปมาหลายครั้ง → เด้งมุมสนาม → เดินหน้าหลบ
            if (switchCount >= 2) {
              unsigned long forwardStart = millis();
              while (millis() - forwardStart <= 500) {
                getIMU();
                heading(100, 90, 0);
              }

              // reset ทุกอย่างหลังหลบมุม
              switchCount = 0;
              vecCurveV = (random(0, 1000) <= 500) ? 5 : 175;
              wentLeft = (vecCurveV == 5);
              lastSwitchTime = millis();
              changeDirDelay = random(800, 1000);
            }


            if (currentTime - lastSwitchTime >= changeDirDelay) {
              vecCurveV = (vecCurveV == 5) ? 175 : 5;
              wentLeft = !wentLeft;
              lastSwitchTime = currentTime;
              changeDirDelay = random(800, 1000);
            }
          }

          state = 0;
        } else if (state == 5) {
          loopTimer = millis();
          while (!(huskylens.updateBlocks() && huskylens.blockSize[1]) && millis() - loopTimer <= 400) {
            getIMU();
            heading(80, 270, 0);
          }
        } else if (state == 6) {  // ส่ายหน้าใช้ความเร็ว
          int ballFounds = false;
          loopTimer = millis();
          if (huskylens.updateBlocks() && huskylens.blockSize[1]) ballFounds = true;
          while (millis() - loopTimer < 800) {
            holonomic(0, 90, 23);
            if (huskylens.updateBlocks() && huskylens.blockSize[1]) {
              ballFounds = true;
              break;
            }
          }
          // getIMU();
          // loopTimer = millis();
          // while (ballFounds == false && abs(pvYaw) > 5 && millis() - loopTimer < 600) {
          //   getIMU();
          //   heading(0, 0, 0);
          //   if (huskylens.updateBlocks() && huskylens.blockSize[1]) {
          //     ballFounds = true;
          //     break;
          //   }
          // }
          loopTimer = millis();
          while (ballFounds == false && millis() - loopTimer < 1700) {
            holonomic(0, 90, -23);
            if (huskylens.updateBlocks() && huskylens.blockSize[1]) {
              ballFounds = true;
              break;
            }
          }
          loopTimer = millis();
          while (ballFounds == false && millis() - loopTimer < 750) {
            holonomic(0, 90, 23);
            if (huskylens.updateBlocks() && huskylens.blockSize[1]) {
              ballFounds = true;
              break;
            }
          }
          // loopTimer = millis();
          // getIMU();
          // while (ballFounds == false && abs(pvYaw) > 5 && millis() - loopTimer < 1000) {
          //   getIMU();
          //   heading(0, 0, 0);
          //   if (huskylens.updateBlocks() && huskylens.blockSize[1]) {
          //     ballFounds = true;
          //     break;
          //   }
          // }
          // loopTimer = millis();
          // while (ballFounds == false && loopTimer - millis() <= 600) {
          //   getIMU();
          //   heading(100, 90, 0);
          //   if (huskylens.updateBlocks() && huskylens.blockSize[1]) {
          //     ballFounds = true;
          //     break;
          //   }
          // }
          state = 0;
        }
      }
    }
  }
}

void BigbumpDef() {

  int setpyball = huskylens.blockInfo[1][0].y;
  int setpxball = huskylens.blockInfo[1][0].y;
  loopTimer = millis();
  while (millis() - loopTimer <= 4800 /*&& huskylens.updateBlocks() && huskylens.blockSize[1]*/) {
    huskylens.updateBlocks();
    holonomic(0, 0, 0);
    if ((huskylens.blockInfo[1][0].y > 58)) {
      // sound(500,1000);
      break;
    }
    // if(huskylens.updateBlocks() && huskylens.blockSize[1] && (abs(huskylens.blockInfo[1][0].x - setpxball) > 10)) break;
  }
  Bigbump();
}
void TurnFindBigbumpDef() {

  int setpyball = huskylens.blockInfo[1][0].y;
  int setpxball = huskylens.blockInfo[1][0].x;
  loopTimer = millis();
  while (millis() - loopTimer <= 4800 /*&& huskylens.updateBlocks() && huskylens.blockSize[1]*/) {
    huskylens.updateBlocks();
    holonomic(0, 0, 0);
    if ((huskylens.blockInfo[1][0].y > 58)) {
      // sound(500,1000);
      break;
    }
    // if(huskylens.updateBlocks() && huskylens.blockSize[1] && (abs(huskylens.blockInfo[1][0].x - setpxball) > 10)) break;
  }
  FindTurnBigbump();
}
void delay5Bigbump() {

  int setpyball = huskylens.blockInfo[1][0].y;
  int setpxball = huskylens.blockInfo[1][0].y;
  loopTimer = millis();
  while (millis() - loopTimer <= 4800 /*&& huskylens.updateBlocks() && huskylens.blockSize[1]*/) {
    holonomic(0, 0, 0);
    // if(huskylens.updateBlocks() && huskylens.blockSize[1] && (abs(huskylens.blockInfo[1][0].x - setpxball) > 10)) break;
  }
  Bigbump();
}