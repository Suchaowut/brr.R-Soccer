void Penalty_Shoot() {
  while (1) {
    int nubL, nubR, vecCurveV;
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
      Yaxis_Error = 180 - huskylens.blockInfo[1][0].y;
      Yaxis_D = Yaxis_Error - Yaxis_PvEror;
      Yaxis_spd = (Yaxis_Error * Yaxis_Kp) + (Yaxis_D * Yaxis_Kd);
      Yaxis_spd = constrain(Yaxis_spd, -100, 100);
      Yaxis_PvEror = Yaxis_Error;

      holonomic(Yaxis_spd, SethaPos, 0);

      if (abs(Yaxis_Error) <= 10) {
        getIMU();
        lastYaw = pvYaw;
        // if (abs(pvYaw) <= 8) {
        // TrackXaxis2();
        // if (abs(Xaxis_Error) <= 8) {  // พร้อมทำงาน
          // wheel(0, 0, 0);
          // getIMU();
          loopTimer = millis();
          while ((huskylens.updateBlocks() && huskylens.blockSize[1])) {
            ballPosX = huskylens.blockInfo[1][0].x;
            ballPosY = huskylens.blockInfo[1][0].y;
            holonomic(100, 90, 0);

            if ((ballPosY > 220 && millis() - loopTimer > 600) || analogRead(SensC) > SenCRef) {
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
            // }
          }
        }
        // } else if (abs(pvYaw) > 8) {
        //   while ((huskylens.updateBlocks() && huskylens.blockSize[1]) && Yaxis_Error <= 20) {
        //     getIMU();
        //     rot_error = 170 - huskylens.blockInfo[1][0].x;
        //     rot_d = rot_error - rot_pError;
        //     rot_pError = rot_error;
        //     rot_w = (rot_error * 0.3) + (rot_i * rot_Ki) + (rot_d * rot_Kd);
        //     rot_w = constrain(rot_w, -60, 60);

        //     Yaxis_Error = 160 - huskylens.blockInfo[1][0].y;
        //     Yaxis_D = Yaxis_Error - Yaxis_PvEror;
        //     Yaxis_spd = (Yaxis_Error * Yaxis_Kp) + (Yaxis_D * Yaxis_Kd);
        //     Yaxis_spd = constrain(Yaxis_spd, -100, 100);
        //     Yaxis_PvEror = Yaxis_Error;

        //     if (Yaxis_Error > 15) holonomic(Yaxis_spd, 90, rot_w);

        //     int targetSpeed = (abs(pvYaw) > 50) ? 90 : 70;
        //     int currentSpeed = 0.8 * currentSpeed + 0.2 * targetSpeed;  // smoothing speed
        //     if (pvYaw < 0) {
        //       vecCurve = 0;
        //     } else {
        //       vecCurve = 180;
        //     }
        //     holonomic(currentSpeed, vecCurve, rot_w);
        //     if (abs(pvYaw) <= 8) break;
        //   }
        // }
      }
    }
  }
}

void Penalty_Save() {
  int FoundRight, FoundCent, FoundLeft, vecCurveV = 180;
  long nubL, nubR;
  while (1) {
    if (analogRead(SensC) > SenCRef) {
      FoundCent = 1;
    }
    if (analogRead(SensL) > SenLRef && analogRead(SensR) < SenRRef) {
      FoundLeft = 1;
      nubL = millis();
      while (millis() - nubL <= 80) {
        if (analogRead(SensR) > SenRRef) {
          heading(0, 0, 0);
          FoundRight = 1;
          break;
        } else if (analogRead(SensC) > SenCRef) {
          heading(0, 0, 0);
          FoundCent = 1;
          break;
        }
      }
    } else if (analogRead(SensL) < SenLRef && analogRead(SensR) > SenRRef) {
      FoundRight = 1;
      nubR = millis();
      while (millis() - nubR <= 80) {
        if (analogRead(SensL) > SenLRef) {
          heading(0, 0, 0);
          FoundLeft = 1;
          break;
        } else if (analogRead(SensC) > SenCRef) {
          heading(0, 0, 0);
          FoundCent = 1;
          break;
        }
      }
    }
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
    Xaxis_Error = huskylens.blockInfo[1][0].x - 150;
    Xaxis_D = Xaxis_Error - Xaxis_PvEror;
    Xaxis_spd = (Xaxis_Error * Xaxis_Kp) + (Xaxis_D * Xaxis_Kd);
    Xaxis_spd = constrain(Xaxis_spd, -100, 100);
    if (abs(Xaxis_spd) < 15 && abs(Xaxis_Error) >= 2) {
      Xaxis_spd = (Xaxis_spd > 0) ? 40 : -40;
    }
    Xaxis_PvEror = Xaxis_Error;

    rot_error = 150 - huskylens.blockInfo[1][0].x;
    rot_d = rot_error - rot_pError;
    rot_pError = rot_error;
    rot_w = (rot_error * 0.3) + (rot_i * rot_Ki) + (rot_d * rot_Kd);
    rot_w = constrain(rot_w, -80, 80);

    // holonomic(Yaxis_spd, SethaPos, 0);
    getIMU();
    if ((huskylens.updateBlocks() && huskylens.blockSize[1])) {
      if (FoundCent == 0 && FoundLeft == 0 && FoundRight == 0) {
        FoundCent = 0;
        FoundLeft = 0;
        FoundRight = 0;
        vecCurveV = 0;
      }
      if (FoundRight == 1 && FoundLeft == 1) {
        FoundRight = 0;
        FoundLeft = 0;
        vecCurveV = 90;
        Xaxis_spd = 60;
      } else if (FoundRight == 1 && FoundCent == 1) {
        FoundRight = 0;
        FoundLeft = 0;
        vecCurveV = 240;
      } else if (FoundCent == 1 && FoundLeft == 1) {
        FoundRight = 0;
        FoundLeft = 0;
        vecCurveV = 300;
      } else if (FoundLeft == 1) {
        FoundLeft = 0;
        vecCurveV = 0;
        Xaxis_spd = 60;
      } else if (FoundRight == 1) {
        FoundRight = 0;
        vecCurveV = 180;
        Xaxis_spd = 60;
      } else if (FoundCent == 1) {
        FoundCent = 0;
        vecCurveV = 270;
        Xaxis_spd = 60;
      }
      heading(Xaxis_spd, vecCurveV, 0);
    } else {
      /*if (FoundCent == 1 && FoundLeft == 1 && FoundRight == 1) {
        FoundCent = 0; FoundLeft = 0; FoundRight = 0;
        vecCurveV = SethaPos;
      }
      else */
      if (FoundRight == 1 && FoundLeft == 1) {
        FoundRight = 0;
        FoundLeft = 0;
        vecCurveV = 90;
      } else if (FoundRight == 1 && FoundCent == 1) {
        FoundRight = 0;
        FoundLeft = 0;
        vecCurveV = 240;
      } else if (FoundCent == 1 && FoundLeft == 1) {
        FoundRight = 0;
        FoundLeft = 0;
        vecCurveV = 300;
      } else if (FoundLeft == 1) {
        FoundLeft = 0;
        vecCurveV = 0;
      } else if (FoundRight == 1) {
        FoundRight = 0;
        vecCurveV = 180;
      } else if (FoundCent == 1) {
        FoundCent = 0;
        vecCurveV = 270;
      }
      heading(80, vecCurveV, 0);
    }
  }
}