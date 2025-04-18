void TrackXaxis2() {
  if (huskylens.updateBlocks() && huskylens.blockSize[1]) {
    Xaxis_Error = huskylens.blockInfo[1][0].x - 160;
    Xaxis_D = Xaxis_Error - Xaxis_PvEror;
    Xaxis_spd = (Xaxis_Error * Xaxis_Kp) + (Xaxis_D * Xaxis_Kd);
    if (abs(Xaxis_spd) < 15 && abs(Xaxis_Error) > 6) {
      Xaxis_spd = (Xaxis_spd > 0) ? 15 : -15;
    } else Xaxis_spd = constrain(Xaxis_spd, -80, 80);
    Xaxis_PvEror = Xaxis_Error;
    getIMU();
    heading(Xaxis_spd, 0, 0);
  }
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
    Yaxis_Error = 190 - huskylens.blockInfo[1][0].y;
    Yaxis_D = Yaxis_Error - Yaxis_PvEror;
    Yaxis_spd = (Yaxis_Error * Yaxis_Kp) + (Yaxis_D * Yaxis_Kd);
    Yaxis_spd = constrain(Yaxis_spd, -100, 100);
    Yaxis_PvEror = Yaxis_Error;

    holonomic(Yaxis_spd, SethaPos, 0);
    if (Yaxis_Error <= 20) {
      getIMU();
      lastYaw = pvYaw;
      if (abs(pvYaw) <= 10) {
        TrackXaxis2();
        if (abs(Xaxis_Error) <= 10) {
          wheel(0, 0, 0);
          SetYaw();
          getIMU();
          // beep();
          lastYaw = pvYaw;
          Dribbling();
        }
      } else if (!(abs(pvYaw) <= 10)) {
        while ((huskylens.updateBlocks() && huskylens.blockSize[1])) {
          getIMU();
          rot_error = 160 - huskylens.blockInfo[1][0].x;
          rot_d = rot_d + rot_error;
          rot_d = constrain(rot_d, -100, 100);
          rot_d = rot_error - rot_pError;
          rot_pError = rot_error;
          rot_w = (rot_error * 0.6) + (rot_i * rot_Ki) + (rot_d * rot_Kd);
          rot_w = constrain(rot_w, -100, 100);

          int targetSpeed = (abs(pvYaw) > 50) ? 60 : 40;
          int currentSpeed = 0.8 * currentSpeed + 0.2 * targetSpeed;  // smoothing speed
          if (lastYaw < 0) {
            vecCurve = 0;
          } else {
            vecCurve = 180;
          }
          holonomic(currentSpeed, vecCurve, rot_w);
          // if (abs(pvYaw) > 50) {
          //   if (lastYaw < 0) {
          //     vecCurve = 0;
          //   } else {
          //     vecCurve = 180;
          //   }
          //   holonomic(60, vecCurve, rot_w);
          // } else {
          //   if (lastYaw < 0) {
          //     vecCurve = 0;
          //   } else {
          //     vecCurve = 180;
          //   }
          //   holonomic(40, vecCurve, rot_w);
          // }
          if (abs(pvYaw) <= 5) break;
        }
      }
    }
  }
}

void Dribbling() {
  float angleToGoal = 90, goalEstX, goalEstY, goalEstWidth;
  while (huskylens.updateBlocks() && huskylens.blockSize[1]) {
    if (huskylens.updateBlocks() && !(huskylens.blockSize[2] || huskylens.blockSize[3])) {
      getIMU();
      heading(100, angleToGoal, 0);
    } else {
      // track goal พร้อมลูก
      getIMU();
      ballPosX = huskylens.blockInfo[1][0].x;
      ballPosY = huskylens.blockInfo[1][0].y;

      if (huskylens.updateBlocks() && huskylens.blockSize[2]) {
        goalEstX = huskylens.blockInfo[2][0].x;
        goalEstY = huskylens.blockInfo[2][0].y;
        goalEstWidth = huskylens.blockInfo[2][0].width;
      } else if (huskylens.updateBlocks() && huskylens.blockSize[3]) {
        goalEstX = huskylens.blockInfo[3][0].x;
        goalEstY = huskylens.blockInfo[3][0].y;
        goalEstWidth = huskylens.blockInfo[3][0].width;
      }

      float goalXLeft = goalEstX - goalEstWidth / 2.0;
      float goalXRight = goalEstX + goalEstWidth / 2.0;

      float distanceToEdge = min(abs(ballPosX - goalXLeft), abs(ballPosX - goalXRight));

      float dx = goalEstX - ballPosX;
      float dy = goalEstY - ballPosY;
      float angleToGoal = atan2(dy, dx) * (180 / PI);
      if (angleToGoal < 0) angleToGoal += 360;
      if (angleToGoal > 180) angleToGoal = 180 - (angleToGoal - 180);
      if (distanceToEdge < 20) {
        angleToGoal = constrain(angleToGoal, 30, 150);
      } else {
        angleToGoal = constrain(angleToGoal, 5, 175);
      }
      if (ballPosY < 210) {
        heading(100, 90, 0);
      } else {
        heading(100, angleToGoal, 0);
      }
      if (abs(ballPosX - goalEstWidth) < 10 && ballPosY > 220 && analogRead(SensC) > SenCRef) {
        holonomic(0, 0, 0);
        shoot();
        delay(100);
        holonomic(60, 270, 0);
        delay(300);
        holonomic(0, 0, 0);
        reload();
        holonomic(0, 0, 0);
        delay(1000);
      }
    }
  }
}

void TouchLine() {
  while (1) {
    if (huskylens.updateBlocks() && huskylens.blockSize[1]) {
      lastYaw = pvYaw;
      if (analogRead(SensC) > SenCRef) {
        holonomic(0, 0, 0);
        delay(100);
        holonomic(50, 270, 0);
        delay(100);
        holonomic(80, 270, 0);
        delay(200);
      } else if (analogRead(SensL) > SenLRef) {
        holonomic(0, 0, 0);
        delay(100);
        holonomic(50, 0, 0);
        delay(100);
        holonomic(80, 0, 0);
        delay(200);
      } else if (analogRead(SensR) > SenRRef) {
        holonomic(0, 180, 0);
        delay(100);
        holonomic(50, 180, 0);
        delay(100);
        holonomic(80, 180, 0);
        delay(200);
      } else {
        AtanTrack2();
      }
    } else {
      getIMU();
      wheel(0, 0, 0);
      int sideRot = 160 - ballPosX;  //คำนวนทิศการหมุนหาลูกบอลเมื่อเจอล่าสุด
      holonomic(0, 0, sideRot / abs(sideRot) * idleSpd);
      // SetYaw();
    }
  }
}