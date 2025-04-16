int lastGoalPos() {
  if (huskylens.blockInfo[2][0].x < 160 || huskylens.blockInfo[3][0].x < 160) return 1;
  else if (huskylens.blockInfo[2][0].x > 160 || huskylens.blockInfo[3][0].x > 160) return 2;
  else return 0;
}

int lastGoalCoord;
int goalX = 160;
void AtanTrack() {
  if ((huskylens.updateBlocks() && huskylens.blockSize[1])) {
    lastGoalCoord = lastGoalPos();
    float ballPosX = huskylens.blockInfo[1][0].x;
    float ballPosY = huskylens.blockInfo[1][0].y;
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
    SetYaw();
    Yaxis_Error = 160 - huskylens.blockInfo[1][0].y;
    Yaxis_D = Yaxis_Error - Yaxis_PvEror;
    Yaxis_spd = (Yaxis_Error * Yaxis_Kp) + (Yaxis_D * Yaxis_Kd);
    Yaxis_spd = constrain(Yaxis_spd, -100, 100);
    Yaxis_PvEror = Yaxis_Error;
    holonomic(Yaxis_spd, SethaPos, 0);
    if (abs(Yaxis_Error) <= 20) {
      getIMU();
      if (pvYaw >= 10 || pvYaw < -10) {
        loopTimer = millis();
        while (millis() - loopTimer <= 100) {
          getIMU();
          heading(0, 0, 0);
        }
        holonomic(0, 0, 0);
        getIMU();
      } else if (SethaPos <= 10) {
        holonomic(80, 300, 0);
      } else if (SethaPos >= 170) {
        holonomic(80, 240, 0);
      } else {
        TrackXaxis();
        if (abs(Xaxis_Error) <= 20) {
          getIMU();
          if (huskylens.updateBlocks() && !(huskylens.blockSize[2] || huskylens.blockSize[3])) {
            FindGoal();
          } else if ((huskylens.updateBlocks() && (huskylens.blockSize[2] || huskylens.blockSize[3]))) {
            goalX = 160;
            ShootNAjA();
          }
        }
      }
    }
  }
}

int RealGoal;
long startTime;
void FindGoal() {
  while (1) {
    startTime = millis();
    while ((huskylens.updateBlocks() && huskylens.blockSize[1]) && !(huskylens.blockSize[2] || huskylens.blockSize[3])) {
      float ballPosX = huskylens.blockInfo[1][0].x;
      float ballPosY = huskylens.blockInfo[1][0].y;
      if (millis() - startTime >= 4000) {
        holonomic(0, 0, 0);
        break;
      }
      if (lastGoalCoord == 1) {
        loopTimer = millis();
        while ((huskylens.updateBlocks() && huskylens.blockSize[1]) && (millis() - loopTimer <= 800)) {
          sound(200, 100);
          getIMU();
          if (pvYaw <= -80) {
            holonomic(0, 0, 0);
            break;
          }
          rot_error = 160 - huskylens.blockInfo[1][0].x;
          rot_d = rot_d + rot_error;
          rot_d = constrain(rot_d, -100, 100);
          rot_d = rot_error - rot_pError;
          rot_pError = rot_error;
          rot_w = (rot_error * rot_Kp) + (rot_i * rot_Ki) + (rot_d * rot_Kd);
          rot_w = constrain(rot_w, 0, 30);
          holonomic(35, 20, rot_w);
          if (!(huskylens.updateBlocks() && huskylens.blockSize[1]) || (huskylens.updateBlocks() && huskylens.blockSize[3] || huskylens.blockSize[2])) {
            if (huskylens.updateBlocks() && huskylens.blockSize[2]) {
              updateGoalAvg(huskylens.blockInfo[2][0].x);
            } else if (huskylens.updateBlocks() && huskylens.blockSize[3]) {
              updateGoalAvg(huskylens.blockInfo[3][0].x);
            }
            holonomic(0, 0, 0);
            break;
          }
        }
        holonomic(0, 0, 0);
        loopTimer = millis();
        while ((huskylens.updateBlocks() && huskylens.blockSize[1]) && (millis() - loopTimer <= 1000)) {
          sound(400, 100);
          getIMU();
          if (pvYaw >= 90) {
            holonomic(0, 0, 0);
            break;
          }
          rot_error = 160 - huskylens.blockInfo[1][0].x;
          rot_d = rot_d + rot_error;
          rot_d = constrain(rot_d, -100, 100);
          rot_d = rot_error - rot_pError;
          rot_pError = rot_error;
          rot_w = (rot_error * rot_Kp) + (rot_i * rot_Ki) + (rot_d * rot_Kd);
          rot_w = constrain(rot_w, -30, 0);
          holonomic(35, 160, rot_w);
          if (!(huskylens.updateBlocks() && huskylens.blockSize[1]) || (huskylens.updateBlocks() && huskylens.blockSize[3] || huskylens.blockSize[2])) {
            if (huskylens.updateBlocks() && huskylens.blockSize[2]) {
              updateGoalAvg(huskylens.blockInfo[2][0].x);
            } else if (huskylens.updateBlocks() && huskylens.blockSize[3]) {
              updateGoalAvg(huskylens.blockInfo[3][0].x);
            }
            holonomic(0, 0, 0);
            break;
          }
        }
        holonomic(0, 0, 0);
      } else {
        loopTimer = millis();
        while ((huskylens.updateBlocks() && huskylens.blockSize[1]) && (millis() - loopTimer <= 800)) {
          sound(400, 100);
          getIMU();
          if (pvYaw <= -80) {
            holonomic(0, 0, 0);
            break;
          }
          rot_error = 160 - huskylens.blockInfo[1][0].x;
          rot_d = rot_d + rot_error;
          rot_d = constrain(rot_d, -100, 100);
          rot_d = rot_error - rot_pError;
          rot_pError = rot_error;
          rot_w = (rot_error * rot_Kp) + (rot_i * rot_Ki) + (rot_d * rot_Kd);
          rot_w = constrain(rot_w, -30, 0);
          holonomic(35, 160, rot_w);
          if(!(huskylens.updateBlocks() && huskylens.blockSize[1])){
            holonomic(0, 0, 0);
            break;
          }
          if ((huskylens.updateBlocks() && huskylens.blockSize[3] || huskylens.blockSize[2])) {
            if (huskylens.updateBlocks() && huskylens.blockSize[2]) {
              updateGoalAvg(huskylens.blockInfo[2][0].x);
            } else if (huskylens.updateBlocks() && huskylens.blockSize[3]) {
              updateGoalAvg(huskylens.blockInfo[3][0].x);
            }
            holonomic(0, 0, 0);
            break;
          }
        }
        holonomic(0, 0, 0);
        loopTimer = millis();
        while ((huskylens.updateBlocks() && huskylens.blockSize[1]) && (millis() - loopTimer <= 1200)) {
          sound(200, 100);
          getIMU();
          if (pvYaw >= 90) {
            holonomic(0, 0, 0);
            break;
          }
          rot_error = 160 - huskylens.blockInfo[1][0].x;
          rot_d = rot_d + rot_error;
          rot_d = constrain(rot_d, -100, 100);
          rot_d = rot_error - rot_pError;
          rot_pError = rot_error;
          rot_w = (rot_error * rot_Kp) + (rot_i * rot_Ki) + (rot_d * rot_Kd);
          rot_w = constrain(rot_w, 0, 30);
          holonomic(35, 20, rot_w);
          if (!(huskylens.updateBlocks() && huskylens.blockSize[1]) || (huskylens.updateBlocks() && huskylens.blockSize[3] || huskylens.blockSize[2])) {
            if (huskylens.updateBlocks() && huskylens.blockSize[2]) {
              updateGoalAvg(huskylens.blockInfo[2][0].x);
            } else if (huskylens.updateBlocks() && huskylens.blockSize[3]) {
              updateGoalAvg(huskylens.blockInfo[3][0].x);
            }
            holonomic(0, 0, 0);
            break;
          }
        }
        holonomic(0, 0, 0);
      }
    }
    break;
  }
  goalX = 160;
  ShootNAjA();
}

float alpha = 0.1;
void updateGoalAvg(int newGoalX) {
  goalX = (1 - alpha) * goalX + alpha * newGoalX;
}

void ShootNAjA() {
  if ((huskylens.updateBlocks() && huskylens.blockSize[1]) && !(huskylens.updateBlocks() && huskylens.blockSize[3] || huskylens.blockSize[2])) {
    while (1) {
        holonomic(0, 0, 0);
        beep();
        shootlana();
        break;
    }
    holonomic(0, 0, 0);
  } else {
    while (1) {
      // if (GoalPos == -1) break;
      ballPosX = huskylens.blockInfo[1][0].x;
      ballPosY = huskylens.blockInfo[1][0].y;

      if (huskylens.updateBlocks() && huskylens.blockSize[2]) {
        updateGoalAvg(huskylens.blockInfo[2][0].x);
        RealGoal = huskylens.blockInfo[2][0].x;
      } else if (huskylens.updateBlocks() && huskylens.blockSize[3]) {
        RealGoal = huskylens.blockInfo[3][0].x;
        updateGoalAvg(huskylens.blockInfo[3][0].x);
      }

      rot_error = 160 - ballPosX;
      rot_d = rot_d + rot_error;
      rot_d = constrain(rot_d, -100, 100);
      rot_d = rot_error - rot_pError;
      rot_pError = rot_error;
      rot_w = (rot_error * rot_Kp) + (rot_i * rot_Ki) + (rot_d * rot_Kd);
      rot_w = constrain(rot_w, -100, 100);
      getIMU();
      if (abs(pvYaw) >= 200) {
        holonomic(0, 0, 0);
        break;
      } else if ((huskylens.updateBlocks() && huskylens.blockSize[1]) && ((ballPosX - goalX < -10) || (ballPosX - RealGoal < -10))) {
        rot_w = constrain(rot_w, -40, 0);
        holonomic(35, 180, rot_w);
      } else if ((huskylens.updateBlocks() && huskylens.blockSize[1]) && ((ballPosX - goalX > 10) || (ballPosX - RealGoal > 10))) {
        rot_w = constrain(rot_w, 0, 40);
        holonomic(35, 0, rot_w);
      } else if ((huskylens.updateBlocks() && huskylens.blockSize[1]) && ((ballPosX - goalX >= -10) || (ballPosX - RealGoal >= -10)) && ((ballPosX - goalX <= 10) || (ballPosX - RealGoal <= 10))) {
        holonomic(0, 0, 0);
        beep();
        shootlana();
        break;
      } else if (!(huskylens.updateBlocks() && huskylens.blockSize[1] && huskylens.blockSize[2] && huskylens.blockSize[3])) {
        holonomic(0, 0, 0);
        break;
      }
    }
    holonomic(0, 0, 0);
  }
}

void shootlana() {
  while (1) {
    if (!(huskylens.updateBlocks() && huskylens.blockSize[1])) {
      holonomic(0, 0, 0);
      break;
    }
    // if ((huskylens.updateBlocks() && huskylens.blockSize[1]) && abs(huskylens.blockInfo[1][0].x - 160) <= 10 && huskylens.blockInfo[1][0].y <= 220) {
    //   holonomic(0, 0, 0);
    //   shoot();
    //   holonomic(0, 0, 0);
    //   reload();
    //   holonomic(0, 0, 0);
    //   delay(1000);
    //   break;
    // }
    if (analogRead(SensC) > SenCRef) {
      holonomic(0, 0, 0);
      if ((huskylens.updateBlocks() && huskylens.blockSize[1]) && abs(160 - huskylens.blockInfo[1][0].x) <= 15 && huskylens.blockInfo[1][0].y >= 220) {
        holonomic(0, 0, 0);
        shoot();
        holonomic(0, 0, 0);
        reload();
        holonomic(0, 0, 0);
        delay(1000);
        break;
      }
      break;
    }
    // else if (!(huskylens.updateBlocks() && huskylens.blockSize[1]) && !(abs(huskylens.blockInfo[1][0].x - 160) <= 10) && !(huskylens.blockInfo[1][0].y <= 220)  && analogRead(SensC) < SenCRef) {
    //   holonomic(50, 90, 0);
    // }
    else {
      holonomic(50, 90, 0);
      if ((huskylens.updateBlocks() && huskylens.blockSize[1]) && abs(160 - huskylens.blockInfo[1][0].x) <= 15 && huskylens.blockInfo[1][0].y >= 220) {
        holonomic(0, 0, 0);
        shoot();
        holonomic(0, 0, 0);
        reload();
        holonomic(0, 0, 0);
        delay(1000);
        break;
      }
    }
  }
  holonomic(0, 0, 0);
}

void Normal_V1() { // แก้ sensor
  while (1) {
    if (huskylens.updateBlocks() && huskylens.blockSize[1]) {
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
        AtanTrack();
      }
    } else {
      getIMU();
      while (analogRead(SensL) < SenLRef && analogRead(SensR) < SenRRef) {
        getIMU();
        holonomic(80, 270, 0);
        SetYaw();
        if (huskylens.updateBlocks() && huskylens.blockSize[1]) {
          break;
        }
        getIMU();
      }

      loopTimer = millis();
      while (millis() - loopTimer <= 3000) {
        if (analogRead(SensC) > SenCRef) {
          holonomic(0, 0, 0);
          delay(100);
          holonomic(50, 270, 0);
          delay(100);
          holonomic(80, 270, 0);
          delay(200);
        }
        getIMU();
        holonomic(40, 90, 0);
        SetYaw();
        if (huskylens.updateBlocks() && huskylens.blockSize[1]) {
          break;
        }
        getIMU();
      }
    }
  }
}