#include "TektiteRotEv.h"

RotEv rotev;

const float CM_PER_RAD = 5.271f / 2.0f;

float prevAngle1 = 0.0f;
float prevAngle2 = 0.0f;
uint32_t prevTime = 0;

#define CURVE_DIA 104.0f
#define TRACK_LENGTH 769.0f

float gyroOffset = 0.0f;

void setup() {
  rotev.begin();
  for (int i = 0; i < 10; i++) {
    prevAngle1 = rotev.enc1Angle();
    prevAngle2 = rotev.enc2Angle();
    delay(50);
  }
  prevTime = micros();
}

#define MOTOR1_DIR -1
#define MOTOR2_DIR 1
#define ENC1_DIR 1
#define ENC2_DIR -1

float posX = 0.0f;
float posY = 0.0f;
float heading = 0.0f;

bool going = false;
uint32_t lastPrint = 0;

float wrapDelta(float delta) {
  if (delta > M_PI) delta -= 2 * M_PI;
  else if (delta < -M_PI) delta += 2 * M_PI;
  return delta;
}

float vel1 = 0.0f;
float vel2 = 0.0f;

#define MAX_CURR 0.3f
#define RPM_6V 500.0f
#define CURR_6V 0.1f
#define RESISTANCE 6.0f/1.6f

const float kE = (6.0f - RESISTANCE*CURR_6V)/(RPM_6V * (2 * M_PI/60.0f));
const float maxVoltIncr = MAX_CURR * RESISTANCE;

float limitCurrPredictive(float vel, float newVolt) {
  float currV = kE * vel;
  float diff = newVolt - currV;
  if (diff > maxVoltIncr) return currV + maxVoltIncr;
  else if (diff < -maxVoltIncr) return currV - maxVoltIncr;
  return newVolt;
}

const float kP_h = 2.8f;
const float kD_h = 0.05f;
const float kP_x = 0.4f;
float kP_y_small = 0.15f;
float kP_y = 0.02f;
uint32_t start = 0;
bool doneCurve = false;

const float Y_BLEND_START_DIST = 100.0f;
const float Y_BLEND_END_DIST = 20.0f;

const float ACCEL_START_SPEED = 0.4f;
const float ACCEL_END_SPEED = 0.8f;
const float ACCEL_DISTANCE = TRACK_LENGTH * 0.7f;

float getSpeedMultiplier(float distance) {
  if (distance >= ACCEL_DISTANCE) return ACCEL_END_SPEED;
  float progress = distance / ACCEL_DISTANCE;
  return ACCEL_START_SPEED + (ACCEL_END_SPEED - ACCEL_START_SPEED) * progress;
}

float getYCoeff(float posX) {
  float distLeft = TRACK_LENGTH - posX;
  if (distLeft >= Y_BLEND_START_DIST) return kP_y;
  if (distLeft <= Y_BLEND_END_DIST) return kP_y_small;
  float t = (Y_BLEND_START_DIST - distLeft) / (Y_BLEND_START_DIST - Y_BLEND_END_DIST);
  return kP_y + (kP_y_small - kP_y) * t;
}

void loop() {
  if (rotev.goButtonPressed()) {
    while (rotev.goButtonPressed()) {
      rotev.ledWrite(0, 0.2, 0);
    }
    rotev.motorEnable(true);
    rotev.motorWrite1(0.0f);
    rotev.motorWrite2(0.0f);
    going = true;
    posX = 0.0f;
    posY = 0.0f;
    heading = 0.03f;
    vel1 = 0.0f;
    vel2 = 0.0f;

    posY = -CURVE_DIA;
    doneCurve = false;

    start = millis();
    float gyroSum = 0.0f;
    int gyroSamples = 0;
    for (int i = 0; i < 10; i++) {
      prevAngle1 = rotev.enc1Angle();
      prevAngle2 = rotev.enc2Angle();
      uint32_t tStart = millis();
      while (millis() - tStart < 533) {
        gyroSum += rotev.readYawRate();
        gyroSamples++;
        delay(1);
      }
    }
    gyroOffset = gyroSum / (float)gyroSamples;

    prevTime = micros();
  } else if (rotev.stopButtonPressed()) {
    rotev.ledWrite(0.2, 0, 0);
    rotev.motorEnable(false);
    going = false;
  } else {
    rotev.ledWrite(0.1, 0.0, 0.1);
  }

  float vbus = rotev.getVoltage();
  float dt = (float)(micros() - prevTime) / 1000000.0f;
  prevTime = micros();

  float angle1 = rotev.enc1Angle();
  float angle2 = rotev.enc2Angle();
  float delta1 = wrapDelta(angle1 - prevAngle1) * ENC1_DIR;
  float delta2 = wrapDelta(angle2 - prevAngle2) * ENC2_DIR;
  float vel1_raw = delta1 / dt;
  float vel2_raw = delta2 / dt;
  vel1 = (vel1 * 0.5f) + (vel1_raw * 0.5f);
  vel2 = (vel2 * 0.5f) + (vel2_raw * 0.5f);
  prevAngle1 = angle1;
  prevAngle2 = angle2;

  float newYawRate = rotev.readYawRate() - gyroOffset;
  heading += newYawRate * dt;
  if (heading < -M_PI) heading += 2 * M_PI;
  else if (heading > M_PI) heading -= 2 * M_PI;

  float averageHeading = heading + newYawRate * dt / 2.0f;
  float delPos = (delta1 + delta2) / 2.0f * CM_PER_RAD;
  posX += delPos * cosf(averageHeading);
  posY += delPos * sinf(averageHeading);

  float yCoeff = getYCoeff(posX);
  float yErr = yCoeff * posY;
  if (yErr > 0.79) yErr = 0.79;
  else if (yErr < -0.79) yErr = -0.79;

  float w = kP_h * (heading + yErr) - kD_h * newYawRate;
  float v = kP_x * (TRACK_LENGTH - posX);
  if (v > 6.0f) v = 6.0f;
  else if (v < -6.0f) v = -6.0f;

  float speedMult = getSpeedMultiplier(posX);
  v *= speedMult;

  if (posX > (TRACK_LENGTH/2.0f + 25.0f) && !doneCurve) {
    doneCurve = true;
    posY = CURVE_DIA - 16.0f;
  }

  if (millis() - start < 500) {
    v *= (float)(millis() - start)/500.0f;
  }

  float volt1 = v - w;
  float volt2 = v + w;
  rotev.motorWrite1(limitCurrPredictive(vel1, volt1)/vbus * MOTOR1_DIR);
  rotev.motorWrite2(limitCurrPredictive(vel2, volt2)/vbus * MOTOR2_DIR);

  if (millis() - lastPrint > 50) {
    Serial.print("curr1:"); Serial.print(rotev.motorCurr1());
    Serial.print(",curr2:"); Serial.print(rotev.motorCurr2());
    Serial.print(",posx:"); Serial.print(posX);
    Serial.print(",posy:"); Serial.print(posY);
    Serial.print(",heading:"); Serial.print(heading);
    Serial.print(",vel1:"); Serial.print(vel1 * CM_PER_RAD);
    Serial.print(",vel2:"); Serial.print(vel2 * CM_PER_RAD);
    Serial.print(",voltage:"); Serial.print(rotev.getVoltage());
    Serial.print(",v:"); Serial.print(v);
    Serial.print(",speedMult:"); Serial.print(speedMult);
    Serial.print(",gyroOff:"); Serial.print(gyroOffset, 5);
    Serial.println();
    lastPrint = millis();
  }
}
