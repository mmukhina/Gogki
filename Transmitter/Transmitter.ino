#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"

// Constants
const byte address[6] = "00001";

#define CALIB_DURATION 2000
#define NO_SIGNAL_TIMEOUT 2000

#define RESET_PIN 5
#define RED_PIN A3
#define GREEN_PIN 3
#define BLUE_PIN A2
#define HEAD_FOLD_POT_PIN A0
#define EYE_POT_PIN A1

#define POT_MIN 0
#define POT_MAX 1024

#define SERVO_MIN 1000
#define SERVO_MAX 2000

#define EYE_MIN 1000
#define EYE_MAX 2000

#define ROTATION_LIMIT 45
#define ROTATION_CENTER 1500
#define ROTATION_LOW 1450
#define ROTATION_HIGH 1550

bool calib_flag = true;
unsigned long calibStartTime = 0;

int prev_x;
int prev_y;
int prev_z;

bool reset_flag = false;

int angleX_offset = 0;
int angleY_offset = 0;
int angleZ_offset = 0;

struct DataStruct {
  int angleX = 0;
  int angleY = 0;
  int angleZ = 0;
  int head_fold;
  int eyes;
};

RF24 radio(7, 8);  // CE, CSN
MPU6050 mpu;
volatile bool mpuFlag = true;  // Flag for interrupt readiness
uint8_t fifoBuffer[45];        // Buffer for FIFO

DataStruct data;

void setup() {
  setColor(0, 0, 255);
  Serial.begin(9600);
  radio.begin();
  radio.openWritingPipe(address);
  radio.setPALevel(RF24_PA_MIN);
  radio.stopListening();

  Wire.begin();

  // Initialize MPU
  mpu.initialize();
  mpu.dmpInitialize();
  mpu.setDMPEnabled(true);
  attachInterrupt(0, dmpReady, RISING);

  pinMode(HEAD_FOLD_POT_PIN, INPUT);
  pinMode(EYE_POT_PIN, INPUT);
  pinMode(RESET_PIN, INPUT);

  pinMode(RED_PIN, OUTPUT);
  pinMode(GREEN_PIN, OUTPUT);
  pinMode(BLUE_PIN, OUTPUT);
}

// Interrupt handler for DMP readiness
void dmpReady() {
  mpuFlag = true;
}

void loop() {
  if (mpuFlag && mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) {
    mpuFlag = false;

    // Check for FIFO overflow
    if (mpu.getFIFOCount() & 1024) {
      setColor(255, 0, 0);
      Serial.println("FIFO overflow!");
      mpu.resetFIFO();
      return;
    }

    // Variables for calculations
    Quaternion q;
    VectorFloat gravity;
    float ypr[3];

    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);

    data.angleX = ypr[1] * (180 / PI);
    data.angleY = ypr[0] * (180 / PI);
    data.angleZ = ypr[2] * (180 / PI);

    if (calib_flag) {
      if (prev_x == data.angleX && prev_y == data.angleY && prev_z == data.angleZ) {
        if (calibStartTime == 0) {
          calibStartTime = millis();
        }
        if (millis() - calibStartTime >= CALIB_DURATION) {
          calib_flag = false;
          Serial.println("Calibration done");
        }
      } else {
        calibStartTime = 0;
      }

      prev_x = data.angleX;
      prev_y = data.angleY;
      prev_z = data.angleZ;

    } else {
      int head_fold_value = analogRead(HEAD_FOLD_POT_PIN);
      int eyes_value = analogRead(EYE_POT_PIN);

      data.head_fold = map(head_fold_value, POT_MIN, POT_MAX, SERVO_MIN, SERVO_MAX);
      data.eyes = map(eyes_value, POT_MIN, POT_MAX, EYE_MIN, EYE_MAX);

      // Handle reset button
      if (digitalRead(RESET_PIN) && !reset_flag) {
        angleX_offset = data.angleX;
        angleY_offset = data.angleY;
        angleZ_offset = data.angleZ;
        reset_flag = true;
      } else if (!digitalRead(RESET_PIN)) {
        reset_flag = false;
      }

      data.angleX -= angleX_offset;
      data.angleY -= angleY_offset;
      data.angleZ -= angleZ_offset;

      data.angleX = wrapAngle(data.angleX);
      data.angleY = wrapAngle(data.angleY);
      data.angleZ = wrapAngle(data.angleZ);

      data.angleX = constrain(data.angleX, -90, 90);
      data.angleY = constrain(data.angleY, -90, 90);
      data.angleZ = constrain(data.angleZ, -90, 90);

      data.angleX = map(data.angleX, -90, 90, SERVO_MIN, SERVO_MAX);
      data.angleZ = map(data.angleZ, -90, 90, SERVO_MIN, SERVO_MAX);

      Serial.print(data.angleX);
      Serial.print(" ");
      Serial.print(data.angleY);
      Serial.print(" ");
      Serial.println(data.angleZ);

      if (data.angleY > ROTATION_LIMIT) {
        data.angleY = ROTATION_HIGH;
      } else if (data.angleY < -ROTATION_LIMIT) {
        data.angleY = ROTATION_LOW;
      } else {
        data.angleY = ROTATION_CENTER;
      }


      if (radio.write(&data, sizeof(data))) {
        setColor(0, 255, 0);
      } else {
        setColor(255, 0, 0);
        //Serial.println("Transmission failed!");
      }
    }
  }
}

void setColor(int redValue, int greenValue, int blueValue) {
  analogWrite(RED_PIN, redValue);
  digitalWrite(GREEN_PIN, greenValue);
  analogWrite(BLUE_PIN, blueValue);
}

int wrapAngle(int angle) {
  if (angle > 180) {
    angle -= 360;
  }
  if (angle < -180) {
    angle += 360;
  }
  return angle;
}
