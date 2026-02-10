#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"

const byte ADDRESS[6] = "00001";

bool reset_flag = false;
int angleX_offset = 0;
int angleY_offset = 0;

#define HEAD_ROTATION_POT_PIN A0
#define EYE_POT_PIN A1
#define RESET_PIN 5

struct DataStruct {
  int angleX;
  int angleY;
  int head_rotation;
  int eye_rotation;
};

RF24 radio(7, 8);  // CE, CSN
MPU6050 mpu;
volatile bool mpuFlag = true;  // Flag for interrupt readiness
uint8_t fifoBuffer[45];        // Buffer for FIFO

DataStruct data;

void setup() {
  Serial.begin(9600);
  radio.begin();
  radio.openWritingPipe(ADDRESS);
  radio.setPALevel(RF24_PA_MIN);
  radio.stopListening();

  Wire.begin();

  // Initialize MPU6050
  mpu.initialize();
  mpu.dmpInitialize();
  mpu.setDMPEnabled(true);
  attachInterrupt(0, dmpReady, RISING);

  data.angleX = 90;
  data.angleY = 90;

  pinMode(HEAD_ROTATION_POT_PIN, INPUT);
  pinMode(EYE_POT_PIN, INPUT);
  pinMode(RESET_PIN, INPUT);
}

// Interrupt handler for DMP readiness
void dmpReady() {
  mpuFlag = true;
}

void loop() {
  if (mpuFlag && mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) {
    // Variables for calculations
    Quaternion q;
    VectorFloat gravity;
    float ypr[3];

    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
    mpuFlag = false;

    // Get current angles
    data.angleX = ypr[1] * (180 / PI);
    data.angleY = ypr[2] * (180 / PI);

    // Read potentiometer values
    data.head_rotation = map(analogRead(HEAD_ROTATION_POT_PIN), 0, 1024, 1200, 1800);
    data.eye_rotation = map(analogRead(EYE_POT_PIN), 0, 1024, 1000, 2000);

    // Handle reset button
    int reset_state = digitalRead(RESET_PIN);
    if (reset_state && !reset_flag) {
      angleX_offset = data.angleX;
      angleY_offset = data.angleY;
      reset_flag = true; 
    }
    if (!reset_state && reset_flag) {
      reset_flag = false;
    }

    data.angleX -= angleX_offset;
    data.angleY -= angleY_offset;

    Serial.println(data.angleX);

    radio.write(&data, sizeof(data));
  }
}
