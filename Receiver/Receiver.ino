#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
#include <Servo.h>

#define MOTOR_MIDDLE_VALUE 1500
#define NO_SIGNAL_TIMEOUT 1000 
#define STEP_SIZE 10

#define LED_PIN 9
#define MOTORX_PIN A1
#define MOTORY_PIN A3
#define MOTORZ_PIN A2
#define MOTOR_HEAD_PIN A4
#define MOTOR_EYES_PIN A5

RF24 radio(7, 8); // CE, CSN

const byte address[6] = "00001";

struct DataStruct {
  int angleX;
  int angleY;
  int angleZ;
  int head_rotation;
  int eye_rotation;
};

DataStruct data;

Servo motorX;
Servo motorY;
Servo motorZ;
Servo motorHead;
Servo motorEyes;

unsigned long lastSignalTime = 0; 

int currentPosX = MOTOR_MIDDLE_VALUE;
int currentPosY = MOTOR_MIDDLE_VALUE;
int currentPosZ = MOTOR_MIDDLE_VALUE;
int currentPosHead = MOTOR_MIDDLE_VALUE;
int currentPosEyes = MOTOR_MIDDLE_VALUE;

void setup() {
  Serial.begin(9600);
  radio.begin();
  radio.openReadingPipe(0, address);
  radio.setPALevel(RF24_PA_MIN);
  radio.startListening();

  motorX.attach(MOTORX_PIN);
  motorY.attach(MOTORY_PIN);
  motorZ.attach(MOTORZ_PIN);
  motorHead.attach(MOTOR_HEAD_PIN);
  motorEyes.attach(MOTOR_EYES_PIN);

  motorX.writeMicroseconds(MOTOR_MIDDLE_VALUE);
  motorY.writeMicroseconds(MOTOR_MIDDLE_VALUE);
  motorZ.writeMicroseconds(MOTOR_MIDDLE_VALUE);
  motorHead.writeMicroseconds(MOTOR_MIDDLE_VALUE);
  motorEyes.writeMicroseconds(MOTOR_MIDDLE_VALUE);

  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH);
  Serial.println("ready");
}

void smoothMove(Servo& motor, int& currentPos, int targetPos) {
  if (currentPos != targetPos) {
    if (currentPos < targetPos) {
      currentPos += STEP_SIZE;
      if (currentPos > targetPos) currentPos = targetPos;
    } else {
      currentPos -= STEP_SIZE;
      if (currentPos < targetPos) currentPos = targetPos;
    }
    motor.writeMicroseconds(currentPos);  
  }
}

void loop() {
  if (radio.available()) {
    radio.read(&data, sizeof(data));

    smoothMove(motorX, currentPosX, data.angleX);
    smoothMove(motorY, currentPosY, data.angleY);
    smoothMove(motorZ, currentPosZ, data.angleZ);
    smoothMove(motorHead, currentPosHead, data.head_rotation);
    smoothMove(motorEyes, currentPosEyes, data.eye_rotation);

    digitalWrite(LED_PIN, HIGH);
    lastSignalTime = millis();
  } else {
    if (millis() - lastSignalTime >= NO_SIGNAL_TIMEOUT) {
      digitalWrite(LED_PIN, LOW);
      Serial.println("no signal");
    }
  }
}
