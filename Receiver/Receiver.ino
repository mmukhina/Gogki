/*
* Arduino Wireless Communication Tutorial
*       Example 1 - Receiver Code
*                
* by Dejan Nedelkovski, www.HowToMechatronics.com
* 
* Library: TMRh20/RF24, https://github.com/tmrh20/RF24/
*/

#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
#include <Servo.h>

#define MOTOR_MIDDLE_VALUE 1500

RF24 radio(7, 8); // CE, CSN

const byte address[6] = "00001";

struct DataStruct {
  int angleX;
  int angleY;
  int head_rotation;
  int eye_rotation;
};

DataStruct data;

Servo motorX;
Servo motorY;
Servo motorHead;
Servo motor4;
Servo motor5;
Servo motor6;
Servo motor7;

void setup() {
  Serial.begin(9600);
  radio.begin();
  radio.openReadingPipe(0, address);
  radio.setPALevel(RF24_PA_MIN);
  radio.startListening();

  motorX.attach(A1);
  motorY.attach(A2);
  motorHead.attach(A3);
  motor4.attach(A4);
  motor5.attach(A5);
  motor6.attach(A6);
  motor7.attach(A7);  

  motorX.writeMicroseconds(MOTOR_MIDDLE_VALUE);
  motorY.writeMicroseconds(MOTOR_MIDDLE_VALUE);
  motorZ.writeMicroseconds(MOTOR_MIDDLE_VALUE);
  motor4.writeMicroseconds(MOTOR_MIDDLE_VALUE);
  motor5.writeMicroseconds(MOTOR_MIDDLE_VALUE);
  motor6.writeMicroseconds(MOTOR_MIDDLE_VALUE);
  motor7.writeMicroseconds(MOTOR_MIDDLE_VALUE);

  pinMode(4, OUTPUT);
  digitalWrite(9, HIGH);
  delay(500);
  digitalWrite(9, LOW);
  Serial.println("ready");
}

void loop() {
  if (radio.available()) {
    radio.read(&data, sizeof(data));

    motorX.write(data.angleX);
    motorY.write(data.angleY);

    Serial.println(data.angleY);

    digitalWrite(9, HIGH);
  } else {
    //digitalWrite(4, LOW);
    Serial.println("no signal");
  }
}