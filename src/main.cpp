#include <Arduino.h>
#include <HerkulexServo.h>
#include <stdio.h>


#define SERVO_ID  0xFE

HerkulexServoBus herkulex_bus(Serial2);
HerkulexServo my_servo(herkulex_bus, SERVO_ID);

uint16_t reponse;
HerkulexStatusError error;
HerkulexStatusDetail detail;

void setup() {

  Serial2.setTX(10);
  Serial2.setRX(9);
  Serial2.begin(666666);
  Serial.begin(9600);
  my_servo.writeRam(HerkulexRamRegister::AlarmLedPolicy, 0x00);

}

void loop() {
  herkulex_bus.update();
  my_servo.setLedColor(HerkulexLed::Green);
  reponse = my_servo.readRam2(HerkulexRamRegister::ID);
  Serial.print("ID: ");
  Serial.println(reponse);
  my_servo.getStatus(error, detail);
  Serial.print("error: "); 
  Serial.println(static_cast<int>(error), HEX); // Convert error to int before printing
  Serial.print("detail: ");
  Serial.println(static_cast<int>(detail),HEX); // Convert detail to int before printing
  delay(1000);
}