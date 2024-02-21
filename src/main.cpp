#include <Arduino.h>

#include <HerkulexServo.h>

#define PIN_SW_RX  9
#define PIN_SW_TX  10
#define SERVO_ID_A 36
#define SERVO_ID_B 37
#define SERVO_ID 254
void individual_move(HerkulexServoBus bus,HerkulexServo servo, uint16_t pos, uint8_t playtime);

HerkulexServoBus herkulex_bus(Serial2);
HerkulexServo    servo_a(herkulex_bus, SERVO_ID_A);
HerkulexServo    servo_b(herkulex_bus, SERVO_ID_B);
HerkulexServo    servo_tout(herkulex_bus, SERVO_ID);

void setup() {
    Serial2.setTX(PIN_SW_TX);
    Serial2.setRX(PIN_SW_RX);
    Serial.begin(9600);
    Serial2.begin(666666);
    delay(500);

  // turn power on
  servo_a.setTorqueOn();

 // individual_move(herkulex_bus,servo_a, 256, 50);
  delay(100 * 11.2f);
}

void loop() {
  
  /*herkulex_bus.prepareIndividualMove();
  servo_a.setPosition(256, 50, HerkulexLed::Cyan);
  herkulex_bus.executeMove();
  herkulex_bus.update();
  delay(100 * 11.2f);
  herkulex_bus.prepareIndividualMove();
  servo_a.setPosition(1024, 50, HerkulexLed::Purple);
  herkulex_bus.executeMove();
  herkulex_bus.update();
  delay(100 * 11.2f);*/

  servo_tout.setLedColor(HerkulexLed::Cyan);
  delay(100);
  servo_tout.setLedColor(HerkulexLed::Purple);
  delay(100);
}

 void individual_move(HerkulexServoBus bus,HerkulexServo servo, uint16_t pos, uint8_t playtime) {
  
  bus.prepareIndividualMove();
  servo.setPosition(pos,playtime, HerkulexLed::Purple);
  bus.executeMove();

 }


 