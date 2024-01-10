#include <Arduino.h>

#include <HerkulexServo.h>

#define PIN_SW_RX  9
#define PIN_SW_TX  10
#define SERVO_ID_A 36



HerkulexServoBus herkulex_bus(Serial2);
HerkulexServo    servo_a(herkulex_bus, SERVO_ID_A);



void setup() {
    Serial2.setTX(PIN_SW_TX);
    Serial2.setRX(PIN_SW_RX);
    Serial.begin(9600);
    Serial2.begin(666666);
    delay(500);

  // turn power on
  servo_a.setTorqueOn();
  

  herkulex_bus.prepareIndividualMove();
  servo_a.setPosition(256, 50,HerkulexLed::Purple);
  herkulex_bus.executeMove();

  delay(100 * 11.2f);

  // turn power off
  servo_a.setTorqueOff();
}

void loop() {
  herkulex_bus.update();
}
