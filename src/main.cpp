#include <Arduino.h>
#include <HerkulexServo.h>


#define SERVO_ID  0xFE

// SoftwareSerial   servo_serial(PIN_SW_RX, PIN_SW_TX);

HerkulexServoBus herkulex_bus(Serial2);
HerkulexServo    my_servo(herkulex_bus, SERVO_ID);


void setup() {
  
  Serial.begin(115200);
  Serial2.setTX(10);
  Serial2.setRX(9);
  Serial2.begin(115200);
}

void loop() {
  herkulex_bus.update();

  my_servo.setLedColor(HerkulexLed::White);
  delay(1000);
  my_servo.setLedColor(HerkulexLed::Off);
  delay(1000);
}
