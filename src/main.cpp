#include <Arduino.h>

#include <HerkulexServo.h>

#define PIN_SW_RX  9
#define PIN_SW_TX  10
#define SERVO_ID_A 36
#define SERVO_ID_B 37
#define SERVO_ID 254


#define bras_gauche 0
#define bras_droit 1


void individual_move(HerkulexServoBus bus,HerkulexServo servo, uint16_t pos, uint8_t playtime);

HerkulexServoBus herkulex_bus(Serial2);

// initialiser le tableau avec tous les servos utilisés
// id : 21 à 27, 31 à 27
// création d'un tableau de servos

/* HerkulexServo* tableauServo[2][7] = {{nullptr}};*/

HerkulexServo    servo_a(herkulex_bus, 36);
HerkulexServo    servo_b(herkulex_bus, 37);
HerkulexServo    servo_tout(herkulex_bus, SERVO_ID);

void setup() {

  // initialiser les servos
  /*for (int i = 0; i < 2; ++i) {
        for (int j = 0; j < 7; ++j) {
            tableauServo[i][j] = new HerkulexServo(herkulex_bus,((i+2)*10+j)); // Crée un nouvel objet HerkulexServo avec le bon id
        }
    }*/

  // initialiser la liaison série
    Serial2.setTX(PIN_SW_TX);
    Serial2.setRX(PIN_SW_RX);
    Serial.begin(9600);
    Serial2.begin(666666);
    delay(500);

  // créer un écouteur d'évenements sur serial, pour lire si des infos servos sont recues
  //Serial2.onReceive(gestion_interruption);
  
  // il faut faire attention dès qu'on recoit un octet qui correspond à --> "D" en ascii (0x44) soit 68 en décimal
  
  // turn power on
  servo_tout.setTorqueOn();

 // individual_move(herkulex_bus,servo_a, 256, 50);
  delay(100 * 11.2f);
}

void loop() {
  /*
  herkulex_bus.prepareIndividualMove();
  servo_a.setPosition(256, 50, HerkulexLed::Cyan);
  herkulex_bus.executeMove();
  herkulex_bus.update();
  delay(100 * 11.2f);
  herkulex_bus.prepareIndividualMove();
  servo_a.setPosition(1024, 50, HerkulexLed::Purple);
  herkulex_bus.executeMove();
  herkulex_bus.update();
  delay(100 * 11.2f);*/
  
  servo_a.setLedColor(HerkulexLed::Cyan);
  servo_b.individual_move(256,100, HerkulexLed::Yellow);

  servo_a.setLedColor(HerkulexLed::Purple);
  servo_b.individual_move(1024,100, HerkulexLed::Yellow);

  servo_b.setLedColor(HerkulexLed::White);
  servo_a.individual_move(256,100, HerkulexLed::Yellow);

  servo_b.setLedColor(HerkulexLed::Green);
  servo_a.individual_move(1024,100, HerkulexLed::Yellow);
  
}

// gestion d'interruptions
void gestion_interruption(){
  // création d'un tableau pour stocker les valeurs

  // lecture des octets un par un jusqu'à l'octet de stop "F" soit 0x46 en ascii 70 en décimal

  // traitement des octets reçus 
  // commandes à executer en fonction des valeurs recues
}

 