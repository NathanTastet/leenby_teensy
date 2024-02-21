#include <Arduino.h>

#include <HerkulexServo.h>

#define PIN_SW_RX  9
#define PIN_SW_TX  10
#define SERVO_ID_A 36
#define SERVO_ID_B 37
#define SERVO_ID 254


#define bras_gauche 0
#define bras_droit 1
#define TAILLE_MESSAGE_MAX 17
#define VITESSE_MAX_DEG_S 360 // en réel on est à max 408 deg/s mais on met une marge de sécurité


void individual_move(HerkulexServoBus bus,HerkulexServo servo, uint16_t pos, uint8_t playtime);

HerkulexServoBus herkulex_bus(Serial2);

// initialiser le tableau avec tous les servos utilisés
// id : 21 à 27, 31 à 27
// création d'un tableau de servos

HerkulexServo* tableauServo[2][8] = {{nullptr}};

HerkulexServo    servo_tout(herkulex_bus, SERVO_ID);

void setup() {

  // initialiser les servos
  for (int i = 0; i < 2; ++i) {
      for (int j = 0; j < 7; ++j) {
          tableauServo[i][j] = new HerkulexServo(herkulex_bus,((i+2)*10+j)); // Crée un nouvel objet HerkulexServo avec le bon id
      }
  }

  // initialiser la liaison série
    Serial2.setTX(PIN_SW_TX);
    Serial2.setRX(PIN_SW_RX);
    Serial.begin(9600);
    Serial2.begin(666666);
    delay(500);
  
  // turn power on
  servo_tout.setTorqueOn();

 // individual_move(herkulex_bus,servo_a, 256, 50);
  delay(100 * 11.2f);
}

void loop() {

  // il faut faire attention dès qu'on recoit un octet qui correspond à --> "D" en ascii (0x44) soit 68 en décimal
  // sur serial
  if(Serial.available()){
    // on lit l'octet
    char octetlu = Serial.read();
    // on traite l'octet
    if(octetlu ==  68){
      // on appelle la fonction de gestion d'interruption
      interruption();
    }
  }

  
}

// gestion d'interruptions
void interruption(){
  while(Serial.available()< TAILLE_MESSAGE_MAX){
    // on attend de recevoir les 16 octets du message complet
  }
  char octets_recus[TAILLE_MESSAGE_MAX];
  // on lit les 16 octets et on les stocke dans un tableau
  for(int i = 0; i <TAILLE_MESSAGE_MAX; i++){
    octets_recus[i] = Serial.read();
    // si on recoit un octet qui correspond à "F" en ascii (0x46) soit 70 en décimal
    // on sort de la boucle (normalement c'est le dernier octet du message reçu)
    if(octets_recus[i] == 70){
      break;
    } 
  }
  
  // lecture des octets un par un jusqu'à l'octet de stop "F" soit 0x46 en ascii 70 en décimal
  // traitement des octets reçus 

  // 17 octets :
  // 1 octet pour le choix bras en short [0]
  // 14 octets pour les 7 angles en long de [1] à [14]
  // 1 octet pour la vitesse en short [15]
  // 1 octet de stop [16]

  char bras = octets_recus[0];
  char vitessept = octets_recus[15];
  // on a une vitesse en % de la vitesse max
  // on a une vitesse max de 360 deg/s
  char vitesse = (vitessept/100)*VITESSE_MAX_DEG_S;

  // les angles sont sur 2 octets uint_16

  for(int i = 1; i < 7; i+=2){
    tableauServo[bras][i]->setLedColor(HerkulexLed::Green);
    uint16_t angle_degre = (octets_recus[i] << 8) | octets_recus[i+1];
    uint16_t tps_ms = (angle_degre/vitesse)*1000; // calcul du temps en s puis x1000 pour avoir le temps en ms
    tableauServo[bras][i]->individual_move(angle_degre,tps_ms, HerkulexLed::Yellow);
  }
  
}

 