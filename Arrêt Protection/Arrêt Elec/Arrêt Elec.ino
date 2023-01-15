#define Thash 800
#define Stop 400
#define Vmax Thash
#include <MARK.h>
#include <Wire.h>  //lib for I2C connection
#include "Ultrasonic.h"
#include "rgb_lcd.h"
Ultrasonic ultrasonic1(8); //Init of ultrasonic snesor on pin 8
Ultrasonic ultrasonic2(10);
Ultrasonic ultrasonic3(12);
const byte infrared = 6;
MARK myrobot;

//Entrées analogiques
pinMode(A2, INPUT);
pinMode(A3, INPUT);

float CourantMesure;
float TensionAnalogique;

// Macros
#define LedToggle digitalWrite(13, !digitalRead(13))
#define MoteurG(Vg) OCR5A=Vg // Vg in [0... 1999]
#define MoteurD(Vd) OCR5B=Vd // VD in [0... 1999]
#define MoteurGD(Vg,Vd) MoteurG(Vg);MoteurD(Vd)
#define StopMoteurGD MoteurGD(Stop,Stop)
Encoder knobLeft(18, 33);            //On définit le pin pour l'encodeur Gauche
Encoder knobRight(31, 19);           //On définit le pin pour l'encodeur Droite

void initMoteurs() {  // MoteurG :OC5A=PIN46-PL3, MoteurD : OC5B=PIN45-PL4
  DDRL = 0x18 ; // PL3 et PL4
  DDRB = 0x80 ; // PB7 LedToggle
  // COM5B_1:0 = 10   -> clear sur egalite++, set sur egalite--
  // WGM5_3:1 = 1000  -> mode 8 => ICR5 defini le TOP
  TCCR5A = (1 << COM5A1) + (1 << COM5B1);
  TCCR5B = (1 << ICNC5) + (1 << WGM53) + (1 << CS50); // CS_12:10  = 001  -> prediv par 1
  ICR5 = Thash; // 1999 correspond a f = 4khz
  StopMoteurGD;
  // Interrution de débordement du timer
  TIMSK5 = 1 << TOIE5;
}

ISR (TIMER5_OVF_vect) { // Pour la lecture du courant
  LedToggle;
}

void setup() {
  pinMode(43, OUTPUT);
  digitalWrite(43, 0);
  initMoteurs();
  sei();
  digitalWrite(43, 1);

}

void loop() {
  long DistanceCapteurAvant;
  long DistanceCapteurDroite;
  long DistanceCapteurGauche;
  DistanceCapteurAvant = ultrasonicF.MeasureInCentimeters();
  DistanceCapteurGauche = ultrasonicG.MeasureInCentimeters();
  DistanceCapteurDroite = ultrasonicD.MeasureInCentimeters();
  newLeft = knobLeft.read();    //Lecture roue gauche
  newRight = knobRight.read();  //Lecture roue droite
  TensionAnalogique = (analogRead(A3) + analogRead(A4));
  CourantMesure = (TensionAnalogique * 5 / 1023 / 100 * 10 ^ -3);
    float K;
    K = 1.025;

    MoteurGD(600,(600*K));

 //---------------Arrêt Elec-------------//
    if (CourantMesure > 1) {
      millis();
      Arret_Elec();
    }
  }
}
