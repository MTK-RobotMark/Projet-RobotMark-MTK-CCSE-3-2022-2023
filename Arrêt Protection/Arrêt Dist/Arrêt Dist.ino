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
// Macros
#define LedToggle digitalWrite(13, !digitalRead(13))
#define MoteurG(Vg) OCR5A=Vg // Vg in [0... 1999]
#define MoteurD(Vd) OCR5B=Vd // VD in [0... 1999]
#define MoteurGD(Vg,Vd) MoteurG(Vg);MoteurD(Vd)
#define StopMoteurGD MoteurGD(Stop,Stop)

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

    float K;
    K = 1.025;

    MoteurGD(600,(600*K));

if (DistanceCapteurDroite <10 || DistanceCapteurGauche < 10 || DistanceCapteurAvant < 10) {
    MoteurGD(400, 400);
 }
  if (digitalRead(infrared)) {
    MoteurGD(400, 400);
  } }
