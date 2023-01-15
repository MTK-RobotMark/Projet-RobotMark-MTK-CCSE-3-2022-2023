//Valeurs Prédéfinies
#define Thash 800
#define Stop 400
#define Vmax Thash
#define VmoyG 700
#define VmoyD 730
#define Angle90 420
#define Imoy 490
#define Inom 1


//Entrées analogiques
pinMode(A2, INPUT);
pinMode(A3, INPUT);

// Macros
#define LedToggle digitalWrite(13, !digitalRead(13))
#define MoteurG(Vg) OCR5A = Vg  // Vg in [0... 1999]
#define MoteurD(Vd) OCR5B = Vd  // VD in [0... 1999]
#define MoteurGD(Vg, Vd) \
  MoteurG(Vg); \
  MoteurD(Vd)
#define StopMoteurGD MoteurGD(Stop, Stop)

//Bibliothèques
#include <MARK.h>        //librairie du robot MARK
#include <Wire.h>        //lib for I2C connection
#include "Ultrasonic.h"  //Pour les Ultrasons
#include "rgb_lcd.h"     //Pour le LCD
#include <Encoder.h>     //Librairie des encodeurs
MARK myrobot;

//Déclaration des capteurs ultrasons
Ultrasonic ultrasonicF(8);   //Init of ultrasonic sensor on pin 8
Ultrasonic ultrasonicG(10);  //Init of ultrasonic sensor on pin 10
Ultrasonic ultrasonicD(12);  //Init of ultrasonic sensor on pin 12

//Encodeur
unsigned long LimEncSecDroite, LimEncSecGauche, ProprieteEncDroite, ProprieteEncGauche, LimDegSec;
Encoder knobLeft(18, 33);            //On définit le pin pour l'encodeur Gauche
Encoder knobRight(31, 19);           //On définit le pin pour l'encodeur Droite
unsigned long positionLeft = -999;   //On définit une position de base
unsigned long positionRight = -999;  //On définit une position de base
#define ProprieteEncDroite 1180      //Valeur de l'encodeur pour 1 tour de la roue droite
#define ProprieteEncGauche 1150      //Valeur de l'encodeur pour 1 tour de la roue gauche
#define LimDegSec 200                //Valeur limite
int DifferentielGauche;              //On mesure la différence de position t à t-1
int DifferentielDroite;              //On mesure la différence de position t à t-1

//Variables pour l'affichage des diverses informations
unsigned long Temps_start_ms;
unsigned long Temps_stop_ms;
unsigned long Duree_ms;
unsigned long Duree_s;
unsigned long Duree_h;
unsigned long SaveR;
unsigned long SaveL;
unsigned long Vf_L;
unsigned long Vf_R;
unsigned long Nm;
unsigned long courant, a;

//Variables pour les ultrasons
unsigned long DistanceCapteurAvant;
unsigned long DistanceCapteurDroite;
unsigned long DistanceCapteurGauche;

const byte infrared = 6;
boolean arret = false;
float CourantMesure;
float TensionAnalogique;

void initMoteurs() {  // MoteurG :OC5A=PIN46-PL3, MoteurD : OC5B=PIN45-PL4
  DDRL = 0x18;        // PL3 et PL4
  DDRB = 0x80;        // PB7 LedToggle
  // COM5B_1:0 = 10   -> clear sur egalite++, set sur egalite--
  // WGM5_3:1 = 1000  -> mode 8 => ICR5 defini le TOP
  TCCR5A = (1 << COM5A1) + (1 << COM5B1);
  TCCR5B = (1 << ICNC5) + (1 << WGM53) + (1 << CS50);  // CS_12:10  = 001  -> prediv par 1
  ICR5 = Thash;                                        // 1999 correspond a f = 4khz
  StopMoteurGD;
  // Interrution de débordement du timer
  TIMSK5 = 1 << TOIE5;
}

ISR(TIMER5_OVF_vect) {  // Pour la lecture du courant
  LedToggle;
}

void affichage() {
  int gps = 0;
  int Energie;
  int Vout;
  Duree_s = Duree_ms / 1000;
  Duree_h = Duree_s / 60 / 24;

  float distance_Robot = (((Vf_R / 1180) + (Vf_L / 1150)) * 0.1);
  long NbtourG = (Vf_L / 1150);  //Nb de tours roue gauche
  long NbtourD = (Vf_R / 1180);  //Nb de tours roue droite
  Vout = Imoy * 2;
  Energie = Vout * Imoy * Duree_h;

  while (1) {
    if ((myrobot.getJoystickY()) < 358) {
      gps++;
      while ((myrobot.getJoystickY()) < 358)
        ;
    }

    else if ((myrobot.getJoystickY()) > 671) {
      gps--;
      while ((myrobot.getJoystickY()) > 671)
        ;
    }
    switch (gps) {
      case 0:
        myrobot.setLcdCursor(0, 0);
        myrobot.lcdPrint("Fin de course");
        myrobot.setLcdCursor(0, 1);
        myrobot.lcdPrint("                ");
        break;


      case 1:
        myrobot.setLcdCursor(0, 0);
        myrobot.lcdPrint("Temps parcours        ");
        myrobot.setLcdCursor(0, 1);
        myrobot.lcdPrint(Duree_s);
        myrobot.lcdPrint("  secondes            ");
        break;

      case 2:
        myrobot.setLcdCursor(0, 0);
        myrobot.lcdPrint(" Ener Elec Conso       ");
        myrobot.setLcdCursor(0, 1);
        myrobot.lcdPrint(Energie);
        break;

      case 3:
        myrobot.setLcdCursor(0, 0);
        myrobot.lcdPrint("le nombre Nm      ");
        myrobot.setLcdCursor(0, 1);
        myrobot.lcdPrint(Nm);
        myrobot.lcdPrint("     fois         ");
        break;

      case 4:
        myrobot.setLcdCursor(0, 0);
        myrobot.lcdPrint("Nb tour roue G    ");
        myrobot.setLcdCursor(0, 1);
        myrobot.lcdPrint(NbtourG);
        myrobot.lcdPrint("   tours          ");
        break;

      case 5:
        myrobot.setLcdCursor(0, 0);
        myrobot.lcdPrint("Nb tour roue R    ");
        myrobot.setLcdCursor(0, 1);
        myrobot.lcdPrint(NbtourD);
        myrobot.lcdPrint("   tours          ");
        break;

      case 6:
        myrobot.setLcdCursor(0, 0);
        myrobot.lcdPrint("    Distance    ");
        myrobot.setLcdCursor(0, 1);
        myrobot.lcdPrint(distance_Robot);
        myrobot.lcdPrint("   m            ");
        break;

      default:
        myrobot.setLcdCursor(0, 0);
        myrobot.lcdPrint(" Retournez en arrière   ");
        myrobot.setLcdCursor(0, 1);
        myrobot.lcdPrint("                        ");
    }
  }
}

//Interruption Arrêt DISTANCE
void Arret_Dist() {
  MoteurGD(Stop, Stop);  //Le robot s'arrête
  myrobot.lcdClear();
  myrobot.setLcdRGB(255, 0, 0);  //Rouge
  myrobot.setLcdCursor(0, 0);
  myrobot.lcdPrint("  Arret Distance   ");
  while (not(myrobot.getJoystickClic()))
    ;  //Attends un appuie sur le joystick
  myrobot.setLcdRGB(255, 255, 255);
}

void Arret_Elec() {
  MoteurGD(Stop, Stop);  //Le robot s'arrête
  myrobot.lcdClear();
  myrobot.setLcdRGB(255, 0, 0);  //Rouge
  myrobot.setLcdCursor(0, 0);
  myrobot.lcdPrint("  Arret Courant  ");
}


void setup() {
  myrobot.begin();
  pinMode(43, OUTPUT);
  digitalWrite(43, 0);
  initMoteurs();  //Appel de la fonction initMoteurs
  sei();          //On autorise les interruptions
  digitalWrite(43, 1);

  LimEncSecDroite = ((LimDegSec / 360) * ProprieteEncDroite);  //Ici on calcule la limite de l'encodeur droite
  LimEncSecGauche = ((LimDegSec / 360) * ProprieteEncGauche);  //Ici on calcule la limite de l'encodeur gauche

  myrobot.setLedBarLevel(myrobot.getBatteryLevel());

  while (myrobot.getInfrared() || (not(myrobot.getJoystickClic()))) {
    MoteurGD(Stop, Stop);
  }
  delay(1000);
  myrobot.setLcdRGB(0, 255, 0);
  myrobot.setLcdCursor(0, 0);
  myrobot.lcdPrint("Appuyez pour depart");
  myrobot.setLcdRGB(0, 0, 255);

  myrobot.lcdClear();
  Temps_start_ms = millis();  //Calcul le début de la course parcours
}


void loop() {
  DistanceCapteurAvant = ultrasonicF.MeasureInCentimeters();
  DistanceCapteurGauche = ultrasonicG.MeasureInCentimeters();
  DistanceCapteurDroite = ultrasonicD.MeasureInCentimeters();
  newLeft = knobLeft.read();    //Lecture roue gauche
  newRight = knobRight.read();  //Lecture roue droite
  TensionAnalogique = (analogRead(A3) + analogRead(A4));
  CourantMesure = (TensionAnalogique * 5 / 1023 / 100 * 10 ^ -3);
  int angle = 0;

  if (arret == false) {
    millis();

    if (DistanceCapteurDroite > 80 && DistanceCapteurGauche > 80) {  // Si le robot est au centre du couloir il va tout droit
      MoteurGD(VmoyG, VmoyD + 2);
      millis();  // La fonction millis() permet l'execution rapide de chaque boucle if et donc une presque exécution simultané des ces mêmes boucles
    }

    if (DistanceCapteurDroite < 80) {  // Si le robot est proche du mur droite il tourne légèrement à gauche
      MoteurGD(VmoyG, VmoyD + 27);
      millis();
    }

    if (DistanceCapteurGauche < 80) {  // Si le robot est proche du mur gauche il tourne légèrement à droite
      MoteurGD(VmoyG + 2, VmoyD);
      millis();
    }

    if (DistanceCapteurAvant < 329 && DistanceCapteurGauche > 90) {
      MoteurGD(VmoyG, VmoyD + 25);
      millis();
    }

    if (DistanceCapteurGauche < 110 && DistanceCapteurDroite > 100) {
      MoteurGD(VmoyG + 60, VmoyD);
      millis();
    }

    if (digitalRead(infrared)) {  // Le robot s'arrête si on le soulève ou passe sur la ligne d'arrivée
      myrobot.setLcdRGB(0, 255, 0);
      arret = true;
      Temps_stop_ms = millis();
      Vf_L = (myrobot.getEncoder("left"));   //On récupère le nombre d'incrémentation de la roue gauche
      Vf_R = (myrobot.getEncoder("right"));  //On récupère le nombre d'incrémentation de la roue droite

      MoteurGD(Stop, Stop);                       //Arret du robot
      Duree_ms = Temps_stop_ms - Temps_start_ms;  //Calcul du temps du parcours
      affichage();                                //Appel de la fonction affichage
    }

    if (DistanceCapteurAvant < 20 || DistanceCapteurDroite < 20 || DistanceCapteurGauche < 20) {
      millis();
      Nm++;  //On incrémente le nombre de fois où le robot est <20cm du mur.
    }


    if (DistanceCapteurAvant < 5 || DistanceCapteurDroite < 5 || DistanceCapteurGauche < 5) {  // Le robot s'arrête s'il arrive face à un mur
      millis();
      Arret_Dist();  //On entre dans l'interruption Arret Distance
    }

    //------------------Interruption Distance Méca et RB-------------------//

    //Si position actuel est differente de la nouvelle position gauche
    if (newLeft != positionLeft) {                  //Si la position a bougé
      DifferentielGauche = newLeft - positionLeft;  //Différence de position
      positionLeft = newLeft;                       //Nouvelle position

      if (DifferentielGauche < LimEncSecGauche) {  //Si la position est différente
        millis();
        myrobot.lcdPrint("Le robot avance");
      }
    } else {                         //Sinon
      MoteurGD(400, 400);            //robot stop
      myrobot.setLcdRGB(255, 0, 0);  //Affichage en couleur rouge
      myrobot.setLcdCursor(0, 0);
      myrobot.lcdPrint("Arret Gauche");  //message d'erreur
    }

    if (newRight != positionRight) {                  //Si la position a bougé
      DifferentielDroite = newRight - positionRight;  //Différence de position
      positionLeft = newLeft;                         //Nouvelle position

      //Si position actuel est differente de la nouvelle position droite
      if (DifferentielDroite < LimEncSecDroite) {  //Si la position est différente
        millis();
        myrobot.lcdPrint("Le robot avance");
      }

      else {                           //Sinon
        MoteurGD(400, 400);            //robot stop
        myrobot.setLcdRGB(255, 0, 0);  //Affichage en couleur rouge
        myrobot.setLcdCursor(0, 0);
        myrobot.lcdPrint("Arret Droite");  //message d'erreur
      }
    }
    //---------------Arrêt Elec-------------//
    if (CourantMesure > 1) {
      millis();
      Arret_Elec();
    }
  }
}
