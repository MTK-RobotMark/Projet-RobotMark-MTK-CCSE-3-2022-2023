//Valeurs Prédéfinies
#define Thash 800
#define Stop 400
#define Vmax Thash

// Macros
#define LedToggle digitalWrite(13, !digitalRead(13))
#define MoteurG(Vg) OCR5A=Vg // Vg in [0... 1999]
#define MoteurD(Vd) OCR5B=Vd // VD in [0... 1999]
#define MoteurGD(Vg,Vd) MoteurG(Vg);MoteurD(Vd)
#define StopMoteurGD MoteurGD(Stop,Stop)

//Bibliothèques
#include <MARK.h> //librairie du robot MARK
#include <Wire.h>  //lib for I2C connection
#include "Ultrasonic.h" //Pour les Ultrasons
#include "rgb_lcd.h" //Pour le LCD

//Déclaration des capteurs ultrasons
Ultrasonic ultrasonicF(8); //Init of ultrasonic sensor on pin 8
Ultrasonic ultrasonicG(10);//Init of ultrasonic sensor on pin 10
Ultrasonic ultrasonicD(12);//Init of ultrasonic sensor on pin 12


//Variables pour mesurer le temps du parcours
unsigned long Temps_start_ms;
unsigned long Temps_stop_ms;
unsigned long Duree_ms;
unsigned long Duree_s;
unsigned long SaveR;
unsigned long SaveL;
unsigned long Vf_L;
unsigned long Vf_R;
//Variables pour les ultrasons
long DistanceCapteurAvant;
long DistanceCapteurDroite;
long DistanceCapteurGauche;
long Nm,compteur;

const byte infrared = 6; //Pas nécessaire pour l'instant
boolean arret = false;
MARK myrobot;

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
  compteur++;
  if(compteur==50){
    compteur=0;
    DistanceCapteurAvant = ultrasonicF.MeasureInCentimeters();
    DistanceCapteurGauche = ultrasonicG.MeasureInCentimeters();
    DistanceCapteurDroite = ultrasonicD.MeasureInCentimeters();
  }
}

void affichage(){
  int gps= 0;
  Duree_s = Duree_ms/1000;
  float distance_Robot = (((Vf_R/1180) + (Vf_L/1150))*0.1);
  long NbtourG = (Vf_L/1150);
  long NbtourD = (Vf_R/1180);

  while(1){
    if((myrobot.getJoystickY())<358){
      gps++;
      while((myrobot.getJoystickY())<358);
    }

    else if ((myrobot.getJoystickY())>671){
      gps--;
      while((myrobot.getJoystickY())>671);
    }
    switch(gps) {
      case 0 :
      myrobot.lcdClear();
      myrobot.setLcdCursor(0, 0);
      myrobot.lcdPrint("Fin de course");
      myrobot.setLcdCursor(0, 1);
      myrobot.lcdPrint("                ");
      break;

      
      case 1 :
      myrobot.setLcdCursor(0, 0);
      myrobot.lcdPrint("Temps parcours        ");
      myrobot.setLcdCursor(0, 1);
      myrobot.lcdPrint(Duree_s);
      myrobot.lcdPrint("  secondes");
      break;
      
      case 2 :
      myrobot.setLcdCursor(0, 0);
      myrobot.lcdPrint(" Ener Elec Conso       ");
      myrobot.setLcdCursor(0, 1);
      myrobot.lcdPrint("                       ");
      break;
      
      case 3 :
      myrobot.setLcdCursor(0, 0);
      myrobot.lcdPrint("le nombre Nm      ");
      myrobot.setLcdCursor(0, 1);
      myrobot.lcdPrint(Nm                  );
      myrobot.lcdPrint("     fois         ");
      break;

      case 4 :
      myrobot.setLcdCursor(0, 0);
      myrobot.lcdPrint("Nb tour roue G    ");
      myrobot.setLcdCursor(0, 1);
      myrobot.lcdPrint(NbtourG             );
      myrobot.lcdPrint("   tours          ");
      break;

      case 5 :
      myrobot.setLcdCursor(0, 0);
      myrobot.lcdPrint("Nb tour roue R    ");
      myrobot.setLcdCursor(0, 1);
      myrobot.lcdPrint(NbtourD             );
      myrobot.lcdPrint("   tours          ");
      break;

      case 6 :
      myrobot.setLcdCursor(0, 0);
      myrobot.lcdPrint("    Distance    ");
      myrobot.setLcdCursor(0, 1);
      myrobot.lcdPrint(distance_Robot    );
      myrobot.lcdPrint("   m            ");
      break;

      default :
      myrobot.setLcdCursor(0, 0);
      myrobot.lcdPrint("   Navigation   ");
      myrobot.setLcdCursor(0, 1);
      myrobot.lcdPrint("   Incorrect    ");


    }
  }
}

//Interruption Arrêt DISTANCE
void Arret_Dist(){
  MoteurGD(Stop, Stop);
  myrobot.setLcdRGB(255,0,0); //Rouge
  myrobot.setLcdCursor(0, 0);
  myrobot.lcdPrint("  Arret Distance   ");
  while(not(myrobot.getJoystickClic()));
  myrobot.setLcdRGB(255, 255, 255); //Red
}

void setup() {
  myrobot.begin();
  pinMode(43, OUTPUT);
  digitalWrite(43, 0);
  initMoteurs();
  sei();
  digitalWrite(43, 1);
  while(myrobot.getInfrared() || (not(myrobot.getJoystickClic()))){
    MoteurGD(Stop,Stop);
  }
  myrobot.setLedBarLevel(myrobot.getBatteryLevel());
  myrobot.setLcdRGB(0, 255, 0);
  myrobot.setLcdCursor(0, 0); myrobot.lcdPrint("Appuyez pour départ");
  myrobot.setLcdRGB(0, 0, 255);
  Temps_start_ms=millis(); //Calcul le début de la course parcours
}


void loop() {
    int x = 0;
  
  if(arret == false){
    millis();

  if (DistanceCapteurAvant < 5 || DistanceCapteurDroite < 5 || DistanceCapteurGauche < 5 ) {  // Le robot s'arrête s'il arrive face à un mur
    millis();
    Arret_Dist(); //On entre dans l'interruption Arret Distance
    }

    if (DistanceCapteurDroite > 50 && DistanceCapteurGauche > 50) { // Si le robot est au centre du couloir il va tout droit
      MoteurGD(600, 621
      );
      millis(); // La fonction millis() permet l'execution rapide de chaque boucle if et donc une presque exécution simultané des ces mêmes boucles
      }
    
   if (DistanceCapteurDroite < 50){ // Si le robot est proche du mur droite il tourne légèrement à gauche
      MoteurGD(600, 624);
      millis();
      }
    
   if (DistanceCapteurGauche < 50){ // Si le robot est proche du mur gauche il tourne légèrement à droite
      MoteurGD(615, 622);
      millis();
      }
    
   if (DistanceCapteurAvant < 70 && DistanceCapteurGauche > 100) {
      delay(200);  
      SaveR = SaveR + myrobot.getEncoder("right"); 
      myrobot.resetEncoder("right");
      while(x < 420){
        delay(20);
        MoteurGD(320,480);
        x = myrobot.getEncoder("right");
        }
       MoteurGD(400,400);
    }   

   if (DistanceCapteurGauche < 80 && DistanceCapteurDroite > 100) { 
      delay(200);
      SaveL = SaveL + myrobot.getEncoder("left"); 
      myrobot.resetEncoder("left");
      while(x < 420){
        delay(20);
        MoteurGD(480,320);
        x = myrobot.getEncoder("left");
        }
       MoteurGD(400,400);
    }
    
   if (digitalRead(infrared)) { // Le robot s'arrête si on le soulève
      myrobot.setLcdRGB(0,255, 0);
      arret = true;
      Temps_stop_ms=millis();
      Vf_L = (myrobot.getEncoder("left") + SaveL);
      Vf_R = (myrobot.getEncoder("right") + SaveR);
      
      MoteurGD(400, 400);
      Duree_ms= Temps_stop_ms - Temps_start_ms;
      affichage();
      }

   if (DistanceCapteurAvant < 20 || DistanceCapteurDroite < 20 || DistanceCapteurGauche < 20 ){
      millis();
      Nm++;
    }
      
  }    
}
