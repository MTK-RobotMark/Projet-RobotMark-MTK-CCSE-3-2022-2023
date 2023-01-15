  #include <Encoder.h> //Librairie des encodeurs
  #include "rgb_lcd.h"
  #define Stop 400 //On définit la valeur d'arrêt du robot
  MARK myrobot;
  Encoder knobLeft(18, 33); //On définit le pin pour l'encodeur Gauche
  Encoder knobRight(31, 19 );//On définit le pin pour l'encodeur Droite
  long positionLeft  = -999; //On définit une position de base       
  long positionRight = -999; //On définit une position de base
  #define ProrieteEncDroite 1180
  #define ProprieteEncGauche 1150
  #define LimDegSec 200
  int DifferentielGauche; //On mesure la différence de position t à t-1
  int DifferentielDroite; //On mesure la différence de position t à t-1
  void setup(){
  LimEncSecDroite=(LimDegSec/360)*ProprieteEncDroite);//Ici on calcule la limite de l'encodeur droite 
  LimEncSecGauche=(LimDegreSec/360)*ProprieteEncGauche);//Ici on calcule la limite de l'encodeur gauche
  }
//Fonction
//Encodeur
  void ArretMec(){ 
	    long newLeft, newRight;//On définit des variables pour les nouvelles positions
	    newLeft = knobLeft.read(); //Lecture roue gauche
	    newRight = knobRight.read();//Lecture roue droite
	//Si position actuel est differente de la nouvelle position gauche
      if (newLeft != positionLeft) { //Si la position a bougé
        DifferentielGauche=newLeft-positionLeft;//Différence de position 
        positionLeft = newLeft; //Nouvelle position
	    if (DifferentielGauche < LimEncSecGauche){ //Si la position est différente
        millis();
        lcd.print("Le robot avance");
	    }
      else{//Sinon
  	    MoteurGD(400,400); //robot stop
	      lcd.setRGB(255, 0, 0); //Affichage en couleur rouge
	      lcd.setCursor(0, 0);
	      lcd.print("Arret Mec Gauche"); //message d'erreur
      }
      if (newRight != positionRight) { //Si la position a bougé
        DifferentielDroite=newRight-positionRight;//Différence de position 
        positionLeft = newLeft; //Nouvelle position
  //Si position actuel est differente de la nouvelle position droite
	    if (DifferentielDroite < LimEncSecDroite){ //Si la position est différente
        millis();
        lcd.print("Le robot avance");
	    }
      else{ //Sinon
  	    MoteurGD(400,400); //robot stop
	      lcd.setRGB(255, 0, 0); //Affichage en couleur rouge
	      lcd.setCursor(0, 0);
	      lcd.print("Arret Mec Droite"); //message d'erreur
      }
  }
	