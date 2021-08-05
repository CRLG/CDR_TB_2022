/*************************************************************************************************************************************************************
 *                    PARTIE 1 : Intégrer des bibliothèques                      *
 *********************************************************************************/
#include <Wire.h>
//#include <Pixy2.h>
#include <Adafruit_MotorShield.h> // inclusion de la librairie pour commander un motor shield
#include <Servo.h> // inclusion de la librairie pour commander ses servomoteurs
#include <Adafruit_NeoPixel.h> //librairie pour les led
#include "Adafruit_VL53L0X.h"
#include <LiquidCrystal_I2C.h>
#ifdef __AVR__
#include <avr/power.h>
#endif

/*************************************************************************************************************************************************************
 *                    PARTIE 2 : Définir des valeurs, des objets, des variables                                                                              *
 *************************************************************************************************************************************************************/
//CONFIGURATION DU ROBOT
//pour debuguer le programme, mettre à true
#define DEBUG false
//pour debuguer le robot sur table (sans capteurs, etc)
#define ARDUINO_ALONE false
//à décommenter pour utiliser le bandeau de LEDS
#define UTILISE_LEDS false
//à décommenter pour utiliser la camera
//#define UTILISE_CAMERA true



//CONFIGURATION DES PORTS UTILISES SUR L'ARDUINO MEGA
/**
 * POUR INFO en utilisant le motor shield v2:
 * la masse (GND) ainsi que le 5v (par defaut) ou le 3.3v sont nécessaire pour faire fonctionner le shield. (le 5v ou le 3v peuvent être choisis par un jumper sur la carte)
 * Le shield utilise le signal SDA et SCL pour le protocole i2c utilisé pour contrôler les moteurs. Sur l'arduino UNO ce sont les pinoches analogiques A4 et A5.
 * Sur l'arduino Mega ces pinoches sont plutôt les pinoches numériques 20 et 21.
 * Il ne faut donc pas utiliser ces pinoches sur ces arduinos avec ce shield pour quoique ce soit d'autre que des capteurs ou actionneurs i2c.
 *
 * Etant donné que le shield utilise l'i2c pour communiquer, vous pouvez connecter d'autres capteurs ou actionneurs i2c sur les signaux SDA et SCL tant qu'ils n'utilisent pas l'adresse 0x60
 * (l'adresse par défaut du shield) ou l'adresse 0x070 (l'adresse utilisable par le shield pour contrôler un groupe de shield)
 *
 * Si vous voulez utiliser les broches du shield déddiées aux servos, elles sont reliées aux pinoches 9 et 10.
 * Si vous n'utilisez pas ces broches, les pinoches 9 et 10 sont libres d'utilisation.
 * Vous pouvez utiliser toutes les autres pinoches non mentionnées ci-dessus.
 */
#define PIN_COULEUR_EQUIPE  A0 //bouton de choix de couleur d'équipe pour le mode autonome
#define PIN_CONTACTEUR_AR_D A1 //contacteurs arrière droit pour le recalage
#define PIN_CONTACTEUR_AR_G A2 //contacteurs arrière gauche pour le recalage
//#define PIN_00              0 //utilisé en cas de débugage
//#define PIN_01              1 //utilisé en cas de débugage
//#define PIN_02              2 //pas encore utilisé mais peut perturber le cadenceur
#define PIN_SERVO_01        3 //PWM - Servo pince gauche
#define PIN_SERVO_02        4 //PWM - pas encore utilisé
#define PIN_SERVO_03        5 //PWM - pas encore utilisé
#define PIN_SERVO_04        6 //PWM - pas encore utilisé
#define PIN_SERVO_05        7 //PWM - pas encore utilisé
#define PIN_SERVO_06        8 //PWM - pas encore utilisé
#define PIN_SERVO_07        9 //PWM - Servo pour les gobelets réservoir
#define PIN_SERVO_08        10 //PWM - Servo pince droite - servo 1 sur le shield motor
#define PIN_SERVO_09        11 //PWM - pas encore utilisé
#define PIN_PWM_GAUCHE      12 //PWM - commande puissance moteur gauche (orange)
#define PIN_PWM_DROIT       13 //PWM - commande puissance moteur droit (gris)
#define PIN_14              14 //pas encore utilisé
#define PIN_15              15 //pas encore utilisé
//#define PIN_RX              18 //ATTENTION utilisé par RX voie de réception de la télécommande pour le mode téléguidé
//#define PIN_TX              19 //ATTENTION utilisé par TX voie d'envoi de la télécommande pour le mode téléguidé
#define PIN_18              16 //pas encore utilisé
#define PIN_19              17 //pas encore utilisé
//#define PIN_SDA             20 //ATTENTION utilisé par le motor shield
//#define PIN_SCL             21 //ATTENTION utilisé par le motor shield
#define PIN_TIRETTE         22 //capteur optique de la tirette pour le mode autonome
#define PIN_LEDS            23 //fil de communication avec le bandeau de leds
#define PIN_24              24 //pas encore utilisé
#define PIN_25              25 //pas encore utilisé
#define PIN_26              26 //pas encore utilisé
#define PIN_27              27 //pas encore utilisé
#define PIN_28              28 //pas encore utilisé
#define PIN_29              29 //pas encore utilisé
#define PIN_30              30 //pas encore utilisé
#define PIN_31              31 //pas encore utilisé
#define PIN_32              32 //pas encore utilisé
#define PIN_33              33 //pas encore utilisé
#define PIN_CMD_1_GAUCHE    34 //commande n°1 moteur gauche (jaune)
#define PIN_CMD_1_DROIT     35 //commande n°1 moteur droit (violet)
#define PIN_CMD_2_GAUCHE    36 //commande n°2 moteur gauche (vert)
#define PIN_CMD_2_DROIT     37 //commande n°2 moteur droit (bleu)
#define PIN_38              38 //pas encore utilisé
#define PIN_39              39 //pas encore utilisé
#define SHT_CAPTEUR_1       40 //pour programmer le capteur de distance n°1
#define PIN_41              41 //pas encore utilisé
#define SHT_CAPTEUR_2       42 //pour programmer le capteur de distance n°2
#define PIN_43              43 //pas encore utilisé
#define SHT_CAPTEUR_3       44 //pour programmer le capteur de distance n°3
#define PIN_45              45 //PWM - pas encore utilisé
#define SHT_CAPTEUR_4       46 //pour programmer le capteur de distance n°4 c'est également une sortie PWM
#define PIN_47              47 //pas encore utilisé
#define PIN_48              48 //pas encore utilisé
#define PIN_49              49 //pas encore utilisé
#define PIN_50              50 //pas encore utilisé
#define PIN_51              51 //pas encore utilisé
#define PIN_52              52 //pas encore utilisé
#define PIN_53              53 //pas encore utilisé

//parametrage du match
#define DUREE_MATCH               100 //en secondes
#define DISTANCE_PAR_PAS_MOTEUR   0.1099 //1 pas = K cm
#define EQUIPE_JAUNE              0
#define EQUIPE_BLEUE              1

//parametrage divers
#define MOTEUR_GAUCHE             2 //identifiant moteur gauche
#define MOTEUR_DROIT              1 //identifiant moteur droit
#define AVANT                     1 //sens des moteurs des roues
#define ARRIERE                   -1 //sens des moteurs des roues
#define STOP                      0 //sens des moteurs des roues
#define NBRE_DE_LEDS              3 //nombre de leds
#define LEDS_OFF                 0 //led eteinte
#define LEDS_BLEU                 1 //couleur bleue pour les leds
#define LEDS_ORANGE               2 //couleur orange pour les leds
#define LEDS_ROUGE                 3 //couleur rouge pour les leds
#define LEDS_VERT                 4 //couleur verte pour les leds
#define LED_GAUCHE                0
#define LED_MILIEU                1
#define LED_DROITE                2

//valeurs des servo_moteurs
#define P_GAUCHE_FERME 180
#define P_GAUCHE_OUVERT 84
#define P_DROITE_FERME 0
#define P_DROITE_OUVERT 100
#define GOBELETS_LEVE 40
#define GOBELETS_MILIEU 100
#define GOBELETS_RANGE 158
#define SERVO_04_INIT 180
#define SERVO_04_TEST 1
#define MANCHON_SORTI 40
#define MANCHON_RENTRE 180
#define SERVO_05_INIT 1
#define SERVO_05_TEST 180

//----------------------------------------------------------------------------------------------
// DECLARATION VARIABLES: MOTEUR, CAMERA, LEDS,...
//----------------------------------------------------------------------------------------------

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////// variables globales diverses
//Création variables globales
bool bProgrammeDemarre; // variable qui indique si le programme est demarre (demarre==true), utilisé par la tirette
int couleur_equipe;
long compteur_temps; //toutes les 10 ms a peu près (temps reel mou)
int temps_match;
float vitesseG_n;
float vitesseD_n;
int step_time;
unsigned long previous_time_10_ms;
unsigned long previous_time_50_ms;
unsigned long previous_time_200_ms;
unsigned long previous_time_500_ms;
unsigned long previous_time_1000_ms;
unsigned long current_time;
unsigned long start_time_match;
int nombre_points;
long somme_steps;
int nb_steps;

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////// servos
//Création des servos
Servo servo_Pince_Gauche;
Servo servo_Pince_Droite;
Servo servo_Gobelets;
Servo servo_04;
Servo servo_05;
Servo servo_06;
Servo servo_07;
Servo servo_08;
Servo servo_09;

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////// moteurs
//Création des 2 moteurs pas à pas
// Création d'une carte moteur avec l'adresse I2C par défaut
Adafruit_MotorShield Carte_Moteur = Adafruit_MotorShield();
// Connection de 2 moteurs pas à pas de 200 pas par tour (1.8 degrés)
Adafruit_StepperMotor *Moteur_G = Carte_Moteur.getStepper(200, 1);// moteur_G (M1 et M2 connectés sur le shield)
Adafruit_StepperMotor *Moteur_D = Carte_Moteur.getStepper(200, 2);// moteur_D (M3 et M4 connectés sur le shield)

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////// leds
//création des leds
#ifdef UTILISE_LEDS
  Adafruit_NeoPixel pixels = Adafruit_NeoPixel(NBRE_DE_LEDS, PIN_LEDS, NEO_GRB + NEO_KHZ800);
#endif

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////// camera
//création caméra
#ifdef UTILISE_CAMERA
  Pixy2 pixy;
#endif

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////// capteurs distance
//adresses à donner aux 4 capteurs de distance
#define CAPTEUR_1_ADDRESS 0x30
#define CAPTEUR_2_ADDRESS 0x31
#define CAPTEUR_3_ADDRESS 0x32
#define CAPTEUR_4_ADDRESS 0x33
//pour stocker les mesures de distance
float capteur1,capteur2, capteur3,capteur4;
//Pour prendre les mesures de distance
uint16_t mesure1, mesure2, mesure3, mesure4;
//pour savoir si une mesure de distance a été lancée
bool capteur_distance_started;

//création des capteurs
Adafruit_VL53L0X CAPTEUR_1 = Adafruit_VL53L0X();
Adafruit_VL53L0X CAPTEUR_2 = Adafruit_VL53L0X();
Adafruit_VL53L0X CAPTEUR_3 = Adafruit_VL53L0X();
Adafruit_VL53L0X CAPTEUR_4 = Adafruit_VL53L0X();

LiquidCrystal_I2C lcd(0x27,20,4);  // set the LCD address to 0x27 for a 16 chars and 2 line display

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////// codeurs

volatile signed int counter1 = 0;  //This variable will increase or decrease depending on the rotation of encoder
volatile signed int counter2 = 0;  //This variable will increase or decrease depending on the rotation of encoder

float dcodeur = 6; // diamètre de la roue codeuse  (à changer) en cm

// calculs
/*float pi = 3.14159265359;
float perimetre_roue_codeuse = pi*dcodeur;
float ntour_par_pas = 1200/perimetre_roue_codeuse;*/
const double unite_distanceG = 0.0157; // unite_distanceG --> distance parcourue en cm pour 1 pas codeur (coté gauche)
const double unite_distanceD = 0.0157; // unite_distanceD --> distance parcourue en cm pour 1 pas codeur (coté gauche)
const double unite_angleG = 0.075; // unite_angle --> angle parcourue en degré pour 1 pas codeur
const double unite_angleD = 0.075; // unite_angle --> angle parcourue en degré pour 1 pas codeur

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////// asservissement distance

float distance; // distance à parcourir en cm (à changer pour faire des tests)

float erreur1 = 0.0; // erreur pour le codeur 1
float erreur2 = 0.0; // erreur pour le codeur 2

int consigne1 = 0; // consigne du moteur1
int consigne2 = 0; // cosnigne du moteur 2

int consigne1_abs;
int consigne2_abs;

float k = 10; // coefficient de proportionalite

float save_1; // variable pour faire un comptage de distance relatif
float save_2;

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////// asservissement angulaire

float angl;

float erreur_angle_1 = 0.0;
float erreur_angle_2 = 0.0;

int consigne_angle_1 = 0.0;
int consigne_angle_2 = 0.0;

int consigne_angle_1_abs;
int consigne_angle_2_abs;

float k_angle = 2.5;

float save_angle_1; // variable pour faire un comptage d'angle relatif
float save_angle_2;

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////// asservissement global
//savoir si le robot a convergé
bool bconvergence = true;
bool prevbconvergence;
int etatCourant;

//pour activer l'asservissement en distance ou en angle
int etat_asservissement = 0;



/*************************************************************************************************************************************************************
 *                    PARTIE 3 : Définition des fonctions                                                                                                     *
 *************************************************************************************************************************************************************/

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////// fonctions des moteurs pas à pas
/*
 * FONCTION VITESSE_MOTEURS_PAS_A_PAS
 * positionne la vitesse des moteurs pas à pas gauche et droite
 * Demande une valeur qui est la vitesse de rotation en tours par minute
 */
void VITESSE_MOTEURS_PAS_A_PAS(int moteur, int valeur_vitesse)
{
  //normalisation de la vitesse entre 0 et 100
  if(valeur_vitesse>100)
    valeur_vitesse=100;
  if(valeur_vitesse<0)
    valeur_vitesse=0;
  //conversion en valeur analogique pour l'arduino entre 0 et 255
  int digital_vitesse=map(valeur_vitesse,0,100,0,255);
  //application de la vitesse au moteur
  if(moteur==MOTEUR_GAUCHE)
    Moteur_G->setSpeed(digital_vitesse);
  if(moteur==MOTEUR_DROIT)
    Moteur_D->setSpeed(digital_vitesse);  
}

/*
 * FONCTION TOURNER_UN_PAS
 * Fait touner le moteur pas à pas droit et gauche d'un pas
 * Demande le sens de rotation du moteur
 */
void TOURNER_UN_PAS(int moteur, int sens)
{
  if(sens==AVANT)
  {
    if(moteur==MOTEUR_GAUCHE)
      Moteur_G->step(1,FORWARD,SINGLE); //en avant
    if(moteur==MOTEUR_DROIT)
      Moteur_D->step(1,BACKWARD,SINGLE); //en avant
  }
  if(sens==ARRIERE)
  {
    if(moteur==MOTEUR_GAUCHE)
      Moteur_G->step(1,BACKWARD,SINGLE); //en arriere
    if(moteur==MOTEUR_DROIT)
      Moteur_D->step(1,FORWARD,SINGLE); //en arriere
  }
}

/*
 * FONCTION ACTION_MOTEUR
 * défini la vitesse des deux moteurs
 * la vitesse peut être négative dans ce cas il s'agit de tourner en arrière
 */
void ACTION_MOTEUR(int moteur, float vitesse)
{
  //vitesse positive exigée
  int vitesse_pos=abs(vitesse);
  //application de la vitesse
  VITESSE_MOTEURS_PAS_A_PAS(moteur, vitesse_pos);

  //vitesse mini supérieure à 0.1
  if(vitesse_pos < 0.1) vitesse=0;
  //on applique le sens de rotation suivant le signe de la vitesse
  if(vitesse>0)
    TOURNER_UN_PAS(moteur, AVANT);
  if(vitesse<0)
    TOURNER_UN_PAS(moteur, ARRIERE);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////// fonctions asservissement distance
/*
 * FONCTION asservissement()
 * fonction d'asservissement en distance
 */
void asservissement()
{
  bool bconvergenceDistanceD = false; // pour savoir s'il a convergé
  bool bconvergenceDistanceG = false;
  
  prevbconvergence = bconvergence;
  
  save_1 = save_1 + (counter2*unite_distanceG);
  save_2 = save_2 + (counter1*unite_distanceD);
  
  counter1 = 0;
  counter2 = 0;
  
  erreur1 = distance - save_1; // calcul de l'erreur pour le moteur1 
  erreur2 = distance - save_2; // calcul de l'erreur pour le moteur2
  
  int erreur1_abs=abs(erreur1);
  int erreur2_abs=abs(erreur2);
  
  //vérification de la convergence à gauche
  // marge d'erreur de 3mm pour l'instant --> test
  if (erreur1_abs > 0.3)
  {
    
    consigne1_abs = (int)(k*erreur1_abs) ;
  
    if (consigne1_abs > 100) { // pour que la consigne ne parte pas en vrille
      consigne1_abs = 100;
    }
  
    if(erreur1>0)
    {
      vitesseG_n = (float)consigne1_abs; // vitesse du moteur
    }
    else
    {
      vitesseG_n=(float)((-1.0)*consigne1_abs);
    }
  }
  else
  {
    bconvergenceDistanceG=true;
    vitesseG_n=0;
  }
  
  //vérification de la convergence à droite
  // marge d'erreur de 3mm pour l'instant --> test
  if (erreur2_abs > 0.3)
  {
    //on etablit la consigne à droite
    consigne2_abs = (int)(k*erreur2_abs) ;
    
    if (consigne2_abs > 100)
    { // pour que la consigne ne parte pas en vrille
      consigne2_abs = 100;
    }
    
    if(erreur2>0)
    {
      vitesseD_n = (float)consigne2_abs; // vitesse du moteur
    }
    else
    {
      vitesseD_n=(float)((-1.0)*consigne2_abs);
    }
  }
  else
  {
    bconvergenceDistanceD=true;
    vitesseD_n=0;
  }
  
  //pour savoir s'il a convergé
  bconvergence=(bconvergenceDistanceD && bconvergenceDistanceG);

}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////// fonctions asservissement d'angle
/*
 * FONCTION asservissement_angle()
 * fonction d'asservissement en angle
 */
void asservissement_angle ()
{
bool bconvergenceDistanceD = false; // pour savoir s'il a convergé
bool bconvergenceDistanceG = false;

prevbconvergence = bconvergence;

counter2 = -counter2; // on inverse le compteur 2 car sinon au lieu de diminuer il augmente --> les roues tourne dans un sens différent

save_angle_1 = save_angle_1 + (counter2*unite_angleG); // calculs de l'avancement effectué sur un interval de temps réduit
save_angle_2 = save_angle_2 + (counter1*unite_angleD);

counter1 = 0; // on met les compteurs à 0
counter2 = 0;

erreur_angle_1 = angl - save_angle_1; // calculs des erreurs
erreur_angle_2 = angl - save_angle_2;

float erreur_angle_1_abs=abs(erreur_angle_1); // les erreurs sont mise en absolue
float erreur_angle_2_abs=abs(erreur_angle_2);

  if (erreur_angle_1_abs > 1) // une boucle de rétroaction
  {
     consigne_angle_1_abs = (float)(k_angle*erreur_angle_1_abs) ; // on fait une correction proportionnelle

     if (consigne_angle_1_abs > 100) 
     { 
      consigne_angle_1_abs = 100;
     }

     if(erreur_angle_1<0)
     {
      vitesseG_n = (float)consigne_angle_1_abs;// vitesse du moteur + sens
     }
     else
     {
      vitesseG_n=(float)((-1.0)*consigne_angle_1_abs); // vitesse du moteur + sens
     }
  }
  else 
  {
  vitesseG_n = 0.0; 
  bconvergenceDistanceG=true; // on dit que le moteur gauche a convergé
  }

if (erreur_angle_2_abs > 1) // 2ème boucle de rétroaction
  {
     consigne_angle_2_abs = (float)(k_angle*erreur_angle_2_abs) ; // on fait une correction proportionnelle

     if (consigne_angle_2_abs > 100) 
     { 
      consigne_angle_2_abs = 100;
     }

     if(erreur_angle_2<0)
     {
      vitesseD_n = (float)((-1.0)*consigne_angle_2_abs);// vitesse du moteur + sens
     }
     else
     {
      vitesseD_n=(float)consigne_angle_2_abs;// vitesse du moteur + sens
     }
  }
  else
  {
    vitesseD_n = 0.0;
    bconvergenceDistanceD=true;// on dit que le moteur gauche a convergé
  }  
bconvergence=(bconvergenceDistanceD && bconvergenceDistanceG); // si les deux moteurs ont convergés on dit que le robot a convergé
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////stratégie de match

// petite consigne pour coder la stratègie : (sinon c'est cassé)

/* !!!! les deux asservissement ne sont pas activé en même temps il faut penser à changer la variable etat_asservissement
 *  
 * ! mettre etat_asservissement = 1 pour asservir en ---distance---
 * ! mettre etat_asservissement = 2 pour asservir en ---angle---
 * 
 * Les deux asservissement ne peuvent pas fonctionner ensemble mais ça reste fonctionnelle
 * 
 * Pour les asservissement en angle il vaut mieux faire plusieurs étapes avec de petits angle 
 * par exemple pour faire un angle de 90° c'est mieux de faire 45° puis ensuite de nouveau 45° c'est plus long mais moins risquer
 * Aussi un angle de 90° fait tourner le robot dans le sens des aiguille d'une montre et inversement
 *
 */

void strategie (int etat) 
{
  switch (etat)         
  {
    case 1 : etat_asservissement = 1;
    distance = 30;
    break;
    case 2 : etat_asservissement = 2;
    angl = 90;
    break;
    case 3 : etat_asservissement = 1;
    distance = 60;
    break;
    case 4 : etat_asservissement = 2;
    angl = 180;
    break;
    case 5 : etat_asservissement = 1;
    distance = 90;
    break;
    case 6 : etat_asservissement = 2;
    angl = 270;
    break;
    case 7 : etat_asservissement = 1;
    distance = 120;
    break;
    case 8 : etat_asservissement = 2;
    angl = 360;
    break;
  }
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////
/*
    Remise à 0 de tous les capteurs en mettant leur pinoche XSHUT à l'état bas pendant 10 ms puis en les mettant à l'état haut pour la remise à 0.
    pour l'adresse
    Garder le capteur #1 évaillé en conservant sa pinoche XSHUT à l'état haut
    mettre tous les autres capteurs endormis en mettant leur pinoche à l'état bas
    Initialiser le capteur #1 avec la fonction begin(nouvelle_adresse_i2c). Donner n'importe quelle adresse inférieure à 0x7F sauf 0x29.
    Conserver le capteur #1 éveillé et réveiller le capteur #2 en mettant sa pinoche XSHUT à l'état haut
    Initialiser le capteur #2 avec la fonction begin(nouvelle_adresse_i2c). Donner n'importe quelle adresse inférieure à 0x7F sauf 0x29 et celle du capteur #1
    et ainsi de suite

    choisir entre les différentes CONFIG de capteur:
    VL53L0X_SENSE_DEFAULT = 0,
    VL53L0X_SENSE_LONG_RANGE,
    VL53L0X_SENSE_HIGH_SPEED,
    VL53L0X_SENSE_HIGH_ACCURACY
 */
void ChangerAdressesCapteursDistance() {

  //reset pour tous
  digitalWrite(SHT_CAPTEUR_1, LOW);    
  digitalWrite(SHT_CAPTEUR_2, LOW);
  digitalWrite(SHT_CAPTEUR_3, LOW);    
  digitalWrite(SHT_CAPTEUR_4, LOW);  
  
  delay(10);
  //reveil pour tous
  digitalWrite(SHT_CAPTEUR_1, HIGH);
  digitalWrite(SHT_CAPTEUR_2, HIGH);
  digitalWrite(SHT_CAPTEUR_3, HIGH);
  digitalWrite(SHT_CAPTEUR_4, HIGH);  
  delay(10);

  //activation capteur 1
  digitalWrite(SHT_CAPTEUR_1, HIGH);
  digitalWrite(SHT_CAPTEUR_2, LOW);
  digitalWrite(SHT_CAPTEUR_3, LOW);
  digitalWrite(SHT_CAPTEUR_4, LOW);

  //changement adresse capteur 1
  if(!CAPTEUR_1.begin(CAPTEUR_1_ADDRESS,false,&Wire,Adafruit_VL53L0X::VL53L0X_SENSE_HIGH_SPEED)) {
    Serial.println(F("Echec pour démarrer le premier VL53L0X"));
    while(1);
  }
  delay(10);

  // activating CAPTEUR_2
  digitalWrite(SHT_CAPTEUR_2, HIGH);
  delay(10);

  //changement adresse CAPTEUR_2
  if(!CAPTEUR_2.begin(CAPTEUR_2_ADDRESS,false,&Wire,Adafruit_VL53L0X::VL53L0X_SENSE_HIGH_SPEED)) {
    Serial.println(F("Echec pour démarrer le second VL53L0X"));
    while(1);
  }
  
  //changement adresse CAPTEUR_3
  digitalWrite(SHT_CAPTEUR_3, HIGH);
  delay(10);

  //initing CAPTEUR_3
  if(!CAPTEUR_3.begin(CAPTEUR_3_ADDRESS,false,&Wire,Adafruit_VL53L0X::VL53L0X_SENSE_HIGH_SPEED)) {
    Serial.println(F("Echec pour démarrer le troisième VL53L0X"));
    while(1);
  }  
  
   
  //changement adresse CAPTEUR_4
  digitalWrite(SHT_CAPTEUR_4, HIGH);
  delay(10);

  //initing CAPTEUR_4
  if(!CAPTEUR_4.begin(CAPTEUR_4_ADDRESS,false,&Wire,Adafruit_VL53L0X::VL53L0X_SENSE_HIGH_SPEED)) {
    Serial.println(F("Echec pour démarrer le quatrième VL53L0X"));
    while(1);
  }  

}

void LectureDistanceObtsacle() {
  
  capteur1=300.;
  capteur2=300.;
  capteur3=300.;
  capteur4=300.;
  mesure1=3000;
  mesure2=3000;
  mesure3=3000;
  mesure4=3000;

 if(capteur_distance_started)
 { 
  // print sensor one reading
  if(DEBUG) Serial.print(F("1: "));
  if(CAPTEUR_1.isRangeComplete())
  {
    if(CAPTEUR_1.readRangeStatus() != 4)
    { 
      // if not out of range
      mesure1=CAPTEUR_1.readRangeResult();
      float temp_mes=mesure1/10.;
         
      if(temp_mes>300.)
      {
        temp_mes=300.; 
      }
      
      capteur1=temp_mes;
      if(DEBUG)
      {
        Serial.print(capteur1);
        Serial.print(F("cm"));
      }    
    }
    else
    {
      if(DEBUG) Serial.print(F("Out of range"));
    }
  }
  else
    if(DEBUG) Serial.print(F("Not ready"));
  
  if(DEBUG) Serial.print(F("\t"));
  
  // print sensor one reading
  if(DEBUG) Serial.print(F("2: "));
  if(CAPTEUR_2.isRangeComplete())
  {
    if(CAPTEUR_2.readRangeStatus() != 4)
    { 
      // if not out of range
      mesure2=CAPTEUR_2.readRangeResult();
      float temp_mes=mesure2/10.;
         
      if(temp_mes>300.)
      {
        temp_mes=300.; 
      }
      
      capteur2=temp_mes;
      if(DEBUG)
      {
        Serial.print(capteur2);
        Serial.print(F("cm"));
      }    
    }
    else
    {
      if(DEBUG) Serial.print(F("Out of range"));
    }
  }
  else
    if(DEBUG) Serial.print(F("Not ready"));
 

   if(DEBUG) Serial.print(F("\t"));

  // print sensor three reading
  if(DEBUG) Serial.print(F("3: "));
  if(CAPTEUR_3.isRangeComplete())
  {
    if(CAPTEUR_3.readRangeStatus() != 4)
    { 
      // if not out of range
      mesure3=CAPTEUR_3.readRangeResult();
      float temp_mes=mesure3/10.;
         
      if(temp_mes>300.)
      {
        temp_mes=300.; 
      }
      
      capteur3=temp_mes;
      if(DEBUG)
      {
        Serial.print(capteur3);
        Serial.print(F("cm"));
      }    
    }
    else
    {
      if(DEBUG) Serial.print(F("Out of range"));
    }
  }
  else
    if(DEBUG) Serial.print(F("Not ready"));
  
  
  if(DEBUG) Serial.print(F("\t"));

  // print sensor four reading
  if(DEBUG) Serial.print(F("4: "));
  if(CAPTEUR_4.isRangeComplete())
  {
    if(CAPTEUR_4.readRangeResult() != 4)
    { 
      // if not out of range
      mesure4=CAPTEUR_4.readRange();
      float temp_mes=mesure4/10.;
         
      if(temp_mes>300.)
      {
        temp_mes=300.; 
      }
      
      capteur4=temp_mes;
      if(DEBUG)
      {
        Serial.print(capteur4);
        Serial.print(F("cm"));
      }    
    }
    else
    {
      if(DEBUG) Serial.print(F("Out of range"));
    }
  }
  else
    if(DEBUG) Serial.print(F("Not ready"));
    
  if(DEBUG) Serial.println();

  capteur_distance_started=false;
 }

 if(!capteur_distance_started)
 {
  CAPTEUR_1.startRange();
  CAPTEUR_2.startRange();
  CAPTEUR_3.startRange();
  CAPTEUR_4.startRange();
  capteur_distance_started=true;
 }

}


bool frontConvergence () /////////////////////////////////savoir s'il a convergé
{
  return ((bconvergence != prevbconvergence)&& bconvergence);
}

void cadenceur()
{
  //actions toutes les 12 millisecondes
  

  //actions toutes les 60 millisecondes
  if(compteur_temps%5==0)
  {
    strategie (etatCourant);
    //on regarde l'état des capteurs
  }

  //actions toutes les 240 millisecondes
  if(compteur_temps%20==0)
  {
    //on s'occupe de la messagerie
    //on lit la distance des obstacles
    LectureDistanceObtsacle();
  }

  //actions toutes les 600 millisecondes
  if(compteur_temps%50==0)
  {
      
  }

  //actions toutes les secondes
  if(compteur_temps%100==0)
  {   
    Serial.println(step_time);
  }
   
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////codeur 1
void ai0() {
  // ai0 is activated if DigitalPin nr 2 is going from LOW to HIGH
  // Check pin 3 to determine the direction
  if(digitalRead(3)==LOW) {
    counter1++;
  }else{
    counter1--;
  }
}

void ai1() {
  // ai0 is activated if DigitalPin nr 3 is going from LOW to HIGH
  // Check with pin 2 to determine the direction
  if(digitalRead(2)==LOW) {
    counter1--;
  }else{
    counter1++;
  }
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////codeur 2
void ai2() {
  if(digitalRead(19)==LOW) {
    counter2++;
  }else{
    counter2--;
  }
}

void ai3() {
  if(digitalRead(18)==LOW) {
    counter2--;
  }else{
    counter2++;
  }
}




/*********************************************************************************
 *                    PARTIE 4 : Fonction setup() de l'arduino                   *
 *********************************************************************************/
void setup()
{
 
    Serial.begin(115200);
    //attendre que le port serie réponde présent pour le debug
    while (! Serial) { delay(1); }
   if(DEBUG) Serial.println("Debug activé\n\n");

//Serial.begin (115200);
Wire.begin();

pinMode(SHT_CAPTEUR_1, OUTPUT);
  pinMode(SHT_CAPTEUR_2, OUTPUT);
  pinMode(SHT_CAPTEUR_3, OUTPUT);
  pinMode(SHT_CAPTEUR_4, OUTPUT);  

  Serial.print("Mise en attente des 4 capteurs");

  digitalWrite(SHT_CAPTEUR_1, LOW);
  digitalWrite(SHT_CAPTEUR_2, LOW);
  digitalWrite(SHT_CAPTEUR_3, LOW);
  digitalWrite(SHT_CAPTEUR_4, LOW);
  Serial.println(" => OK");

  Serial.print("Changement d'adresse des capteurs distance!");
  ChangerAdressesCapteursDistance();
  capteur_distance_started=false;
  Serial.println(" => OK");



  //on utilise les pinoches réservées aux moteurs des roues
  pinMode(PIN_PWM_GAUCHE,OUTPUT); //commande puissance moteur gauche
  pinMode(PIN_PWM_DROIT,OUTPUT); //commande puissance moteur droit
  pinMode(PIN_CMD_1_GAUCHE,OUTPUT); //commande n°1 moteur gauche
  pinMode(PIN_CMD_1_DROIT,OUTPUT); //commande n°1 moteur droit
  pinMode(PIN_CMD_2_GAUCHE,OUTPUT); //commande n°2 moteur gauche
  pinMode(PIN_CMD_2_DROIT,OUTPUT); //commande n°2 moteur droit

  Serial.print("Initialisation servo pince gauche");
  servo_Pince_Gauche.attach(PIN_SERVO_08);
 /* servo_Pince_Gauche.write(P_GAUCHE_FERME);
  delay (1500);*/
  servo_Pince_Gauche.write(P_GAUCHE_OUVERT);
  Serial.println(" => OK");

  Serial.print("Initialisation servo pince droite");
  servo_Pince_Droite.attach(PIN_SERVO_07);
  /*servo_Pince_Droite.write(P_DROITE_FERME);
  delay (1500);*/
  servo_Pince_Droite.write(P_DROITE_OUVERT);
  Serial.println(" => OK");

  Serial.print("Lancement carte moteur");
  //initialisation de la carte moteur pour utiliser les moteurs pas à pas
  Carte_Moteur.begin();  //initialise le moteur avec une frequence par défaut 1.6KHz
  Serial.println(" => OK");

  Serial.print("Lancement du timer pour cadencer les actions");
    //initialisation du chronométrage du match
    /*MsTimer2::set(1, cadenceur); // période = duree du match, on activera avec la tirette
    MsTimer2::start();*/
  step_time=0;
  previous_time_10_ms=millis();
  previous_time_50_ms=previous_time_10_ms;
  previous_time_200_ms=previous_time_10_ms;
  previous_time_500_ms=previous_time_10_ms;
  previous_time_1000_ms=previous_time_10_ms;
    current_time=0;
    //ticks=0;
    compteur_temps=0;
Serial.println(" => OK");

    Serial.print("Initialisation de la tirette, le bouton de couleur d'équipe");
    //initialisation de la tirette      
    pinMode(PIN_TIRETTE, INPUT);  //en entrée
    pinMode(PIN_COULEUR_EQUIPE,INPUT_PULLUP); // choix de la couleur
    couleur_equipe=EQUIPE_BLEUE; //equipe violette par defaut
    bProgrammeDemarre=false; // le programme n'est pas demarre quand l'arduino s'allume
    Serial.println(" => OK");
  
  /*pinMode(PIN_CONTACTEUR_AR_D,INPUT_PULLUP); // contacteur arrière droit
  pinMode(PIN_CONTACTEUR_AR_G,INPUT_PULLUP); // contacteur arrière gauche*/
   
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////réception de l'information des roues codeuses

  Serial.print("Initialisation de la roue codeuse droite");
  pinMode(2, INPUT_PULLUP);           // set pin to input et pullup résistance
  pinMode(3, INPUT_PULLUP);           // set pin to input et pullup résistance
  
  //Setting up interrupt
  //A rising pulse from encodenren activated ai0(). AttachInterrupt 0 is DigitalPin nr 2 on moust Arduino.
  attachInterrupt(0, ai0, RISING);
  
  //B rising pulse from encodenren activated ai1(). AttachInterrupt 1 is DigitalPin nr 3 on moust Arduino.
  attachInterrupt(1, ai1, RISING);

  Serial.println(" => OK");


  Serial.print("Initialisation de la roue codeuse gauche");
  pinMode(18, INPUT_PULLUP);           // set pin to input et pullup résistance
  pinMode(19, INPUT_PULLUP);           // set pin to input et pullup résistance
  
  //Setting up interrupt
  //A rising pulse from encodenren activated ai0(). AttachInterrupt 0 is DigitalPin nr 2 on moust Arduino.
  attachInterrupt(5, ai2, RISING);
  
  //B rising pulse from encodenren activated ai1(). AttachInterrupt 1 is DigitalPin nr 3 on moust Arduino.
  attachInterrupt(4, ai3, RISING);

  Serial.println(" => OK");

  Serial.print("Initialisation de l'écran");
    //initialisation écran LCD
  lcd.init();

  // Affichage des distances sur l'écran LCD
  lcd.backlight();
  lcd.setCursor(0,0); lcd.print("ROBOT TETES BRIQUEES");
  lcd.setCursor(0,1); lcd.print("STRATEGIE 1");
  lcd.setCursor(0,3); lcd.print("READY!");
  Serial.println(" => OK");


///////////
  nombre_points=5; //on a déjà 5 points en posant le phare
  start_time_match=previous_time_10_ms;
  vitesseG_n=0;
  vitesseD_n=0;
  temps_match=0;
  prevbconvergence = bconvergence;
  etatCourant = 0;
  save_1 = 0.0;
  save_2 = 0.0;
  save_angle_2 = 0.0;
  save_angle_1 = 0.0;
  distance = 0.0;
  angl = 0.0;
  
  somme_steps=0;
  nb_steps=0;
 
  if(analogRead(PIN_COULEUR_EQUIPE)<800)
  {
    couleur_equipe=EQUIPE_BLEUE;//bouton vers le haut
          lcd.clear();
      lcd.setCursor(0,0); lcd.print("ROBOT TETES BRIQUEES");
      lcd.setCursor(0,1); lcd.print("STRATEGIE 1");
      lcd.setCursor(0,2); lcd.print("COULEUR: BLEU");
      lcd.setCursor(0,3); lcd.print("READY!");
  }
  else
  {
    couleur_equipe=EQUIPE_JAUNE;
          lcd.clear();
      lcd.setCursor(0,0); lcd.print("ROBOT TETES BRIQUEES");
      lcd.setCursor(0,1); lcd.print("STRATEGIE 1");
      lcd.setCursor(0,2); lcd.print("COULEUR: JAUNE");
      lcd.setCursor(0,3); lcd.print("READY!");
  }
      
  
}

/*********************************************************************************
 *                    PARTIE 4 : Fonction loo() de l'arduino                     *
 *********************************************************************************/
void loop() {

    //timer
    current_time=millis();

    //cadenceur
    //taches toutes les 10 ms
    if((current_time-previous_time_10_ms)>=10)
    {
      step_time=(current_time-previous_time_10_ms);
      if(bProgrammeDemarre)
      {
        somme_steps += step_time;
        nb_steps++;
      }
      previous_time_10_ms=current_time;
      //on relève les pas codeurs
      //on fait une boucle d'asservissement
      if (etat_asservissement == 1) asservissement();
      
      if (etat_asservissement == 2) asservissement_angle();
      
      if (frontConvergence()) etatCourant++;
    }
    //taches toutes les 50 ms
    if((current_time-previous_time_50_ms)>=50)
    {
      previous_time_50_ms=current_time;
      strategie (etatCourant);
      //on regarde l'état des capteurs
      //TODO
    }
    //taches toutes les 200 ms
    if((current_time-previous_time_200_ms)>=200)
    {
      previous_time_200_ms=current_time;
      //on s'occupe de la messagerie
      //TODO
      //on lit la distance des obstacles
      //LectureDistanceObtsacle();
    }
    //taches toutes les 500 ms
    if((current_time-previous_time_500_ms)>=500)
    {
      previous_time_500_ms=current_time;
    }
    //taches toutes les 1 s
    if((current_time-previous_time_1000_ms)>=1000)
    {
      previous_time_1000_ms=current_time;
      Serial.println(step_time);
    }

   
  //les actions lancées dans la boucle infinie
   ACTION_MOTEUR(MOTEUR_GAUCHE, vitesseG_n);
   ACTION_MOTEUR(MOTEUR_DROIT, vitesseD_n);

   //---------------------------------------------
   //CHOIX COULEUR
   //---------------------------------------------
  if(!bProgrammeDemarre)
  {
    int valeur_bouton=analogRead(PIN_COULEUR_EQUIPE);     
    if((valeur_bouton<800)&&(couleur_equipe!=EQUIPE_BLEUE)) //bouton vers le haut
    {
      couleur_equipe=EQUIPE_BLEUE;
      lcd.clear();
      lcd.setCursor(0,0); lcd.print("ROBOT TETES BRIQUEES");
      lcd.setCursor(0,1); lcd.print("STRATEGIE 1");
      lcd.setCursor(0,2); lcd.print("COULEUR: BLEU");
      lcd.setCursor(0,3); lcd.print("READY!");
      #ifdef UTILSE_LEDS
        METTRE_LEDS_A(LEDS_BLEU);
      #endif
    }
    if((valeur_bouton>=800)&&(couleur_equipe!=EQUIPE_JAUNE)) //bouton vers le haut
    {
      couleur_equipe=EQUIPE_JAUNE;
      lcd.clear();
      lcd.setCursor(0,0); lcd.print("ROBOT TETES BRIQUEES");
      lcd.setCursor(0,1); lcd.print("STRATEGIE 1");
      lcd.setCursor(0,2); lcd.print("COULEUR: JAUNE");
      lcd.setCursor(0,3); lcd.print("READY!");
      #ifdef UTILSE_LEDS
        METTRE_LEDS_A(LEDS_ORANGE);
      #endif
    }
  }


  //---------------------------------------------
  //LANCEMENT PROGRAMME
  //---------------------------------------------
  bool decision_debut_match=false;
  if(!bProgrammeDemarre)
  {
    if(ARDUINO_ALONE)
    {
      decision_debut_match=(Serial.read()=='s');
    }
    else
    {
      decision_debut_match=(digitalRead(PIN_TIRETTE)==LOW);
    }
  }
  
  
  if ((decision_debut_match)&&(!bProgrammeDemarre))
  {
    bProgrammeDemarre=true; // le programme est demarre
    
    //on vérifie la couleur de l'équipe avant de lancer le programme (pour être sûr)
    int valeur=analogRead(PIN_COULEUR_EQUIPE);
    if(valeur<800) //bouton vers le haut
    couleur_equipe=EQUIPE_BLEUE;
    else
    couleur_equipe=EQUIPE_JAUNE;
    
    etatCourant = 1;
    temps_match = 0;

    start_time_match=current_time;
    
    //initialisation écran LCD
    lcd.clear();
    lcd.setCursor(0,0); lcd.print("LANCEMENT MATCH");
  }

  //calcul du temps match
  int temps_match_ecoule=(current_time-start_time_match)/1000;
  if ((temps_match_ecoule >= DUREE_MATCH) && bProgrammeDemarre)
  {
    vitesseD_n = 0;
    vitesseG_n = 0;
    ACTION_MOTEUR(MOTEUR_GAUCHE, vitesseG_n);
    ACTION_MOTEUR(MOTEUR_DROIT, vitesseD_n);

    //moyenne d'un pas du cadenceur
    float moyenne=(float)(somme_steps/nb_steps);
    Serial.print(F("Moyenne d'un pas cadenceur: "));
    Serial.print(moyenne);
    Serial.print(F(" ms"));

    //initialisation écran LCD
    lcd.clear();
    lcd.setCursor(0,0); lcd.print("FIN DU MATCH!");
    lcd.setCursor(0,2); lcd.print("NBRE DE PTS:");
    lcd.setCursor(13,2); lcd.print(nombre_points);
    
    while (1)
    {
    }
  }

}
