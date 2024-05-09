#include <AccelStepper.h>
#include <MultiStepper.h>
#include <VL53L0X_mod.h>
//POSITION-------------------------------------------------------------------------------------------
void turnLeft(int angle);
void turnLeft(int angle);
void moveTo(float distance);
void end();
void editSpeed(unsigned int speed = 1);
//STRATEGIE------------------------------------------------------------------------------------------
unsigned int step = 0;
bool positionSet = false;
unsigned int side = 0;  // 1 = left | 2 = right
//#######################################PARAMETRES##################################################
unsigned int robot = 2; // 0 = PAMI n°0 zone | 1 = PAMI n°1 demo | 2 = PAMI n°2 jardinière
unsigned int enableTimer = true; // false = OFF | true = ON
//####################################################################################################
//TIMER----------------------------------------------------------------------------------------------
//long int t1 = 0;
//long int t2 = 0;
long int timeMatch = 0;
// Generally, you should use "unsigned long" for variables that hold time
// The value will quickly become too large for an int to store
unsigned long previousMillis = 0;  // will store last time LED was updated
// constants won't change:
const long interval = 500;  // interval at which to blink (milliseconds)

//LASER----------------------------------------------------------------------------------------------
// The number of sensors in your system.
const uint8_t sensorCount = 2;
// The Arduino pin connected to the XSHUT pin of each sensor.
const uint8_t xshutPins[sensorCount] = { 4, 7 };
VL53L0X_mod sensors[sensorCount];

uint16_t sensorLeftNew = 65535;
uint16_t sensorRightNew = 65535;

uint16_t sensorLeft = 65535;
uint16_t sensorRight = 65535;

//MOTEURS--------------------------------------------------------------------------------------------
// Configuration des broches pour le moteur X
#define X_STEP_PIN 3  // Broche STEP du moteur X
#define X_DIR_PIN 6   // Broche DIR du moteur X
#define ENABLE_PIN 8  // Broche ENABLE du moteur

#define M0_PIN 9   // Broche M0 du moteur X
#define M1_PIN 10  // Broche M1 du moteur X
#define M2_PIN 11  // Broche M2 du moteur X

// Configuration des broches pour le moteur Y
#define Y_STEP_PIN 2  // Broche STEP du moteur Y
#define Y_DIR_PIN 5   // Broche DIR du moteur Y

// Configuration des paramètres des moteurs
#define MOTOR_STEPS 200  // Nombre de pas par tour du moteur
#define RPM 50           // Vitesse de rotation en tours par minute
#define MICROSTEPS 16    // Configuration du microstepping (1/16 de pas)
#define WHEEL_DIAMETER_MM 62   // Diamètre des roues en millimètres
#define WHEEL_DISTANCE_MM 107.75  // Distance entre les deux roues en millimètres

// Définition des objets AccelStepper pour chaque moteur
AccelStepper stepperX(AccelStepper::DRIVER, X_STEP_PIN, X_DIR_PIN);
AccelStepper stepperY(AccelStepper::DRIVER, Y_STEP_PIN, Y_DIR_PIN);

// Créer un objet MultiStepper pour synchroniser les moteurs
MultiStepper multiStepper;

long positions[2];
//TIRETTE----------------------------------------------------------------------------------------------
const int TIRETTE = 13;
int etat = 0;
//TEST-------------------------------------------------------------------------------------------------
unsigned long time;
unsigned long waitTime;
//SETUP================================================================================================
void setup() {
  Serial.begin(9600);
  Serial.println("GOOD : Let's go!!");

  //LASER----------------------------------------------------------------------------------------------
  while (!Serial) {}
  Wire.begin();

  // Disable/reset all sensors by driving their XSHUT pins low.
  for (uint8_t i = 0; i < sensorCount; i++) {
    pinMode(xshutPins[i], OUTPUT);
    digitalWrite(xshutPins[i], LOW);
  }

  // Enable, initialize, and start each sensor, one by one.
  for (uint8_t i = 0; i < sensorCount; i++) {
    // Stop driving this sensor's XSHUT low. This should allow the carrier
    // board to pull it high. (We do NOT want to drive XSHUT high since it is
    // not level shifted.) Then wait a bit for the sensor to start up.
    pinMode(xshutPins[i], INPUT);
    delay(10);

    sensors[i].setTimeout(500);
    if (!sensors[i].init()) {
      Serial.print("ERROR : Failed to detect and initialize sensor ");
      Serial.println(i);
      while (1) {}
    }

    // Each sensor must have its address changed to a unique value other than
    // the default of 0x29 (except for the last one, which could be left at
    // the default). To make it simple, we'll just count up from 0x2A.
    sensors[i].setAddress(0x30 + i);

    //sensors[i].startContinuous(1000);
    //sensors[i].startContinuous();
    sensors[i].setMeasurementTimingBudget(20000);
  }

  Serial.println(F("GOOD : Sensors Initialized"));

  //MOTEURS--------------------------------------------------------------------------------------------

  // Configuration des broches pour le moteur X
  pinMode(ENABLE_PIN, OUTPUT);

  // Configuration des paramètres des moteurs X et Y
  editSpeed(5);

  // Activation des sorties des drivers pour les moteurs X et Y
  digitalWrite(ENABLE_PIN, LOW);  // Activer le driver du moteur X (LOW = activé)
  digitalWrite(ENABLE_PIN, LOW);  // Activer le driver du moteur Y (LOW = activé)

  // Ajouter les moteurs à la MultiStepper (X et Y)
  multiStepper.addStepper(stepperX);
  multiStepper.addStepper(stepperY);
 
  //TIRETTE----------------------------------------------------------------------------------------------
  pinMode(TIRETTE, INPUT_PULLUP);
  while(digitalRead(TIRETTE) != 0){
    Serial.println("ERROR : tirette absente");
  }

  //CHOIX DU COTE----------------------------------------------------------------------------------------------
  delay(1000);

    sensorLeft = sensors[1].readRangeSingleMillimeters();   //distance sensor left
    sensorRight = sensors[0].readRangeSingleMillimeters();  //distance sensor right
  
  while ((sensorLeft >= 30 || sensorLeft <= 10) && (sensorRight >= 45 || sensorRight <= 10)) {
    Serial.print("no side choose");
    Serial.println();
    sensorLeft = sensors[1].readRangeSingleMillimeters();   //distance sensor left
    sensorRight = sensors[0].readRangeSingleMillimeters();  //distance sensor right
  }
  
  delay(1000);
  if (sensorLeft <= 40) {
    side = 0;
    Serial.println("GOOD : Left");
    moveTo(-25);
    multiStepper.runSpeedToPosition(); // Attendre que les moteurs atteignent les positions cibles
    delay(500);
    moveTo(13.5);
    multiStepper.runSpeedToPosition(); // Attendre que les moteurs atteignent les positions cibles
    delay(500);
    if(robot == 0 || robot == 2){
      turnLeft(90);
      multiStepper.runSpeedToPosition(); // Attendre que les moteurs atteignent les positions cibles
    }
    
  } else {
    side = 1;
    Serial.println("GOOD : Right");
    moveTo(-25);
    multiStepper.runSpeedToPosition(); // Attendre que les moteurs atteignent les positions cibles
    delay(500);
    moveTo(13.5);
    multiStepper.runSpeedToPosition(); // Attendre que les moteurs atteignent les positions cibles
    delay(500);
    if(robot == 0 || robot == 2){
      turnRight(90);
      multiStepper.runSpeedToPosition(); // Attendre que les moteurs atteignent les positions cibles
    }
  }
  
  editSpeed(1);

  //START----------------------------------------------------------------------------------------------
  while(digitalRead(TIRETTE) != 0){
    Serial.println("ERROR : tirette absente");
  }
  while(digitalRead(TIRETTE) != 1){
    Serial.println("GOOD : tirette ON");
  }
  Serial.println("GOOD : tirette OFF");

  if(enableTimer){
    delay(90000); // attendre 90s que ce soit au tour du pami !!!!!!!!!!!!!!!!!!!!!!!!
  }
  timeMatch = millis();
  
  //TEST-------------------------------------------------------------------------------------------------
  waitTime = millis();/*
  while(1){
    if( sensors[1].readRangeNoBlocking(sensorLeftNew)){ //distance sensor left
      sensorLeft = sensorLeftNew;
    }
    if(sensors[0].readRangeNoBlocking(sensorRightNew)){  //distance sensor right
          sensorRight = sensorRightNew;
    }
    Serial.print(sensorLeft);
    Serial.print(" | ");
    Serial.println(sensorRight);
  }*/
}

//LOOP==================================================================================================
void loop() {

  if((millis() - timeMatch) >= (10000-500)){
    Serial.println("ERROR : FiIN DE MATCH");
    stepperX.stop();
    stepperY.stop();
    end();
    while(1);

  }

  unsigned int currentMillis = millis();
  if (currentMillis - previousMillis >= interval) {
    multiStepper.run();
    sensorLeft = sensors[1].readRangeSingleMillimeters();   //distance sensor left
    multiStepper.run();
    sensorRight = sensors[0].readRangeSingleMillimeters();  //distance sensor right
    multiStepper.run();
    previousMillis = currentMillis;
  } 

  if (((sensorLeft >= 30 && sensorLeft <= 100) || (sensorRight >= 45 && sensorRight <= 100))){
  }else {
    if (!positionSet) {
      //Serial.println("GOOD : positionSet");
      if(robot == 0){// PAMI n°0 qui va tt droit dans la première zone
        //Serial.println("PAMI n°0");
        if (side == 0) {
          switch (step) {
            case 0:
              moveTo(650);
              break;
            case 1:
              moveTo(120);
              multiStepper.runSpeedToPosition(); // Attendre que les moteurs atteignent les positions cibles
              break;
            case 2:
              end();
          }
        } else if (side == 1) {
          switch (step) {
            case 0:
              moveTo(650);
              break;
            case 1:
              moveTo(120);
              multiStepper.runSpeedToPosition(); // Attendre que les moteurs atteignent les positions cibles
              break;
            case 2:
              end();
          }
        }
      }else if(robot == 1){// PAMI n°1 qui va dans la zone plus loin
        //Serial.println("PAMI n°1");
        if (side == 0) {

          switch (step) {
            case 0:
              moveTo(150);
              break;
            case 1:
              turnLeft(90);
              break;
            case 2:
              moveTo(650);
              break;
            case 3:
              turnRight(90);
              break;
            case 4:
              moveTo(1550);
              break;
            case 5:
              turnLeft(90);
              break;
            case 6:
              moveTo(200);
              break;
            case 7:
              end();
          }
        } else if (side == 1) {
          switch (step) {
            case 0:
              moveTo(150);
              break;
            case 1:
              turnRight(90);
              break;
            case 2:
              moveTo(650);
              break;
            case 3:
              turnLeft(90);
              break;
            case 4:
              moveTo(1550);
              break;
            case 5:
              turnRight(90);
              break;
            case 6:
              moveTo(200);
              break;
            case 7:
              end();  
          }
        }
      }else if(robot == 2){// PAMI n°2 qui va dans la jardinière
        //Serial.println("PAMI n°2");
        if (side == 0) {
          switch (step) {
            case 0:
              moveTo(650);
              break;
            case 1:
              turnLeft(90);
              delay(200);
              moveTo(75);
              multiStepper.runSpeedToPosition(); // Attendre que les moteurs atteignent les positions cibles
              break;
            case 2:
              end();
          }
        } else if (side == 1) {
          switch (step) {
            case 0:
              moveTo(650);
              break;
            case 1:
              turnRight(90);
              delay(200);
              moveTo(75);
              multiStepper.runSpeedToPosition(); // Attendre que les moteurs atteignent les positions cibles
              break;
            case 2:
              end();
          }
        }
      }
      positionSet = true;
    }
    if ((stepperX.distanceToGo() == 0 && stepperY.distanceToGo() == 0) || Serial.read() == 49) {
      //Serial.println("GOOD : point de passage");
      //delay(1000);
      step++;
      positionSet = false;
    } else {
      multiStepper.run();
    }
  }
}

//FONCTION================================================================================================
// Fonction pour faire tourner le robot à gauche d'un angle donné en degrés
void turnLeft(int angle) {
  float distance = (angle / 360.0) * PI * WHEEL_DISTANCE_MM;  // Calcul de la distance à parcourir par chaque roue

  long steps = distance * MOTOR_STEPS / (PI * WHEEL_DIAMETER_MM) * MICROSTEPS-130;  // Nombre de pas nécessaires pour chaque roue

  stepperX.setCurrentPosition(0);
  stepperY.setCurrentPosition(0);

  // Calcul des positions cibles pour chaque moteur
  long positions[2];
  positions[1] = steps;  // Moteur X (roue droite)
  positions[0] = steps;  // Moteur Y (roue gauche)

  // Déplacer les moteurs vers les positions cibles en utilisant MultiStepper
  multiStepper.moveTo(positions);
  multiStepper.runSpeedToPosition(); // Attendre que les moteurs atteignent les positions cibles
  //delay(25);
}

// Fonction pour faire tourner le robot à droite d'un angle donné en degrés
void turnRight(int angle) {
  float distance = (angle / 360.0) * PI * WHEEL_DISTANCE_MM;  // Calcul de la distance à parcourir par chaque roue

  long steps = distance * MOTOR_STEPS / (PI * WHEEL_DIAMETER_MM) * MICROSTEPS-130;  // Nombre de pas nécessaires pour chaque roue

  stepperX.setCurrentPosition(0);
  stepperY.setCurrentPosition(0);

  // Calcul des positions cibles pour chaque moteur
  long positions[2];
  positions[1] = -steps;  // Moteur X (roue droite)
  positions[0] = -steps;  // Moteur Y (roue gauche)

  // Déplacer les moteurs vers les positions cibles en utilisant MultiStepper
  multiStepper.moveTo(positions);
  multiStepper.runSpeedToPosition(); // Attendre que les moteurs atteignent les positions cibles
  //delay(25);
  
}

// Fonction pour faire avancer le robot sur une distance donnée en mm
void  moveTo(float distance) {
  float distanceSteps = (distance / (PI * WHEEL_DIAMETER_MM)) * MOTOR_STEPS * MICROSTEPS;  // Calcul du nombre de pas nécessaires

  stepperX.setCurrentPosition(0);
  stepperY.setCurrentPosition(0);

  // Définir les positions cibles pour chaque moteur dans la MultiStepper
  long positions[2];
  positions[1] = distanceSteps;
  positions[0] = -distanceSteps;

  multiStepper.moveTo(positions);  // Déplacer les moteurs vers les positions cibles
  
}
void end(){
  Serial.println("GOOD : bien arrivé");
  digitalWrite(ENABLE_PIN, HIGH);  // Désactiver le driver du moteur X (HIGH = désactivé)
  digitalWrite(ENABLE_PIN, HIGH);  // Désactiver le driver du moteur Y (HIGH = désactivé)
  time = millis();
  while(1){
    Serial.print("GOOD : bien arrivé | temps de parcours : \t");
    Serial.println((time - waitTime)/1000);
    delay(1000); 
  }
}
void editSpeed(unsigned int speed = 1){
  stepperX.setMaxSpeed((float(MOTOR_STEPS * MICROSTEPS) / 60) * RPM/speed);
  stepperX.setAcceleration((float(MOTOR_STEPS * MICROSTEPS) / 60) * RPM/(speed*4));
  stepperY.setMaxSpeed((float(MOTOR_STEPS * MICROSTEPS) / 60) * RPM/speed);
  stepperY.setAcceleration((float(MOTOR_STEPS * MICROSTEPS) / 60) * RPM/(speed*4));
}
