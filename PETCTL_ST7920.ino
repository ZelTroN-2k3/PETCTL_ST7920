/******************************************************
   PETCTL - Exemple modifié pour U8glib + ST7920
   https://github.com/ZelTroN-2k3/PETCTL_ST7920
*******************************************************/
/*================================================================================================
  CFG_ENC_DT            2 pin encoder DT 
  CFG_ENC_CLK           3 pin encoder CLK 
  CFG_ENC_SW            4 pin encoder SW 

  CFG_STEP_DIR_PIN      5 pin stepper driver DIR 
  CFG_STEP_STEP_PIN     6 pin stepper driver STEP 
  CFG_STEP_EN_PIN       7 pin stepper driver EN 

  CFG_ENDSTOP_PIN       8 pin ENDSTOP 
  CFG_EMENDSTOP_PIN    12 pin EMENDSTOP 

  CFG_HEATER_PIN        9 pin MOSFET

  Display ST7920       10 pin CS
  Display ST7920       11 pin DATA
  Display ST7920       13 pin CLK

  CFG_TERM_PIN         A0 pin Termistor
  CFG_SOUND_PIN        A1 pin Buzzer

/*================================================================================================
                                           +-----+
                              +------------| USB |------------+
                              |            +-----+            |
  pin CLK     display ST7920  | [ ]D13/SCK        MISO/D12[ ] |  CFG_EMENDSTOP_PIN pin EMENDSTOP 
                              | [ ]3.3V           MOSI/D11[ ]~|  display ST7920    pin DATA
                              | [ ]V.ref     ___    SS/D10[ ]~|  display ST7920    pin CS
  pin Termistor CFG_TERM_PIN  | [ ]A0       / N \       D9[ ]~|  CFG_HEATER_PIN    pin MOSFET
  pin Buzzer   CFG_SOUND_PIN  | [ ]A1      /  A  \      D8[ ] |  CFG_ENDSTOP_PIN   pin ENDSTOP 
                              | [ ]A2      \  N  /      D7[ ] |  CFG_STEP_EN_PIN   pin stepper
                              | [ ]A3       \_0_/       D6[ ]~|  CFG_STEP_STEP_PIN pin stepper
                              | [ ]A4/SDA               D5[ ]~|  CFG_STEP_DIR_PIN  pin stepper
                              | [ ]A5/SCL               D4[ ] |  CFG_ENC_SW        pin encoder
                              | [ ]A6              INT1/D3[ ]~|  CFG_ENC_CLK       pin encoder
                              | [ ]A7              INT0/D2[ ] |  CFG_ENC_DT        pin encoder
                         +5v  | [ ]5V                  GND[ ] |
                              | [ ]RST                 RST[ ] | 
                         GND  | [ ]GND   5V MOSI GND   TX1[ ] | 
                              | [ ]Vin   [ ] [ ] [ ]   RX1[ ] | 
                              |          [ ] [ ] [ ]          |
                              |          MISO SCK RST         |
                              | NANO-V3                       |
                              +-------------------------------+
================================================================================================*/
#include "PETCTL_cfg.h"
#define SPEED_MAX 10

#define DRIVER_STEP_TIME 6  // changer le délai à 6 µs
#include "GyverStepper.h"
GStepper<STEPPER2WIRE> stepper(200 * CFG_STEP_DIV, CFG_STEP_STEP_PIN, CFG_STEP_DIR_PIN, CFG_STEP_EN_PIN);

#include "GyverTimers.h"

#include <U8glib.h>
// -------------------------------------------------------------------------------------
//  Création de l'objet U8g2 pour ST7920  "Adapté à votre câblage : (CLK, DATA, CS)"
//  Maintenant pour l’écran ST7920 128x64 en 3 fils software SPI:
// -------------------------------------------------------------------------------------
// U8GLIB_ST7920_128X64 u8g(/* clock=*/ 13, /* data=*/ 11, /* cs=*/ 10, U8G_PIN_NONE);  
U8GLIB_ST7920_128X64_1X u8g(/* clock=*/ 13, /* data=*/ 11, /* cs=*/ 10);  

#define CLK CFG_ENC_CLK
#define DT CFG_ENC_DT
#define SW CFG_ENC_SW

#include "GyverEncoder.h"
Encoder enc1(CLK, DT, SW);
int value = 0;

// Termistor definition
float prevTemp, curTemp = 0;
float targetTemp = CFG_TEMP_INIT;

// Déplacement / extrém
float finalLength = 0;

// PID
#include "GyverPID.h"
GyverPID regulator(CFG_PID_P, CFG_PID_I, CFG_PID_D, 200);

boolean Heat = false;

#define GEAR_RATIO ((float)CFG_RED_G1 * (float)CFG_RED_G2 * (float)CFG_RED_G3)

/* Bobin round length 74 * Pi = 232.478 232.478 mm - bobin round length */
#define BOBIN_ROUND_LENGTH ((float)3.1415926 * (float)CFG_BOBIN_DIAM)
const float REDCONST = BOBIN_ROUND_LENGTH /(360 * GEAR_RATIO * 1000);
boolean runMotor=false;
long SpeedX10 = (float)CFG_SPEED_INIT * 10;

/* Interactive statuses */
#define CHANGE_NO           0
#define CHANGE_TEMPERATURE  1
#define CHANGE_SPEED        2
int whatToChange = CHANGE_NO;
unsigned long interactive = millis();

// Arrêt d'urgence
#define OVERHEAT          1
#define THERMISTOR_ERROR  2

// Prototypes
void encRotationToValue (long* value, int inc = 1, long minValue = 0, long maxValue = 0);
void motorCTL(long setSpeedX10);
void emStop(int reason);
float getMilage();
float getTemp();
float simpleKalman(float newVal);
//bool reverseState = false;
// Indicateurs de statut
bool thermistorConnected = false;
bool pidActive = false;
bool encoderConnected = true; // Supposons que l'encodeur est connecté si le code fonctionne

// ====================================================
// SETUP
// ====================================================
void setup() {
  #if defined(SERIAL_DEBUG_TEMP) || defined(SERIAL_DEBUG_STEPPER) || defined(SERIAL_DEBUG_TEMP_PID)
    Serial.begin(9600);
  #endif //SERIAL_DEBUG_TEMP || SERIAL_DEBUG_STEPPER

  #if defined(SERIAL_DEBUG_STEPPER)
    Serial.print("Gear ratio: ");
    Serial.println(GEAR_RATIO);
    Serial.println("[deg/s],\t[step/s],\t[deg],\t[mm/s],\t[deg/s],\t[deg]");
  #endif //SERIAL_DEBUG_STEPPER

  #if defined(__LGT8F__)
    analogReadResolution(10);
  #endif

    pinMode(CFG_ENDSTOP_PIN, INPUT_PULLUP);
    pinMode(CFG_EMENDSTOP_PIN, INPUT_PULLUP);
    pinMode(CFG_SOUND_PIN, OUTPUT);
    
    // Stepper init
    stepper.setRunMode(KEEP_SPEED);   // mode régulateur de vitesse

  #if defined(CFG_STEP_INVERT)
    stepper.reverse(true);            // inverse le sens si nécessaire
  #endif //CFG_STEP_INVERT

    stepper.autoPower(true);
    stepper.setAcceleration(300);
    stepper.setSpeedDeg(mmStoDeg((float)SPEED_MAX));
    Timer2.setPeriod(stepper.getMinPeriod() / 2);
    stepper.brake();
    Timer2.enableISR();
    stepper.reset();                  // arrêter et réinitialiser la position à 0

    // stepper.enable();  // force la broche EN à l'état actif (selon la logique de la lib)

  // Écran de démarrage (simple)
  #if defined(CFG_SOUND_START)
    beepE();
  #endif

  // === ICI on redessine Boot Screen à l'écran ===
  u8g.firstPage();
  do {
    drawBootScreen();
  } while(u8g.nextPage());

    enc1.setType(CFG_ENC_TYPE);
    enc1.setPinMode(LOW_PULL);

    regulator.setpoint = targetTemp;
}

void updateThermistorStatus() {
  float temp = getTemp();
  
  // Définir des seuils réalistes pour la température (par exemple, entre -40°C et 250°C)
  if (temp > -40.0 && temp < 250.0) {
    thermistorConnected = true;
  } else {
    thermistorConnected = false;
  }
}

void updatePIDStatus() {
  pidActive = Heat;
}

/*****************************************************
   Interruption Timer
*****************************************************/
ISR(TIMER2_A) {
  enc1.tick();
  stepper.tick(); // cochez ici
}

/*****************************************************
   yield()
*****************************************************/
void yield() {
  // "coup de pouce" en cas de blocage
  stepper.tick();
}

// ====================================================
// LOOP principal
// ====================================================
void loop() {
    enc1.tick();
    stepper.tick();

    // Mise à jour des indicateurs de statut
    updateThermistorStatus();
    updatePIDStatus();

    long newTargetTemp = targetTemp;
    long newSpeedX10 = SpeedX10;
    float rest;

  // exemple de toggle sur clic
  //if (enc1.isClick()) {
  //  reverseState = !reverseState;     // on change l'état
  //  stepper.reverse(reverseState);    // on applique au moteur
  //}

  // Gestion du double/clic encodeur :
  if (enc1.isDouble()) {
    whatToChange = CHANGE_SPEED;
    interactiveSet();                  // On "force" la mise à jour de l'affichage
  }
  if (enc1.isSingle()) {
    whatToChange = CHANGE_TEMPERATURE;
    interactiveSet();
  }
  if (!isInteractive()) {
    whatToChange = CHANGE_NO;
  }

  // Ajustement via encodeur selon le mode
  if (whatToChange == CHANGE_TEMPERATURE) {
    encRotationToValue(&newTargetTemp, 1, CFG_TEMP_MIN, CFG_TEMP_MAX - 10);
    if (enc1.isHolded()) {
      Heat = !Heat;
      printHeaterStatus(Heat);
    }

    if (newTargetTemp != (long)targetTemp) {
      targetTemp = newTargetTemp;
      regulator.setpoint = newTargetTemp;
      printTargetTemp(newTargetTemp);
    }

  } else if (whatToChange == CHANGE_SPEED) {
    encRotationToValue(&newSpeedX10, 1, 0, SPEED_MAX * 10);
    if (enc1.isHolded()) {
      runMotor = !runMotor;
      if (runMotor) {
        motorCTL(newSpeedX10);
      } else {
        motorCTL(-1);
        runMotor = false;
      }
      interactiveSet();
    }
    if (newSpeedX10 != SpeedX10) {
      SpeedX10 = newSpeedX10;
      if (runMotor) motorCTL(newSpeedX10); // en degrés/sec
      printSpeed(newSpeedX10);
    }
  }

  // Mise à jour du "métrage"
  if (runMotor) {
    printMilage(stepper.getCurrentDeg());
  }

  // Mesure de température
  curTemp = getTemp();
  stepper.tick(); // keep stepping

  if (curTemp > CFG_TEMP_MAX - 10) emStop(OVERHEAT);
  if (curTemp < -10) emStop(THERMISTOR_ERROR);

  regulator.input = curTemp;
  if (curTemp != prevTemp) {
    prevTemp = curTemp;
    printCurrentTemp(curTemp);
  }

  // Contrôle du chauffage
  if (Heat) {
    int pidOut = (int)constrain(regulator.getResultTimer(), 0, 255);
    analogWrite(CFG_HEATER_PIN, pidOut);
    debugTemp(curTemp, pidOut);
  } else {
    analogWrite(CFG_HEATER_PIN, 0);
    debugTemp(curTemp, 0);
  }

  // Vérif EndStop principal
  if(!digitalRead(CFG_ENDSTOP_PIN)) {
    if(!runMotor) {
      // ...
      // On ne dessine plus ici, on stocke seulement
      if (finalLength == 0) {
        // rien de spécial
      }
    } else {
      if (finalLength > 0) {
        rest = finalLength - getMilage();
        if (rest >= 0) {
          // On continue
        } else {
          runMotor = false;
          motorCTL(0);
          Heat = false;
          printHeaterStatus(Heat);
          finalLength = 0;
          beepI();
        }
      } else {
        finalLength = getMilage() + CFG_PULL_EXTRA_LENGTH;
      }
    }
  } else {
    finalLength = 0;
  }

  // Vérif EndStop d'urgence
  if(!digitalRead(CFG_EMENDSTOP_PIN)) {
    if(!runMotor) {
      // ...
    } else {
      runMotor = false;
      motorCTL(-1);
      Heat = false;
      printHeaterStatus(Heat);
      beepI();
      beepI();
    }
  }

  // Forcer les états pour le test decocher le commentaire //
  //thermistorConnected = false;  // ou false
  //pidActive = true;           // ou true
  //encoderConnected = false;     // ou false

  // === ICI on redessine tout à l'écran ===
  u8g.firstPage();
  do {
    drawScreen();
  } while(u8g.nextPage());
}

/*****************************************************
   Fonctions "debug" etc.
*****************************************************/
void debugTemp(float temp, int out) {
  #if defined(SERIAL_DEBUG_TEMP)
    static long debug_time;
    if (debug_time < millis() ) {
      debug_time = millis() + 200;
      Serial.print(temp);
    #if defined(SERIAL_DEBUG_TEMP_PID)
      Serial.print(' ');
      Serial.print(out);
    #endif
      Serial.println();
    }
  #endif
}

// Convertit mm/s en degrés/s pour stepper
long mmStoDeg(float mmS) {
  return mmS / (REDCONST * 1000);
}

void beepE() {
  digitalWrite(CFG_SOUND_PIN, 1);
  delay(50);
  digitalWrite(CFG_SOUND_PIN, 0);
  delay(50);
}

void beepI() {
  beepE();
  beepE();
}
void beepT() {
  digitalWrite(CFG_SOUND_PIN, 1);
  delay(600);
  digitalWrite(CFG_SOUND_PIN, 0);
  delay(200);
}

void beepO() {
  beepT();
  beepT();
  beepT();
}

/*****************************************************
   emStop (arrêt d'urgence)
   => bloque le programme dans une boucle infinie
*****************************************************/
void emStop(int reason) {
  runMotor = false;
  motorCTL(-1);
  stepper.disable();
  Heat = false;
  analogWrite(CFG_HEATER_PIN, 0);

  // Mettre à jour les indicateurs de statut
  thermistorConnected = false; // Supposons qu'en arrêt d'urgence, on considère le thermistor comme déconnecté
  pidActive = false;
  
  // === ICI on redessine Boot Error à l'écran ===
  drawError(reason);             // On dessine un écran "Halt" avant de bloquer

  // Bips d'alerte
  for(;;) {
    beepO();
    delay(60000);
  }
}
 
/*****************************************************
   getMilage: convertit en mm (ou en m?) la position
*****************************************************/
float getMilage() {
  return stepper.getCurrentDeg() * REDCONST;
}

/*****************************************************
   motorCTL
   Param setSpeedX10:
     > 0 => démarre à cette vitesse
       0 => stop
      -1 => brake
*****************************************************/
void motorCTL(long setSpeedX10) {
  #if defined(SERIAL_DEBUG_STEPPER)
    Serial.print(stepper.getSpeedDeg());
    Serial.print(",\t");
    Serial.print(stepper.getSpeed());
    Serial.print(",\t");
    Serial.print(stepper.getCurrent());
    Serial.print(",\t");
  #endif // SERIAL_DEBUG_STEPPER

  if (setSpeedX10 > 0) {
    stepper.setSpeedDeg(mmStoDeg((float)setSpeedX10 / 10.0), SMOOTH);
  } else if (setSpeedX10 == 0) {
    stepper.stop();
  } else {
    stepper.brake();
  }
  
  #if defined(SERIAL_DEBUG_STEPPER)
    Serial.print((float)setSpeedX10/10);
    Serial.print(",\t");
    Serial.print(stepper.getSpeedDeg());
    Serial.print(",\t");
    Serial.print(stepper.getCurrent());
    Serial.println(" ");
  #endif // SERIAL_DEBUG_STEPPER

}

/*****************************************************
   Fonctions d'affichage "vides"
   (on ne dessine plus directement, on le fera dans
    drawScreen() à la fin de loop())
*****************************************************/
void printHeaterStatus(boolean status) {
  // Ne dessine plus rien directement
  // On se contente de modifier la variable Heat si besoin
}


void printTargetTemp(float t){
  // Ne dessine plus rien directement
}

void printCurrentTemp(float t) {
  // Ne dessine plus rien directement
}

void printSpeed(long s){
  // Ne dessine plus rien directement
}

void printMilage(float m){
  // Ne dessine plus rien directement
}

/*****************************************************
   encRotationToValue
*****************************************************/
void encRotationToValue (long* value, int inc, long minValue, long maxValue) {
  if (enc1.isRight())  { *value += inc;      interactiveSet(); }
  if (enc1.isFastR())  { *value += inc * 5;  interactiveSet(); }
  if (enc1.isLeft())   { *value -= inc;      interactiveSet(); }
  if (enc1.isFastL())  { *value -= inc * 5;  interactiveSet(); }
  if (*value < minValue) *value = minValue;
  if (*value > maxValue) *value = maxValue;
}

/*****************************************************
   Système "interactive" (15s)
*****************************************************/
void interactiveSet() {
  interactive = millis() + 15000;
}
boolean isInteractive() {
  return millis() < interactive;
}

/*****************************************************
   getTemp - Thermistance + filtrage simple Kalman
*****************************************************/
float getTemp() {
  uint8_t i;
  float average;

  average = analogRead(CFG_TERM_PIN);
  // convert the value to resistance
  average = 1023 / average - 1;
  average = CFG_TERM_SERIAL_R / average;

  float steinhart;
  steinhart = average / CFG_TERM_VALUE;     // (R/Ro)
  steinhart = log(steinhart);                  // ln(R/Ro)
  steinhart /= CFG_TERM_B_COEFF;                   // 1/B * ln(R/Ro)
  steinhart += 1.0 / (CFG_TERM_VALUE_TEMP + 273.15); // + (1/To)
  steinhart = 1.0 / steinhart;                 // Invert
  steinhart -= 273.15;                         // convert absolute temp to C

  return simpleKalman(steinhart);
}

/*****************************************************
   Filtre simple Kalman
*****************************************************/
// https://alexgyver.ru/lessons/filters/
float _err_measure = 0.8;  // bruit de mesure approximatif
float _q = 0.02;   // le taux de variation des valeurs est de 0,001-1, faites-le varier vous-même

float simpleKalman(float newVal) {
  float _kalman_gain, _current_estimate;
  static float _err_estimate = _err_measure;
  static float _last_estimate;
  _kalman_gain = (float)_err_estimate / (_err_estimate + _err_measure);
  _current_estimate = _last_estimate + (float)_kalman_gain * (newVal - _last_estimate);
  _err_estimate =  (1.0 - _kalman_gain) * _err_estimate + fabs(_last_estimate - _current_estimate) * _q;
  _last_estimate = _current_estimate;
  return _current_estimate;
}

/************************************************************************************************
   drawError()
   => dessine un écran "Halt" "Overheat" "Thermistor" avant de bloquer, en fonction de la raison
*************************************************************************************************/
void drawError(int reason) {
  if (runMotor == false && (reason == OVERHEAT || reason == THERMISTOR_ERROR)) {
    u8g.firstPage();
    do {
      u8g.setFont(u8g_font_9x15Br);
      u8g.setPrintPos(30, 20);
      u8g.print("*HALT!*");

      u8g.setFont(u8g_font_6x12);
      u8g.setPrintPos(32, 40);
      switch (reason) {
        case OVERHEAT:
          u8g.print("Overheat");
          break;
        case THERMISTOR_ERROR:
          u8g.print("Thermistor");
          break;
      }
    } while(u8g.nextPage());
  }
}
/*****************************************************
   drawBootScreen()
   => dessine TOUT l'écran en fonction des variables
*****************************************************/
void drawBootScreen() {
  // On peut faire un petit écran de démarrage en utilisant U8glib:
  u8g.firstPage();
  do {
    // On choisit une police grande :
    u8g.setFont(u8g_font_9x15Br);
    // We want the reference position for printing the string to be in the upper left corner of that string
    u8g.setFontPosTop(); 
    // X i Y coordinates where the string wants to print out on the display
    u8g.setPrintPos(20, 10); u8g.print("    PETCTL");
    u8g.setFont(u8g_font_6x12);
    u8g.setPrintPos(25, 40); u8g.print("mvb V0.11a");
  } while(u8g.nextPage());
  delay(4000); // 2 secondes de splash
} 

/*****************************************************
   drawScreen()
   => dessine TOUT l'écran en fonction des variables
*****************************************************/
void drawScreen() {
  // Réinitialiser la couleur à blanc pour dessiner les éléments généraux
  u8g.setColorIndex(1); // Blanc
    
  // 1) Dessiner une bordure tout autour de l’écran
  u8g.drawFrame(0, 0, 128, 64);

  // 2) Dessiner des lignes horizontales
  u8g.drawLine(0, 13, 128, 13);   // Ligne horizontale en haut

  // 3) Dessiner une ligne verticale pour séparer les sections (par exemple à x=74)
  u8g.drawLine(74, 0, 74, 13);    // Ligne verticale en haut
  u8g.drawLine(0, 52, 128, 52);   // Ligne verticale centrale

  // Températures en gros (ou moyen)
  // Température courante
  // 3) Afficher les températures
  { 
    u8g.setFont(u8g_font_9x15Br);
    char buf[10];
    dtostrf(curTemp, 4, 1, buf);
    // y=30 (baseline) => on voit le texte vers 30
    u8g.setPrintPos(8, 12);
    u8g.print(buf);
    u8g.setPrintPos(2, 8);
    u8g.setFont(u8g_font_5x8r); u8g.print("C"); // current
  }

  // Température cible
  {
    u8g.setFont(u8g_font_9x15Br);
    // Dessin "Heat" ou pas. -> si Heat = true => on affiche "*"
    u8g.setPrintPos(76, 10);
    if (Heat) {
      u8g.print("*");             // Heat ON
    } else {     
      u8g.print(".");             // Heat OFF
    }

    char buf[10];
    dtostrf(targetTemp, 4, 0, buf);
    // x=70 => un peu plus loin
    u8g.setPrintPos(80, 12);
    // Inversion si whatToChange == CHANGE_TEMPERATURE ?
    if (whatToChange == CHANGE_TEMPERATURE && isInteractive()) {
      // Dessiner un fond noir
      u8g.drawBox(86, 1, 41, 12); // (x,y,width,height)
      u8g.setColorIndex(0); // Écrire en "couleur inversée"
      u8g.setPrintPos(80, 12);
      u8g.print(buf);
      u8g.setPrintPos(116, 8);
      u8g.setFont(u8g_font_5x8r); u8g.print("C");
      u8g.setColorIndex(1); // Revenir en couleur normale
    } else {
      u8g.print(buf);
      u8g.setPrintPos(116, 8);
      u8g.setFont(u8g_font_5x8r); u8g.print("C");
    }
  }

  // Vitesse en mm/s
  // 4) Afficher l'indicateur "runMotor"
  {
    u8g.setFont(u8g_font_9x15Br); //font_blipfest_07n
    // l’indicateur runMotor ou pas. -> si Run = true => on affiche "*"
    //u8g.setFont(u8g_font_6x12);       // Police petite
    u8g.setPrintPos(3, 32);           // Position x=3, y=32
    if (runMotor) {
      u8g.print("*");                 // Moteur ON
    } else {
      u8g.print(".");                 // Moteur OFF
    }
    
      
    //u8g.setFont(u8g_font_6x12);
    float spd = (float)SpeedX10 / 10.0;
    char buf[10];
    dtostrf(spd, 4, 1, buf);
    u8g.setPrintPos(3, 32);
    if (whatToChange == CHANGE_SPEED && isInteractive()) {
      // Surligne
      u8g.drawBox(0, 37, 60, 16);
      u8g.setColorIndex(0);
      u8g.setPrintPos(3, 32);
      u8g.print(buf);
      u8g.setFont(u8g_font_6x12);      
      u8g.print(" mm/s");
      u8g.setColorIndex(1);
    } else {
      u8g.print(buf);
      u8g.setFont(u8g_font_6x12);      
      u8g.print(" mm/s");
    }
  }

  // Avancement (métrage)
  // 5) Afficher l'avancement (métrage)
  {
    u8g.setFont(u8g_font_9x15Br);
    float dist = getMilage(); // en mètres ?
    // Dans votre code, getMilage() renvoie "deg * REDCONST"
    // A vous de voir l'unité exacte (mm, m, etc.)
    char buf[10];
    dtostrf(dist, 4, 2, buf);
    u8g.setPrintPos(12, 48);
    u8g.print(buf);
    u8g.setFont(u8g_font_5x8r);
    u8g.print(" m");
  }

/*
  // Fin de courses (debug ?) => par exemple
  // endstop = CFG_ENDSTOP_PIN, emendstop = CFG_EMENDSTOP_PIN
  // 6) Afficher les Endstops
  if (!digitalRead(CFG_ENDSTOP_PIN)) {
    u8g.setFont(u8g_font_4x6r);
    u8g.setPrintPos(105, 62);
    u8g.print("END");
  }
  if (!digitalRead(CFG_EMENDSTOP_PIN)) {
    u8g.setFont(u8g_font_4x6r);
    u8g.setPrintPos(105, 62);
    u8g.print("X");
  }
*/
/*
  // 7.1) Afficher la connexion au Thermistor
  u8g.setFont(u8g_font_5x8r);
  u8g.setPrintPos(4, 61); // Position x=5, y=10
  u8g.print("T:");
  if (thermistorConnected) {
    u8g.setPrintPos(14, 61);
    u8g.print("OK");
  } else {
    u8g.setPrintPos(14, 61);
    u8g.print("ERR");
  }
  
  // 7.2) Afficher l'état du PID
  u8g.setPrintPos(30, 61); // Position x=100, y=10
  u8g.print("PID:");
  if (pidActive) {
    u8g.setPrintPos(50, 61);
    u8g.print("ON");
  } else {
    u8g.setPrintPos(50, 61);
    u8g.print("OFF");
  }
  
  // 7.3) Afficher l'état de l'Encodeur
  // (Supposons que si le code fonctionne, l'encodeur est connecté)
  u8g.setPrintPos(70, 61); // Position x=5, y=52
  u8g.print("ENC:");
  if (encoderConnected) {
    u8g.setPrintPos(90, 61);
    u8g.print("OK");
  } else {
    u8g.setPrintPos(90, 61);
    u8g.print("ERR");
  } */

  // 6) Afficher les Endstops avec boîtes
  drawEndstopStatus();
  // 6.1) Afficher les Endstops avec boîtes
  drawEMEndstopStatus();

  // 7) Afficher les indicateurs de statut avec boîtes et inversion des couleurs
  drawThermistorStatus();
  drawPIDStatus();
  drawEncoderStatus();  
}

/*****************************************************
// Fin de courses (debug ?) => par exemple
// endstop = CFG_ENDSTOP_PIN
// 6) Afficher les Endstops
*****************************************************/
void drawEndstopStatus() {
  if (!digitalRead(CFG_ENDSTOP_PIN)) { // Endstop activé (LOW)
    // Dessiner la boîte de fond en blanc
    u8g.setColorIndex(1); // Blanc
    u8g.drawBox(97, 54, 17, 8); // (x, y, width, height)
    
    // Dessiner "END" en noir
    u8g.setColorIndex(0); // Noir
    u8g.setFont(u8g_font_4x6r);
    u8g.setPrintPos(100, 61); // Ajustez la position selon la police
    u8g.print("END");
  }
  else { // Endstop désactivé (HIGH)
    // Dessiner la boîte de fond en noir
    u8g.setColorIndex(1); // Blanc
    u8g.drawFrame(97, 54, 17, 8); // (x, y, width, height)
    
    // Dessiner "END" en blanc
    u8g.setColorIndex(1); // Blanc
    u8g.setFont(u8g_font_4x6r);
    u8g.setPrintPos(100, 61);
    u8g.print("END");
  }
  // Réinitialiser l'indice de couleur à blanc pour éviter les interférences
  u8g.setColorIndex(1);
}

/*****************************************************
// Fin de courses (debug ?) => par exemple
// emendstop = CFG_EMENDSTOP_PIN
// 6.1) Afficher les Endstops
*****************************************************/
void drawEMEndstopStatus() { 
  if (!digitalRead(CFG_EMENDSTOP_PIN)) { // EM Endstop activé (LOW)
    // Dessiner la boîte de fond en blanc
    u8g.setColorIndex(1); // Blanc
    u8g.drawBox(115, 54, 11, 8); // (x, y, width, height)
    
    // Dessiner "X" en noir
    u8g.setColorIndex(0); // Noir
    u8g.setFont(u8g_font_4x6r);
    u8g.setPrintPos(119, 61); // Ajustez la position selon la police
    u8g.print("X");
  }
  else { // EM Endstop désactivé (HIGH)
    // Dessiner la boîte de fond en noir
    u8g.setColorIndex(1); // Blanc
    u8g.drawFrame(115, 54, 11, 8); // (x, y, width, height)
    
    // Dessiner "X" en blanc
    u8g.setColorIndex(1); // Blanc
    u8g.setFont(u8g_font_4x6r);
    u8g.setPrintPos(119, 60);
    u8g.print("x");
  }
  // Réinitialiser l'indice de couleur à blanc pour éviter les interférences
  u8g.setColorIndex(1);
}

/*****************************************************
// 7.1) Afficher la connexion au Thermistor
*****************************************************/
void drawThermistorStatus() {
  if (thermistorConnected) {
    // Dessiner la boîte de fond en blanc
    u8g.setColorIndex(1); // Blanc
    u8g.drawBox(2, 54, 30, 8); // (x, y, largeur, hauteur)
    
    // Dessiner "T:" et "OK" en noir
    u8g.setColorIndex(0); // Noir
    u8g.setFont(u8g_font_4x6r);
    u8g.setPrintPos(6, 61); // Position x=6, y=61
    u8g.print("T:");
    u8g.setPrintPos(14, 61);
    u8g.print("OK ");
  }
  else {
    // Dessiner la boîte de fond en noir
    u8g.setColorIndex(0); // Noir
    u8g.drawBox(2, 54, 30, 8); // (x, y, largeur, hauteur)
    
    // Dessiner "T:" et "ERR" en blanc
    u8g.setColorIndex(1); // Blanc
    u8g.setFont(u8g_font_4x6r);
    u8g.setPrintPos(6, 61); // Position x=6, y=61
    u8g.print("T:");
    u8g.setPrintPos(14, 61);
    u8g.print("ERR");
  }
}

/*****************************************************
// 7.2) Afficher l'état du PID
*****************************************************/
void drawPIDStatus() {
  if (pidActive) {
    // Dessiner la boîte de fond en blanc
    u8g.setColorIndex(1); // Blanc
    u8g.drawBox(33, 54, 30, 8); // (x, y, largeur, hauteur)
    
    // Dessiner "PID:" et "ON" en noir
    u8g.setColorIndex(0); // Noir
    u8g.setFont(u8g_font_4x6r);
    u8g.setPrintPos(34, 61); // Position x=33, y=61
    u8g.print("PID:");
    u8g.setPrintPos(50, 61);
    u8g.print("ON ");
  }
  else {
    // Dessiner la boîte de fond en noir
    u8g.setColorIndex(0); // Noir
    u8g.drawBox(33, 54, 30, 8); // (x, y, largeur, hauteur)
    
    // Dessiner "PID:" et "OFF" en blanc
    u8g.setColorIndex(1); // Blanc
    u8g.setFont(u8g_font_4x6r);
    u8g.setPrintPos(34, 61); // Position x=33, y=61
    u8g.print("PID:");
    u8g.setPrintPos(50, 61);
    u8g.print("OFF");
  }
}

/*****************************************************************
// 7.3) Afficher l'état de l'Encodeur
// (Supposons que si le code fonctionne, l'encodeur est connecté)
******************************************************************/
void drawEncoderStatus() {
  if (encoderConnected) {
    // Dessiner la boîte de fond en blanc
    u8g.setColorIndex(1); // Blanc
    u8g.drawBox(64, 54, 32, 8); // (x, y, largeur, hauteur)
    
    // Dessiner "ENC:" et "OK" en noir
    u8g.setColorIndex(0); // Noir
    u8g.setFont(u8g_font_4x6r);
    u8g.setPrintPos(66, 61); // Position x=65, y=61
    u8g.print("ENC:");
    u8g.setPrintPos(82, 61);
    u8g.print("OK ");
  }
  else {
    // Dessiner la boîte de fond en noir
    u8g.setColorIndex(0); // Noir
    u8g.drawBox(64, 54, 32, 8); // (x, y, largeur, hauteur)
    
    // Dessiner "ENC:" et "ERR" en blanc
    u8g.setColorIndex(1); // Blanc
    u8g.setFont(u8g_font_4x6r);
    u8g.setPrintPos(66, 61); // Position x=65, y=61
    u8g.print("ENC:");
    u8g.setPrintPos(82, 61);
    u8g.print("ERR");
  }
}

