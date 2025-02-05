/****************************************************************************************************
 *  PETCTL - Exemple modifié pour U8glib + ST7920
 *  Source : https://github.com/ZelTroN-2k3/PETCTL_ST7920
 *
 *  ┌───────────────────────────────────────────────────────────────────────────────────────────────┐
 *  │ Liste du matériel :                                                                           │
 *  │   • Arduino Nano                                                                              │
 *  │   • Écran ST7920 128x64                                                                       │
 *  │   • Moteur NEMA 17 (17HS3401S)                                                                │
 *  │   • Pilote de moteur pas à pas A4988                                                          │
 *  │   • Encodeur rotatif KY-040                                                                   │
 *  │   • Capteur photoélectrique LM393                                                             │
 *  │   • MOSFET IRFZ44N                                                                            │
 *  │   • Thermistance NTC 100K ohm B3950                                                           │
 *  │   • Cartouche chauffante 12V/40W                                                              │
 *  │                                                                                               │
 *  │ Le programme suivant est prévu pour un Arduino Nano, avec les éléments ci-dessus connectés,   │
 *  │ à savoir :                                                                                    │
 *  │   - Écran ST79220                                                                             │
 *  │   - NEMA 17                                                                                   │
 *  │   - Pilote A4988                                                                              │
 *  │   - Encodeur rotatif KY-040                                                                   │
 *  │   - Capteur photoélectrique LM393                                                             │
 *  │   - MOSFET IRFZ44N                                                                            │
 *  │   - Thermistance NTC 100K ohm B3950                                                           │
 *  │   - Cartouche chauffante 12V/40W                                                              │
 *  └───────────────────────────────────────────────────────────────────────────────────────────────┘
 *
 *  ┌───────────────────────────────────────────────────────────────────────────────────────────────┐
 *  │ Récap des connexions (définitions) :                                                          │
 *  │   CFG_ENC_DT            =  2   // pin encoder DT                                              │
 *  │   CFG_ENC_CLK           =  3   // pin encoder CLK                                             │
 *  │   CFG_ENC_SW            =  4   // pin encodeur SW                                             │
 *  │                                                                                               │
 *  │   CFG_STEP_DIR_PIN      =  5   // pin stepper driver DIR                                      │
 *  │   CFG_STEP_STEP_PIN     =  6   // pin stepper driver STEP                                     │
 *  │   CFG_STEP_EN_PIN       =  7   // pin stepper driver EN                                       │
 *  │                                                                                               │
 *  │   CFG_ENDSTOP_PIN       =  8   // pin ENDSTOP                                                 │
 *  │   CFG_EMENDSTOP_PIN     = 12   // pin EMENDSTOP (Axe d'urgence)                               │
 *  │                                                                                               │
 *  │   CFG_HEATER_PIN        =  9   // pin MOSFET (contrôle de chauffe)                            │
 *  │                                                                                               │
 *  │   Display ST7920        = 10   // pin CS                                                      │
 *  │   Display ST7920        = 11   // pin DATA                                                    │
 *  │   Display ST7920        = 13   // pin CLK                                                     │
 *  │                                                                                               │
 *  │   CFG_TERM_PIN          = A0  // pin Thermistor                                               │
 *  │   CFG_SOUND_PIN         = A1  // pin Buzzer                                                   │
 *  └───────────────────────────────────────────────────────────────────────────────────────────────┘
 ****************************************************************************************************/


/****************************************************************************************************
 * ARDUINO NANO - Schéma de câblage (ST7920, Endstops, Stepper, etc.)
 *
 *                                           +-----+
 *                              +------------| USB |------------+
 *                              |            +-----+            |
 *  pin CLK     display ST7920  | [ ]D13/SCK        MISO/D12[ ] |  CFG_EMENDSTOP_PIN pin EMENDSTOP 
 *                       +3.3v  | [ ]3.3V           MOSI/D11[ ]~|  display ST7920    pin DATA
 *                              | [ ]V.ref     ___    SS/D10[ ]~|  display ST7920    pin CS
 *  pin Thermistor CFG_TERM_PIN | [ ]A0       / N \       D9[ ]~|  CFG_HEATER_PIN    pin MOSFET
 *  pin Buzzer   CFG_SOUND_PIN  | [ ]A1      /  A  \      D8[ ] |  CFG_ENDSTOP_PIN   pin ENDSTOP 
 *                              | [ ]A2      \  N  /      D7[ ] |  CFG_STEP_EN_PIN   pin stepper
 *                              | [ ]A3       \_0_/       D6[ ]~|  CFG_STEP_STEP_PIN pin stepper
 *                              | [ ]A4/SDA               D5[ ]~|  CFG_STEP_DIR_PIN  pin stepper
 *                              | [ ]A5/SCL               D4[ ] |  CFG_ENC_SW        pin encodeur
 *                              | [ ]A6              INT1/D3[ ]~|  CFG_ENC_CLK       pin encodeur
 *                              | [ ]A7              INT0/D2[ ] |  CFG_ENC_DT        pin encodeur
 *                         +5v  | [ ]5V                  GND[ ] |
 *                              | [ ]RST                 RST[ ] | 
 *                         GND  | [ ]GND   5V MOSI GND   TX1[ ] | 
 *                              | [ ]Vin   [ ] [ ] [ ]   RX1[ ] | 
 *                              |          [ ] [ ] [ ]          |
 *                              |          MISO SCK RST         |
 *                              | NANO-V3                       |
 *                              +-------------------------------+
 ****************************************************************************************************/

#include "PETCTL_cfg.h"
#define SPEED_MAX 10

#define _developer_ "PETCTL ST7920"
#define _version_   "mvb v0.11c"


#define DRIVER_STEP_TIME 6  // changer le délai à 6 µs
#include "GyverStepper.h"
GStepper<STEPPER2WIRE> stepper(200 * CFG_STEP_DIV, CFG_STEP_STEP_PIN, CFG_STEP_DIR_PIN, CFG_STEP_EN_PIN);

#include "GyverTimers.h"
#include "GyverEncoder.h"
#include "GyverPID.h"
#include <U8glib.h>
// -------------------------------------------------------------------------------------
//  Création de l'objet U8g2 pour ST7920  "Adapté à votre câblage : (CLK, DATA, CS)"
//  Maintenant pour l’écran ST7920 128x64 en 3 fils software SPI:
// -------------------------------------------------------------------------------------
U8GLIB_ST7920_128X64 u8g(/* clock=*/ 13, /* data=*/ 11, /* cs=*/ 10, U8G_PIN_NONE);  
//U8GLIB_ST7920_128X64_1X u8g(/* clock=*/ 13, /* data=*/ 11, /* cs=*/ 10);  

#define CLK CFG_ENC_CLK
#define DT CFG_ENC_DT
#define SW CFG_ENC_SW

// ENCODER
Encoder enc1(CLK, DT, SW);
int value = 0;

// Termistor definition
float prevTemp, curTemp = 0;
float targetTemp = CFG_TEMP_INIT;

// Déplacement / extrém
float finalLength = 0;

// PID
GyverPID regulator(CFG_PID_P, CFG_PID_I, CFG_PID_D, 200);

boolean Heat = false;

#define GEAR_RATIO ((float)CFG_RED_G1 * (float)CFG_RED_G2 * (float)CFG_RED_G3)

/* Longueur de la bobine ronde 74*Pi = 232,478 232,478 mm - longueur de la bobine ronde */
#define BOBIN_ROUND_LENGTH ((float)3.1415926 * (float)CFG_BOBIN_DIAM)
const float REDCONST = BOBIN_ROUND_LENGTH /(360 * GEAR_RATIO * 1000);
boolean runMotor=false;
long SpeedX10 = (float)CFG_SPEED_INIT * 10;

/* Statuts interactifs */
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

bool encoderState = false;    // Variable pour suivre l'état de l'encodeur
int encoderStep = 0;          // Étapes d'affichage des "*

// ====================================================
// SETUP
// ====================================================
void setup() {
  #if defined(SERIAL_DEBUG_TEMP) || defined(SERIAL_DEBUG_STEPPER) || defined(SERIAL_DEBUG_TEMP_PID)
    Serial.begin(9600);
  #endif //SERIAL_DEBUG_TEMP || SERIAL_DEBUG_STEPPER || SERIAL_DEBUG_TEMP_PID

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
    stepper.setAcceleration(300); // 300
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
        //CFG_PULL_EXTRA_LENGTH = 0.07 Longueur supplémentaire à tirer après le déclenchement de la butée [m]
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
  //pidActive = true;             // ou true
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
/*    digitalWrite(CFG_SOUND_PIN, HIGH);
    delay(170);
    digitalWrite(CFG_SOUND_PIN, LOW);
    delay(10);
    digitalWrite(CFG_SOUND_PIN, HIGH);
    delay(80);
    digitalWrite(CFG_SOUND_PIN, LOW);
    delay(100);
    digitalWrite(CFG_SOUND_PIN, HIGH);
    delay(80);
    digitalWrite(CFG_SOUND_PIN, LOW);  
    delay(50);
*/
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
  encoderStep = 0; // Réinitialiser l'état de l'encodeur à chaque arrêt d'urgence
  runMotor = false;
  motorCTL(-1);
  stepper.disable();
  Heat = false;
  analogWrite(CFG_HEATER_PIN, 0);

  // Mettre à jour les indicateurs de statut
  thermistorConnected = false;
  pidActive = false;

  // === Initialisation du timer ===
  unsigned long timerStart = millis();
  unsigned long lastBeepTime = millis();
  unsigned long lastUpdateTime = millis();
  bool beepState = false; // État du bip non bloquant

  Serial.println("Arrêt d'urgence déclenché. Appuyez sur l'encodeur pour redémarrer.");

  // Boucle d'arrêt d'urgence
  while (true) {
    unsigned long currentTime = millis();

    // Mettre à jour l'affichage de l'erreur toutes les 500ms
    if (currentTime - lastUpdateTime >= 500) {
      lastUpdateTime = currentTime;
      drawError(reason, timerStart);
    }

    // Bip non bloquant toutes les 2 secondes
    if (currentTime - lastBeepTime >= 2000) {
      lastBeepTime = currentTime;
      beepState = !beepState;
      digitalWrite(CFG_SOUND_PIN, beepState);
    }
    
    // Vérifier si l'utilisateur appuie sur l'encodeur
    if (digitalRead(CFG_ENC_SW) == LOW) {
      Serial.println("Bouton détecté, attente de 2 secondes...");
      encoderStep = 1; // Mettre à jour immédiatement
      drawError(reason, timerStart);
      unsigned long pressStart = millis();
      while (digitalRead(CFG_ENC_SW) == LOW) {
        if (millis() - pressStart >= 2000) {
          Serial.println("Bouton maintenu pendant 2 secondes, validation...");
          encoderStep = 2; // Mise à jour de l'état de maintien
          drawError(reason, timerStart);
          delay(100); // Petit délai pour éviter une fausse détection
          
          // Attendre que l'utilisateur relâche le bouton
          while (digitalRead(CFG_ENC_SW) == LOW);
          
          Serial.println("Bouton relâché, sortie de l'arrêt d'urgence.");
          encoderStep = 3; // Détection du relâchement
          drawError(reason, timerStart);
          runMotor = false;
          Heat = false;
          thermistorConnected = true;
          pidActive = true;
          digitalWrite(CFG_SOUND_PIN, LOW); // Désactiver le bip
          
          // Redessiner l'écran principal
          Serial.println("Redessiner l'écran principal...");
          u8g.firstPage();
          do {
            drawScreen();
          } while (u8g.nextPage());
          
          delay(500); // Ajout d'un délai pour éviter un retour trop rapide
          
          return; // Quitter proprement la fonction
        }
      }
    }
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
  steinhart = average / CFG_TERM_VALUE;               // (R/Ro)
  steinhart = log(steinhart);                         // ln(R/Ro)
  steinhart /= CFG_TERM_B_COEFF;                      // 1/B * ln(R/Ro)
  steinhart += 1.0 / (CFG_TERM_VALUE_TEMP + 273.15);  // + (1/To)
  steinhart = 1.0 / steinhart;                        // Invert
  steinhart -= 273.15;                                // convert absolute temp to C

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
void drawError(int reason, unsigned long timerStart) {
  u8g.firstPage();
  do {
    u8g.setColorIndex(1); // Blanc  
    u8g.drawFrame(0, 0, 128, 64); // Dessiner une bordure
    u8g.drawLine(1, 15, 126, 15); // Ligne horizontale en haut
    u8g.drawLine(1, 55, 126, 55); // Ligne horizontale en bas
    
    u8g.setFont(u8g_font_9x15Br);
    u8g.setPrintPos(8, 13);
    u8g.print("*!WARNINGS!*");

    u8g.setFont(u8g_font_6x12r);
    u8g.setPrintPos(5, 25);
    switch (reason) {
      case OVERHEAT:
        u8g.print("ERREUR: Overheat");
        break;
      case THERMISTOR_ERROR:
        u8g.print("ERREUR: Thermistor");
        break;
    }

    // Affichage du timer
    unsigned long elapsed = (millis() - timerStart) / 1000;
    char timerBuffer[10];
    u8g.setFont(u8g_font_4x6r); 
    sprintf(timerBuffer, "%02lu:%02lu", elapsed / 60, elapsed % 60);
    u8g.setPrintPos(98, 62);
    u8g.print(timerBuffer);

    // Affichage des Informations
    u8g.setPrintPos(2, 39); u8g.print("   Emergency stop triggered.");
    u8g.setPrintPos(2, 45); u8g.print("   Press encoder to restart.");
    u8g.setPrintPos(2, 51); u8g.print("   Button held for 4 seconds.");

    // Affichage des actions de l'encodeur
    if (encoderStep >= 1) {
      u8g.setPrintPos(2, 62); u8g.print("*");
    } else {
      u8g.setPrintPos(2, 62); u8g.print("O");
    }
    if (encoderStep >= 2) {
      u8g.setPrintPos(8, 62); u8g.print("*");
    } else {
      u8g.setPrintPos(8, 62); u8g.print("O");
    }
    if (encoderStep >= 3) {
      u8g.setPrintPos(14, 62); u8g.print("*");
    } else{
      u8g.setPrintPos(14, 62); u8g.print("O");
    }
  } while (u8g.nextPage());
}

/*****************************************************
   drawBootScreen()
   => dessine TOUT l'écran en fonction des variables
*****************************************************/
void drawBootScreen() {
  // On peut faire un petit écran de démarrage en utilisant U8glib:
  u8g.firstPage();
  do {
    u8g.setFont(u8g_font_9x15Br); // On choisit une police grande :
    // We want the reference position for printing the string to be in the upper left corner of that string
    u8g.setFontPosTop(); 
    // X i Y coordinates where the string wants to print out on the display
    u8g.setPrintPos(5, 10); u8g.print(_developer_);
    u8g.setFont(u8g_font_6x12r);
    u8g.setPrintPos(25, 40); u8g.print(_version_);
  } while(u8g.nextPage());
  delay(5000); // 5 secondes de splash
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
  u8g.drawLine(1, 13, 126, 13); // Ligne horizontale en haut
  u8g.drawLine(1, 34, 126, 34); // Ligne horizontale en milieu
  u8g.drawLine(1, 52, 126, 52); // Ligne horizontale en bas

  // 3) Dessiner une ligne verticale pour séparer les sections (par exemple à x=74)
  u8g.drawLine(74, 1, 74, 12);  // Ligne verticale en haut
  u8g.drawLine(64, 14, 64, 51); // Ligne verticale en milieu

  drawCurrentTemperature();         // 3.0) Afficher les températures
  drawTargetTemperature();          // 3.1) Température cible
  drawShowCurrentStepperPosition(); // 3.2) Afficher la position actuelle du stepper
  drawShowRunMotorIndicator();      // 4.0) Afficher l'indicateur "runMotor"
  drawShowProgressFootage();        // 5.0) Afficher l'avancement (métrage)
  drawEndstopStatus();              // 6.0) Afficher les Endstops avec boîtes
  drawEMEndstopStatus();            // 6.1) Afficher les Endstops avec boîtes
  drawThermistorStatus();           // 7.0) Afficher les indicateurs de statut avec boîtes et inversion des couleurs
  drawPIDStatus();                  // 7.1) Afficher l'état du PID
  drawEncoderStatus();              // 7.2) Afficher l'état de l'Encodeur
}

/***************************************************** 
  // Température courante en gros (ou moyen)
  // 3.0) Afficher les températures
*****************************************************/
void drawCurrentTemperature() { 
  u8g.setFont(u8g_font_9x15Br);
  char buf[10];
  dtostrf(curTemp, 4, 1, buf);
  u8g.setPrintPos(8, 12);
  u8g.print(buf);
  u8g.setPrintPos(2, 8);
  u8g.setFont(u8g_font_5x8r); u8g.print("C"); // current
}

/***************************************************** 
  // 3.1) Température cible
*****************************************************/
void drawTargetTemperature() {
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
  } 
  else {
    u8g.print(buf);
    u8g.setPrintPos(116, 8);
    u8g.setFont(u8g_font_5x8r); u8g.print("C");
  }
}

/***************************************************** 
  // drawbargraph()
*****************************************************/
void drawbargraph(int n) {
  byte y = map(n, 0, 1855, 1, 14);
  //u8g.setFont(u8g_font_5x8r);

  // Pointer
  u8g.setPrintPos(10, 44); 
  for (int i = 1; i <= y; i++) {
    int pointerX = 10 + (i - 1) * 3;
    u8g.drawBox(pointerX, 44, 2, 6);
  }
}

/***************************************************** 
  // Vitesse actuelle du moteur pas à pas Vitesse en /s.
  // 3.2) Afficher la position actuelle du stepper
*****************************************************/
void drawShowCurrentStepperPosition() {
  u8g.setFont(u8g_font_4x6r);
  u8g.setPrintPos(4, 41);
  u8g.print("step/s:");

  u8g.setFont(u8g_font_5x8r);
  long currentPosition = stepper.getSpeed();
  char buf[10];
  dtostrf(currentPosition, 4, 0, buf);
  u8g.setPrintPos(40, 42);
  u8g.print(buf);
  u8g.setFont(u8g_font_4x6r);
  //u8g.print(":step/s");

  // Affichage de la barre de progression
  drawbargraph(currentPosition);
}

/***************************************************** 
  // Vitesse en mm/s
  // 4.0) Afficher l'indicateur "runMotor"
*****************************************************/
void drawShowRunMotorIndicator() {
  float spd = (float)SpeedX10 / 10.0;
  char buf[10];
  dtostrf(spd, 4, 1, buf);

  // Déterminer si on est en mode interactif pour la vitesse
  bool interactiveSpeed = (whatToChange == CHANGE_SPEED && isInteractive());

  // Si mode interactif, dessiner le fond noir et inverser la couleur
  if (interactiveSpeed)
  {
    u8g.drawBox(1, 14, 63, 20); // (x, y, largeur, hauteur)
    u8g.setColorIndex(0);       // Texte en "couleur inversée"
  }

  // Affichage commun
  // 1. Afficher l'indicateur de l'état du moteur
  u8g.setFont(u8g_font_9x15Br);
  u8g.setPrintPos(3, 30);
  u8g.print(runMotor ? "*" : "."); // Affiche "*" si runMotor est vrai, sinon "."

  // 2. Afficher le libellé "Speed:"
  u8g.setFont(u8g_font_4x6r);
  u8g.setPrintPos(4, 20);
  u8g.print("Speed:");

  // 3. Afficher la valeur de la vitesse
  u8g.setFont(u8g_font_9x15Br);
  u8g.setPrintPos(4, 32);
  u8g.print(buf);

  // 4. Afficher l'unité " mm/s"
  u8g.setFont(u8g_font_4x6r);
  u8g.print(" mm/s");

  // Si la couleur a été modifiée, la rétablir à la normale
  if (interactiveSpeed)
  {
    u8g.setColorIndex(1);
  }
}

/*****************************************************  
  // Avancement (métrage)
  // 5.0) Afficher l'avancement (métrage)
*****************************************************/
void drawShowProgressFootage() {
  // Vérifier si l'endstop principal est activé (LOW)
  if (!digitalRead(CFG_ENDSTOP_PIN))
  {
    // Calculer le reste (en mètres) et le convertir en mm
    float rest = finalLength - getMilage();
    float rest_mm = rest * 1000.0;

    // Afficher le décompte en mm
    u8g.setFont(u8g_font_4x6r);

    // Dessiner un fond noir
    u8g.drawBox(65, 14, 63, 20); // (x,y,width,height)
    u8g.setColorIndex(0);        // Écrire en "couleur inversée"

    u8g.setPrintPos(68, 20);
    u8g.print("Extra:"); // Par exemple "Extra:" pour indiquer la longueur supplémentaire

    u8g.setFont(u8g_font_9x15Br);
    char buf[10];
    dtostrf(rest_mm, 4, 0, buf); // Afficher sans décimales
    u8g.setPrintPos(68, 32);
    u8g.print(buf);
    u8g.setFont(u8g_font_4x6r);
    u8g.print(" mm");
  }
  else {
    u8g.setFont(u8g_font_4x6r);
    u8g.setPrintPos(68, 20);
    u8g.print("Progress:");

    u8g.setFont(u8g_font_9x15Br);
    float dist = getMilage(); // en mètres ?
    // Dans votre code, getMilage() renvoie "deg * REDCONST"
    // A vous de voir l'unité exacte (mm, m, etc.)
    char buf[10];
    dtostrf(dist, 4, 2, buf);
    u8g.setPrintPos(76, 32);
    u8g.print(buf);
    u8g.setFont(u8g_font_4x6r);
    u8g.print(" m");
  }
}

/*****************************************************
// Fin de courses (debug ?) => par exemple
// endstop = CFG_ENDSTOP_PIN
// 6.0) Afficher les Endstops
*****************************************************/
void drawEndstopStatus() {
  int currentColor = 1;  // On suppose que 1 est la couleur par défaut (blanc)

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
}

/*****************************************************
// Fin de courses (debug ?) => par exemple
// emendstop = CFG_EMENDSTOP_PIN
// 6.1) Afficher les Endstops
*****************************************************/
void drawEMEndstopStatus() { 
  int currentColor = 1;  // On suppose que 1 est la couleur par défaut (blanc)
  
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
}

/*****************************************************
// 7.0) Afficher la connexion au Thermistor
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
// 7.1) Afficher l'état du PID
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
// 7.2) Afficher l'état de l'Encodeur
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
