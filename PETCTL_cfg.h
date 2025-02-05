
#define CFG_STEP_DIV          8 /* Stepper driver microstep devision */

#define CFG_STEP_STEP_PIN     6 /* Which pin stepper driver STEP pin connected */
#define CFG_STEP_DIR_PIN      5 /* Which pin stepper driver DIR pin connected */
#define CFG_STEP_EN_PIN       7 /* Which pin stepper driver EN pin connected */

#define CFG_STEP_INVERT         /* Invert stepper rotation direction (comment out to disable invertion)*/

#define CFG_ENC_CLK           3 /* Which pin encoder CLK pin connected */
#define CFG_ENC_DT            2 /* Which pin encoder DT pin connected */
#define CFG_ENC_SW            4 /* Which pin encoder SW pin connected */

#define CFG_ENDSTOP_PIN       8 /* À quelle broche ENDSTOP est-elle connectée */
#define CFG_EMENDSTOP_PIN    12 /* À quelle broche la butée EMENDSTOP est-elle connectée */

#define CFG_TERM_PIN         A0 /* Which pin termistor connected to*/

#define CFG_ENC_TYPE         TYPE2 /* Type of encoder: TYPE1 or TYPE2 */

#define CFG_HEATER_PIN        9 /* À quelle broche le MOSFET de chauffage est-il connecté ? */
#define CFG_BOBIN_DIAM       74 /* Diamètre de la bobine de filament cible [mm] */
#define CFG_SPEED_INIT       2.5 /* Vitesse de traction initiale [mm/s] */
#define CFG_SOUND_PIN        A1 /* Connexion de la broche du buzzer */

#define CFG_TEMP_INIT        180 /* Initial target temperature [degree C]*/
#define CFG_TEMP_MAX         290 /* Maximum allowed temperature [degree C], allowed to set to 10 degree less */
#define CFG_TEMP_MIN         105 /* Minimum allowed temperature to set [degree C] */


#define CFG_TERM_VALUE 100000 /* Thermistor resistance at 25 degrees C [Om] */
#define CFG_TERM_VALUE_TEMP 25 /* Thermistor temperature for nominal resistance (almost always 25 C) [degree C] */
#define CFG_TERM_B_COEFF 4388 /* The beta coefficient of the thermistor (usually 3000-4000) */
#define CFG_TERM_SERIAL_R 4700 /* the value of the 'other' resistor [Om] */

#define CFG_PULL_EXTRA_LENGTH 0.07 /* Extra length to pull after end stop triggered [m] */


// ----------------------------------------------------------------------------------- 
// Coefficients du régulateur PID
// PID p: 12.69  PID i: 0.71 PID d: 57.11
// ----------------------------------------------------------------------------------- 
#define CFG_PID_P 12.69
#define CFG_PID_I 0.71
#define CFG_PID_D 57.11

// ----------------------------------------------------------------------------------- 
// Activer le son de démarrage (commenter pour désactiver). 
// Spécial pour GEORGIY (@nehilo011) :)
// ----------------------------------------------------------------------------------- 
#define CFG_SOUND_START

// ----------------------------------------------------------------------------------- 
// Choisir le type de réducteur.
// Un seul CFG_RED_RA, CFG_RED_PP1 ou CFG_RED_PP2 peut être décommenté
// -----------------------------------------------------------------------------------
/* RobertSa/Anatoly reductor variant (1:139.21875 ratio)*/
//#define CFG_RED_RA
/* PETPull Zneipas classic old reductor variant (1:30.9375 ratio)*/
//#define CFG_RED_PP1
/* PETPull-2 Zneipas reductor variant (1:65.68(18) ratio)*/
#define CFG_RED_PP2

/* NE CHANGEZ RIEN APRES CETTE LIGNE SI VOUS N'ÊTES PAS SÛR DE 146% */

// -----------------------------------------------------------------------------------
// ver la sortie de débogage série Activer/Désactive
// -----------------------------------------------------------------------------------
//#define SERIAL_DEBUG_TEMP 
//#define SERIAL_DEBUG_TEMP_PID 
//#define SERIAL_DEBUG_STEPPER 

/* Rapport de démultiplication pour la variante de réducteur PETPull-2 Zneipas */
/* 
  8 dents de l'engrenage sur l'arbre du moteur pas à pas interagissent avec
    34 dents de l'engrenage 1er.
  11 dents de l'engrenage 1er interagissent avec
    34 dents de l'engrenage 2e.
  11 dents de l'engrenage 2e interagissent avec
    55 dents de la bobine cible

  rapport de réduction 65,68(18)
*/
#if defined(CFG_RED_PP2)
#define CFG_RED_G1 34/8
#define CFG_RED_G2 34/11
#define CFG_RED_G3 55/11
#endif //CFG_RED_PP2

/* Rapport de démultiplication pour la variante de réducteur RobertSa/Anatoly */
/* 
  8 dents de l'engrenage sur l'arbre pas à pas interagissent avec
    l'engrenage à 36 dents du 1er engrenage.
  8 dents du 1er engrenage interagissent avec
    l'engrenage à 36 dents du 2e engrenage.
  8 dents du 2e engrenage interagissent avec
    55 dents de la bobine cible

  rapport de réduction 139,21875
*/
#if defined(CFG_RED_RA)
#define CFG_RED_G1 36/8
#define CFG_RED_G2 36/8
#define CFG_RED_G3 55/8
#endif //CFG_RED_RA

/* Rapport de démultiplication pour la variante de réducteur PetPull Zneipas */
/*
  8 dents de l'engrenage sur l'arbre du moteur pas à pas interagissent avec
  36 dents de l'engrenage du 1er engrenage.
  8 dents de l'engrenage du 1er engrenage interagissent avec
  55 dents de la bobine cible
  CFG_RED_G2 1 - à exclure) 2e engrenage

  rapport de réduction 30,9375
*/
#if defined(CFG_RED_PP1)
#define CFG_RED_G1 36/8
#define CFG_RED_G2 1
#define CFG_RED_G3 55/8
#endif //CFG_RED_PP1
