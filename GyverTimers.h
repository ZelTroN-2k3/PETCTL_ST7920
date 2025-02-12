/*
 Configuration et surveillance des interruptions sur les temporisateurs matériels ATmega328p, ATmega2560
 Documentation : https://alexgyver.ru/gyvertimers/
 GitHub : https://github.com/GyverLibs/GyverTimers
 Possibilités :
 - Les trois temporisateurs sur ATmega328 et les six temporisateurs sur ATmega2560 sont pris en charge ;
 - Réglage de la période (µs) et de la fréquence (Hz) des interruptions :
 - Temporisateurs 8 bits : 61 Hz - 1 MHz (16 384 µs.. 1 µs) ;
 - Temporisateurs 16 bits : 0,24 Hz - 1 MHz (4 200 000 µs. 1 µs) ;
 - Réglage automatique du réglage de la période à partir de la fréquence d'horloge (F_CPU) ;
 - La fonction renvoie la période/fréquence exacte à l'état stable pour le débogage (la fréquence est limitée par la résolution du minuteur) ;
 - Prise en charge du fonctionnement multicanal : un temporisateur appelle 2 (ATmega328) ou
 3 (ATmega2560, timers 1, 3, 4, 5) interruptions avec déphasage réglable de 0 à 360 degrés ;
 - Action d'interruption de broche de minuterie matérielle configurable : haute, basse, bascule.
 Permet de générer un méandre (un et deux temps) ;
 - Contrôle du fonctionnement de la minuterie : démarrage/arrêt/pause/continuation/initialisation ;

 Egor 'Nich1con' Zaharov pour AlexGyver, alex@alexgyver.ru
 https://alexgyver.ru/
 Licence MIT

 Versions:
 v1.1 - erreur corrigée dans le calcul des périodes
 v1.2 - code divisé en h et cpp
 v1.3 - bug mineur corrigé
 v1.4 - tableau corrigé des fréquences et des périodes
 v1.5 - redémarrage et reprise corrigés
 v1.6 - le déphasage est déplacé vers une méthode distincte
 v1.7 - documentation corrigée
 v1.8 - bug corrigé avec la période maximale
 v1.9 - bug corrigé avec le retour d'une période 2x
*/

/***************************************************************************************
 *                             Arduino NANO 16 MHz (ATmega328)
 * -------------------------------------------------------------------------------------
 *  Timer   | Résolution | Fréquences            | Périodes               | Sorties    | Broche Arduino | Broche µC
 * -------------------------------------------------------------------------------------
 *  Timer0  | 8 bits     | 61 Hz - 1 MHz         | 16 384..1 µs           | CHANNEL_A  | D6             | PD6
 *          |            |                       |                        | CHANNEL_B  | D5             | PD5
 * -------------------------------------------------------------------------------------
 *  Timer1  | 16 bits    | 0,24 Hz - 1 MHz       | 4 200 000..1 µs        | CHANNEL_A  | D9             | PB1
 *          |            |                       |                        | CHANNEL_B  | D10            | PB2
 * -------------------------------------------------------------------------------------
 *  Timer2  | 8 bits     | 61 Hz - 1 MHz         | 16 384..1 µs           | CHANNEL_A  | D11            | PB3
 *          |            |                       |                        | CHANNEL_B  | D3             | PD3
 * -------------------------------------------------------------------------------------
 *
 *                            Arduino MEGA 16 MHz (ATmega2560)
 * -------------------------------------------------------------------------------------
 *  Timer   | Résolution | Fréquences            | Périodes               | Sorties    | Broche Arduino | Broche µC
 * -------------------------------------------------------------------------------------
 *  Timer0  | 8 bits     | 61 Hz - 1 MHz         | 16 384..1 µs           | CHANNEL_A  | 13             | PB7
 *          |            |                       |                        | CHANNEL_B  | 4              | PG5
 * -------------------------------------------------------------------------------------
 *  Timer1  | 16 bits    | 0,24 Hz - 1 MHz       | 4 200 000..1 µs        | CHANNEL_A  | 11             | PB5
 *          |            |                       |                        | CHANNEL_B  | 12             | PB6
 *          |            |                       |                        | CHANNEL_C  | 13             | PB7
 * -------------------------------------------------------------------------------------
 *  Timer2  | 8 bits     | 61 Hz - 1 MHz         | 16 384..1 µs           | CHANNEL_A  | 10             | PB4
 *          |            |                       |                        | CHANNEL_B  | 9              | PH6
 * -------------------------------------------------------------------------------------
 *  Timer3  | 16 bits    | 0,24 Hz - 1 MHz       | 4 200 000..1 µs        | CHANNEL_A  | 5              | PE3
 *          |            |                       |                        | CHANNEL_B  | 2              | PE4
 *          |            |                       |                        | CHANNEL_C  | 3              | PE5
 * -------------------------------------------------------------------------------------
 *  Timer4  | 16 bits    | 0,24 Hz - 1 MHz       | 4 200 000..1 µs        | CHANNEL_A  | 6              | PH3
 *          |            |                       |                        | CHANNEL_B  | 7              | PH4
 *          |            |                       |                        | CHANNEL_C  | 8              | PH5
 * -------------------------------------------------------------------------------------
 *  Timer5  | 16 bits    | 0,24 Hz - 1 MHz       | 4 200 000..1 µs        | CHANNEL_A  | 46             | PL3
 *          |            |                       |                        | CHANNEL_B  | 45             | PL4
 *          |            |                       |                        | CHANNEL_C  | 44             | PL5
 * -------------------------------------------------------------------------------------
 *
 * Remarques :
 *  - Les valeurs de fréquences (ex. 61 Hz - 1 MHz) et de périodes (ex. 16 384..1 µs)
 *    dépendent du réglage des préscalers et des modes des timers.
 *  - Les canaux (CHANNEL_A, CHANNEL_B, CHANNEL_C) correspondent aux sorties PWM/compare 
 *    disponibles sur chaque timer.
 *  - Les broches indiquées (Broche Arduino / Broche µC) se réfèrent à l’ATmega concerné.
 ***************************************************************************************/


/*******************************************************************************************************
 *  Fonctionnalités (traduction du russe) :
 *  
 *   setPeriod(période) 
 *      - Définit la période en microsecondes et démarre le timer.
 *      - Retourne la période réellement appliquée (la précision est limitée par la résolution du timer).
 *   setFrequency(fréquence) 
 *      - Définit la fréquence en Hertz et démarre le timer.
 *      - Retourne la fréquence réellement appliquée (la précision est limitée par la résolution du timer).
 *   setFrequencyFloat(fréquence float)
 *      - Définit la fréquence en Hertz avec un nombre à virgule (décimal) et démarre le timer.
 *      - Retourne la fréquence réellement appliquée (la précision est limitée par la résolution du timer).
 *   enableISR(source)
 *      - Active les interruptions du timer sur le canal CHANNEL_A ou CHANNEL_B
 *        (+ CHANNEL_C sur Mega2560).
 *   disableISR(source)
 *      - Désactive les interruptions sur le canal CHANNEL_A ou CHANNEL_B.
 *        Le timer continue de compter (sans paramètre, désactive le canal A).
 *   pause()
 *      - Met le timer en pause sans réinitialiser son compteur.
 *   resume()
 *      - Reprend le comptage après une pause.
 *   stop()
 *      - Arrête le comptage et réinitialise le compteur.
 *   restart()
 *      - Redémarre le timer (remise à zéro du compteur).
 *   setDefault()
 *      - Restaure les paramètres par défaut d’Arduino pour le timer.
 *   outputEnable(canal, mode)
 *      - Active la sortie du timer sur CHANNEL_A ou CHANNEL_B (+ CHANNEL_C sur Mega2560).
 *      - Le mode peut être : TOGGLE_PIN (inverser l’état), CLEAR_PIN (forcer à LOW), SET_PIN (forcer à HIGH).
 *   outputDisable(canal)
 *      - Désactive la sortie du timer sur CHANNEL_A ou CHANNEL_B (+ CHANNEL_C sur Mega2560, voir tableau des timers).
 *   outputState(canal, état)
 *      - Change l’état de la sortie du canal : HIGH ou LOW.
 *   phaseShift(source, phase)
 *      - Décale la phase du canal de 0 à 360 degrés (pour les timers 8 bits, seul le canal B est décalable).
 *   
 *******************************************************************************************************/


#define MAX_PERIOD_8 (1000000UL * 1024UL / F_CPU * 256UL)		// 16384 (61 Гц) на 16 МГц
#define MAX_PERIOD_16 (1000000UL * 1024UL / F_CPU * 65536UL)	// 4194304 (0.24 Гц) на 16 МГц

#ifndef GyverTimers_h
#define GyverTimers_h
#include <Arduino.h>

/* ==========  Константы ========== */
#define CHANNEL_A 0x00
#define CHANNEL_B 0x01
#define CHANNEL_C 0x02

#define TOGGLE_PIN 0x01
#define CLEAR_PIN 0x02
#define SET_PIN 0x03

#define TIMER0_A  TIMER0_COMPA_vect
#define TIMER0_B  TIMER0_COMPB_vect
#define TIMER1_A  TIMER1_COMPA_vect
#define TIMER1_B  TIMER1_COMPB_vect
#define TIMER2_A  TIMER2_COMPA_vect
#define TIMER2_B  TIMER2_COMPB_vect

#if defined(__AVR_ATmega2560__)
#define TIMER1_C  TIMER1_COMPC_vect
#define TIMER3_A  TIMER3_COMPA_vect
#define TIMER3_B  TIMER3_COMPB_vect
#define TIMER3_C  TIMER3_COMPC_vect
#define TIMER4_A  TIMER4_COMPA_vect
#define TIMER4_B  TIMER4_COMPB_vect
#define TIMER4_C  TIMER4_COMPC_vect
#define TIMER5_A  TIMER5_COMPA_vect
#define TIMER5_B  TIMER5_COMPB_vect
#define TIMER5_C  TIMER5_COMPC_vect
#endif

#define GYVERTIMERS_INLINE
/*inline __attribute__((always_inline))*/

/* ================ Сlasses of timers ================ */
class Timer_0 {                       					  // Timer 0
public:
    uint32_t setPeriod(uint32_t _timer0_period);          // Set timer period [us]
    uint32_t setFrequency(uint32_t _timer0_frequency);    // Set timer frequency [Hz]
    float setFrequencyFloat(float _timer0_frequency);  	  // Set timer float frequency [Hz]
    
    GYVERTIMERS_INLINE
    void enableISR(uint8_t source = CHANNEL_A);           // Enable timer interrupt , channel A or B 
    
    GYVERTIMERS_INLINE
    void disableISR(uint8_t source = CHANNEL_A);          // Disable timer interrupt , channel A or B
    
    GYVERTIMERS_INLINE
    void pause(void);                   				  // Disable timer clock , not cleaning the counter
    
    GYVERTIMERS_INLINE
    void resume(void);                 				      // Return clock timer settings
    
    GYVERTIMERS_INLINE
    void stop(void);                   					  // Disable timer clock , and cleaning the counter
    
    GYVERTIMERS_INLINE
    void restart(void);                  				  // Return clock timer settings & reset counter
    
    GYVERTIMERS_INLINE
    void setDefault(void);               			      // Set default timer settings
    
    GYVERTIMERS_INLINE
    void outputEnable(uint8_t channel, uint8_t mode);	  // Enable and configurate timer hardware output
    
    GYVERTIMERS_INLINE
    void outputDisable(uint8_t channel);				  // Disable timer hardware output
    
    GYVERTIMERS_INLINE
    void outputState(uint8_t channel,bool state);		  // Set High / Low on the timer output 
    
    GYVERTIMERS_INLINE
    void phaseShift(uint8_t source, uint16_t phase);
    
private:
    uint8_t _timer0_clock = 0x00;           			  // Variable to store timer clock settings
};

class Timer_1 {                      					  // Timer 1
public:
    uint32_t setPeriod(uint32_t _timer1_period);      	  // Set timer period [Hz]
    uint32_t setFrequency(uint32_t _timer1_frequency);    // Set timer frequency [Hz]
    float setFrequencyFloat(float _timer1_frequency);     // Set timer float frequency [Hz]
    
    GYVERTIMERS_INLINE
    void enableISR(uint8_t source = CHANNEL_A);       	  // Enable timer interrupt , channel A or B 
    
    GYVERTIMERS_INLINE
    void disableISR(uint8_t source = CHANNEL_A);          // Disable timer interrupt , channel A or B
    
    GYVERTIMERS_INLINE
    void pause(void);                   				  // Disable timer clock , not cleaning the counter
    
    GYVERTIMERS_INLINE
    void resume(void);                    				  // Return clock timer settings
    
    GYVERTIMERS_INLINE
    void stop(void);                    				  // Disable timer clock , and cleaning the counter	
    
    GYVERTIMERS_INLINE
    void restart(void);                   				  // Return clock timer settings & reset counter
    
    GYVERTIMERS_INLINE
    void setDefault(void);                  			  // Set default timer settings
    
    GYVERTIMERS_INLINE
    void outputEnable(uint8_t channel, uint8_t mode);	  // Enable and configurate timer hardware output
    
    GYVERTIMERS_INLINE
    void outputDisable(uint8_t channel);				  // Disable timer hardware output
    
    GYVERTIMERS_INLINE
    void outputState(uint8_t channel,bool state);		  // Set High / Low on the timer output  
    
    GYVERTIMERS_INLINE
    void phaseShift(uint8_t source, uint16_t phase);
    
private:
    uint8_t _timer1_clock = 0x00;             			  // Variable to store timer clock settings
};

class Timer_2 {                       					  // Timer 2
public:
    uint32_t setPeriod(uint32_t _timer2_period);      	  // Set timer period [Hz]
    uint32_t setFrequency(uint32_t _timer2_frequency);    // Set timer frequency [Hz]
    float setFrequencyFloat(float _timer2_frequency);     // Set timer float frequency [Hz]
    
    GYVERTIMERS_INLINE
    void enableISR(uint8_t source = CHANNEL_A);      	  // Enable timer interrupt , channel A or B 
    
    GYVERTIMERS_INLINE
    void disableISR(uint8_t source = CHANNEL_A);          // Disable timer interrupt , channel A or B
    
    GYVERTIMERS_INLINE
    void pause(void);                   				  // Disable timer clock , not cleaning the counter
    
    GYVERTIMERS_INLINE
    void resume(void);                 				      // Return clock timer settings
    
    GYVERTIMERS_INLINE
    void stop(void);                    				  // Disable timer clock , and cleaning the counter
    
    GYVERTIMERS_INLINE
    void restart(void);                  				  // Return clock timer settings & reset counter
    
    GYVERTIMERS_INLINE
    void setDefault(void);                  			  // Set default timer settings
    
    GYVERTIMERS_INLINE
    void outputEnable(uint8_t channel, uint8_t mode);	  // Enable and configurate timer hardware output
    
    GYVERTIMERS_INLINE
    void outputDisable(uint8_t channel);				  // Disable timer hardware output
    
    GYVERTIMERS_INLINE
    void outputState(uint8_t channel,bool state);		  // Set High / Low on the timer output  
    
    GYVERTIMERS_INLINE
    void phaseShift(uint8_t source, uint16_t phase);
    
private:
    uint8_t _timer2_clock = 0x00;             			  // Variable to store timer clock settings	
};

#if defined(__AVR_ATmega2560__)
class Timer_3 {                       					  // Timer 3
public:
    uint32_t setPeriod(uint32_t _timer3_period);     	  // Set timer period [Hz]
    uint32_t setFrequency(uint32_t _timer3_frequency);    // Set timer frequency [Hz]
    float setFrequencyFloat(float _timer3_frequency);     // Set timer float frequency [Hz]
    
    GYVERTIMERS_INLINE
    void enableISR(uint8_t source = CHANNEL_A);       	  // Enable timer interrupt , channel A or B
    
    GYVERTIMERS_INLINE
    void disableISR(uint8_t source = CHANNEL_A);          // Disable timer interrupt , channel A or B
    
    GYVERTIMERS_INLINE
    void pause(void);                  					  // Disable timer clock , not cleaning the counter
    
    GYVERTIMERS_INLINE
    void resume(void);                   				  // Return clock timer settings
    
    GYVERTIMERS_INLINE
    void stop(void);                    				  // Disable timer clock , and cleaning the counter
    
    GYVERTIMERS_INLINE
    void restart(void);                   				  // Return clock timer settings & reset counter
    
    GYVERTIMERS_INLINE
    void setDefault(void);                				  // Set default timer settings
    
    GYVERTIMERS_INLINE
    void outputEnable(uint8_t channel, uint8_t mode);	  // Enable and configurate timer hardware output
    
    GYVERTIMERS_INLINE
    void outputDisable(uint8_t channel);				  // Disable timer hardware output
    
    GYVERTIMERS_INLINE
    void outputState(uint8_t channel,bool state);		  // Set High / Low on the timer output 
    
    GYVERTIMERS_INLINE
    void phaseShift(uint8_t source, uint16_t phase);
    
private:
    uint8_t _timer3_clock = 0x00;             			  // Variable to store timer clock settings
};

class Timer_4 {                      					  // Timer 4
public:
    uint32_t setPeriod(uint32_t _timer4_period);      	  // Set timer period [Hz]
    uint32_t setFrequency(uint32_t _timer4_frequency);    // Set timer frequency [Hz]
    float setFrequencyFloat(float _timer4_frequency);     // Set timer float frequency [Hz]
    
    GYVERTIMERS_INLINE
    void enableISR(uint8_t source = CHANNEL_A);       	  // Enable timer interrupt , channel A or B
    
    GYVERTIMERS_INLINE
    void disableISR(uint8_t source = CHANNEL_A);          // Disable timer interrupt , channel A or B
    
    GYVERTIMERS_INLINE
    void pause(void);                  					  // Disable timer clock , not cleaning the counter
    
    GYVERTIMERS_INLINE
    void resume(void);                    				  // Return clock timer settings
    
    GYVERTIMERS_INLINE
    void stop(void);                   				      // Disable timer clock , and cleaning the counter
    
    GYVERTIMERS_INLINE
    void restart(void);                        			  // Return clock timer settings & reset counter
    
    GYVERTIMERS_INLINE
    void setDefault(void);                  			  // Set default timer settings
    
    GYVERTIMERS_INLINE
    void outputEnable(uint8_t channel, uint8_t mode);	  // Enable and configurate timer hardware output
    
    GYVERTIMERS_INLINE
    void outputDisable(uint8_t channel);				  // Disable timer hardware output
    
    GYVERTIMERS_INLINE
    void outputState(uint8_t channel,bool state);		  // Set High / Low on the timer output 
    
    GYVERTIMERS_INLINE
    void phaseShift(uint8_t source, uint16_t phase);
    
private:
    uint8_t _timer4_clock = 0x00;            			  // Variable to store timer clock settings
};

class Timer_5 {                     					  // Timer 5
public:
    uint32_t setPeriod(uint32_t _timer5_period);          // Set timer period [Hz]
    uint32_t setFrequency(uint32_t _timer5_frequency);    // Set timer frequency [Hz]
    float setFrequencyFloat(float _timer5_frequency);     // Set timer float frequency [Hz]
    
    GYVERTIMERS_INLINE
    void enableISR(uint8_t source = CHANNEL_A);       	  // Enable timer interrupt , channel A or B
    
    GYVERTIMERS_INLINE
    void disableISR(uint8_t source = CHANNEL_A);          // Disable timer interrupt , channel A or B
    
    GYVERTIMERS_INLINE
    void pause(void);                  					  // Disable timer clock , not cleaning the counter
    
    GYVERTIMERS_INLINE
    void resume(void);                    				  // Return clock timer settings
    
    GYVERTIMERS_INLINE
    void stop(void);                    				  // Disable timer clock , and cleaning the counter
    
    GYVERTIMERS_INLINE
    void restart(void);                  				  // Return clock timer settings & reset counter
    
    GYVERTIMERS_INLINE
    void setDefault(void);                 				  // Set default timer settings
    
    GYVERTIMERS_INLINE
    void outputEnable(uint8_t channel, uint8_t mode);	  // Enable and configurate timer hardware output
    
    GYVERTIMERS_INLINE
    void outputDisable(uint8_t channel);				  // Disable timer hardware output
    
    GYVERTIMERS_INLINE
    void outputState(uint8_t channel,bool state);		  // Set High / Low on the timer output 
    
    GYVERTIMERS_INLINE
    void phaseShift(uint8_t source, uint16_t phase);
    
private:
    uint8_t _timer5_clock = 0x00;                		  // Variable to store timer clock settings
};

#endif

extern Timer_0 Timer0;
extern Timer_1 Timer1;
extern Timer_2 Timer2;

#if defined(__AVR_ATmega2560__)
extern Timer_3 Timer3;
extern Timer_4 Timer4;
extern Timer_5 Timer5;
#endif

#endif
