# Il s’agit d’un contrôleur qui fait fonctionner une machine qui convertit le ruban découpé dans des bouteilles en PET en fibre adaptée à l’impression 3D.
![GitHub Issues or Pull Requests](https://img.shields.io/github/issues/ZelTroN-2k3/10kHz-to-225MHz-VFO-RF-Generator-with-Si5351---Version-2.1)
![GitHub License](https://img.shields.io/github/license/ZelTroN-2k3/10kHz-to-225MHz-VFO-RF-Generator-with-Si5351---Version-2.1)
![GitHub Repo stars](https://img.shields.io/github/stars/ZelTroN-2k3/10kHz-to-225MHz-VFO-RF-Generator-with-Si5351---Version-2.1?style=flat)
![GitHub forks](https://img.shields.io/github/forks/ZelTroN-2k3/10kHz-to-225MHz-VFO-RF-Generator-with-Si5351---Version-2.1?style=flat)
![GitHub Issues or Pull Requests](https://github.com/ZelTroN-2k3/PETCTL_ST7920/blob/v0.11c/PETCTL-image2.jpg)

# Source code Original de l'auteur: https://github.com/mvbasov/PETCTL/tree/github
![GitHub Issues or Pull Requests](https://github.com/ZelTroN-2k3/PETCTL_ST7920/blob/v0.11c/image_ecran4.jpg)

# Comment gérer
Les commandes se sont avérées très pratiques (pour moi) :
- réglage de la température en un clic. La température cible est mise en évidence dans le coin droit. Dans ce mode, un appui long allume/éteint le chauffage, l'étoile au début de la 1ère ligne s'allume/s'éteint.
- deux clics pour régler la vitesse. La vitesse au milieu est mise en évidence. Dans ce mode, un appui long allume/éteint le moteur, l'étoile au début de la 2ème ligne s'allume/s'éteint.
- Si vous n'appuyez sur rien pendant 15 secondes, il passe en mode veille comme sur la photo.
- Si le capteur de fin de bande est connecté, alors après son déclenchement, le moteur s'éteint après avoir tiré CFG_PULL_EXTRA_LENGTH [m] le chauffage s'éteint également
- Si le capteur de rupture de fibre est connecté, après son déclenchement, le moteur s'éteint immédiatement et le chauffage s'éteint également

les fichiers suivants :

1. **PETCTL_ST7920.ino** – Le fichier principal du programme Arduino.
2. **PETCTL_cfg.h** – Probablement un fichier de configuration.
3. **GyverEncoder.cpp / GyverEncoder.h** – Bibliothèque pour gérer l'encodeur rotatif.
4. **GyverTimers.cpp / GyverTimers.h** – Gestion des timers.
5. **GyverPID.h** – Probablement pour la régulation PID.
6. **GyverStepper.h** – Gestion des moteurs pas à pas.
7. **FastIO.h** – Optimisation des entrées/sorties.

fichier principal **PETCTL_ST7920.ino** 
fichier de configuration **PETCTL_cfg.h**.

### Analyse du Code

#### 1. **Fichier Principal - `PETCTL_ST7920.ino`**
Le fichier principal commence par une description du projet et des connexions matérielles :
- **Encodeur rotatif KY-040** : broches **2 (DT)**, **3 (CLK)**, **4 (SW)**.
- **Moteur pas à pas NEMA 17 (via A4988)** :
  - **DIR** → broche **5**
  - **STEP** → broche **6**
  - **EN** → broche **7**
- **Capteur photoélectrique LM393 (ENDSTOP)** : broche **8**.
- **Bouton d'arrêt d'urgence (EMENDSTOP)** : broche **12**.
- **MOSFET IRFZ44N** (chauffage) : broche **9**.
- **Écran ST7920 (SPI)** :
  - **CS** → broche **10**
  - **DATA** → broche **11**
  - **CLK** → broche **13**
- **Thermistance NTC 100K** : broche **A0**.
- **Buzzer** : broche **A1**.

fichier de configuration **`PETCTL_cfg.h`** gérer les périphériques via des bibliothèques.

---

#### 2. **Fichier de Configuration - `PETCTL_cfg.h`**
Ce fichier définit plusieurs paramètres matériels :
- **Micro-pas du moteur** : `CFG_STEP_DIV = 8`
- **Broches du moteur pas à pas** :
  - `CFG_STEP_STEP_PIN = 6`
  - `CFG_STEP_DIR_PIN = 5`
  - `CFG_STEP_EN_PIN = 7`
- **Inversion du sens du moteur** : `#define CFG_STEP_INVERT`
- **Broches de l’encodeur rotatif** :
  - `CFG_ENC_CLK = 3`
  - `CFG_ENC_DT = 2`
  - `CFG_ENC_SW = 4`
- **Fin de course et arrêt d'urgence** :
  - `CFG_ENDSTOP_PIN = 8`
  - `CFG_EMENDSTOP_PIN = 12`
- **Capteurs et chauffage** :
  - `CFG_TERM_PIN = A0`
  - `CFG_HEATER_PIN = 9`
- **Autres paramètres** :
  - `CFG_BOBIN_DIAM = 74` mm → Diamètre de la bobine de filament
  - `CFG_SPEED_INIT = 2.5` mm/s → Vitesse initiale du moteur

---

### **Prochaines Étapes**
programme principal** en détail pour :
1. Identifier **comment il contrôle le moteur pas à pas, le chauffage et la gestion de température**.
2. Vérifier **la gestion de l’encodeur rotatif**.
3. Examiner l'affichage sur l'**écran ST7920**.


### **Analyse du Code**

Ton programme **gère plusieurs fonctionnalités essentielles** 
projet de transformation de bouteilles en filament :

---

### **1. Contrôle du moteur pas à pas (NEMA 17 via A4988)**
- Utilise la bibliothèque **GyverStepper** pour contrôler le moteur.
- La fonction principale de contrôle est **`motorCTL(long setSpeedX10)`** :
  - **setSpeedX10 > 0** → Le moteur démarre à la vitesse donnée.
  - **setSpeedX10 == 0** → Le moteur s’arrête.
  - **setSpeedX10 == -1** → Freinage du moteur.
- Conversion **mm/s en degrés/s** via **`mmStoDeg(float mmS)`**.
- Lecture de la position via **`getMilage()`** (convertit la position du moteur en mm).

---

### **2. Gestion du chauffage (MOSFET IRFZ44N + cartouche chauffante 12V/40W)**
- Le chauffage est contrôlé par un **PID** défini dans **GyverPID**.
- La température est mesurée avec une **thermistance NTC 100K ohm B3950** connectée sur **A0**.
- La fonction **`getTemp()`** lit la température, applique la conversion de Steinhart-Hart, puis un **filtrage de Kalman** pour stabiliser les valeurs.
- **Arrêt d’urgence en cas de surchauffe** (`curTemp > CFG_TEMP_MAX - 10`).

---

### **3. Sécurités (Endstops et arrêt d’urgence)**
- **Endstop principal** (`CFG_ENDSTOP_PIN = 8`) : Détecte la fin du filament et arrête le moteur.
- **Endstop d’urgence** (`CFG_EMENDSTOP_PIN = 12`) : Arrête immédiatement le moteur et le chauffage.
- La fonction **`emStop(int reason)`** bloque l’exécution en cas de problème (surchauffe, thermistance défectueuse).

---

### **4. Gestion de l’encodeur rotatif KY-040**
- Utilisation de **GyverEncoder**.
- Fonction **`encRotationToValue(long* value, int inc, long minValue, long maxValue)`** pour modifier les valeurs en fonction de la rotation.
- **Modes de réglage** (température, vitesse, etc.).
- Vérification si l’encodeur est actif via **`isInteractive()`** (15s d’activation après interaction).

---

### **5. Affichage sur écran ST7920 (via U8glib)**
- L’affichage est **mis à jour en une seule fois** à la fin de la loop via **`drawScreen()`**.
- Les indicateurs à l’écran :
  - **Température actuelle et cible** (`printCurrentTemp()`, `printTargetTemp()`).
  - **Statut du moteur** (`printSpeed()`, `printMilage()`).
  - **Statut des capteurs et encodeurs** :
    - `drawThermistorStatus()`
    - `drawPIDStatus()`
    - `drawEncoderStatus()`
    - `drawEndstopStatus()`
- Un **écran d’erreur (Halt, Overheat, Thermistor)** est affiché en cas de problème.

