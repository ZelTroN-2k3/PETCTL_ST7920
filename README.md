# Il s’agit d’un contrôleur qui fait fonctionner une machine qui convertit le ruban découpé dans des bouteilles en PET en fibre adaptée à l’impression 3D.
![GitHub Issues or Pull Requests](https://img.shields.io/github/issues/ZelTroN-2k3/10kHz-to-225MHz-VFO-RF-Generator-with-Si5351---Version-2.1)
![GitHub License](https://img.shields.io/github/license/ZelTroN-2k3/10kHz-to-225MHz-VFO-RF-Generator-with-Si5351---Version-2.1)
![GitHub Repo stars](https://img.shields.io/github/stars/ZelTroN-2k3/10kHz-to-225MHz-VFO-RF-Generator-with-Si5351---Version-2.1?style=flat)
![GitHub forks](https://img.shields.io/github/forks/ZelTroN-2k3/10kHz-to-225MHz-VFO-RF-Generator-with-Si5351---Version-2.1?style=flat)
![GitHub Issues or Pull Requests](https://github.com/ZelTroN-2k3/PETCTL_ST7920/blob/v0.11a/image_ecran2.jpg)

- Version 0.11a
https://github.com/ZelTroN-2k3/PETCTL_ST7920/blob/v0.11a/image_ecran1.jpg

- Le croquis utilise 23750 octets (77%) de l'espace de stockage de programmes.
- Le maximum est de 30720 octets.
  Les variables globales utilisent 553 octets (27%) de mémoire dynamique,
  ce qui laisse 1495 octets pour les variables locales. Le maximum est de 2048 octets.

# Source code Original de l'auteur: https://github.com/mvbasov/PETCTL/tree/github
# Comment gérer
Les commandes se sont avérées très pratiques (pour moi) :
- réglage de la température en un clic. La température cible est mise en évidence dans le coin droit. Dans ce mode, un appui long allume/éteint le chauffage, l'étoile au début de la 1ère ligne s'allume/s'éteint.
- deux clics pour régler la vitesse. La vitesse au milieu est mise en évidence. Dans ce mode, un appui long allume/éteint le moteur, l'étoile au début de la 2ème ligne s'allume/s'éteint.
- Si vous n'appuyez sur rien pendant 15 secondes, il passe en mode veille comme sur la photo.
- Si le capteur de fin de bande est connecté, alors après son déclenchement, le moteur s'éteint après avoir tiré CFG_PULL_EXTRA_LENGTH [m] le chauffage s'éteint également
- Si le capteur de rupture de fibre est connecté, après son déclenchement, le moteur s'éteint immédiatement et le chauffage s'éteint également

# Disposition générale de la machine
La machine est montée sur une planche de meuble (en résineux) de dimensions 43 x 29,5 cm, de sorte qu'elle peut être rangée dans une boîte ménagère pliable en plastique standard.
