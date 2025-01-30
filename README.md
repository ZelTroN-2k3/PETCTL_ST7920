# Il s’agit d’un contrôleur qui fait fonctionner une machine qui convertit le ruban découpé dans des bouteilles en PET en fibre adaptée à l’impression 3D.

# Comment gérer
Les commandes se sont avérées très pratiques (pour moi) :
- réglage de la température en un clic. La température cible est mise en évidence dans le coin droit. Dans ce mode, un appui long allume/éteint le chauffage, l'étoile au début de la 1ère ligne s'allume/s'éteint.
- deux clics pour régler la vitesse. La vitesse au milieu est mise en évidence. Dans ce mode, un appui long allume/éteint le moteur, l'étoile au début de la 2ème ligne s'allume/s'éteint.
- Si vous n'appuyez sur rien pendant 15 secondes, il passe en mode veille comme sur la photo.
- Si le capteur de fin de bande est connecté, alors après son déclenchement, le moteur s'éteint après avoir tiré CFG_PULL_EXTRA_LENGTH [m] le chauffage s'éteint également
- Si le capteur de rupture de fibre est connecté, après son déclenchement, le moteur s'éteint immédiatement et le chauffage s'éteint également

# Disposition générale de la machine
La machine est montée sur une planche de meuble (en résineux) de dimensions 43 x 29,5 cm, de sorte qu'elle peut être rangée dans une boîte ménagère pliable en plastique standard.
