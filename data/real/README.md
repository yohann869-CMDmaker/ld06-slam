# Scans réels

Déposer ici les scans de la salle, un fichier par scan, avec le récapitulatif `poses.csv`
(bouton **Enregistrer tous les scans…** de l'application).

Trois façons d'acquérir les scans (détails dans le README principal) :

- **Programme de test du module** (méthode utilisée) : copier la sortie Teleplot du moniteur
  série et la coller dans l'application (Ctrl+V).
- **USB à la demande** : programme `firmware/ESP32_LD06_Capture` et bouton
  **Capturer un scan** de l'application.
- **Wi-Fi** : RobotControl, bouton `Export CSV`.

Format des fichiers (identique à l'export RobotControl) :

```
# Robot position: x=0.0000m, y=0.0000m, theta=0.00deg
# LD06 scan n=452 speed=3602deg/s crc_errors=0
angle_deg,distance_m,intensity
359.620,1.2340,212
...
```

- `angle_deg` est déjà en convention mathématique (sens antihoraire) : la librairie LD06 a
  fait la conversion `360 - angle` sur l'ESP32. Ne pas l'inverser une seconde fois.
- **Poses.** Sans instrument de mesure, la pose de chaque scan est obtenue par recalage manuel
  dans l'application : le premier scan définit le repère (0, 0, 0), les suivants sont déplacés
  jusqu'à ce que leurs murs se superposent. Pour estimer l'incertitude de ces poses de
  référence, recaler un même scan deux fois en partant de zéro et noter l'écart obtenu.
- Robot immobile pendant l'acquisition (un tour dure environ 100 ms, sans horodatage par point),
  et personne qui passe dans la zone scannée.

Vérification d'orientation à faire avant toute campagne : une boîte à 50 cm devant le robot
doit apparaître vers 0° (x ≈ 0,5 ; y ≈ 0), une boîte à gauche vers 90° (y > 0).
