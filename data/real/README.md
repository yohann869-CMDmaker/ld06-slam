# Scans réels

Déposer ici les scans exportés depuis RobotControl (`Export CSV`), un fichier par scan.

Format attendu (export RobotControl) :

```
# Robot position: x=0.0000m, y=0.0000m, theta=0.00deg
angle_deg,distance_m,intensity
0.360,1.2340,212
...
```

- `angle_deg` est déjà en convention mathématique (sens antihoraire) : la librairie
  LD06 a fait la conversion `360 - angle` sur l'ESP32. Ne pas l'inverser une seconde fois.
- La pose de la ligne `#` est celle saisie dans RobotControl (`/pose`) : elle n'est pas
  mesurée. Noter dans `poses.md` comment elle a été relevée (mètre ruban, repères au sol)
  et son incertitude estimée.
- Robot immobile pendant l'acquisition (un tour dure environ 100 ms, sans horodatage par point).

Vérification d'orientation à faire avant toute campagne : une boîte à 50 cm devant le robot
doit apparaître vers 0° (x ≈ 0,5 ; y ≈ 0), une boîte à gauche vers 90° (y > 0).
