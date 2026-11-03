# Export LidarScope : cartographie lidar LD06

Créé le 02/11/2026 à 21:02:21.

## Conventions (valables pour tous les fichiers)
- Distances en **mètres**, angles en **degrés**, sens **trigonométrique** (antihoraire), 0° = avant du robot.
- Repère **lidar** : +X = avant du robot, +Y = sa gauche. Repère **carte** : origine au coin de référence de la salle.
- Pose `(x, y, θ)` : `x_carte = x + cos θ·x_lidar − sin θ·y_lidar`, `y_carte = y + sin θ·x_lidar + cos θ·y_lidar`.
- `distance = 0` = pas d'écho. Intensité vide / `null` = non transmise (format Teleplot).
- `session.lidar.json` contient la description complète des colonnes et les lignes série **brutes**.

## Réglages de cartographie (étapes du cours)
- Prétraitement : portée 0.70 à 12.0 m, intensité ≥ 123
- Grid map : résolution 5 cm, état initial « inconnu », modèle probabiliste, p(occupé | impact) = 0.85, p(occupé | traversée) = 0.45, suppression des cellules isolées : oui
- Carte vectorielle : saut 21 cm, tolérance de découpe 8 cm, longueur min 53 cm, fusion si angle ≤ 21°

## Contenu

### `scan_01_scan_1/`

- **Scan 1** : pose (1.60 m, -4.40 m, -86.0°), 708/708 points avec écho, source `printScanTeleplot` : Reçu au format Teleplot (>lidar:x:y;...|xy, mm) : seuls x et y sont transmis. intensity = null (inconnue). Angle des points sans écho interpolé entre voisins. y corrigé du miroir de la bibliothèque LD06 (qui envoie y = -d·sin θ).
- Grid map 254 x 281 cellules : 362 occupées, 23978 libres, 47034 inconnues (surface explorée 60.85 m²)
- Carte vectorielle : 16 segments, longueur totale 19.23 m
- Fichiers : `session.lidar.json`, `points.csv`, `carte.png`, `grid_map_maptool.csv`, `grid_map_ternaire.csv`, `grid_map_proba.csv`, `carte_vectorielle.txt`, `scans_tp/scan_01.csv`

### `scan_02_scan_2/`

- **Scan 2** : pose (1.60 m, -4.20 m, -90.0°), 325/325 points avec écho, source `printScanTeleplot` : Reçu au format Teleplot (>lidar:x:y;...|xy, mm) : seuls x et y sont transmis. intensity = null (inconnue). Angle des points sans écho interpolé entre voisins. y corrigé du miroir de la bibliothèque LD06 (qui envoie y = -d·sin θ).
- Grid map 263 x 289 cellules : 138 occupées, 21785 libres, 54084 inconnues (surface explorée 54.81 m²)
- Carte vectorielle : 10 segments, longueur totale 22.43 m
- Fichiers : `session.lidar.json`, `points.csv`, `carte.png`, `grid_map_maptool.csv`, `grid_map_ternaire.csv`, `grid_map_proba.csv`, `carte_vectorielle.txt`, `scans_tp/scan_02.csv`

### `scan_03_scan_3/`

- **Scan 3** : pose (0.20 m, 1.20 m, 0.0°), 324/324 points avec écho, source `printScanTeleplot` : Reçu au format Teleplot (>lidar:x:y;...|xy, mm) : seuls x et y sont transmis. intensity = null (inconnue). Angle des points sans écho interpolé entre voisins. y corrigé du miroir de la bibliothèque LD06 (qui envoie y = -d·sin θ).
- Grid map 219 x 273 cellules : 141 occupées, 5895 libres, 53751 inconnues (surface explorée 15.09 m²)
- Carte vectorielle : 0 segments, longueur totale 0 m
- Fichiers : `session.lidar.json`, `points.csv`, `carte.png`, `grid_map_maptool.csv`, `grid_map_ternaire.csv`, `grid_map_proba.csv`, `scans_tp/scan_03.csv`

### `scan_04_scan_4/`

- **Scan 4** : pose (0.40 m, 3.40 m, 0.0°), 325/325 points avec écho, source `printScanTeleplot` : Reçu au format Teleplot (>lidar:x:y;...|xy, mm) : seuls x et y sont transmis. intensity = null (inconnue). Angle des points sans écho interpolé entre voisins. y corrigé du miroir de la bibliothèque LD06 (qui envoie y = -d·sin θ).
- Grid map 92 x 179 cellules : 121 occupées, 1994 libres, 14353 inconnues (surface explorée 5.29 m²)
- Carte vectorielle : 3 segments, longueur totale 2.64 m
- Fichiers : `session.lidar.json`, `points.csv`, `carte.png`, `grid_map_maptool.csv`, `grid_map_ternaire.csv`, `grid_map_proba.csv`, `carte_vectorielle.txt`, `scans_tp/scan_04.csv`

### `scan_05_scan_5/`

- **Scan 5** : pose (1.20 m, 3.60 m, 0.0°), 840/840 points avec écho, source `printScanTeleplot` : Reçu au format Teleplot (>lidar:x:y;...|xy, mm) : seuls x et y sont transmis. intensity = null (inconnue). Angle des points sans écho interpolé entre voisins. y corrigé du miroir de la bibliothèque LD06 (qui envoie y = -d·sin θ).
- Grid map 220 x 248 cellules : 226 occupées, 8166 libres, 46168 inconnues (surface explorée 20.98 m²)
- Carte vectorielle : 7 segments, longueur totale 7.7 m
- Fichiers : `session.lidar.json`, `points.csv`, `carte.png`, `grid_map_maptool.csv`, `grid_map_ternaire.csv`, `grid_map_proba.csv`, `carte_vectorielle.txt`, `scans_tp/scan_05.csv`

### `scan_06_scan_6/`

- **Scan 6** : pose (4.20 m, -2.25 m, 0.0°), 734/734 points avec écho, source `printScanTeleplot` : Reçu au format Teleplot (>lidar:x:y;...|xy, mm) : seuls x et y sont transmis. intensity = null (inconnue). Angle des points sans écho interpolé entre voisins. y corrigé du miroir de la bibliothèque LD06 (qui envoie y = -d·sin θ).
- Grid map 361 x 266 cellules : 268 occupées, 14040 libres, 81718 inconnues (surface explorée 35.77 m²)
- Carte vectorielle : 10 segments, longueur totale 12.07 m
- Fichiers : `session.lidar.json`, `points.csv`, `carte.png`, `grid_map_maptool.csv`, `grid_map_ternaire.csv`, `grid_map_proba.csv`, `carte_vectorielle.txt`, `scans_tp/scan_06.csv`

### `scan_07_scan_7/`

- **Scan 7** : pose (6.80 m, 1.40 m, 5.0°), 324/324 points avec écho, source `printScanTeleplot` : Reçu au format Teleplot (>lidar:x:y;...|xy, mm) : seuls x et y sont transmis. intensity = null (inconnue). Angle des points sans écho interpolé entre voisins. y corrigé du miroir de la bibliothèque LD06 (qui envoie y = -d·sin θ).
- Grid map 153 x 217 cellules : 212 occupées, 6213 libres, 26776 inconnues (surface explorée 16.06 m²)
- Carte vectorielle : 4 segments, longueur totale 8.61 m
- Fichiers : `session.lidar.json`, `points.csv`, `carte.png`, `grid_map_maptool.csv`, `grid_map_ternaire.csv`, `grid_map_proba.csv`, `carte_vectorielle.txt`, `scans_tp/scan_07.csv`

### `scan_08_scan_8/`

- **Scan 8** : pose (2.60 m, -6.65 m, 0.0°), 324/324 points avec écho, source `printScanTeleplot` : Reçu au format Teleplot (>lidar:x:y;...|xy, mm) : seuls x et y sont transmis. intensity = null (inconnue). Angle des points sans écho interpolé entre voisins. y corrigé du miroir de la bibliothèque LD06 (qui envoie y = -d·sin θ).
- Grid map 184 x 220 cellules : 139 occupées, 6789 libres, 33552 inconnues (surface explorée 17.32 m²)
- Carte vectorielle : 3 segments, longueur totale 2.26 m
- Fichiers : `session.lidar.json`, `points.csv`, `carte.png`, `grid_map_maptool.csv`, `grid_map_ternaire.csv`, `grid_map_proba.csv`, `carte_vectorielle.txt`, `scans_tp/scan_08.csv`

### `scan_09_scan_9/`

- **Scan 9** : pose (2.70 m, -6.60 m, 3.0°), 325/325 points avec écho, source `printScanTeleplot` : Reçu au format Teleplot (>lidar:x:y;...|xy, mm) : seuls x et y sont transmis. intensity = null (inconnue). Angle des points sans écho interpolé entre voisins. y corrigé du miroir de la bibliothèque LD06 (qui envoie y = -d·sin θ).
- Grid map 174 x 273 cellules : 140 occupées, 6976 libres, 40386 inconnues (surface explorée 17.79 m²)
- Carte vectorielle : 4 segments, longueur totale 4.22 m
- Fichiers : `session.lidar.json`, `points.csv`, `carte.png`, `grid_map_maptool.csv`, `grid_map_ternaire.csv`, `grid_map_proba.csv`, `carte_vectorielle.txt`, `scans_tp/scan_09.csv`

### `scan_10_scan_10/`

- **Scan 10** : pose (3.60 m, -6.35 m, -7.0°), 733/733 points avec écho, source `printScanTeleplot` : Reçu au format Teleplot (>lidar:x:y;...|xy, mm) : seuls x et y sont transmis. intensity = null (inconnue). Angle des points sans écho interpolé entre voisins. y corrigé du miroir de la bibliothèque LD06 (qui envoie y = -d·sin θ).
- Grid map 209 x 276 cellules : 252 occupées, 8750 libres, 48682 inconnues (surface explorée 22.51 m²)
- Carte vectorielle : 4 segments, longueur totale 4.09 m
- Fichiers : `session.lidar.json`, `points.csv`, `carte.png`, `grid_map_maptool.csv`, `grid_map_ternaire.csv`, `grid_map_proba.csv`, `carte_vectorielle.txt`, `scans_tp/scan_10.csv`

### `scan_12_scan_12/`

- **Scan 12** : pose (-0.15 m, -5.45 m, 0.0°), 325/325 points avec écho, source `printScanTeleplot` : Reçu au format Teleplot (>lidar:x:y;...|xy, mm) : seuls x et y sont transmis. intensity = null (inconnue). Angle des points sans écho interpolé entre voisins. y corrigé du miroir de la bibliothèque LD06 (qui envoie y = -d·sin θ).
- Grid map 116 x 278 cellules : 186 occupées, 6632 libres, 25430 inconnues (surface explorée 17.05 m²)
- Carte vectorielle : 2 segments, longueur totale 2.2 m
- Fichiers : `session.lidar.json`, `points.csv`, `carte.png`, `grid_map_maptool.csv`, `grid_map_ternaire.csv`, `grid_map_proba.csv`, `carte_vectorielle.txt`, `scans_tp/scan_12.csv`

### `scan_13_scan_13/`

- **Scan 13** : pose (0.40 m, -2.05 m, 2.0°), 323/324 points avec écho, source `printScanTeleplot` : Reçu au format Teleplot (>lidar:x:y;...|xy, mm) : seuls x et y sont transmis. intensity = null (inconnue). Angle des points sans écho interpolé entre voisins. y corrigé du miroir de la bibliothèque LD06 (qui envoie y = -d·sin θ).
- Grid map 252 x 274 cellules : 126 occupées, 11703 libres, 57219 inconnues (surface explorée 29.57 m²)
- Carte vectorielle : 4 segments, longueur totale 2.92 m
- Fichiers : `session.lidar.json`, `points.csv`, `carte.png`, `grid_map_maptool.csv`, `grid_map_ternaire.csv`, `grid_map_proba.csv`, `carte_vectorielle.txt`, `scans_tp/scan_13.csv`

### `scan_14_scan_14/`

- **Scan 14** : pose (-1.00 m, -2.70 m, -5.0°), 325/325 points avec écho, source `printScanTeleplot` : Reçu au format Teleplot (>lidar:x:y;...|xy, mm) : seuls x et y sont transmis. intensity = null (inconnue). Angle des points sans écho interpolé entre voisins. y corrigé du miroir de la bibliothèque LD06 (qui envoie y = -d·sin θ).
- Grid map 239 x 292 cellules : 73 occupées, 22778 libres, 46937 inconnues (surface explorée 57.13 m²)
- Carte vectorielle : 8 segments, longueur totale 32.81 m
- Fichiers : `session.lidar.json`, `points.csv`, `carte.png`, `grid_map_maptool.csv`, `grid_map_ternaire.csv`, `grid_map_proba.csv`, `carte_vectorielle.txt`, `scans_tp/scan_14.csv`

### `scan_15_scan_15/`

- **Scan 15** : pose (0.35 m, 0.45 m, 10.0°), 325/325 points avec écho, source `printScanTeleplot` : Reçu au format Teleplot (>lidar:x:y;...|xy, mm) : seuls x et y sont transmis. intensity = null (inconnue). Angle des points sans écho interpolé entre voisins. y corrigé du miroir de la bibliothèque LD06 (qui envoie y = -d·sin θ).
- Grid map 227 x 387 cellules : 119 occupées, 23169 libres, 64561 inconnues (surface explorée 58.22 m²)
- Carte vectorielle : 12 segments, longueur totale 36.59 m
- Fichiers : `session.lidar.json`, `points.csv`, `carte.png`, `grid_map_maptool.csv`, `grid_map_ternaire.csv`, `grid_map_proba.csv`, `carte_vectorielle.txt`, `scans_tp/scan_15.csv`

### `scan_16_scan_16/`

- **Scan 16** : pose (0.00 m, 0.00 m, 0.0°), 707/707 points avec écho, source `printScanTeleplot` : Reçu au format Teleplot (>lidar:x:y;...|xy, mm) : seuls x et y sont transmis. intensity = null (inconnue). Angle des points sans écho interpolé entre voisins. y corrigé du miroir de la bibliothèque LD06 (qui envoie y = -d·sin θ).
- Grid map 225 x 329 cellules : 371 occupées, 29780 libres, 43874 inconnues (surface explorée 75.38 m²)
- Carte vectorielle : 12 segments, longueur totale 34.63 m
- Fichiers : `session.lidar.json`, `points.csv`, `carte.png`, `grid_map_maptool.csv`, `grid_map_ternaire.csv`, `grid_map_proba.csv`, `carte_vectorielle.txt`, `scans_tp/scan_16.csv`

### `scan_17_scan_17/`

- **Scan 17** : pose (0.00 m, 0.00 m, -2.0°), 708/708 points avec écho, source `printScanTeleplot` : Reçu au format Teleplot (>lidar:x:y;...|xy, mm) : seuls x et y sont transmis. intensity = null (inconnue). Angle des points sans écho interpolé entre voisins. y corrigé du miroir de la bibliothèque LD06 (qui envoie y = -d·sin θ).
- Grid map 232 x 291 cellules : 492 occupées, 32229 libres, 34791 inconnues (surface explorée 81.8 m²)
- Carte vectorielle : 12 segments, longueur totale 36.06 m
- Fichiers : `session.lidar.json`, `points.csv`, `carte.png`, `grid_map_maptool.csv`, `grid_map_ternaire.csv`, `grid_map_proba.csv`, `carte_vectorielle.txt`, `scans_tp/scan_17.csv`

### `ensemble/`

- **Scan 1** : pose (1.60 m, -4.40 m, -86.0°), 708/708 points avec écho, source `printScanTeleplot` : Reçu au format Teleplot (>lidar:x:y;...|xy, mm) : seuls x et y sont transmis. intensity = null (inconnue). Angle des points sans écho interpolé entre voisins. y corrigé du miroir de la bibliothèque LD06 (qui envoie y = -d·sin θ).
- **Scan 2** : pose (1.60 m, -4.20 m, -90.0°), 325/325 points avec écho, source `printScanTeleplot` : Reçu au format Teleplot (>lidar:x:y;...|xy, mm) : seuls x et y sont transmis. intensity = null (inconnue). Angle des points sans écho interpolé entre voisins. y corrigé du miroir de la bibliothèque LD06 (qui envoie y = -d·sin θ).
- **Scan 3** : pose (0.20 m, 1.20 m, 0.0°), 324/324 points avec écho, source `printScanTeleplot` : Reçu au format Teleplot (>lidar:x:y;...|xy, mm) : seuls x et y sont transmis. intensity = null (inconnue). Angle des points sans écho interpolé entre voisins. y corrigé du miroir de la bibliothèque LD06 (qui envoie y = -d·sin θ).
- **Scan 4** : pose (0.40 m, 3.40 m, 0.0°), 325/325 points avec écho, source `printScanTeleplot` : Reçu au format Teleplot (>lidar:x:y;...|xy, mm) : seuls x et y sont transmis. intensity = null (inconnue). Angle des points sans écho interpolé entre voisins. y corrigé du miroir de la bibliothèque LD06 (qui envoie y = -d·sin θ).
- **Scan 5** : pose (1.20 m, 3.60 m, 0.0°), 840/840 points avec écho, source `printScanTeleplot` : Reçu au format Teleplot (>lidar:x:y;...|xy, mm) : seuls x et y sont transmis. intensity = null (inconnue). Angle des points sans écho interpolé entre voisins. y corrigé du miroir de la bibliothèque LD06 (qui envoie y = -d·sin θ).
- **Scan 6** : pose (4.20 m, -2.25 m, 0.0°), 734/734 points avec écho, source `printScanTeleplot` : Reçu au format Teleplot (>lidar:x:y;...|xy, mm) : seuls x et y sont transmis. intensity = null (inconnue). Angle des points sans écho interpolé entre voisins. y corrigé du miroir de la bibliothèque LD06 (qui envoie y = -d·sin θ).
- **Scan 7** : pose (6.80 m, 1.40 m, 5.0°), 324/324 points avec écho, source `printScanTeleplot` : Reçu au format Teleplot (>lidar:x:y;...|xy, mm) : seuls x et y sont transmis. intensity = null (inconnue). Angle des points sans écho interpolé entre voisins. y corrigé du miroir de la bibliothèque LD06 (qui envoie y = -d·sin θ).
- **Scan 8** : pose (2.60 m, -6.65 m, 0.0°), 324/324 points avec écho, source `printScanTeleplot` : Reçu au format Teleplot (>lidar:x:y;...|xy, mm) : seuls x et y sont transmis. intensity = null (inconnue). Angle des points sans écho interpolé entre voisins. y corrigé du miroir de la bibliothèque LD06 (qui envoie y = -d·sin θ).
- **Scan 9** : pose (2.70 m, -6.60 m, 3.0°), 325/325 points avec écho, source `printScanTeleplot` : Reçu au format Teleplot (>lidar:x:y;...|xy, mm) : seuls x et y sont transmis. intensity = null (inconnue). Angle des points sans écho interpolé entre voisins. y corrigé du miroir de la bibliothèque LD06 (qui envoie y = -d·sin θ).
- **Scan 10** : pose (3.60 m, -6.35 m, -7.0°), 733/733 points avec écho, source `printScanTeleplot` : Reçu au format Teleplot (>lidar:x:y;...|xy, mm) : seuls x et y sont transmis. intensity = null (inconnue). Angle des points sans écho interpolé entre voisins. y corrigé du miroir de la bibliothèque LD06 (qui envoie y = -d·sin θ).
- **Scan 12** : pose (-0.15 m, -5.45 m, 0.0°), 325/325 points avec écho, source `printScanTeleplot` : Reçu au format Teleplot (>lidar:x:y;...|xy, mm) : seuls x et y sont transmis. intensity = null (inconnue). Angle des points sans écho interpolé entre voisins. y corrigé du miroir de la bibliothèque LD06 (qui envoie y = -d·sin θ).
- **Scan 13** : pose (0.40 m, -2.05 m, 2.0°), 323/324 points avec écho, source `printScanTeleplot` : Reçu au format Teleplot (>lidar:x:y;...|xy, mm) : seuls x et y sont transmis. intensity = null (inconnue). Angle des points sans écho interpolé entre voisins. y corrigé du miroir de la bibliothèque LD06 (qui envoie y = -d·sin θ).
- **Scan 14** : pose (-1.00 m, -2.70 m, -5.0°), 325/325 points avec écho, source `printScanTeleplot` : Reçu au format Teleplot (>lidar:x:y;...|xy, mm) : seuls x et y sont transmis. intensity = null (inconnue). Angle des points sans écho interpolé entre voisins. y corrigé du miroir de la bibliothèque LD06 (qui envoie y = -d·sin θ).
- **Scan 15** : pose (0.35 m, 0.45 m, 10.0°), 325/325 points avec écho, source `printScanTeleplot` : Reçu au format Teleplot (>lidar:x:y;...|xy, mm) : seuls x et y sont transmis. intensity = null (inconnue). Angle des points sans écho interpolé entre voisins. y corrigé du miroir de la bibliothèque LD06 (qui envoie y = -d·sin θ).
- **Scan 16** : pose (0.00 m, 0.00 m, 0.0°), 707/707 points avec écho, source `printScanTeleplot` : Reçu au format Teleplot (>lidar:x:y;...|xy, mm) : seuls x et y sont transmis. intensity = null (inconnue). Angle des points sans écho interpolé entre voisins. y corrigé du miroir de la bibliothèque LD06 (qui envoie y = -d·sin θ).
- **Scan 17** : pose (0.00 m, 0.00 m, -2.0°), 708/708 points avec écho, source `printScanTeleplot` : Reçu au format Teleplot (>lidar:x:y;...|xy, mm) : seuls x et y sont transmis. intensity = null (inconnue). Angle des points sans écho interpolé entre voisins. y corrigé du miroir de la bibliothèque LD06 (qui envoie y = -d·sin θ).
- Grid map 375 x 393 cellules : 3629 occupées, 46432 libres, 97314 inconnues (surface explorée 125.15 m²)
- Carte vectorielle : 39 segments, longueur totale 84.09 m
- Fichiers : `session.lidar.json`, `points.csv`, `carte.png`, `grid_map_maptool.csv`, `grid_map_ternaire.csv`, `grid_map_proba.csv`, `carte_vectorielle.txt`, `scans_tp/scan_01.csv`, `scans_tp/scan_02.csv`, `scans_tp/scan_03.csv`, `scans_tp/scan_04.csv`, `scans_tp/scan_05.csv`, `scans_tp/scan_06.csv`, `scans_tp/scan_07.csv`, `scans_tp/scan_08.csv`, `scans_tp/scan_09.csv`, `scans_tp/scan_10.csv`, `scans_tp/scan_12.csv`, `scans_tp/scan_13.csv`, `scans_tp/scan_14.csv`, `scans_tp/scan_15.csv`, `scans_tp/scan_16.csv`, `scans_tp/scan_17.csv`
