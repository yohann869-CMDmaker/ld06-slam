# Export LidarScope : cartographie lidar LD06

Créé le 03/11/2026 à 12:04:20.

## Conventions (valables pour tous les fichiers)
- Distances en **mètres**, angles en **degrés**, sens **trigonométrique** (antihoraire), 0° = avant du robot.
- Repère **lidar** : +X = avant du robot, +Y = sa gauche. Repère **carte** : origine au coin de référence de la salle.
- Pose `(x, y, θ)` : `x_carte = x + cos θ·x_lidar − sin θ·y_lidar`, `y_carte = y + sin θ·x_lidar + cos θ·y_lidar`.
- `distance = 0` = pas d'écho. Intensité vide / `null` = non transmise (format Teleplot).
- `session.lidar.json` contient la description complète des colonnes et les lignes série **brutes**.

## Réglages de cartographie (étapes du cours)
- Prétraitement : portée 0.67 à 12.0 m, intensité ≥ 0
- Grid map : résolution 5 cm, état initial « inconnu », modèle probabiliste, p(occupé | impact) = 0.70, p(occupé | traversée) = 0.40, suppression des cellules isolées : non
- Carte vectorielle : saut 27 cm, tolérance de découpe 10 cm, longueur min 10 cm, fusion si angle ≤ 24°

## Contenu

### `scan_01_scan_1/`

- **Scan 1** : pose (-8.76 m, -0.23 m, 4.0°), 870/870 points avec écho, source `printScanCSV` : Reçu au format CSV de la bibliothèque LD06 (N,angle,distance_mm,intensité) : mesures complètes.
- Grid map 274 x 169 cellules : 367 occupées, 9294 libres, 36645 inconnues (surface explorée 24.15 m²)
- Carte vectorielle : 34 segments, longueur totale 13.55 m
- Fichiers : `session.lidar.json`, `points.csv`, `carte.png`, `grid_map_maptool.csv`, `grid_map_ternaire.csv`, `grid_map_proba.csv`, `carte_vectorielle.txt`, `scans_tp/scan_01.csv`

### `scan_02_scan_2/`

- **Scan 2** : pose (-10.04 m, 0.58 m, 10.0°), 304/304 points avec écho, source `printScanCSV` : Reçu au format CSV de la bibliothèque LD06 (N,angle,distance_mm,intensité) : mesures complètes.
- Grid map 209 x 91 cellules : 105 occupées, 2513 libres, 16401 inconnues (surface explorée 6.55 m²)
- Carte vectorielle : 7 segments, longueur totale 3.96 m
- Fichiers : `session.lidar.json`, `points.csv`, `carte.png`, `grid_map_maptool.csv`, `grid_map_ternaire.csv`, `grid_map_proba.csv`, `carte_vectorielle.txt`, `scans_tp/scan_02.csv`

### `scan_03_scan_3/`

- **Scan 3** : pose (-9.17 m, -3.36 m, 2.0°), 305/305 points avec écho, source `printScanCSV` : Reçu au format CSV de la bibliothèque LD06 (N,angle,distance_mm,intensité) : mesures complètes.
- Grid map 264 x 190 cellules : 233 occupées, 8095 libres, 41832 inconnues (surface explorée 20.82 m²)
- Carte vectorielle : 9 segments, longueur totale 8.04 m
- Fichiers : `session.lidar.json`, `points.csv`, `carte.png`, `grid_map_maptool.csv`, `grid_map_ternaire.csv`, `grid_map_proba.csv`, `carte_vectorielle.txt`, `scans_tp/scan_03.csv`

### `scan_05_scan_5/`

- **Scan 5** : pose (-4.42 m, -3.33 m, 5.0°), 304/304 points avec écho, source `printScanCSV` : Reçu au format CSV de la bibliothèque LD06 (N,angle,distance_mm,intensité) : mesures complètes.
- Grid map 272 x 248 cellules : 277 occupées, 12180 libres, 54999 inconnues (surface explorée 31.14 m²)
- Carte vectorielle : 17 segments, longueur totale 15.55 m
- Fichiers : `session.lidar.json`, `points.csv`, `carte.png`, `grid_map_maptool.csv`, `grid_map_ternaire.csv`, `grid_map_proba.csv`, `carte_vectorielle.txt`, `scans_tp/scan_05.csv`

### `scan_06_scan_6/`

- **Scan 6** : pose (-3.48 m, -0.10 m, 0.5°), 302/303 points avec écho, source `printScanCSV` : Reçu au format CSV de la bibliothèque LD06 (N,angle,distance_mm,intensité) : mesures complètes.
- Grid map 274 x 247 cellules : 263 occupées, 10576 libres, 56839 inconnues (surface explorée 27.1 m²)
- Carte vectorielle : 12 segments, longueur totale 11.29 m
- Fichiers : `session.lidar.json`, `points.csv`, `carte.png`, `grid_map_maptool.csv`, `grid_map_ternaire.csv`, `grid_map_proba.csv`, `carte_vectorielle.txt`, `scans_tp/scan_06.csv`

### `scan_07_scan_7/`

- **Scan 7** : pose (-6.18 m, -0.16 m, 0.0°), 792/792 points avec écho, source `printScanCSV` : Reçu au format CSV de la bibliothèque LD06 (N,angle,distance_mm,intensité) : mesures complètes.
- Grid map 273 x 212 cellules : 371 occupées, 10038 libres, 47467 inconnues (surface explorée 26.02 m²)
- Carte vectorielle : 33 segments, longueur totale 11.35 m
- Fichiers : `session.lidar.json`, `points.csv`, `carte.png`, `grid_map_maptool.csv`, `grid_map_ternaire.csv`, `grid_map_proba.csv`, `carte_vectorielle.txt`, `scans_tp/scan_07.csv`

### `scan_08_scan_8/`

- **Scan 8** : pose (-5.97 m, -2.33 m, 7.0°), 758/758 points avec écho, source `printScanCSV` : Reçu au format CSV de la bibliothèque LD06 (N,angle,distance_mm,intensité) : mesures complètes.
- Grid map 274 x 252 cellules : 563 occupées, 27899 libres, 40586 inconnues (surface explorée 71.16 m²)
- Carte vectorielle : 37 segments, longueur totale 27.92 m
- Fichiers : `session.lidar.json`, `points.csv`, `carte.png`, `grid_map_maptool.csv`, `grid_map_ternaire.csv`, `grid_map_proba.csv`, `carte_vectorielle.txt`, `scans_tp/scan_08.csv`

### `scan_09_scan_9/`

- **Scan 9** : pose (-4.74 m, 0.66 m, 7.0°), 766/766 points avec écho, source `printScanCSV` : Reçu au format CSV de la bibliothèque LD06 (N,angle,distance_mm,intensité) : mesures complètes.
- Grid map 274 x 214 cellules : 484 occupées, 31288 libres, 26864 inconnues (surface explorée 79.43 m²)
- Carte vectorielle : 7 segments, longueur totale 40.42 m
- Fichiers : `session.lidar.json`, `points.csv`, `carte.png`, `grid_map_maptool.csv`, `grid_map_ternaire.csv`, `grid_map_proba.csv`, `carte_vectorielle.txt`, `scans_tp/scan_09.csv`

### `scan_10_scan_10/`

- **Scan 10** : pose (0.59 m, -5.74 m, 3.5°), 784/784 points avec écho, source `printScanCSV` : Reçu au format CSV de la bibliothèque LD06 (N,angle,distance_mm,intensité) : mesures complètes.
- Grid map 272 x 157 cellules : 392 occupées, 9460 libres, 32852 inconnues (surface explorée 24.63 m²)
- Carte vectorielle : 28 segments, longueur totale 17.25 m
- Fichiers : `session.lidar.json`, `points.csv`, `carte.png`, `grid_map_maptool.csv`, `grid_map_ternaire.csv`, `grid_map_proba.csv`, `carte_vectorielle.txt`, `scans_tp/scan_10.csv`

### `scan_11_scan_11/`

- **Scan 11** : pose (0.36 m, -7.49 m, 0.5°), 304/304 points avec écho, source `printScanCSV` : Reçu au format CSV de la bibliothèque LD06 (N,angle,distance_mm,intensité) : mesures complètes.
- Grid map 173 x 149 cellules : 209 occupées, 4994 libres, 20574 inconnues (surface explorée 13.01 m²)
- Carte vectorielle : 6 segments, longueur totale 8.91 m
- Fichiers : `session.lidar.json`, `points.csv`, `carte.png`, `grid_map_maptool.csv`, `grid_map_ternaire.csv`, `grid_map_proba.csv`, `carte_vectorielle.txt`, `scans_tp/scan_11.csv`

### `scan_12_scan_12/`

- **Scan 12** : pose (0.62 m, -3.22 m, 0.0°), 693/693 points avec écho, source `printScanCSV` : Reçu au format CSV de la bibliothèque LD06 (N,angle,distance_mm,intensité) : mesures complètes.
- Grid map 275 x 214 cellules : 485 occupées, 28779 libres, 29586 inconnues (surface explorée 73.16 m²)
- Carte vectorielle : 14 segments, longueur totale 37.62 m
- Fichiers : `session.lidar.json`, `points.csv`, `carte.png`, `grid_map_maptool.csv`, `grid_map_ternaire.csv`, `grid_map_proba.csv`, `carte_vectorielle.txt`, `scans_tp/scan_12.csv`

### `scan_13_scan_4/`

- **scan 4** : pose (-4.76 m, 0.69 m, 89.0°), 454/454 points avec écho, source `fichier` : Importé depuis un fichier.
- Grid map 273 x 215 cellules : 387 occupées, 28550 libres, 29758 inconnues (surface explorée 72.34 m²)
- Carte vectorielle : 6 segments, longueur totale 41.79 m
- Fichiers : `session.lidar.json`, `points.csv`, `carte.png`, `grid_map_maptool.csv`, `grid_map_ternaire.csv`, `grid_map_proba.csv`, `carte_vectorielle.txt`, `scans_tp/scan_13.csv`

### `ensemble/`

- **Scan 1** : pose (-8.76 m, -0.23 m, 4.0°), 870/870 points avec écho, source `printScanCSV` : Reçu au format CSV de la bibliothèque LD06 (N,angle,distance_mm,intensité) : mesures complètes.
- **Scan 2** : pose (-10.04 m, 0.58 m, 10.0°), 304/304 points avec écho, source `printScanCSV` : Reçu au format CSV de la bibliothèque LD06 (N,angle,distance_mm,intensité) : mesures complètes.
- **Scan 3** : pose (-9.17 m, -3.36 m, 2.0°), 305/305 points avec écho, source `printScanCSV` : Reçu au format CSV de la bibliothèque LD06 (N,angle,distance_mm,intensité) : mesures complètes.
- **Scan 5** : pose (-4.42 m, -3.33 m, 5.0°), 304/304 points avec écho, source `printScanCSV` : Reçu au format CSV de la bibliothèque LD06 (N,angle,distance_mm,intensité) : mesures complètes.
- **Scan 6** : pose (-3.48 m, -0.10 m, 0.5°), 302/303 points avec écho, source `printScanCSV` : Reçu au format CSV de la bibliothèque LD06 (N,angle,distance_mm,intensité) : mesures complètes.
- **Scan 7** : pose (-6.18 m, -0.16 m, 0.0°), 792/792 points avec écho, source `printScanCSV` : Reçu au format CSV de la bibliothèque LD06 (N,angle,distance_mm,intensité) : mesures complètes.
- **Scan 8** : pose (-5.97 m, -2.33 m, 7.0°), 758/758 points avec écho, source `printScanCSV` : Reçu au format CSV de la bibliothèque LD06 (N,angle,distance_mm,intensité) : mesures complètes.
- **Scan 9** : pose (-4.74 m, 0.66 m, 7.0°), 766/766 points avec écho, source `printScanCSV` : Reçu au format CSV de la bibliothèque LD06 (N,angle,distance_mm,intensité) : mesures complètes.
- **Scan 10** : pose (0.59 m, -5.74 m, 3.5°), 784/784 points avec écho, source `printScanCSV` : Reçu au format CSV de la bibliothèque LD06 (N,angle,distance_mm,intensité) : mesures complètes.
- **Scan 11** : pose (0.36 m, -7.49 m, 0.5°), 304/304 points avec écho, source `printScanCSV` : Reçu au format CSV de la bibliothèque LD06 (N,angle,distance_mm,intensité) : mesures complètes.
- **Scan 12** : pose (0.62 m, -3.22 m, 0.0°), 693/693 points avec écho, source `printScanCSV` : Reçu au format CSV de la bibliothèque LD06 (N,angle,distance_mm,intensité) : mesures complètes.
- **scan 4** : pose (-4.76 m, 0.69 m, 89.0°), 454/454 points avec écho, source `fichier` : Importé depuis un fichier.
- Grid map 281 x 252 cellules : 1581 occupées, 45624 libres, 23607 inconnues (surface explorée 118.01 m²)
- Carte vectorielle : 31 segments, longueur totale 69.56 m
- Fichiers : `session.lidar.json`, `points.csv`, `carte.png`, `grid_map_maptool.csv`, `grid_map_ternaire.csv`, `grid_map_proba.csv`, `carte_vectorielle.txt`, `scans_tp/scan_01.csv`, `scans_tp/scan_02.csv`, `scans_tp/scan_03.csv`, `scans_tp/scan_05.csv`, `scans_tp/scan_06.csv`, `scans_tp/scan_07.csv`, `scans_tp/scan_08.csv`, `scans_tp/scan_09.csv`, `scans_tp/scan_10.csv`, `scans_tp/scan_11.csv`, `scans_tp/scan_12.csv`, `scans_tp/scan_13.csv`
