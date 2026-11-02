# LD06 SLAM — Cartographie et localisation 2D

Projet du module *Cartographie et localisation appliquée à la robotique mobile*.
Outils pour exploiter les scans du lidar LD06 : affichage de nuages de points,
grid map, carte vectorielle, puis (jours 2 et 3) scan matching et SLAM.

Compte rendu : [jour 1](docs/jour1.md).

## Prérequis

- Node.js 22 ou plus (testé avec Node 24). Aucune dépendance à installer.
- Un navigateur récent (Chrome, Edge, Firefox).

## Application web

```bash
node serve.mjs
```

Puis ouvrir http://localhost:8080/ (le serveur est nécessaire : les modules JavaScript ne se
chargent pas depuis `file://`).

1. **Jeu simulé** charge les 6 scans de `data/sim/`, ou **Ouvrir des scans…** charge des CSV
   RobotControl, MapTool ou capturés sur le port série (glisser-déposer possible).
2. Sélectionner un scan pour modifier sa pose : champs x, y, θ ; glisser le scan ;
   Maj + glisser pour le tourner ; flèches et Q/A, E au clavier.
3. **Grid map** : créer depuis le scan sélectionné, mettre à jour la carte avec le scan
   sélectionné, ou construire avec tous les scans visibles. Export CSV/PNG, import CSV
   (y compris les grilles MapTool).
4. **Carte vectorielle** : mêmes actions ; export/import TXT (`x1,y1,x2,y2`), carte de
   référence en pointillés (par ex. `data/sim/scene_A.txt` ou une scène MapTool).

## Ligne de commande

```bash
node scripts/build-maps.mjs --out out/sim data/sim/scan_01.csv data/sim/scan_02.csv
```

Construit `grid_map.csv`, `grid_map.png`, `vector_map.txt` et `apercu.png` à partir des scans
(pose lue dans chaque fichier). Options : `--res 0.05`, `--grid carte.csv` et
`--vector carte.txt` pour mettre à jour des cartes existantes, `--ref scene.txt`,
`--no-absence`, `--min-intensity N`.

Autres scripts :

| Commande | Rôle |
| --- | --- |
| `node scripts/gen-sim.mjs` | Régénère le jeu simulé (`data/sim/`) |
| `node scripts/figures-jour1.mjs` | Figures du compte rendu (`docs/img/`) et mesures de précision |

## Tests

```bash
node --test "test/*.test.js"
```

## Structure

| Dossier | Contenu |
| --- | --- |
| `src/core/` | Librairie d'algorithmes (JS pur, utilisée par l'appli web et par Node) |
| `app/` | Application web interactive |
| `scripts/` | Outils en ligne de commande (données simulées, construction de cartes, figures) |
| `test/` | Tests unitaires (`node --test`) |
| `data/` | Scans : `sim/` simulés avec pose connue, `real/` acquis sur le robot |
| `docs/` | Compte rendu et figures |

| Module | Rôle |
| --- | --- |
| `pose.js` | Poses, changements de repère, composition, erreur de pose |
| `scan.js` | Lecture des CSV, filtrage, polaire → cartésien → monde |
| `sim.js` | Simulateur de scans LD06 (lancer de rayons, bruit) |
| `gridmap.js` | Grid map probabiliste (log-odds, Bresenham, mise à jour, nettoyage) |
| `geometry.js` | Segments : angle, proximité, fusion, intersection, découpage |
| `segmentation.js` | Nuage de points → segments (clusters, split-and-merge) |
| `vectormap.js` | Carte vectorielle (fusion, gestion des absences) |
