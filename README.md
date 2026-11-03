# LD06 SLAM — Cartographie et localisation 2D

Projet du module *Cartographie et localisation appliquée à la robotique mobile*.
Outils pour exploiter les scans du lidar LD06 : affichage de nuages de points,
grid map, carte vectorielle, puis (jours 2 et 3) scan matching et SLAM.

Comptes rendus : [jour 1](docs/jour1.md), [jour 2](docs/jour2.md) ([banc d'essai](docs/jour2_benchmark.md)), [journal des erreurs et corrections](docs/journal.md), [algorithmes expliqués](docs/algorithmes.md).

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

## Acquisition des scans avec le programme de test fourni (Teleplot)

Le programme `ESP32_LD06_Test.ino` du module envoie chaque tour sur le port série sous la
forme `>lidar:x:y;x:y;…|xy` (format Teleplot). L'application lit directement ce texte :

1. Téléverser `ESP32_LD06_Test.ino` et ouvrir le moniteur série de l'IDE Arduino à 115200 bauds.
2. Poser le robot, attendre 2 à 3 secondes sans le toucher.
3. Cliquer dans la zone de sortie du moniteur, **Ctrl+A** puis **Ctrl+C**.
4. Dans l'application, **Ctrl+V** sur la carte (ou bouton **Coller un scan**). Le nouveau scan
   apparaît dans sa couleur.
5. Déplacer le robot (0,5 à 1 m, moins de 45° de rotation) et recommencer à l'étape 2.

L'application garde la dernière ligne complète (celle coupée par la copie est ignorée) et
corrige le miroir des x/y de la librairie. Le programme de test perd des trames, car il envoie
du texte plus lentement que le lidar ne produit ses mesures : chaque tour a des trous (jusqu'à
74° sur un tour réel). Un scan correspond toujours à **un seul tour** : fusionner plusieurs
tours biaiserait le scan dès que le robot a bougé ou tourné, même un peu, entre deux tours.

Ensuite, pour chaque scan : recalage manuel et enregistrement, comme aux étapes 5 et 6 ci-dessous.

## Acquisition des scans par USB (programme de capture)

Variante sans perte de trames : un programme qui n'envoie un scan que sur demande.

1. Dans l'IDE Arduino (librairie `LD06.zip` installée, carte « ESP32 Dev Module »), téléverser
   [`firmware/ESP32_LD06_Capture/ESP32_LD06_Capture.ino`](firmware/ESP32_LD06_Capture/ESP32_LD06_Capture.ino).
   Même câblage que `ESP32_LD06_Test.ino` : TX du lidar sur GPIO33, PWM sur GPIO32.
2. **Fermer le moniteur série de l'IDE** (le port USB ne peut être ouvert que par un programme).
3. Dans l'application ouverte avec **Chrome ou Edge**, cliquer sur **Connecter le robot (USB)**
   et choisir le port de l'ESP32. La ligne d'état affiche la vitesse du lidar et le nombre de
   points par tour.
4. Poser le robot, ne plus le toucher, cliquer sur **Capturer un scan**. Recommencer après
   chaque déplacement (0,5 à 1 m, moins de 45° de rotation entre deux scans).
5. Le premier scan est la référence, en pose (0, 0, 0). Chaque nouveau scan part de la pose
   du scan sélectionné : le recaler à la main (glisser, Maj + glisser, flèches, Q/A et E)
   jusqu'à ce que ses murs se superposent aux précédents. Sa pose x, y, θ est alors lue
   dans l'éditeur de pose.
6. **Enregistrer tous les scans…** écrit chaque scan avec sa pose dans un dossier
   (par exemple `data/real/`) ainsi qu'un récapitulatif `poses.csv`. Les fichiers se
   rechargent avec **Ouvrir des scans…** en gardant leurs poses.

Le programme peut aussi être piloté depuis le moniteur série (115200 bauds) :
`s` envoie un scan, `i` affiche la vitesse, le nombre de points et les erreurs CRC,
`r` compte les octets bruts reçus du lidar en 300 ms (diagnostic de câblage ou d'alimentation).

## Cartes propres et exports du jour 1

- **Vue propre (style MapTool)** (section Affichage) : grid map nettoyée (poussières, faisceaux
  sortis par les fenêtres et petits trous retirés) et carte vectorielle nettoyée (murs redressés
  sur la direction dominante, morceaux fusionnés, coins fermés, segments hors de la pièce retirés).
- **Exporter tout le jour 1** : 7 figures PNG avec titre, échelle et légende (nuages de points,
  grid map brute et propre, segments extraits, carte vectorielle brute et propre, plan de la
  salle), les grid maps (CSV) et cartes vectorielles (TXT) brutes et propres, `poses.csv`, et deux
  fichiers pour MapTool (`maptool_carte_vectorielle.txt`, `maptool_grid_map.csv`). Pour les
  ouvrir dans MapTool, régler la largeur et la hauteur indiquées en tête du fichier, puis importer.

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
