# Jour 2 — Localisation (scan matching)

Objectif (énoncé et présentation du module) : aligner un scan lidar sur une référence, d'abord
à la main puis automatiquement, sur trois types de référence, avec une évaluation chiffrée.
Code : `src/core/localization.js` (méthodes), `src/core/matching.js` (champ de distance, ICP),
application : panneau **Localisation (jour 2)**, banc d'essai : `scripts/benchmark-jour2.mjs`.

## Carte de référence figée

Pour localiser un scan, il faut une carte qui ne bouge plus. Les cartes « des scans visibles »
de l'application sont reconstruites à chaque changement de pose ; la **carte de référence** est
au contraire figée une fois pour toutes et enregistrée dans un fichier. Les nouveaux scans
(pris au hasard dans la salle) sont ensuite localisés sur elle, sans la modifier.

Contenu du fichier `data/reference/carte_reference.json` (module `src/core/refmap.js`) :

| Partie | Contenu |
| --- | --- |
| Grid map | Valeurs 0..255 par cellule (0 libre, 128 inconnu, 255 occupé), compressées par plages de valeurs identiques : 85 000 cellules tiennent en 12 ko |
| Carte vectorielle | Segments `[x1, y1, x2, y2]` en mètres, et coins `[x, y, θ]` |
| Traçabilité | Nom, date, version propre ou brute, scans ayant servi et leurs poses recalées |

Construction de la carte de la salle :

```bash
node scripts/build-reference.mjs
```

Le script relit tous les scans réels de `data/real/` avec leur pose de départ (posée à la main),
les recale automatiquement (multi-résolution + ICP, chaque scan face à tous les autres, en
2 secondes), construit la grid map et la carte vectorielle, garde leur version propre et écrit
la carte, les poses recalées (`data/reference/poses_recalees.csv`) et un aperçu.

### Scans ajoutés pour améliorer la carte (3 novembre)

Aux 7 scans du 2 novembre s'ajoutent 11 scans pris le 3 novembre et exportés depuis
LidarScope (export brut gardé dans `data/real/lidarscope_20261103/`). Import :

```bash
node scripts/import-lidarscope.mjs data/real/lidarscope_20261103 --prefix scan_1103_ls
```

1. **Un seul tour par scan.** 6 des 11 fichiers enchaînent 2 à 3 rotations (693 à 870 points
   pour environ 300 par tour). `singleTour` (`src/core/scan.js`) garde la plus longue suite de
   points, dans l'ordre d'acquisition, qui tient dans 360° : un tour complet sans doublon
   d'angle, et le plus complet si des trames ont été perdues.
2. **Changement de repère.** LidarScope a son propre repère (origine et axes choisis dans
   l'outil). L'export contient aussi notre scan `scan_1102_175527`, réimporté : il sert d'ancre.
   T = (pose dans notre repère) ∘ (pose LidarScope)⁻¹ = (−0,610 m ; −4,672 m ; −89,0°), appliqué
   aux 11 autres scans.
3. **Recalage commun** des 18 scans (`build-reference.mjs`).

Trois départs ont été comparés pour le recalage commun (même algorithme, seules les poses de
départ changent) :

| Départ | Points cohérents (10 cm) | Cohérence à 5 cm | Cellules occupées |
| --- | --- | --- | --- |
| Nos 7 scans à leurs poses recalées d'hier + poses LidarScope | 80 % | 68 % | 1413 |
| Nos 7 scans à leurs poses recalées d'hier + nouveaux scans localisés sur l'ancienne carte | 82 % | 70 % | 1402 |
| **Nos 7 scans à leurs poses manuelles + poses LidarScope** (retenu) | **82 %** | **71 %** | **1376** |

Moins de cellules occupées signifie des murs plus fins, donc mieux superposés. Le départ retenu
donne aussi la carte la plus propre à l'œil : mur du haut en un seul trait, rectangle central
fermé.

**Erreur corrigée dans l'ancienne carte.** Le recalage automatique du 2 novembre avait fait
glisser deux scans du haut de la salle (`180026` et `180105`) d'environ 50 cm le long des murs :
dans cette zone, des murs longs et parallèles laissent la translation mal contrainte. Les
nouveaux scans lèvent l'ambiguïté : en y, ces deux scans reviennent à 9 cm et 0 cm de leurs
poses manuelles (ils en étaient à 62 et 49 cm). Le recalage automatique n'est donc fiable
qu'avec assez de recouvrement entre scans.

| Mesure | 7 scans (2 nov.) | 18 scans (3 nov.) |
| --- | --- | --- |
| Points cohérents entre scans | 45 % → 60 % | 75 % → 82 % |
| Grid map | 279 × 304 cellules de 5 cm | 280 × 304 cellules de 5 cm |
| Carte vectorielle | 31 segments, 9 coins | 50 segments, 17 coins |
| Emprise | 12,6 m × 13,9 m | 12,7 m × 14,1 m |

![Carte de référence de la salle](img/j2_carte_reference.png)

*Carte à 18 scans. Sol blanc, murs gris (grid map propre), segments noirs, coins orange, poses
des scans en bleu. Ancienne carte à 7 scans : [j2_carte_reference_7scans.png](img/j2_carte_reference_7scans.png)
(rectangle central ouvert et décalé).*

Dans l'application, section **Carte de référence (jour 2)** :

- **Charger la carte de la salle** : relit `data/reference/carte_reference.json` ;
- **Charger…** : une autre carte de référence enregistrée ;
- **Figer les cartes actuelles** : les cartes des scans visibles (version propre ou brute)
  deviennent la référence ; **Enregistrer…** la télécharge ;
- la carte s'affiche en fond (sol blanc, murs gris) et le panneau Localisation propose les
  références « Carte de référence : vectorielle » et « Carte de référence : grid map ».

Vérification sur les 18 scans : chacun est placé à 25 cm, 15 cm et 8° de sa pose recalée
(29 cm d'écart), puis localisé sur la carte de référence. Réussite = moins de 5 cm et de 2°.
Les scans ont servi à construire la carte : c'est un contrôle de cohérence, pas encore
l'évaluation sur des scans de test indépendants.

| Référence | Méthode | Réussite (7 scans) | Réussite (18 scans) | Erreur médiane (18 scans) | Temps |
| --- | --- | --- | --- | --- | --- |
| Carte vectorielle | Multi-résolution | 13/18 | **17/18** | 1,0 cm / 0,25° | 87 ms |
| Carte vectorielle | Calcul direct | 13/18 | 10/18 | 1,8 cm / 0,51° | 9 ms |
| Grid map | Multi-résolution | 13/18 | 13/18 | 3,2 cm / 0,25° | 110 ms |
| Grid map | Descente de gradient | 11/18 | 13/18 | 3,3 cm / 0,14° | 9 ms |

- La carte à 18 scans améliore surtout la multi-résolution sur la carte vectorielle : c'est la
  méthode proposée par défaut avec la carte de référence. Le seul échec (`ls02`) est à 5,1 cm.
- Le calcul direct est local : au-delà de sa marge de 25 cm il décroche sur les scans qui
  voient peu de murs (6 à 38 % de leurs points sur la carte). La carte à 50 segments lui
  propose aussi plus de segments voisins où se tromper.
- Sur la grid map, l'erreur médiane de 3 cm vient de l'épaisseur des murs (cellules de 5 cm).
- Peu de points d'un scan tombent sur la carte (40 à 60 %) même quand la pose est juste : la
  carte propre ne garde que les murs, sans les pieds de tables et de chaises.

## Références

| Référence | Représentation utilisée | Construction dans l'application |
| --- | --- | --- |
| Grid map | Centres des cellules occupées | Avec les scans visibles autres que le scan à localiser |
| Carte vectorielle | Segments (échantillonnés tous les 2 cm pour les méthodes à base de points) | Idem, ou carte importée (`Référence…`, par ex. une scène MapTool) |
| Scan de référence | Points du scan précédent dans la liste | Scan pris près de la position du scan à localiser |
| Carte de référence | Grid map ou segments de la carte figée | Chargée depuis un fichier, ne change plus |

Pour chaque référence, un **champ de distance** (distance de chaque cellule au point de
référence le plus proche, transformée de chanfrein en deux passes) donne le score d'une pose :
moyenne de exp(−d²/2σ²) sur les points du scan transformés. C'est le « score basé sur la
distance aux cellules occupées » du cours, lissé par σ.

## Méthodes (document « Scan matching », §8)

| Méthode | Principe | Références |
| --- | --- | --- |
| Force brute | Toutes les poses d'une grille 2 cm × 0,5° dans la fenêtre ±50 cm, ±20° | toutes |
| Multi-résolution | Grille grossière (10 cm, 2°, champ lissé σ = 15 cm), puis fine (1 cm, 0,25°) autour des 3 meilleurs candidats | toutes |
| Descente de gradient | Montée sur le score, gradient numérique sur le champ interpolé, pas adaptatif, σ décroissant 15 → 8 → 4 cm | toutes |
| Filtre particulaire (probabiliste) | 400 hypothèses autour de la pose initiale, pondérées par la vraisemblance, rééchantillonnage systématique, bruit décroissant (recuit) | toutes |
| ICP | Plus proche voisin + transformation rigide en forme close, distance d'association décroissante | toutes |
| Calcul direct | Moindres carrés point-droite : chaque point proche d'un segment donne une équation linéaire en (dx, dy, dθ), système 3 × 3 résolu directement ; passes à marge décroissante 25 → 10 → 4 cm | carte vectorielle |

Le **calcul direct** généralise l'algorithme de localisation absolue de l'article « Enhancing
Robot Navigation with a Low-Cost LIDAR and Absolute Localization Algorithm » (équipe Mars
Attacks!, Coupe de Robotique 2024) : l'article sélectionne les points proches des bords d'une
table rectangulaire connue (boîtes AABB), fait une régression linéaire par paire de bords
parallèles et en déduit (x, y, θ) ; il propose comme amélioration plusieurs passes à marge
décroissante (0,100 → 0,025 → 0,010 m). Ici les segments de la carte sont quelconques et la
régression devient un système point-droite à trois inconnues.

## Évaluation chiffrée

Protocole et résultats complets : [jour2_benchmark.md](jour2_benchmark.md) (données brutes en CSV).
Jeu simulé à pose vraie connue ; mêmes scans et **mêmes poses initiales** pour toutes les méthodes ;
4 niveaux de perturbation de 5 cm / 2° à 45 cm / 18° ; réussite = moins de 5 cm et moins de 2°.

Ce qu'il en ressort, conforme au cours :

- **Force brute et multi-résolution** : 100 % de réussite à tous les niveaux, mais 0,2 à 0,4 s
  par scan ; la précision est limitée par le pas de la grille de recherche.
- **ICP et calcul direct** : les plus précis (≈ 1 mm, 0,02 à 0,1°) et rapides (≈ 10 ms), mais
  locaux : la réussite tombe à 50–75 % quand la pose initiale est à 45 cm / 18°.
- **Descente de gradient** : 5 ms, 98 à 100 % de réussite dans la fenêtre grâce au σ décroissant.
- **Grid map** : erreur médiane ≈ 1 cm, liée à la résolution de 5 cm des cellules.
- **Filtre particulaire** : robuste aux grands écarts mais peu précis seul (≈ 3 cm) : il sert
  d'initialisation à une méthode locale.

Combinaison retenue dans l'application (« Recaler automatiquement ») : recherche
multi-résolution sur une large fenêtre, puis ICP, comme la stratégie « Full Search ICP » puis
« Limited search ICP » de la présentation RoboCup MSL 2022.

## Données réelles

Sur les scans de la salle, la pose vraie n'est pas connue. La référence est la pose recalée à la
main ; son incertitude se mesure en recalant deux fois le même scan à partir de zéro. Le tableau
de comparaison de l'application affiche, pour chaque méthode, la correction appliquée, l'écart
à la pose du fichier et la part des points du scan à moins de 5 cm de la carte.
