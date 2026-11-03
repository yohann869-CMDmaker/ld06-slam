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
| Grid map | Valeurs 0..255 par cellule (0 libre, 128 inconnu, 255 occupé), compressées par plages de valeurs identiques : 85 000 cellules tiennent en 10 ko |
| Carte vectorielle | Segments `[x1, y1, x2, y2]` en mètres, et coins `[x, y, θ]` |
| Traçabilité | Nom, date, version propre ou brute, scans ayant servi et leurs poses recalées |

Construction de la carte de la salle :

```bash
node scripts/build-reference.mjs
```

Le script relit les 7 scans réels de `data/real/` avec leurs poses manuelles, les recale
automatiquement (multi-résolution + ICP, chaque scan face à tous les autres), construit la grid
map et la carte vectorielle, garde leur version propre et écrit la carte, les poses recalées
(`data/reference/poses_recalees.csv`) et un aperçu.

| Mesure | Valeur |
| --- | --- |
| Points cohérents entre scans | 45 % (poses manuelles) → 60 % (poses recalées) |
| Grid map | 279 × 304 cellules de 5 cm |
| Carte vectorielle | 31 segments, 9 coins |
| Emprise | 12,6 m × 13,9 m |

![Carte de référence de la salle](img/j2_carte_reference.png)

*Sol blanc, murs gris (grid map propre), segments noirs, coins orange, poses des 7 scans en bleu.*

Dans l'application, section **Carte de référence (jour 2)** :

- **Charger la carte de la salle** : relit `data/reference/carte_reference.json` ;
- **Charger…** : une autre carte de référence enregistrée ;
- **Figer les cartes actuelles** : les cartes des scans visibles (version propre ou brute)
  deviennent la référence ; **Enregistrer…** la télécharge ;
- la carte s'affiche en fond (sol blanc, murs gris) et le panneau Localisation propose les
  références « Carte de référence : vectorielle » et « Carte de référence : grid map ».

Vérification : scan `scan_1102_180105` placé à 25 cm, 15 cm et 8° de sa pose recalée, puis
localisé par calcul direct sur la carte vectorielle de référence : arrivée à 0,6 cm et 0,25° de
la pose recalée, en 18 ms. Seuls 42 % des points du scan tombent à moins de 5 cm d'un segment :
la carte propre ne garde que les murs, les pieds de tables et de chaises n'y sont pas.

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
