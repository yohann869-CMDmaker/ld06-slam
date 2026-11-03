# Jour 2 — Localisation (scan matching)

Objectif (énoncé et présentation du module) : aligner un scan lidar sur une référence, d'abord
à la main puis automatiquement, sur trois types de référence, avec une évaluation chiffrée.
Code : `src/core/localization.js` (méthodes), `src/core/matching.js` (champ de distance, ICP),
application : panneau **Localisation (jour 2)**, banc d'essai : `scripts/benchmark-jour2.mjs`.

## Références

| Référence | Représentation utilisée | Construction dans l'application |
| --- | --- | --- |
| Grid map | Centres des cellules occupées | Avec les scans visibles autres que le scan à localiser |
| Carte vectorielle | Segments (échantillonnés tous les 2 cm pour les méthodes à base de points) | Idem, ou carte importée (`Référence…`, par ex. une scène MapTool) |
| Scan de référence | Points du scan précédent dans la liste | Scan pris près de la position du scan à localiser |

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
