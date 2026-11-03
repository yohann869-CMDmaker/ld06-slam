# Les algorithmes du projet, expliqués

Ce document explique chaque algorithme utilisé : le principe, les étapes, les réglages et
pourquoi ils ont été choisis, les forces et les limites, et où se trouve le code. Les
commentaires du code reprennent les mêmes étapes, numérotées de la même façon.

## Vue d'ensemble

```
scan (angle, distance)  ──► points du robot ──► points du monde ──► grid map / carte vectorielle   (jour 1)
                                     │                                    │
                                     └────────── localisation ◄───────────┘                       (jour 2)
                                         (trouver la pose du robot)
```

| Étape | Fichier | Fonctions |
| --- | --- | --- |
| Lecture et conversion des scans | `src/core/scan.js`, `src/core/pose.js` | `parseScanCSV`, `scanToLocalXY`, `localToWorld` |
| Grid map | `src/core/gridmap.js` | `GridMap.integrateScan`, `bresenham` |
| Segments | `src/core/segmentation.js` | `clusterize`, `extractSegments`, `fitLine` |
| Opérations sur segments | `src/core/geometry.js` | `angleBetweenDeg`, `areClose`, `mergeSegments`, `intersect`, `cutSegment` |
| Carte vectorielle | `src/core/vectormap.js` | `VectorMap.integrateScan` |
| Cartes propres, objets fixes, coins | `src/core/cleanup.js` | `cleanMaps`, `cleanGrid`, `cleanVectorMap`, `classifySegments`, `findCorners` |
| Recalage des scans | `src/core/matching.js` | `DistanceField`, `matchScan`, `icp`, `autoAlign` |
| Localisation (jour 2) | `src/core/localization.js` | `localize` et les six méthodes |

---

## Jour 1 — construire les cartes

### 1. Du scan aux points dans le monde

Le LD06 donne pour chaque mesure un angle a (déjà converti dans le sens antihoraire par la
librairie) et une distance r.

1. Repère du robot : `x = r·cos(a)`, `y = r·sin(a)`.
2. Repère du monde, pour un robot en (X, Y) orienté de θ :
   `x_monde = X + cos θ·x − sin θ·y` et `y_monde = Y + sin θ·x + cos θ·y` (rotation puis translation).
3. Filtrage : on jette `r = 0` (pas d'impact), `r < 15 cm` (reflets sur le robot), `r > 12 m`
   (portée du LD06) et les intensités trop faibles (seuil réglable).

### 2. Grid map probabiliste

**Représentation.** Chaque cellule (5 cm) stocke le *log-odds* `l = ln(p / (1 − p))` de sa
probabilité d'être occupée : l = 0 veut dire p = 0,5 (inconnu). Avantage : la mise à jour
bayésienne devient une simple addition.

**Intégration d'un scan** (`integrateScan`) :

1. chaque point → sa cellule d'impact ;
2. pour chaque faisceau, cellules entre le robot et l'impact (tracé de **Bresenham**, qui
   avance cellule par cellule en suivant l'erreur accumulée par rapport à la vraie droite) :
   `l ← l − 0,40` (vues libres) ;
3. cellule d'impact : `l ← l + 0,85` (vue occupée) ;
4. `l` est borné dans [−4 ; +4] pour que la carte puisse encore changer d'avis (objet déplacé).

**Réglages issus des essais** :

- une cellule n'est mise à jour qu'une fois par scan, et si un faisceau s'y arrête, l'occupation
  l'emporte sur les faisceaux qui la traversent (sinon les murs vus en rasant s'effacent) ;
- rien n'est marqué libre dans les 5 derniers cm avant l'impact (environ 5 fois le bruit du
  lidar) : un impact bruité juste derrière un mur ne libère pas la cellule du mur ;
- faisceaux intermédiaires : entre deux impacts voisins sur une même surface, on trace des
  faisceaux supplémentaires « libres » pour ne pas laisser de hachures loin du robot ;
- classement : occupé si p ≥ 0,6, libre si p ≤ 0,45, inconnu entre les deux.

### 3. Du nuage de points aux segments

1. **Clusters** : on parcourt les points dans l'ordre des angles et on coupe quand deux points
   successifs sont trop éloignés. Le seuil grandit avec la distance (Borges & Aldon) :
   `D = r·sin(dθ) / sin(λ − dθ) + 3σ`, avec λ = 10° (incidence la plus rasante acceptée) et
   σ = 1 cm (bruit). Loin du robot, deux points voisins d'un même mur sont naturellement plus espacés.
2. **Split** (Douglas-Peucker) : dans un cluster, on trace la corde entre le premier et le
   dernier point ; si un point s'en écarte de plus de 3 cm, on coupe le cluster en deux à cet
   endroit (c'est un coin) et on recommence sur chaque moitié.
3. **Ajustement** par moindres carrés totaux : la droite passe par le barycentre et sa
   direction φ vérifie `tan(2φ) = 2·Sxy / (Sxx − Syy)` (S : sommes des écarts au barycentre).
   Contrairement à `y = ax + b`, cela fonctionne aussi pour un mur vertical.
4. **Merge** : deux morceaux voisins qui tiennent sur une même droite sont refusionnés.

Cas particuliers traités : le mur qui contient l'angle 0° est en deux morceaux (début et fin
du tour), qu'on recolle ; si toute la pièce forme une boucle fermée, on la fait recommencer sur un coin.

### 4. Opérations sur les segments

- **Angle entre deux segments** : angle entre leurs droites, ramené dans [0° ; 90°] (un mur
  n'a pas de sens de parcours).
- **Proches** : angle ≤ 10°, écart perpendiculaire ≤ 8 cm et trou le long du mur ≤ 20 cm.
- **Fusion** : direction moyenne pondérée par le nombre de points, calculée sur l'angle double
  (sinon 0° et 180° donneraient 90°) ; la droite passe par le barycentre des milieux ; la
  longueur va de la plus petite à la plus grande projection des quatre extrémités.
- **Intersection** : on résout `P1 + t·r = Q1 + u·s` ; t et u entre 0 et 1 = sur les deux segments.
- **Découpage** : seulement si l'intersection est sur les deux segments et strictement à
  l'intérieur de celui qu'on coupe (tolérance 1 µm) ; on garde le plus long côté, celui de P1
  en cas d'égalité ; jamais pour des segments colinéaires.

### 5. Carte vectorielle

Pour chaque scan :

1. **Absences** : chaque faisceau (raccourci de 15 cm avant son impact) qui traverse un segment
   de la carte prouve que ce segment n'existe plus à cet endroit : on le découpe ;
2. extraction des segments du scan (§3) ;
3. chaque nouveau segment est fusionné avec les segments proches, sinon ajouté ;
4. les segments de moins de 15 cm sont supprimés.

### 6. Cartes propres et points d'intérêt

- **Grid map propre** : retrait des groupes de moins de 3 cellules occupées ; *ouverture
  morphologique* de l'espace libre (érosion puis dilatation de 2 cellules) qui efface les
  faisceaux fins sortis par les fenêtres (le lidar ne voit pas les vitres) ; on ne garde que les
  zones libres reliées à une position du robot ; petits trous bouchés ; murs fermés le long du sol.
- **Carte vectorielle propre** : direction dominante des murs (histogramme des orientations
  pondéré par la longueur) ; segments à moins de 7° redressés sur elle ou sa perpendiculaire ;
  fusion ; coins fermés ; segments hors de la pièce retirés ; oubli des segments vus par trop
  peu de scans (option).
- **Murs et objets fixes** (`classifySegments`, `drawObjects`, chaîne complète `cleanMaps`) :
  la grid map efface les meubles, car des faisceaux d'autres scans passent dessous ou à côté.
  Les segments, eux, rassemblent les points de tous les scans qui ont vu le meuble.
  1. *Extérieur* de la salle : cellules inconnues reliées au bord de la carte (remplissage qui
     s'arrête sur les murs et le sol). L'intérieur d'une table, entouré de sol, n'en fait pas partie.
  2. *Mur* : segment dont la moitié au moins est à moins de 25 cm de l'extérieur.
  3. *Objet fixe* : autre segment vu par au moins 2 scans, ou qui forme un coin avec un segment
     gardé (face d'un meuble vue une seule fois).
  4. Le reste (vu par un seul scan, isolé : chaise, sac, personne) est retiré.
  5. Les objets fixes sont redessinés dans la grid map propre (Bresenham le long du segment).
  Les cartes propres sont construites **sans découpe par les absences** : la découpe efface les
  meubles traversés par des faisceaux avant qu'ils aient pu être reconnus.
- **Coins** : intersection de deux murs presque perpendiculaires dont les extrémités se
  rejoignent, avec la direction de la bissectrice (point d'intérêt orienté du cours).

---

## Jour 2 — se localiser

### 7. Le problème

On connaît une **référence** (grid map, carte vectorielle ou scan de référence) et un **scan** dans
le repère du robot. On cherche la pose (x, y, θ) telle que les points du scan, ramenés dans le
monde, tombent sur la référence. Les quatre étapes du cours : transformation, correspondance,
score, optimisation.

### 8. Le champ de distance et le score (base commune)

**Champ de distance** : une grille qui donne, pour chaque cellule, la distance au point de
référence le plus proche. Elle se calcule une seule fois par deux balayages (transformée de
chanfrein) : chaque cellule prend le minimum entre sa valeur et celle d'un voisin déjà vu, plus
la distance à ce voisin. Ensuite, la distance d'un point du scan à la référence n'est plus
qu'une lecture de case (interpolée entre les cases voisines).

**Score d'une pose** : moyenne sur les points du scan de `exp(−d² / 2σ²)`. Un point exactement
sur la référence rapporte 1, un point loin rapporte 0. σ règle la tolérance : grand σ = score
lisse, facile à optimiser mais peu précis ; petit σ = précis.

Pour la grid map, la référence est l'ensemble des cellules occupées ; pour la carte vectorielle,
des points pris tous les 2 cm sur les segments ; pour un scan, ses points.

### 9. Les six méthodes

| Méthode | Comment elle cherche | Points forts | Limites |
| --- | --- | --- | --- |
| **Force brute** | Teste toutes les poses d'une grille 2 cm × 0,5° dans ±50 cm, ±20° (≈ 210 000 poses) et garde la meilleure | Ne se trompe pas de maximum dans la fenêtre | Lente (≈ 0,4 s), précision = pas de la grille |
| **Multi-résolution** | Grille grossière (10 cm, 2°, score lissé), puis fine (1 cm, 0,25°) autour des 3 meilleurs candidats | Aussi robuste, 2 à 3 fois plus rapide, plus précise | Encore 0,2 s |
| **Descente de gradient** | Mesure la pente du score en x, y, θ et avance dans le sens de la montée ; pas qui s'allonge si ça monte, raccourcit sinon ; σ de 15 puis 8 puis 4 cm | ≈ 5 ms | Locale : peut s'arrêter sur un faux maximum |
| **Filtre particulaire** | 400 hypothèses de pose pondérées par la vraisemblance du scan, rééchantillonnées, agitées d'un bruit qui diminue (recuit), 15 fois | Garde plusieurs hypothèses : robuste aux pièges | Peu précise seule (≈ 3 cm) |
| **ICP** | Apparie chaque point à son plus proche voisin, calcule directement la meilleure rotation et translation, recommence | ≈ 1 mm, ≈ 10 ms | Locale |
| **Calcul direct** | Moindres carrés point-droite sur la carte vectorielle (ci-dessous) | ≈ 1 mm, ≈ 7 ms, sans grille de recherche | Locale ; impossible si un seul mur est visible |

**Formule de l'ICP.** Avec les couples (Pk du scan, Qk de la référence), leurs barycentres P̄ et
Q̄ et les écarts a = Pk − P̄, b = Qk − Q̄ :
`dθ = atan2(Σ(ax·by − ay·bx), Σ(ax·bx + ay·by))` et `t = Q̄ − R(dθ)·P̄`.

**Calcul direct (inspiré de l'article du cours).** L'article « Mars Attacks! » (Coupe de
Robotique 2024) localise un robot sur une table rectangulaire : il garde les points proches de
chaque bord, fait une droite de régression par paire de bords parallèles et lit directement
l'erreur (x, y, θ) dans la pente et l'ordonnée à l'origine. Il suggère plusieurs passes avec
une marge de plus en plus petite. Généralisation à des murs quelconques :

1. chaque point du scan est associé au mur le plus proche, s'il en est à moins de la marge ;
2. sa distance signée au mur est `r = n·(w − a)` (n : normale du mur, w : le point, a : un point du mur) ;
3. une petite correction (dx, dy, dθ) change cette distance en `r + nx·dx + ny·dy + dθ·n·(−(wy − y), wx − x)` ;
4. on cherche la correction qui rend toutes ces distances les plus petites possibles :
   c'est un système linéaire 3 × 3, résolu directement ;
5. on recommence quatre fois, puis on resserre la marge : 25 cm, 10 cm, puis 4 cm.

### 10. Recalage automatique des scans entre eux

Bouton « Recaler automatiquement » (`autoAlign`) : le premier scan sert de repère.

1. Un scan encore en (0, 0, 0) est placé par **multi-résolution sur une large fenêtre**
   (±2,5 m, orientation quelconque) contre les scans déjà placés, puis affiné par **ICP**.
2. Tous les scans sont ensuite affinés localement (±50 cm, ±15°) face à **tous les autres**,
   en deux passes, ce qui répartit les petites erreurs au lieu de les accumuler.

C'est la stratégie de la présentation RoboCup MSL 2022 : une recherche complète pour
l'initialisation, puis une recherche limitée.

### 11. Évaluation (exigée par l'énoncé du jour 2)

`scripts/benchmark-jour2.mjs` : scans simulés à pose vraie connue ; pour chaque scan, les
cartes sont construites avec les autres scans ; mêmes poses initiales pour toutes les méthodes,
de 5 cm / 2° à 45 cm / 18° d'écart ; réussite = moins de 5 cm et moins de 2°. Résultats et
analyse : [jour2.md](jour2.md) et [jour2_benchmark.md](jour2_benchmark.md).
