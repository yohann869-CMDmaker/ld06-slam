# Jour 3 — Banc d'essai de l'exploration autonome (simulation)

Généré par `node scripts/benchmark-explore.mjs`. Le robot part sans carte et choisit seul ses
commandes (méthode des frontières sur la grid map du SLAM). Mouvement vrai : 10 % d'erreur sur
la distance et l'angle, 5 cm de glissement latéral et 1,5° de dérive de cap par mètre ; le SLAM
ne reçoit que la commande. **Couverture** : part du sol libre de la scène (grille de 10 cm, hors
meubles et murs) que la carte a vue libre. **Réussite** : couverture ≥ 95 % sans collision.
**Choix de la frontière** : la plus proche (Yamauchi 1997) ou la plus utile, longueur / (distance + 1 m)
(Holz et al. 2010).

## A. Salle de classe simulée (10 m × 8 m)

| Choix de la frontière | Départ | Pas | Distance | Couverture | Collisions | ATE | Erreur finale | Boucles | Temps / pas | Réussite |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
| la plus proche | (1.5 ; 2.2 ; 0°) | 33 | 8.0 m | **100.0 %** | 0 | 0.7 cm | 1.5 cm | 2 | 79 ms | oui |
| la plus proche | (8 ; 6 ; 180°) | 35 | 8.4 m | **100.0 %** | 0 | 1.9 cm | 3.4 cm | 0 | 56 ms | oui |
| la plus proche | (5 ; 1 ; 90°) | 39 | 10.1 m | **100.0 %** | 0 | 0.7 cm | 1.3 cm | 0 | 96 ms | oui |
| utilité | (1.5 ; 2.2 ; 0°) | 22 | 6.0 m | **100.0 %** | 0 | 1.0 cm | 1.7 cm | 0 | 55 ms | oui |
| utilité | (8 ; 6 ; 180°) | 34 | 8.8 m | **100.0 %** | 0 | 2.7 cm | 4.8 cm | 0 | 54 ms | oui |
| utilité | (5 ; 1 ; 90°) | 39 | 10.1 m | **100.0 %** | 0 | 0.7 cm | 1.3 cm | 0 | 96 ms | oui |

## B. Trois pièces reliées par des portes de 1 m (14 m × 8 m)

| Choix de la frontière | Départ | Pas | Distance | Couverture | Collisions | ATE | Erreur finale | Boucles | Temps / pas | Réussite |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
| la plus proche | (1.5 ; 1.5 ; 0°) | 104 | 26.7 m | **100.0 %** | 0 | 1.1 cm | 1.1 cm | 8 | 147 ms | oui |
| la plus proche | (5.5 ; 6.5 ; -90°) | 120 | 32.8 m | **100.0 %** | 0 | 1.0 cm | 0.6 cm | 12 | 138 ms | oui |
| la plus proche | (12 ; 6.5 ; 180°) | 149 | 34.4 m | **99.6 %** | 0 | 1.0 cm | 0.4 cm | 18 | 124 ms | oui |
| utilité | (1.5 ; 1.5 ; 0°) | 112 | 29.7 m | **100.0 %** | 0 | 1.3 cm | 3.8 cm | 7 | 98 ms | oui |
| utilité | (5.5 ; 6.5 ; -90°) | 144 | 34.6 m | **99.9 %** | 0 | 1.7 cm | 3.1 cm | 15 | 153 ms | oui |
| utilité | (12 ; 6.5 ; 180°) | 125 | 30.1 m | **98.5 %** | 0 | 1.2 cm | 1.7 cm | 13 | 128 ms | oui |

Figures (premier départ de chaque scène) : grid map construite par le SLAM, trajectoire vraie
(gris pointillé) et estimée (bleu) ; sur la vue en cours : zones frontières (vert, taille selon
leur longueur) et chemin prévu vers la plus proche (orange).

| Salle de classe : carte finale | Trois pièces : carte finale |
| --- | --- |
| ![salle](img/j3_explo_salle.png) | ![pièces](img/j3_explo_pieces.png) |

![Trois pièces : exploration en cours](img/j3_explo_pieces_en_cours.png)
