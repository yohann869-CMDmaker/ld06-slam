# Localisation globale : banc d'essai

Généré par `node scripts/benchmark-global.mjs`. Chaque scan est localisé **sans pose de départ**
sur une carte construite **sans lui** (grid map de localisation des autres scans). Réussite :
moins de 5 cm et moins de 2°. Scans réels : pose de référence = pose recalée
(incertitude de 2 à 5 cm) ; cartes faites avec les scans pris au sol.

| Jeu | Scans | Variante | Réussite | Erreur médiane (réussites) | Poses testées | Temps moyen | Ambiguës signalées |
| --- | --- | --- | --- | --- | --- | --- | --- |
| simulé | tous (6) | orientations des murs (4 × 9 angles) | 6/6 (100 %) | 0.7 cm / 0.17° | 29 196 | 742 ms | 0 |
| simulé | tous (6) | tous les angles (180) | 6/6 (100 %) | 1.6 cm / 0.25° | 145 980 | 908 ms | 0 |
| salle | au sol (13) | orientations des murs (4 × 9 angles) | 10/13 (77 %) | 1.3 cm / 0.32° | 75 888 | 799 ms | 7 |
| salle | au sol (13) | tous les angles (180) | 9/13 (69 %) | 1.4 cm / 0.19° | 379 440 | 1637 ms | 5 |
| salle | en hauteur (5) | orientations des murs (4 × 9 angles) | 5/5 (100 %) | 1.6 cm / 0.12° | 77 148 | 792 ms | 0 |
| salle | en hauteur (5) | tous les angles (180) | 5/5 (100 %) | 1.9 cm / 0.18° | 385 740 | 1507 ms | 0 |

## Confiance

| Jeu | Variante | Réponses fiables : justes | Réponses ambiguës : justes | Échecs signalés ambigus |
| --- | --- | --- | --- | --- |
| simulé | orientations des murs (4 × 9 angles) | 6/6 | 0/0 | 0/0 |
| simulé | tous les angles (180) | 6/6 | 0/0 | 0/0 |
| salle | orientations des murs (4 × 9 angles) | 11/11 | 4/7 | 3/3 |
| salle | tous les angles (180) | 11/13 | 3/5 | 2/4 |

## Détail par scan

Confiance : points qui départagent la 1re et la 2e candidate (sur la carte pour l'une et pas
pour l'autre). Ambiguë si la 2e en a au moins moitié autant que la 1re, ou si moins de 8 points départagent.

| Jeu | Scan | Variante | Erreur | Points sur la carte | Points 1re / 2e | Temps |
| --- | --- | --- | --- | --- | --- | --- |
| simulé | scan_01.csv | murs | 0.7 cm / 0.2° | 81 % | 155 / 68 | 532 ms |
| simulé | scan_01.csv | tous | 0.7 cm / 0.0° | 82 % | 156 / 68 | 581 ms |
| simulé | scan_02.csv | murs | 1.6 cm / 0.0° | 82 % | 173 / 46 | 793 ms |
| simulé | scan_02.csv | tous | 1.6 cm / 0.0° | 82 % | 173 / 46 | 610 ms |
| simulé | scan_03.csv | murs | 1.6 cm / 0.1° | 91 % | 113 / 5 | 681 ms |
| simulé | scan_03.csv | tous | 3.8 cm / 0.5° | 93 % | 122 / 6 | 1065 ms |
| simulé | scan_04.csv | murs | 0.7 cm / 0.1° | 90 % | 161 / 18 | 634 ms |
| simulé | scan_04.csv | tous | 1.6 cm / 0.3° | 88 % | 155 / 19 | 1036 ms |
| simulé | scan_05.csv | murs | 0.7 cm / 0.3° | 97 % | 153 / 0 | 942 ms |
| simulé | scan_05.csv | tous | 0.7 cm / 0.3° | 97 % | 153 / 0 | 1058 ms |
| simulé | scan_06.csv | murs | 0.7 cm / 0.2° | 96 % | 128 / 13 | 867 ms |
| simulé | scan_06.csv | tous | 0.7 cm / 0.3° | 96 % | 126 / 13 | 1099 ms |
| salle | scan_1102_175527.csv (en hauteur) | murs | 1.6 cm / 0.4° | 89 % | 124 / 29 | 987 ms |
| salle | scan_1102_175527.csv (en hauteur) | tous | 1.6 cm / 0.4° | 89 % | 124 / 29 | 1850 ms |
| salle | scan_1102_175930.csv (en hauteur) | murs | 2.8 cm / 0.1° | 79 % | 131 / 14 | 929 ms |
| salle | scan_1102_175930.csv (en hauteur) | tous | 2.8 cm / 0.2° | 78 % | 133 / 15 | 1727 ms |
| salle | scan_1102_180026.csv | murs | **échec** 6.8 cm / 0.7° | 52 % | 122 / 73 (ambiguë) | 908 ms |
| salle | scan_1102_180026.csv | tous | **échec** 6.8 cm / 0.7° | 52 % | 161 / 83 (ambiguë) | 1737 ms |
| salle | scan_1102_180105.csv | murs | 4.5 cm / 0.5° | 36 % | 53 / 45 (ambiguë) | 945 ms |
| salle | scan_1102_180105.csv | tous | 4.5 cm / 0.5° | 36 % | 53 / 45 (ambiguë) | 1748 ms |
| salle | scan_1102_180212.csv | murs | 2.6 cm / 0.1° | 40 % | 76 / 72 (ambiguë) | 1066 ms |
| salle | scan_1102_180212.csv | tous | **échec** 547.1 cm / 89.8° | 39 % | 72 / 33 | 1950 ms |
| salle | scan_1102_180308.csv | murs | 0.9 cm / 0.1° | 56 % | 35 / 14 | 968 ms |
| salle | scan_1102_180308.csv | tous | 0.4 cm / 0.1° | 56 % | 33 / 13 | 1711 ms |
| salle | scan_1102_180357.csv | murs | 2.9 cm / 0.1° | 35 % | 128 / 78 (ambiguë) | 900 ms |
| salle | scan_1102_180357.csv | tous | 2.9 cm / 0.0° | 35 % | 121 / 68 (ambiguë) | 1566 ms |
| salle | scan_1103_ls01.csv | murs | 1.2 cm / 0.5° | 48 % | 103 / 20 | 705 ms |
| salle | scan_1103_ls01.csv | tous | 1.4 cm / 0.2° | 48 % | 111 / 52 | 1666 ms |
| salle | scan_1103_ls02.csv | murs | **échec** 1453.4 cm / 87.4° | 70 % | 50 / 41 (ambiguë) | 758 ms |
| salle | scan_1103_ls02.csv | tous | **échec** 1453.4 cm / 87.4° | 70 % | 50 / 41 (ambiguë) | 1489 ms |
| salle | scan_1103_ls03.csv | murs | **échec** 53.3 cm / 0.4° | 47 % | 34 / 20 (ambiguë) | 714 ms |
| salle | scan_1103_ls03.csv | tous | **échec** 49.6 cm / 1.1° | 48 % | 20 / 6 | 1573 ms |
| salle | scan_1103_ls05.csv | murs | 1.2 cm / 0.6° | 52 % | 67 / 14 | 590 ms |
| salle | scan_1103_ls05.csv | tous | 1.2 cm / 0.6° | 52 % | 66 / 14 | 1558 ms |
| salle | scan_1103_ls06.csv | murs | 1.1 cm / 0.2° | 50 % | 70 / 25 | 808 ms |
| salle | scan_1103_ls06.csv | tous | 1.1 cm / 0.2° | 50 % | 70 / 25 | 1683 ms |
| salle | scan_1103_ls07.csv | murs | 1.4 cm / 0.3° | 47 % | 54 / 41 (ambiguë) | 684 ms |
| salle | scan_1103_ls07.csv | tous | 1.4 cm / 0.3° | 47 % | 53 / 42 (ambiguë) | 1616 ms |
| salle | scan_1103_ls08.csv (en hauteur) | murs | 1.9 cm / 0.5° | 67 % | 41 / 5 | 718 ms |
| salle | scan_1103_ls08.csv (en hauteur) | tous | 1.9 cm / 0.5° | 67 % | 41 / 5 | 1326 ms |
| salle | scan_1103_ls09.csv (en hauteur) | murs | 0.5 cm / 0.0° | 93 % | 98 / 15 | 744 ms |
| salle | scan_1103_ls09.csv (en hauteur) | tous | 1.9 cm / 0.0° | 91 % | 98 / 22 | 1665 ms |
| salle | scan_1103_ls10.csv | murs | 1.3 cm / 0.9° | 66 % | 58 / 7 | 609 ms |
| salle | scan_1103_ls10.csv | tous | 1.8 cm / 1.0° | 67 % | 61 / 7 | 1660 ms |
| salle | scan_1103_ls11.csv | murs | 1.3 cm / 0.3° | 69 % | 51 / 19 | 739 ms |
| salle | scan_1103_ls11.csv | tous | 1.6 cm / 0.1° | 69 % | 52 / 20 | 1324 ms |
| salle | scan_1103_ls12.csv (en hauteur) | murs | 0.7 cm / 0.1° | 79 % | 76 / 11 | 581 ms |
| salle | scan_1103_ls12.csv (en hauteur) | tous | 0.7 cm / 0.1° | 79 % | 73 / 20 | 968 ms |
