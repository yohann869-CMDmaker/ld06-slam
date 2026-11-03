# Jour 2 — Banc d'essai sur données réelles

Généré par `node scripts/benchmark-reel.mjs`. Même protocole que le banc d'essai simulé
([jour2_benchmark.md](jour2_benchmark.md)) : 4 niveaux de perturbation de la pose initiale, 2 tirages par niveau,
fenêtre ±50 cm / ±20° ; **réussite** : moins de 5 cm **et** moins de 2°.

## A. Scans de la carte (13 scans pris au sol, chacun sur une carte construite sans lui)

Pose de référence : pose recalée par le recalage commun des 18 scans. Références : grid map de
localisation et carte vectorielle propre (murs et meubles nommés) des autres scans au sol, scan
de référence = scan au sol le plus proche.

| Référence | Méthode | Réussite | Erreur position médiane | Erreur angle médiane | Temps moyen |
| --- | --- | --- | --- | --- | --- |
| Grid map de localisation | Force brute | 77 % | 3.2 cm | 0.50° | 339 ms |
| Grid map de localisation | Multi-résolution | 85 % | 2.6 cm | 0.25° | 122 ms |
| Grid map de localisation | Descente de gradient | 76 % | 2.8 cm | 0.45° | 5 ms |
| Grid map de localisation | Filtre particulaire (probabiliste) | 22 % | 7.1 cm | 1.22° | 38 ms |
| Grid map de localisation | ICP | 56 % | 4.4 cm | 0.77° | 10 ms |
| Carte vectorielle | Calcul direct (moindres carrés point-droite) | 57 % | 3.1 cm | 0.61° | 5 ms |
| Carte vectorielle | Force brute | 80 % | 2.0 cm | 0.50° | 362 ms |
| Carte vectorielle | Multi-résolution | 78 % | 1.7 cm | 0.25° | 127 ms |
| Carte vectorielle | Descente de gradient | 74 % | 1.9 cm | 0.50° | 6 ms |
| Carte vectorielle | Filtre particulaire (probabiliste) | 37 % | 7.1 cm | 1.28° | 38 ms |
| Carte vectorielle | ICP | 56 % | 3.6 cm | 0.61° | 8 ms |
| Scan de référence | ICP | 49 % | 4.2 cm | 1.33° | 7 ms |
| Scan de référence | Force brute | 88 % | 2.1 cm | 0.50° | 347 ms |
| Scan de référence | Multi-résolution | 86 % | 1.6 cm | 0.50° | 121 ms |
| Scan de référence | Descente de gradient | 71 % | 2.3 cm | 0.58° | 9 ms |
| Scan de référence | Filtre particulaire (probabiliste) | 18 % | 8.3 cm | 1.43° | 39 ms |

### Sensibilité à l'initialisation (taux de réussite)

| Référence | Méthode | 5 cm / 2° | 15 cm / 5° | 30 cm / 10° | 45 cm / 18° |
| --- | --- | --- | --- | --- | --- |
| Grid map de localisation | Force brute | 69 % | 77 % | 77 % | 85 % |
| Grid map de localisation | Multi-résolution | 85 % | 85 % | 85 % | 85 % |
| Grid map de localisation | Descente de gradient | 92 % | 85 % | 77 % | 50 % |
| Grid map de localisation | Filtre particulaire (probabiliste) | 27 % | 19 % | 19 % | 23 % |
| Grid map de localisation | ICP | 92 % | 85 % | 46 % | 0 % |
| Carte vectorielle | Calcul direct (moindres carrés point-droite) | 85 % | 85 % | 54 % | 4 % |
| Carte vectorielle | Force brute | 77 % | 77 % | 81 % | 85 % |
| Carte vectorielle | Multi-résolution | 77 % | 77 % | 77 % | 81 % |
| Carte vectorielle | Descente de gradient | 85 % | 85 % | 77 % | 50 % |
| Carte vectorielle | Filtre particulaire (probabiliste) | 38 % | 23 % | 42 % | 42 % |
| Carte vectorielle | ICP | 85 % | 88 % | 42 % | 8 % |
| Scan de référence | ICP | 88 % | 69 % | 35 % | 4 % |
| Scan de référence | Force brute | 88 % | 88 % | 88 % | 88 % |
| Scan de référence | Multi-résolution | 85 % | 85 % | 85 % | 88 % |
| Scan de référence | Descente de gradient | 88 % | 92 % | 77 % | 27 % |
| Scan de référence | Filtre particulaire (probabiliste) | 35 % | 19 % | 15 % | 4 % |

### Incertitude des poses de référence réelles

Chaque scan est relocalisé (multi-résolution puis ICP), **depuis sa pose recalée**, sur la grid map
construite sans lui : le déplacement mesure le désaccord entre cette carte indépendante et le recalage
commun, donc l'incertitude des poses de référence. La pose manuelle (posée à la main, sans mesure)
est donnée pour comparaison.

| Scan | Désaccord carte sans le scan | Pose manuelle / pose recalée |
| --- | --- | --- |
| scan_1102_180026.csv | 6.1 cm / 0.50° | 9.5 cm / 0.7° |
| scan_1102_180105.csv | 2.2 cm / 0.25° | 18.2 cm / 0.9° |
| scan_1102_180212.csv | 3.6 cm / 0.00° | 19.8 cm / 2.0° |
| scan_1102_180308.csv | 1.4 cm / 0.22° | 13.9 cm / 1.4° |
| scan_1102_180357.csv | 2.0 cm / 0.00° | 3.6 cm / 0.3° |
| scan_1103_ls01.csv | 1.4 cm / 0.17° | 4.4 cm / 1.0° |
| scan_1103_ls02.csv | 4.1 cm / 1.25° | 8.4 cm / 1.3° |
| scan_1103_ls03.csv | 49.5 cm / 0.51° | 3.9 cm / 0.0° |
| scan_1103_ls05.csv | 1.2 cm / 0.64° | 7.1 cm / 0.5° |
| scan_1103_ls06.csv | 3.2 cm / 0.25° | 24.4 cm / 0.8° |
| scan_1103_ls07.csv | 1.4 cm / 0.32° | 8.5 cm / 1.4° |
| scan_1103_ls10.csv | 1.3 cm / 0.91° | 2.2 cm / 1.5° |
| scan_1103_ls11.csv | 1.3 cm / 0.28° | 1.8 cm / 1.1° |
| **médiane** | **2.0 cm / 0.28°** | 8.4 cm / 1.0° |
| maximum | 49.5 cm / 1.25° | 24.4 cm / 2.0° |

## B. Jeu de test indépendant (16 scans du 2 novembre, `data/test/`)

Scans jamais utilisés pour la carte. **Localisation globale** (sans pose de départ) sur la carte de
référence, puis comparaison aux poses posées à la main dans LidarScope, ramenées dans notre repère
(changement de repère estimé sur les réponses fiables : (-0.599 m ; -2.669 m ; -173.80°)).
Un placement à la main est approximatif (de l'ordre de 20 à 30 cm) : cette comparaison vérifie que
la bonne place est trouvée, pas la précision au centimètre.

| Scan | Pris | Pose trouvée | Points sur la carte | Confiance (points 1re / 2e) | Écart à la pose LidarScope | Temps |
| --- | --- | --- | --- | --- | --- | --- |
| scan_1102_test01.csv | en hauteur ? (détecté) | (-2.67 ; 1.13 ; 98.6°) | 57 % | fiable (41 / 8) | 40.7 cm / 1.6° | 468 ms |
| scan_1102_test02.csv | en hauteur ? (détecté) | (-2.67 ; 1.13 ; 98.6°) | 67 % | fiable (73 / 3) | 21.0 cm / 2.4° | 537 ms |
| scan_1102_test03.csv | au sol | (0.20 ; 5.35 ; -177.4°) | 29 % | **ambiguë** (51 / 41) | 926.9 cm / 3.6° | 481 ms |
| scan_1102_test04.csv | au sol | (-4.65 ; 4.96 ; 5.8°) | 50 % | **ambiguë** (61 / 35) | 1175.9 cm / 179.6° | 559 ms |
| scan_1102_test05.csv | au sol | (0.59 ; -0.64 ; -83.6°) | 54 % | **ambiguë** (22 / 19) | 607.2 cm / 90.2° | 433 ms |
| scan_1102_test06.csv | au sol | (-4.86 ; -0.89 ; -172.7°) | 48 % | fiable (57 / 17) | 15.7 cm / 1.1° | 614 ms |
| scan_1102_test07.csv | au sol | (-7.35 ; -4.78 ; -169.2°) | 62 % | fiable (34 / 2) | 13.7 cm / 0.4° | 476 ms |
| scan_1102_test08.csv | au sol | (-3.79 ; 3.84 ; -171.8°) | 48 % | fiable (67 / 5) | 21.4 cm / 2.0° | 463 ms |
| scan_1102_test09.csv | au sol | (-3.79 ; 3.83 ; -171.5°) | 48 % | fiable (69 / 10) | 31.5 cm / 0.7° | 433 ms |
| scan_1102_test10.csv | au sol | (-4.75 ; 3.17 ; -178.5°) | 42 % | **ambiguë** (60 / 32) | 14.0 cm / 2.3° | 453 ms |
| scan_1102_test12.csv | au sol | (-0.98 ; 2.47 ; -172.5°) | 46 % | fiable (51 / 16) | 30.7 cm / 1.3° | 417 ms |
| scan_1102_test13.csv | au sol | (-1.14 ; -0.67 ; -173.2°) | 47 % | fiable (68 / 12) | 8.4 cm / 1.4° | 569 ms |
| scan_1102_test14.csv | en hauteur ? (détecté) | (0.05 ; 0.13 ; -178.4°) | 85 % | fiable (78 / 18) | 4.8 cm / 0.4° | 672 ms |
| scan_1102_test15.csv | en hauteur ? (détecté) | (-1.05 ; -3.32 ; -165.1°) | 86 % | fiable (74 / 14) | 21.7 cm / 1.3° | 555 ms |
| scan_1102_test16.csv | en hauteur ? (détecté) | (-0.70 ; -2.83 ; -176.2°) | 87 % | fiable (68 / 12) | 19.2 cm / 2.4° | 378 ms |
| scan_1102_test17.csv | en hauteur ? (détecté) | (-0.69 ; -2.81 ; -175.8°) | 87 % | fiable (66 / 7) | 16.1 cm / 0.0° | 435 ms |

Réponses fiables : 12/16, toutes à moins de 40.7 cm et 2.4° de la pose LidarScope (médiane 21.0 cm). Réponses ambiguës : 4, dont 3 à plus de 1 m de la pose LidarScope.

### Méthodes locales autour de la pose trouvée (réponses fiables)

Pose de départ perturbée autour de la pose trouvée par la localisation globale ; réussite = retour à
moins de 5 cm et 2° de cette pose (convergence et répétabilité, faute de pose vraie mesurée).

| Référence | Méthode | Réussite | Écart médian | Écart angle médian | Temps moyen |
| --- | --- | --- | --- | --- | --- |
| Grid map de localisation | Force brute | 100 % | 1.1 cm | 0.00° | 334 ms |
| Grid map de localisation | Multi-résolution | 100 % | 0.7 cm | 0.00° | 103 ms |
| Grid map de localisation | Descente de gradient | 92 % | 0.9 cm | 0.11° | 4 ms |
| Grid map de localisation | Filtre particulaire (probabiliste) | 33 % | 6.0 cm | 0.76° | 34 ms |
| Grid map de localisation | ICP | 56 % | 2.3 cm | 0.41° | 7 ms |
| Carte vectorielle | Calcul direct (moindres carrés point-droite) | 73 % | 2.7 cm | 0.22° | 5 ms |
| Carte vectorielle | Force brute | 98 % | 2.2 cm | 0.00° | 331 ms |
| Carte vectorielle | Multi-résolution | 100 % | 2.5 cm | 0.25° | 105 ms |
| Carte vectorielle | Descente de gradient | 94 % | 2.6 cm | 0.14° | 5 ms |
| Carte vectorielle | Filtre particulaire (probabiliste) | 52 % | 5.0 cm | 0.45° | 33 ms |
| Carte vectorielle | ICP | 63 % | 2.9 cm | 0.33° | 8 ms |
| Scan de référence | ICP | 40 % | 13.0 cm | 1.63° | 4 ms |
| Scan de référence | Force brute | 75 % | 3.1 cm | 0.50° | 325 ms |
| Scan de référence | Multi-résolution | 83 % | 2.1 cm | 0.25° | 102 ms |
| Scan de référence | Descente de gradient | 67 % | 2.7 cm | 0.42° | 5 ms |
| Scan de référence | Filtre particulaire (probabiliste) | 27 % | 9.8 cm | 1.22° | 36 ms |

#### Sensibilité à l'initialisation (taux de réussite)

| Référence | Méthode | 5 cm / 2° | 15 cm / 5° | 30 cm / 10° | 45 cm / 18° |
| --- | --- | --- | --- | --- | --- |
| Grid map de localisation | Force brute | 100 % | 100 % | 100 % | 100 % |
| Grid map de localisation | Multi-résolution | 100 % | 100 % | 100 % | 100 % |
| Grid map de localisation | Descente de gradient | 100 % | 100 % | 100 % | 67 % |
| Grid map de localisation | Filtre particulaire (probabiliste) | 50 % | 42 % | 17 % | 25 % |
| Grid map de localisation | ICP | 100 % | 100 % | 25 % | 0 % |
| Carte vectorielle | Calcul direct (moindres carrés point-droite) | 100 % | 100 % | 83 % | 8 % |
| Carte vectorielle | Force brute | 92 % | 100 % | 100 % | 100 % |
| Carte vectorielle | Multi-résolution | 100 % | 100 % | 100 % | 100 % |
| Carte vectorielle | Descente de gradient | 100 % | 100 % | 100 % | 75 % |
| Carte vectorielle | Filtre particulaire (probabiliste) | 67 % | 42 % | 33 % | 67 % |
| Carte vectorielle | ICP | 100 % | 100 % | 50 % | 0 % |
| Scan de référence | ICP | 75 % | 67 % | 17 % | 0 % |
| Scan de référence | Force brute | 75 % | 75 % | 83 % | 67 % |
| Scan de référence | Multi-résolution | 83 % | 83 % | 83 % | 83 % |
| Scan de référence | Descente de gradient | 83 % | 75 % | 67 % | 42 % |
| Scan de référence | Filtre particulaire (probabiliste) | 33 % | 25 % | 33 % | 17 % |

2448 localisations en 248 s. Données brutes : `jour2_reel.csv`.
