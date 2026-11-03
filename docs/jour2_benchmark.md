# Jour 2 — Banc d'essai de la localisation

Jeu simulé (salle 8 m × 6 m, 6 scans, bruit σ = 1 cm, pose vraie connue). Pour chaque scan, les
cartes de référence sont construites avec **les autres scans** à leur pose vraie ; le scan de référence est le scan
le plus proche. Poses initiales : 4 niveaux de perturbation × 3 tirages par scan, **identiques pour toutes
les méthodes**. Fenêtre de recherche : ±50 cm, ±20°.

**Réussite** : erreur de position < 5 cm **et** erreur angulaire < 2°.

## Résultats par méthode

| Référence | Méthode | Réussite | Erreur position médiane | Erreur angle médiane | Temps moyen |
| --- | --- | --- | --- | --- | --- |
| Grid map | Force brute | 100 % | 12.8 mm | 0.00° | 259 ms |
| Grid map | Multi-résolution | 100 % | 13.2 mm | 0.25° | 103 ms |
| Grid map | Descente de gradient | 97 % | 10.2 mm | 0.14° | 3 ms |
| Grid map | Filtre particulaire (probabiliste) | 81 % | 28.8 mm | 0.68° | 25 ms |
| Grid map | ICP | 89 % | 22.0 mm | 0.59° | 8 ms |
| Grid map | PL-ICP (ICP point-droite, Censi 2008) | 74 % | 28.0 mm | 0.29° | 3 ms |
| Grid map | NDT (Biber 2003) | 92 % | 25.4 mm | 0.02° | 13 ms |
| Grid map | Gauss-Newton sur la carte (Hector 2011) | 100 % | 11.2 mm | 0.12° | 2 ms |
| Carte vectorielle | Calcul direct (moindres carrés point-droite) | 86 % | 1.3 mm | 0.02° | 3 ms |
| Carte vectorielle | Force brute | 100 % | 9.2 mm | 0.00° | 254 ms |
| Carte vectorielle | Multi-résolution | 100 % | 4.3 mm | 0.00° | 103 ms |
| Carte vectorielle | Descente de gradient | 99 % | 2.3 mm | 0.01° | 3 ms |
| Carte vectorielle | Filtre particulaire (probabiliste) | 89 % | 28.4 mm | 0.48° | 22 ms |
| Carte vectorielle | ICP | 92 % | 1.3 mm | 0.04° | 9 ms |
| Carte vectorielle | PL-ICP (ICP point-droite, Censi 2008) | 94 % | 1.1 mm | 0.02° | 4 ms |
| Carte vectorielle | NDT (Biber 2003) | 83 % | 2.2 mm | 0.03° | 11 ms |
| Carte vectorielle | Gauss-Newton sur la carte (Hector 2011) | 100 % | 6.8 mm | 0.06° | 2 ms |
| Scan de référence | ICP | 96 % | 1.0 mm | 0.06° | 6 ms |
| Scan de référence | Force brute | 100 % | 10.0 mm | 0.00° | 249 ms |
| Scan de référence | Multi-résolution | 100 % | 7.2 mm | 0.00° | 100 ms |
| Scan de référence | Descente de gradient | 100 % | 6.0 mm | 0.06° | 3 ms |
| Scan de référence | Filtre particulaire (probabiliste) | 67 % | 36.2 mm | 1.11° | 23 ms |
| Scan de référence | PL-ICP (ICP point-droite, Censi 2008) | 93 % | 3.1 mm | 0.05° | 3 ms |
| Scan de référence | NDT (Biber 2003) | 89 % | 2.2 mm | 0.04° | 13 ms |
| Scan de référence | Gauss-Newton sur la carte (Hector 2011) | 100 % | 10.2 mm | 0.12° | 1 ms |

## Sensibilité à l'initialisation (taux de réussite)

| Référence | Méthode | 5 cm / 2° | 15 cm / 5° | 30 cm / 10° | 45 cm / 18° |
| --- | --- | --- | --- | --- | --- |
| Grid map | Force brute | 100 % | 100 % | 100 % | 100 % |
| Grid map | Multi-résolution | 100 % | 100 % | 100 % | 100 % |
| Grid map | Descente de gradient | 100 % | 100 % | 100 % | 89 % |
| Grid map | Filtre particulaire (probabiliste) | 83 % | 83 % | 61 % | 94 % |
| Grid map | ICP | 100 % | 100 % | 100 % | 56 % |
| Grid map | PL-ICP (ICP point-droite, Censi 2008) | 83 % | 83 % | 78 % | 50 % |
| Grid map | NDT (Biber 2003) | 100 % | 100 % | 94 % | 72 % |
| Grid map | Gauss-Newton sur la carte (Hector 2011) | 100 % | 100 % | 100 % | 100 % |
| Carte vectorielle | Calcul direct (moindres carrés point-droite) | 100 % | 100 % | 100 % | 44 % |
| Carte vectorielle | Force brute | 100 % | 100 % | 100 % | 100 % |
| Carte vectorielle | Multi-résolution | 100 % | 100 % | 100 % | 100 % |
| Carte vectorielle | Descente de gradient | 100 % | 100 % | 100 % | 94 % |
| Carte vectorielle | Filtre particulaire (probabiliste) | 83 % | 94 % | 78 % | 100 % |
| Carte vectorielle | ICP | 100 % | 100 % | 100 % | 67 % |
| Carte vectorielle | PL-ICP (ICP point-droite, Censi 2008) | 100 % | 100 % | 100 % | 78 % |
| Carte vectorielle | NDT (Biber 2003) | 94 % | 94 % | 89 % | 56 % |
| Carte vectorielle | Gauss-Newton sur la carte (Hector 2011) | 100 % | 100 % | 100 % | 100 % |
| Scan de référence | ICP | 100 % | 100 % | 100 % | 83 % |
| Scan de référence | Force brute | 100 % | 100 % | 100 % | 100 % |
| Scan de référence | Multi-résolution | 100 % | 100 % | 100 % | 100 % |
| Scan de référence | Descente de gradient | 100 % | 100 % | 100 % | 100 % |
| Scan de référence | Filtre particulaire (probabiliste) | 72 % | 61 % | 44 % | 89 % |
| Scan de référence | PL-ICP (ICP point-droite, Censi 2008) | 100 % | 100 % | 100 % | 72 % |
| Scan de référence | NDT (Biber 2003) | 100 % | 100 % | 83 % | 72 % |
| Scan de référence | Gauss-Newton sur la carte (Hector 2011) | 100 % | 100 % | 100 % | 100 % |

1800 localisations en 88 s (Node.js). Données brutes : `jour2_benchmark.csv`.
Régénérer : `node scripts/benchmark-jour2.mjs [tirages par niveau]`.
