# Jour 2 — Banc d'essai de la localisation

Jeu simulé (salle 8 m × 6 m, 6 scans, bruit σ = 1 cm, pose vraie connue). Pour chaque scan, les
cartes de référence sont construites avec **les autres scans** à leur pose vraie ; le scan de référence est le scan
le plus proche. Poses initiales : 4 niveaux de perturbation × 2 tirages par scan, **identiques pour toutes
les méthodes**. Fenêtre de recherche : ±50 cm, ±20°.

**Réussite** : erreur de position < 5 cm **et** erreur angulaire < 2°.

## Résultats par méthode

| Référence | Méthode | Réussite | Erreur position médiane | Erreur angle médiane | Temps moyen |
| --- | --- | --- | --- | --- | --- |
| Grid map | Force brute | 100 % | 11.8 mm | 0.00° | 417 ms |
| Grid map | Multi-résolution | 100 % | 12.4 mm | 0.25° | 176 ms |
| Grid map | Descente de gradient | 100 % | 10.4 mm | 0.13° | 5 ms |
| Grid map | Filtre particulaire (probabiliste) | 73 % | 32.7 mm | 0.78° | 42 ms |
| Grid map | ICP | 88 % | 23.1 mm | 0.51° | 13 ms |
| Carte vectorielle | Calcul direct (moindres carrés point-droite) | 90 % | 1.3 mm | 0.02° | 7 ms |
| Carte vectorielle | Force brute | 100 % | 8.6 mm | 0.00° | 442 ms |
| Carte vectorielle | Multi-résolution | 100 % | 4.5 mm | 0.00° | 177 ms |
| Carte vectorielle | Descente de gradient | 100 % | 2.2 mm | 0.01° | 4 ms |
| Carte vectorielle | Filtre particulaire (probabiliste) | 88 % | 27.4 mm | 0.41° | 39 ms |
| Carte vectorielle | ICP | 90 % | 1.3 mm | 0.04° | 17 ms |
| Scan de référence | ICP | 94 % | 1.0 mm | 0.10° | 12 ms |
| Scan de référence | Force brute | 100 % | 10.3 mm | 0.00° | 396 ms |
| Scan de référence | Multi-résolution | 100 % | 7.0 mm | 0.00° | 157 ms |
| Scan de référence | Descente de gradient | 98 % | 6.0 mm | 0.06° | 5 ms |
| Scan de référence | Filtre particulaire (probabiliste) | 65 % | 38.6 mm | 1.09° | 38 ms |

## Sensibilité à l'initialisation (taux de réussite)

| Référence | Méthode | 5 cm / 2° | 15 cm / 5° | 30 cm / 10° | 45 cm / 18° |
| --- | --- | --- | --- | --- | --- |
| Grid map | Force brute | 100 % | 100 % | 100 % | 100 % |
| Grid map | Multi-résolution | 100 % | 100 % | 100 % | 100 % |
| Grid map | Descente de gradient | 100 % | 100 % | 100 % | 100 % |
| Grid map | Filtre particulaire (probabiliste) | 75 % | 67 % | 67 % | 83 % |
| Grid map | ICP | 100 % | 100 % | 100 % | 50 % |
| Carte vectorielle | Calcul direct (moindres carrés point-droite) | 100 % | 100 % | 100 % | 58 % |
| Carte vectorielle | Force brute | 100 % | 100 % | 100 % | 100 % |
| Carte vectorielle | Multi-résolution | 100 % | 100 % | 100 % | 100 % |
| Carte vectorielle | Descente de gradient | 100 % | 100 % | 100 % | 100 % |
| Carte vectorielle | Filtre particulaire (probabiliste) | 92 % | 92 % | 67 % | 100 % |
| Carte vectorielle | ICP | 100 % | 100 % | 100 % | 58 % |
| Scan de référence | ICP | 100 % | 100 % | 100 % | 75 % |
| Scan de référence | Force brute | 100 % | 100 % | 100 % | 100 % |
| Scan de référence | Multi-résolution | 100 % | 100 % | 100 % | 100 % |
| Scan de référence | Descente de gradient | 100 % | 100 % | 100 % | 92 % |
| Scan de référence | Filtre particulaire (probabiliste) | 67 % | 67 % | 42 % | 83 % |

768 localisations en 94 s (Node.js). Données brutes : `jour2_benchmark.csv`.
Régénérer : `node scripts/benchmark-jour2.mjs [tirages par niveau]`.
