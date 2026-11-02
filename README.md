# LD06 SLAM — Cartographie et localisation 2D

Projet du module *Cartographie et localisation appliquée à la robotique mobile*.
Outils pour exploiter les scans du lidar LD06 : affichage de nuages de points,
grid map, carte vectorielle, puis (jours 2 et 3) scan matching et SLAM.

## Prérequis

- Node.js 22 ou plus (testé avec Node 24). Aucune dépendance à installer.
- Un navigateur récent (Chrome, Edge, Firefox).

## Lancer

```bash
node serve.mjs
```

Puis ouvrir http://localhost:8080/ dans le navigateur.

## Tests

```bash
node --test "test/*.test.js"
```

## Structure

| Dossier | Contenu |
| --- | --- |
| `src/core/` | Librairie d'algorithmes (JS pur, utilisée par l'appli web et par Node) |
| `app/` | Application web interactive |
| `scripts/` | Outils en ligne de commande (génération de données simulées, construction de cartes) |
| `test/` | Tests unitaires (`node --test`) |
| `data/` | Scans : `sim/` simulés avec pose connue, `real/` acquis sur le robot |
| `docs/` | Compte rendu et figures |
