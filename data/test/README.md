# Jeu de test indépendant

16 scans de la salle pris le 2 novembre avec LidarScope (programme de test, format Teleplot),
**jamais utilisés pour construire la carte de référence** : ils servent à évaluer la
localisation (jour 2) sur des scans que la carte n'a jamais vus.

- `lidarscope_20261102/` : export brut (dossier `scans_tp` de l'export LidarScope du
  2 novembre, 21 h 02) et son `LISEZMOI.md`.
- `scan_1102_testNN.csv` : les mêmes scans réduits à **un seul tour** (`singleTour`), importés par

  ```bash
  node scripts/import-lidarscope.mjs data/test/lidarscope_20261102 --out data/test --prefix scan_1102_test
  ```

  La pose écrite dans chaque fichier est celle posée **à la main dans LidarScope**, dans le repère
  de LidarScope du 2 novembre (aucun scan commun avec `data/real` pour la ramener dans notre
  repère). `scripts/benchmark-reel.mjs` estime ce changement de repère à partir des réponses
  fiables de la localisation globale, puis compare.

D'après leurs mesures (rien à moins de 1 m), `test01`, `test02`, `test14`, `test15`, `test16` et
`test17` semblent pris **en hauteur** (robot posé sur un meuble ; `test14` au même endroit que le
premier scan de la carte, sur l'armoire ; `test01` et `test02` sur la table du milieu).

Ne pas mettre ces fichiers dans `data/real` : ils entreraient dans la carte de référence et le
test ne serait plus indépendant.
