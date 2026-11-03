# Passation — état du projet et suite

Fichier à lire en premier pour reprendre le projet (personne ou IA). Mis à jour le 4 novembre 2026.

- Dépôt : https://github.com/yohann869-CMDmaker/ld06-slam (branche `main`, à jour avec GitHub).
- Rendu du module : **dimanche 22 novembre 2026, 23:59**, un `.zip` (lien du dépôt ou copie avec
  historique, outils + instructions, synthèse personnelle).
- Langage : JavaScript pur (Node 22+, aucune dépendance) ; application web `node serve.mjs`
  (port 8080, ou 8081 s'il est pris) ; tests `node --test "test/*.test.js"` (110 tests, tous verts
  le 4 novembre).
- Le dossier `ld06-slam/ld06-slam/` (non suivi) est un clone vide (« Initial commit ») sans
  rapport avec le projet : à ignorer ou supprimer.

## Ce qui est fait

| Jour | Demandé par l'énoncé | État | Où |
| --- | --- | --- | --- |
| 1 | Extraction des scans réels | Fait : 7 scans (2 nov., USB) + 11 scans LidarScope (3 nov.), jeu de test indépendant de 16 scans | `data/real/`, `data/test/`, `firmware/`, `scripts/capture.py`, `scripts/import-lidarscope.mjs` |
| 1 | Visualisation multi-scans, couleurs, pose modifiable | Fait | `app/` |
| 1 | Grid map (création + mise à jour) | Fait (log-odds, Bresenham, nettoyage, import/export, MapTool) | `src/core/gridmap.js` |
| 1 | Segments : extraction, angle, proximité, fusion, intersection, découpage avec tolérances et cas limites | Fait et testé | `src/core/geometry.js`, `segmentation.js` |
| 1 | Carte vectorielle | Fait (fusion, gestion des absences, carte propre, objets fixes étiquetés) | `src/core/vectormap.js`, `cleanup.js` |
| 2 | Localisation sur grid map, carte vectorielle, scan de référence + évaluation chiffrée | Fait : 6 méthodes (brute force, gradient, multi-résolution, probabiliste, ICP, calcul direct), banc d'essai simulé et réel, sensibilité à l'initialisation, localisation globale | `src/core/localization.js`, `globalloc.js`, `docs/jour2*.md` |
| 3 | SLAM (localisation + cartographie simultanées) | Fait : suivi sur carte locale, relocalisation, scans clés, graphe de poses Gauss-Newton avec fermeture de boucle ; validé en simulation (salle : 0,6 cm ; couloir 54 m : 27 cm avec boucle) | `src/core/slam.js`, `posegraph.js`, `docs/jour3*.md` |
| 3 | Carte complète de la salle | Carte de référence faite à partir des 18 scans réels recalés (`data/reference/`) ; **pas encore** produite par le SLAM lui-même | voir « À faire » |

Comptes rendus : `docs/jour1.md`, `docs/jour2.md`, `docs/jour3.md` ; algorithmes expliqués :
`docs/algorithmes.md` ; erreurs rencontrées et corrections : `docs/journal.md`.

Constat important (jour 3) : sur les 18 clichés réels (épars, deux hauteurs, salle presque
symétrique), un SLAM parti de zéro est ambigu. Il faut un **trajet continu au sol**, un scan
tous les 20 à 30 cm : c'est le mode « SLAM en direct » de l'application, pas encore essayé sur
le vrai robot.

## À faire pour le rendu (par priorité)

1. **SLAM en direct sur le robot en salle** (prochaine séance avec le robot) : `python
   scripts/live.py`, case « SLAM en direct », pousser le robot lentement au sol sur une boucle
   dans la salle. Enregistrer les scans clés, exporter la carte (grid + vectorielle), comparer
   à la carte de référence (écart des murs, cohérence au retour). Ajouter le résultat et les
   figures à `docs/jour3.md`.
2. **Incertitude des références réelles** : recaler deux fois les mêmes scans (énoncé jour 2 :
   « indiquez l'incertitude des références mesurées sur le robot »). Vérifier ce qui est déjà
   dans `docs/jour2_reel.md` et compléter si besoin.
3. **Synthèse finale** `docs/synthese.md` (exigée par le livrable) : pour chaque outil, résultats
   visuels, choix, fonctionnalités, méthodes ; puis ce que j'ai apprécié, moins apprécié, ce que
   j'améliorerais. S'appuyer sur les comptes rendus existants ; la partie personnelle doit être
   écrite ou validée par Yohan.
4. **Livrable** : script ou instructions pour produire le `.zip` (lien GitHub + copie du dépôt
   avec `.git`, README à jour).
5. Bonus (facultatif) : déplacement automatique du robot (exploration simple, ex. suivi de mur ou
   frontières de la grid map) pendant le SLAM en direct, via le Wi-Fi de l'ESP32 / RobotControl.

## Idée pour plus tard : une application pour aider les étudiants

Proposée par Yohan le 4 novembre, **à faire après le rendu du TP** (ne pas mélanger avec le dépôt
du module tant que le rendu n'est pas fait).

- Idée : réutiliser `src/core/` et l'application web pour un outil pédagogique (visualiser pas à
  pas grid map, segments, scan matching, SLAM ; charger ses propres scans LD06 ; exercices).
- Préalable : un **état de l'art** de ce qui existe déjà (outils pédagogiques et simulateurs
  SLAM 2D en ligne, ROS/rviz, Gazebo, les outils du module MapTool et RobotControl, cours
  interactifs, bibliothèques comme les implémentations Python de PythonRobotics, etc.) pour
  trouver ce qui manque vraiment avant de coder.
- À décider avec Yohan : public visé (ce module seulement ou plus large), web seul ou avec le
  robot, dépôt séparé.
