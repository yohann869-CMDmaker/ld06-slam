# Passation — état du projet et suite

Fichier à lire en premier pour reprendre le projet (personne ou IA). Mis à jour le 4 novembre 2026.

- Dépôt : https://github.com/yohann869-CMDmaker/ld06-slam (branche `main`, à jour avec GitHub).
- Rendu du module : **dimanche 22 novembre 2026, 23:59**, un `.zip` (lien du dépôt ou copie avec
  historique, outils + instructions, synthèse personnelle).
- Langage : JavaScript pur (Node 22+, aucune dépendance) ; application web `node serve.mjs`
  (port 8080, ou 8081 s'il est pris) ; tests `node --test "test/*.test.js"` (110 tests, tous verts
  le 4 novembre ; 124 depuis les méthodes de l'état de l'art).
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
| 3 | Bonus : déplacer le robot automatiquement | Fait en simulation : exploration par frontières sur la carte du SLAM (6 départs sur 6 réussis, couverture 99,6-100 %, 0 collision, 1-2 cm d'erreur) ; pilotage Wi-Fi du vrai robot (`firmware/ESP32_RobotWifi` : notre robot commande ses moteurs directement par l'ESP32, pas par la carte I2C du module) avec étalonnage automatique des vitesses, vérifié avec un faux robot (`scripts/mock-robot.mjs`) ; **pas encore essayé sur le vrai robot** | `src/core/explore.js`, `docs/jour3.md` (§ Bonus), `docs/jour3_explore.md` |

Comptes rendus : `docs/jour1.md`, `docs/jour2.md`, `docs/jour3.md` ; algorithmes expliqués :
`docs/algorithmes.md` ; erreurs rencontrées et corrections : `docs/journal.md` ; **état de l'art**
(articles, thèses, projets GitHub) : `docs/etat_de_l_art.md`.

Ajouts du 4 novembre (soir), après l'état de l'art :
- 3 méthodes de localisation de plus (`src/core/advancedmatch.js`) : PL-ICP (Censi 2008), NDT
  (Biber 2003), Gauss-Newton sur la carte (Hector 2011), passées dans les bancs d'essai du jour 2
  (`docs/jour2.md`, section « Ajout ») ; le Gauss-Newton réussit 100 % en simulation en 1-2 ms ;
- SLAM 4 à 7 fois plus rapide : suivi par Gauss-Newton d'abord, multi-résolution en secours
  (`docs/jour3.md`, section « Amélioration du SLAM ») ;
- export de la grid map au format ROS (PGM + YAML) dans l'appli et en ligne de commande ;
- exploration : choix de la frontière par utilité essayé, sans gain, gardé en option.

Ajouts du 5 novembre (« il faut connaître la position du robot ») : a priori de position dans la
localisation globale (18 / 18 au lieu de 17 / 18 sur les scans réels) et dans le SLAM ; graphe de
poses hors ligne (Lu & Milios) qui donne une carte complète de la salle à partir des clichés et de
leurs poses approximatives (erreur médiane 4,2 cm). Détails : `docs/jour3.md`, dernière section, et
`docs/jour3_apriori.md`. Le SLAM en ligne sur clichés épars reste impossible : trajet continu
nécessaire (essai avec le robot toujours à faire, robots rares en salle).

Constat important (jour 3) : sur les 18 clichés réels (épars, deux hauteurs, salle presque
symétrique), un SLAM parti de zéro est ambigu. Il faut un **trajet continu au sol**, un scan
tous les 20 à 30 cm : c'est le mode « SLAM en direct » de l'application, pas encore essayé sur
le vrai robot.

## Premier essai sur le vrai robot (4 novembre)

- Notre robot n'a pas la carte moteurs I2C du module : moteurs commandés directement par l'ESP32
  (AVG 18/19, AVD 5/17, ARG 15/2, ARD 16/4). Programme `firmware/ESP32_RobotWifi` (routes du module +
  `/speed`). Connexion Wi-Fi, scans et mouvements : OK.
- Pilotage par pas de 30 cm : SLAM perdu dès les premiers pas (pas trop grands, vitesse réelle
  inconnue, robot entouré de chaises : scans pauvres, beaucoup de points à moins de 50 cm).
- Remplacé par le **pilotage continu** (touche maintenue, SLAM à 5-6 scans/s), validé avec le faux
  robot (pose au centimètre). **À réessayer sur le robot** : retéléverser `ESP32_RobotWifi`, espace
  dégagé, vitesse 100-120, rotations douces ; enregistrer les scans dans `data/slam_wifi/`.

## À faire pour le rendu (par priorité)

1. **Carte de la salle par SLAM avec le vrai robot** (prochaine séance avec le robot ; c'est ce
   qui manque pour finir le jour 3). Deux façons, la seconde est le bonus :
   - USB : `python scripts/live.py`, case « SLAM en direct », pousser le robot lentement au sol
     sur une boucle dans la salle ;
   - Wi-Fi : téléverser `firmware/ESP32_RobotWifi` (câblage des moteurs en tête du fichier), connecter le PC
     au Wi-Fi `DON MIGUEL3`, `node serve.mjs`, bouton « Explorer avec le robot (Wi-Fi) », rester à
     côté (Échap = arrêt). Mesurer d'abord à peu près la vitesse du robot (cm parcourus en 1 s)
     pour remplir les champs ; ils se corrigent ensuite tout seuls.
   Puis enregistrer les scans clés, exporter la carte (grid + vectorielle), comparer à la carte
   de référence (écart des murs, cohérence au retour), ajouter résultats et figures à
   `docs/jour3.md` (remplacer la mention « pas encore essayé sur le vrai robot »).
2. **Incertitude des références réelles** : déjà mesurée (± 2,0 cm et 0,28° en médiane, voir
   `docs/jour2_reel.md`) ; rien à faire sauf vérification.
3. **Synthèse finale** `docs/synthese.md` (exigée par le livrable) : pour chaque outil, résultats
   visuels, choix, fonctionnalités, méthodes ; puis ce que j'ai apprécié, moins apprécié, ce que
   j'améliorerais. S'appuyer sur les comptes rendus existants ; la partie personnelle doit être
   écrite ou validée par Yohan.
4. **Livrable** : script ou instructions pour produire le `.zip` (lien GitHub + copie du dépôt
   avec `.git`, README à jour).
5. Améliorations possibles de l'exploration (facultatif) : choisir la zone frontière selon sa
   taille et sa distance (moins d'allers-retours), utiliser les translations latérales du
   Mecanum, limiter l'exploration à une zone dessinée à la main (vitres).

## Idée pour plus tard : une application pour aider les étudiants

Proposée par Yohan le 4 novembre. **Commencée** le 4 novembre dans le dossier voisin
`../carto-edu` (dépôt git séparé, local, pas encore sur GitHub) : état de l'art des outils
pédagogiques (`ETAT_DE_L_ART.md`), conception et parcours de 9 fiches (`CONCEPTION.md`), prototype
web avec 4 fiches (lidar → points, grid map pas à pas, ICP pas à pas, filtre particulaire avec
MCL augmenté). Lancer : `node serve.mjs` dans `carto-edu`, puis http://localhost:8090/.
Son dossier `core/` est une copie de `src/core` : la recopier après chaque changement du TP.

- Idée : réutiliser `src/core/` et l'application web pour un outil pédagogique (visualiser pas à
  pas grid map, segments, scan matching, SLAM ; charger ses propres scans LD06 ; exercices).
- Suite : fiches 3, 6, 7, 8, 9 (segments, comparaison des méthodes, SLAM, exploration, mes
  données), exercices vérifiés automatiquement, publication (GitHub Pages) après le rendu.
- À décider avec Yohan : public visé (ce module seulement ou plus large), nom, dépôt GitHub.
