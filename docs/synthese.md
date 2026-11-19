# Synthèse personnelle — Cartographie et localisation

Module *Cartographie et localisation appliquée à la robotique mobile*, novembre 2026. Synthèse
demandée par le livrable : pour chaque outil, les résultats visuels obtenus, les choix effectués,
les fonctionnalités intégrées et les méthodes implémentées ; puis ce que je retiens du module.
Les chiffres viennent des comptes rendus ([jour 1](jour1.md), [jour 2](jour2.md), [jour 3](jour3.md)) ;
chaque tableau y est régénérable par un script.

**Environnement choisi.** JavaScript (modules ES) sans aucune dépendance : une seule librairie
d'algorithmes (`src/core/`) sert à la fois à l'application web (affichage, manipulation, robot en
direct) et à Node.js (tests, bancs d'essai, figures). Même technologie que les outils du module
(MapTool, RobotControl), conventions identiques (pose x, y, θ ; mètres et degrés ; angles
antihoraires).

## 1. Acquisition des scans et programmes du robot

![Le robot](media/robot_dessus.jpg)

- **Résultats.** 18 scans réels pour la carte de la salle (`data/real/`), 16 scans d'un jeu de test
  indépendant (`data/test/`), 159 scans clés d'une séance de SLAM avec le robot en mouvement
  (`data/slam_usb_20261104/`).
- **Choix.** Un scan = **un seul tour** de lidar : fusionner plusieurs tours biaiserait le scan dès que
  le robot bouge un peu. Le programme de test fourni perdait des trames (trous jusqu'à 74°) : j'ai
  écrit un programme de capture qui n'envoie un scan que sur demande, puis un programme USB à 921 600
  bauds pour suivre le lidar en continu.
- **Fonctionnalités.** Lecture de tous les formats du module (RobotControl, MapTool, sortie Teleplot
  collée depuis le moniteur série, échange binaire Wi-Fi) ; capture USB et Wi-Fi.
- **Difficulté et solution.** Notre robot n'a pas la carte moteurs I2C du programme fourni : ses
  broches I2C sont des broches moteur, et les roues tournaient seules. J'ai relevé le câblage et écrit
  `ESP32_RobotWifi` et `ESP32_RobotUSB`, qui commandent les moteurs directement par l'ESP32 en gardant
  les routes et le format de RobotControl.

## 2. Outil de visualisation des nuages de points (jour 1)

![Six scans superposés](img/j1_nuages.png)

- **Résultats.** Plusieurs scans affichés ensemble, une couleur par scan, à leur pose.
- **Fonctionnalités.** Pose de chaque scan modifiable (champs x, y, θ ; glisser ; Maj + glisser pour
  tourner ; clavier) ; recalage automatique de tous les scans ; enregistrement des scans avec leurs
  poses (`poses.csv`) ; scan en temps réel du robot affiché en direct.
- **Méthode.** Polaire → repère du robot → repère monde, `P = R(θ)·p + T`.

## 3. Outil grid map (jour 1)

| Un scan | Six scans |
| --- | --- |
| ![Grid map d'un scan](img/j1_grid_scan1.png) | ![Grid map de six scans](img/j1_grid_tous.png) |

- **Résultats.** Carte d'occupation de la salle ; chaque nouveau scan met la carte à jour.
- **Choix.** Grille probabiliste en **log-odds** : la mise à jour bayésienne devient une addition
  (impact +0,85, cellule traversée −0,40, bornes ±4 pour que la carte puisse « changer d'avis ») ;
  cellules de 5 cm (le bruit du LD06 est d'environ 1 cm).
- **Fonctionnalités.** Créer une carte depuis un scan, la mettre à jour avec un autre scan, carte de
  tous les scans ; **durée de vie** des cellules (décroissance vers l'inconnu), filtre des cellules
  isolées, carte propre ; exports CSV, PNG, ROS (PGM + YAML) ; import des grilles MapTool.
- **Méthode.** Bresenham du robot vers chaque impact (cellules traversées), puis la cellule d'impact.

## 4. Outil segments (jour 1)

![Segmentation d'un scan](img/j1_segments_scan1.png)

- **Résultats.** Un segment par mur ; toutes les opérations demandées, testées sur leurs cas limites.
- **Choix.** Découpage en groupes de points par un seuil qui grandit avec la distance (Borges & Aldon) ;
  split-and-merge (Douglas-Peucker, 3 cm) ; ajustement par moindres carrés **totaux**, valable pour
  un mur vertical ; tolérance numérique EPS pour les intersections et les découpages.
- **Fonctionnalités.** Angle entre segments, segments « proches » en angle et en position, fusion
  pondérée, intersection, découpage d'un segment par un autre (intersection strictement intérieure,
  extrémités et colinéaires sans effet, longueurs égales à EPS près : côté du premier point).
- **Vérification.** Tests automatiques de chaque cas limite de l'énoncé (`test/geometry.test.js`).

## 5. Outil carte vectorielle (jour 1)

![Carte vectorielle des six scans](img/j1_vecto_tous.png)

- **Résultats.** Carte de segments de la salle, mise à jour scan par scan.
- **Méthode** (étapes du document de cours). Gestion des absences : chaque faisceau découpe les
  segments qu'il traverse ; extraction des segments du scan ; fusion en cascade avec les segments
  proches de la carte ; suppression des segments trop courts. Chaque segment garde son poids et les
  scans qui l'ont vu.
- **Fonctionnalités.** Carte propre (murs redressés, coins fermés, meubles fixes gardés et nommés :
  table, armoire, tableau), import et export TXT, scènes MapTool en référence.

## 6. Outil de localisation (jour 2)

![Carte de référence de la salle](img/j2_carte_reference.png)

- **Résultats.** Sur les trois types de référence (grid map, carte vectorielle, scan de référence),
  avec les mêmes scans et les mêmes poses de départ ; réussite = moins de 5 cm **et** moins de 2° :

  | Référence | Méthode retenue (Gauss-Newton sur la carte) | La plus précise |
  | --- | --- | --- |
  | Grid map | 100 % de réussite, 11 mm et 0,12°, 2 ms | Descente de gradient : 10 mm, 97 % |
  | Carte vectorielle | 100 %, 7 mm et 0,06°, 2 ms | PL-ICP : 1,1 mm, 94 % |
  | Scan de référence | 100 %, 10 mm et 0,12°, 1 ms | ICP : 1,0 mm, 96 % |

  Le Gauss-Newton est retenu pour le SLAM : il ne rate jamais dans la fenêtre testée et répond en 1 à
  2 ms ; les méthodes les plus précises ratent plus souvent quand la pose de départ est loin.

  Sur données réelles, incertitude des poses de référence : ± 2,0 cm et 0,28° en médiane. Sans pose de
  départ, 17 scans réels sur 18 retrouvés dans toute la salle, 18 sur 18 avec la pose approximative.
- **Méthodes.** Force brute, descente de gradient, multi-résolution, filtre particulaire, ICP, calcul
  direct (moindres carrés point-droite) ; puis, après l'état de l'art, PL-ICP, NDT et Gauss-Newton
  sur la carte. Localisation globale avec verdict « fiable » ou « ambiguë ».
- **Choix.** Un même champ de distance et un même score pour toutes les méthodes, pour les comparer
  équitablement ; sensibilité mesurée de 5 cm / 2° à 45 cm / 18° de perturbation.
- **Fonctionnalités.** Localiser le scan sélectionné, comparer les méthodes, localisation globale,
  exporter les résultats en CSV ; carte de référence figée et enregistrable.

## 7. SLAM et carte de la salle (jour 3)

| Carte du SLAM recalée sur la référence | Carte finale reconstruite |
| --- | --- |
| ![Superposition](img/slam_usb_20261104_superposition.png) | ![Carte finale](img/slam_usb_20261104_finale_superposition.png) |

- **Résultats.** En simulation, 0,5 cm d'erreur dans la salle. Avec le **vrai robot**, piloté par câble
  pendant 26 minutes (57 m, 159 scans clés), la carte du SLAM est à **4,3 cm** de la carte de
  référence faite séparément ; après filtrage et reconstruction, les murs vus sont à 2,8 cm et les
  murs reconstruits à 3,9 cm de la référence.
- **Méthodes.** Suivi sur une carte locale (Gauss-Newton puis multi-résolution et ICP), scans clés,
  graphe de poses optimisé par Gauss-Newton avec précision par direction, fermeture de boucle,
  relocalisation ; filtres (objets de moins de 40 cm) et reconstruction du plan d'après la
  littérature (Okorn 2010, Mura 2014, Ochmann 2016, 2019).
- **Choix.** Un SLAM en ligne a besoin d'un **trajet continu** : les clichés épars de la carte de
  référence ne suffisaient pas (salle presque symétrique, scans pris en hauteur). Pour ces clichés,
  j'ai utilisé un graphe de poses hors ligne (Lu & Milios), et pour la carte complète, un trajet
  continu avec le robot.
- **Fonctionnalités.** SLAM sur des scans enregistrés ; SLAM en direct pendant le pilotage au clavier
  (câble ou Wi-Fi) ; barre d'état (liaison, pose, scans par seconde, état du SLAM).

## 8. Bonus : robot autonome

![Exploration de trois pièces](img/j3_explo_pieces.png)

- **Résultats.** Exploration par frontières : 12 départs sur 12 réussis en simulation (98,5 à 100 % de
  la surface vue, aucune collision, 1 à 3 cm d'erreur de pose). Sur le vrai robot en Wi-Fi,
  l'exploration a été essayée et a fonctionné, mais cette séance n'a pas été enregistrée.
- **Méthode.** Frontières entre le connu et l'inconnu (Yamauchi 1997), plus court chemin sur la grille
  gonflée du rayon du robot, vitesses du robot estimées et corrigées à chaque pas par le SLAM (le
  robot n'a pas de codeurs).

## 9. Ce que je retiens du module

### Ce que j'ai apprécié

J'ai bien aimé ce cours. Il était entraînant, et nous avions la chance de travailler sur de vrais
robots : cela change de la théorie seule. J'ai apprécié les explications du cours (grid map, carte
vectorielle, localisation) et le fait de pouvoir les appliquer directement, jusqu'au SLAM et à la
navigation du robot.

### Ce que j'ai moins apprécié

Le principal problème ne venait pas du cours lui-même : il n'y avait pas assez de robots pour tout le
monde, et nous étions souvent obligés d'attendre pour faire nos essais.

J'ai aussi rencontré des difficultés pendant les essais. Le robot perdait souvent le suivi du SLAM,
surtout pendant les rotations, et quand j'avançais trop vite : les scans n'arrivaient pas assez vite
pour suivre le mouvement. En reprenant une rotation plus doucement, le robot retrouvait sa position
sur la carte, et le SLAM fonctionnait bien de cette façon.

### Ce que j'améliorerais

*(Pistes : des codeurs sur les roues pour une vraie odométrie ; plus de temps ou de robots pour les
essais réels ; enregistrer chaque séance réelle, y compris l'exploration autonome ; vérifier l'échelle
de la carte au mètre ruban ; écarter chaises et personnes pendant le SLAM plutôt qu'après.)*
