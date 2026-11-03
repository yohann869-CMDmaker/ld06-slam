# État de l'art — cartographie et localisation 2D par lidar

Recherche faite le 4 novembre 2026 pour situer notre projet et choisir des améliorations. Pour
chaque famille de méthodes : la référence fondatrice, ce qu'elle apporte, ce que notre projet en
fait déjà, et ce qu'on en retient. Les outils pédagogiques existants sont étudiés à part, pour
l'application destinée aux étudiants (dépôt séparé `carto-edu`, fichier `ETAT_DE_L_ART.md`).

## 1. Cartes

### Grid map (carte d'occupation)

- **Elfes (1989)**, thèse CMU *Occupancy Grids: A Probabilistic Framework for Robot Perception
  and Navigation* et article *Using Occupancy Grids for Mobile Robot Perception and Navigation*
  (IEEE Computer 22(6)). Idée fondatrice : l'espace découpé en cellules, chacune avec une
  probabilité d'occupation mise à jour par la règle de Bayes à chaque mesure, depuis plusieurs
  points de vue. Le cours de Stachniss (Bonn) et *Probabilistic Robotics* (Thrun, Burgard, Fox)
  en donnent la forme log-odds que nous utilisons.
- **Nous** : log-odds bornés, Bresenham, marge libre avant l'impact, faisceaux intermédiaires,
  nettoyage, durée de vie (`decay`). C'est la méthode classique, complète.
- **Format d'échange** : ROS `map_server` / Nav2 enregistre une carte en image PGM plus un YAML
  (`resolution`, `origin`, `occupied_thresh` = 0,65, `free_thresh` = 0,196, `negate`). →
  **ajouté** : export PGM + YAML, la carte s'ouvre dans ROS / RViz / Nav2.

### Carte vectorielle (segments)

- **Nguyen, Martinelli, Tomatis, Siegwart (IROS 2005)**, *A Comparison of Line Extraction
  Algorithms using 2D Laser Rangefinder* : six méthodes comparées ; split-and-merge et
  l'incrémental sont les meilleurs compromis vitesse / justesse ; RANSAC, Hough et EM moins
  fiables. Une étude de 2019 (arXiv 1910.10711, extraction de polylignes au maximum de
  vraisemblance) conteste ce classement : l'*iterative endpoint fit* ferait mieux que le
  split-and-merge.
- **Garulli et al. (Sienne, CDC / ECMR 2005)** : SLAM par EKF sur des droites, en gardant les
  extrémités des segments pour l'association. **Vázquez-Martín et al. (2008)** : segments et coins
  (intersections de droites, réelles ou virtuelles) comme amers.
- **Nous** : clusters adaptatifs + split-and-merge, fusion, découpage par les faisceaux (absences),
  coins. Proche de l'état de l'art pour une carte d'intérieur ; pas d'EKF sur les segments (notre
  SLAM est un graphe de poses, plus robuste que l'EKF, voir § 3).

## 2. Localisation par scan matching

| Méthode | Référence | Principe | Dans notre projet |
| --- | --- | --- | --- |
| Recherche corrélative multi-résolution | Olson, *Real-time correlative scan matching*, ICRA 2009 ; M3RSM, ICRA 2015 | Score sur une grille de vraisemblance, recherche exhaustive à deux niveaux (grossier puis fin), même résultat que la force brute | Méthodes « force brute » et « multi-résolution » (champ de distance, score gaussien) |
| ICP point à point | Besl & McKay 1992 ; Lu & Milios 1997 | Couples plus proches voisins, transformation en forme close | Méthode « ICP » |
| **PL-ICP** (point-droite) | **Censi**, *An ICP variant using a point-to-line metric*, ICRA 2008 ; logiciel CSM | Distance de chaque point à la droite locale de la référence, solution exacte ; plus précis, moins d'itérations, moins robuste aux grands écarts | **Ajouté** (méthode « PL-ICP ») |
| **NDT** | **Biber & Straßer**, *The Normal Distributions Transform*, IROS 2003 | Une gaussienne par cellule (moyenne, covariance des points) : densité continue et dérivable, optimisée par Newton, sans correspondances | **Ajouté** (méthode « NDT ») |
| **Gauss-Newton sur la carte** | **Kohlbrecher et al.**, *Hector SLAM*, SSRR 2011 | Extrémités des faisceaux recalées sur la carte interpolée, Gauss-Newton, plusieurs résolutions, sans odométrie | **Ajouté** (méthode « Gauss-Newton (Hector) » sur le champ de distance) |
| Descente de gradient | — | Pente numérique du score, pas adaptatif | Méthode « descente de gradient » |
| Filtre particulaire (MCL) | Dellaert, Fox, Burgard, Thrun 1999 ; **Fox**, *KLD-sampling*, NIPS 2001 / IJRR 2003 (AMCL) | Nuage d'hypothèses ; KLD : nombre de particules adapté à l'incertitude | Méthode « filtre particulaire » (taille fixe) ; localisation globale à part (`globalloc.js`) |
| Calcul direct | Article « Mars Attacks! » (Coupe de Robotique 2024) | Moindres carrés point-droite sur des murs connus | Méthode « calcul direct » (carte vectorielle) |

## 3. SLAM

- **Lu & Milios (1997)** posent le SLAM comme un graphe de poses ; **Grisetti, Kümmerle,
  Stachniss, Burgard**, *A Tutorial on Graph-Based SLAM* (IEEE ITS Magazine, 2010) en donne la
  résolution par moindres carrés (Gauss-Newton). **Nous** : graphe de poses, Gauss-Newton,
  information par lien d'après les normales des points (Censi 2007 pour la covariance de l'ICP).
- **GMapping** (Grisetti, Stachniss, Burgard, IEEE T-RO 2007) : filtre particulaire
  Rao-Blackwellisé, une carte par particule, proposition améliorée par le scan matching,
  rééchantillonnage seulement quand le nombre effectif de particules baisse. Lourd en mémoire ;
  utile pour comparer, pas retenu.
- **Hector SLAM** (2011) : scan-to-map Gauss-Newton multi-résolution, sans odométrie ni fermeture
  de boucle. Notre suivi sur carte locale en est proche ; nous ajoutons la fermeture de boucle.
- **Karto / Open Karto** (Konolige et al., 2010, *Sparse Pose Adjustment*) et **SLAM Toolbox**
  (Macenski & Jambrecic, JOSS 2021) : graphe de poses, recalage corrélatif, cartographie
  « à vie » (continuer une carte, retirer les nœuds en trop), mode localisation, fusion de cartes.
- **Cartographer** (Hess, Kohler, Rapp, Andor, *Real-Time Loop Closure in 2D LIDAR SLAM*, ICRA
  2016) : sous-cartes (grilles de probabilité), fermeture de boucle par **branch-and-bound** sur
  des grilles précalculées, assez rapide pour du temps réel. **Nous** : recherche de boucle limitée
  aux scans clés à moins de 3 m (plus simple, suffisant dans une salle).
- **tinySLAM / CoreSLAM / BreezySLAM** (Simon D. Levy, Python / C) : SLAM minimal (grille +
  recherche aléatoire), réputé le plus léger ; vinySLAM (OSLL, `slam_constructor`) l'améliore.
  Bonne référence pour un robot à petit microcontrôleur.

## 4. Exploration

- **Yamauchi**, *A Frontier-Based Approach for Autonomous Exploration*, CIRA 1997 : aller à la
  frontière entre l'espace libre connu et l'inconnu. Les travaux suivants (Holz et al., 2010,
  *Evaluating the efficiency of frontier-based exploration*) en font un cas particulier de
  l'exploration par gain d'information : choisir la frontière qui apporte le plus pour le moins de
  trajet. **Nous** : frontière la plus proche ; → **essayé** : choix par utilité (longueur de la
  zone / (distance + 1 m)). Sur nos deux scènes simulées, il ne raccourcit pas le trajet en
  moyenne (trois pièces : 94 m au total sur trois départs dans les deux cas) : gardé en option,
  la plus proche reste le défaut.

## 5. Projets proches (GitHub)

| Projet | Intérêt pour nous |
| --- | --- |
| [ldrobotSensorTeam/ldlidar_stl_ros2](https://github.com/topics/ld06) | Pilote officiel LD06 / LD19 pour ROS 2 : vérifier nos conventions d'angle |
| [simondlevy/BreezySLAM](https://github.com/simondlevy/breezyslam) | SLAM minimal en Python / C, export PGM |
| AtsushiSakai/PythonRobotics (arXiv 1808.10703) | Implémentations courtes et animées : ICP, graph SLAM, FastSLAM, filtres |
| [mocheng/slam-playground](https://github.com/mocheng/slam-playground) | SLAM 2D éducatif : ICP + graphe de poses |
| [entusedglue3125/lidr-slam-mapper](https://github.com/entusedglue3125/lidr-slam-mapper) | ICP + grille log-odds + tableau de bord web (YDLIDAR) : architecture très proche de la nôtre |
| [JiahengZhao/realtimeCSM](https://github.com/JiahengZhao/realtimeCSM) | Implémentation de la recherche corrélative d'Olson |
| SLAM Toolbox, Cartographer, Hector, GMapping (ROS) | Références industrielles |
| MATLAB Navigation Toolbox : *SLAM Map Builder* | Appli interactive : pause, correction à la main d'un recalage ou d'une boucle, puis reconstruction. → idée reprise pour l'appli étudiants |

Un relevé sur Robotics Stack Exchange montre le même montage que le nôtre (LD06 sur l'UART d'un
ESP32, données envoyées en WebSocket à un PC) : notre relais USB et notre client Wi-Fi répondent
au même besoin sans ROS.

## 6. Ce que nous avons ajouté après cette étude

1. **PL-ICP** (Censi 2008), **NDT** (Biber 2003) et **Gauss-Newton sur le champ de distance**
   (Hector 2011) : trois méthodes de localisation de plus, comparées aux six existantes par le banc
   d'essai du jour 2 ([jour2_benchmark.md](jour2_benchmark.md)).
2. **Export ROS** de la grid map (PGM + YAML), lisible par `map_server` / Nav2 / RViz.
3. **Exploration par utilité** (longueur de la frontière / distance), en option : pas de gain
   mesuré sur nos scènes ([jour3_explore.md](jour3_explore.md)).

Pistes non retenues (pour la suite) : KLD-sampling pour le filtre particulaire, sous-cartes et
branch-and-bound (Cartographer), cartographie « à vie » (SLAM Toolbox), filtre particulaire
Rao-Blackwellisé (GMapping).

## Sources

- Elfes 1989 : [thèse (CMU)](https://faculty.iiit.ac.in/~mkrishna/ElfesOccupancyGrids.pdf),
  [NASA NTRS](https://ntrs.nasa.gov/citations/19900019759)
- Nguyen et al. 2005 : [EPFL Infoscience](https://infoscience.epfl.ch/record/97571/files/?ln=en) ;
  polylignes 2019 : [arXiv 1910.10711](https://arxiv.org/pdf/1910.10711)
- Garulli et al. 2005 : [CDC 2005](https://www3.diism.unisi.it/~anto/public/papers/CDC05SLAM.pdf)
- Olson 2009 / 2015 : [M3RSM (PDF)](https://april.eecs.umich.edu/pdfs/olson2015scanmatch.pdf),
  [DOI 10.1109/ROBOT.2009.5152375](https://api.crossref.org/works/10.1109%2FROBOT.2009.5152375)
- Censi 2008, PL-ICP : [Caltech](https://resolver.caltech.edu/CaltechAUTHORS:20100512-152648883), [CSM](https://mirror.umd.edu/roswiki/csm.html)
- Biber & Straßer 2003, NDT : [IEEE](https://ieeexplore.ieee.org/document/1249285),
  [PCL NDT2D](https://docs.ros.org/en/hydro/api/pcl/html/classpcl_1_1NormalDistributionsTransform2D.html)
- Kohlbrecher et al. 2011, Hector : [PDF](https://web.sim.informatik.tu-darmstadt.de/publ/download/2011_SSRR_KohlbrecherMeyerStrykKlingauf_Flexible_SLAM_System.pdf)
- Fox, KLD-sampling : [IJRR 2003](https://rse-lab.cs.washington.edu/postscripts/adaptive-ijrr-2003.pdf),
  [NIPS 2001](https://papers.nips.cc/paper_files/paper/2001/hash/c5b2cebf15b205503560c4e8e6d1ea78-Abstract.html)
- Grisetti et al. 2010, graph SLAM : [PDF (Bonn)](https://www.ipb.uni-bonn.de/wp-content/papercite-data/pdf/grisetti10titsmag.pdf)
- Grisetti et al. 2007, GMapping : [PDF (Bonn)](https://www.ipb.uni-bonn.de/wp-content/papercite-data/pdf/grisetti07tro.pdf),
  [OpenSLAM](https://openslam-org.github.io/gmapping)
- Hess et al. 2016, Cartographer : [Google Research](https://research.google/pubs/real-time-loop-closure-in-2d-lidar-slam/),
  [documentation](https://google-cartographer-ros.readthedocs.io/en/latest/algo_walkthrough.html)
- Macenski & Jambrecic 2021, SLAM Toolbox : [JOSS](https://joss.theoj.org/papers/10.21105/joss.02783.pdf),
  [ROSCon 2019](https://roscon.ros.org/2019/talks/roscon2019_slamtoolbox.pdf)
- BreezySLAM : [GitHub](https://github.com/simondlevy/breezyslam) ; slam_constructor :
  [ROS index](https://index.ros.org/p/slam_constructor/)
- Yamauchi 1997 : [PDF](https://typeset.io/pdf/a-frontier-based-approach-for-autonomous-exploration-7rz3ldf0tq.pdf) ;
  Holz et al. 2010 : [PDF](https://www.ais.uni-bonn.de/~holz/papers/holz_2010_isr.pdf)
- ROS map_server : [wiki](https://wiki.ros.org/map_server) ; MATLAB SLAM Map Builder :
  [documentation](https://mathworks.com/help/nav/ref/slammapbuilder-app.html)
- PythonRobotics : [arXiv 1808.10703](https://arxiv.org/abs/1808.10703v2) ;
  LD06 + ESP32 + WebSocket : [Robotics Stack Exchange](https://answers.ros.org/question/412708/i-have-an-issue-with-lidar-ld06-and-laserscan/)
- Cours de Stachniss (Bonn) : [Online Training – Mobile Robotics](https://www.ipb.uni-bonn.de/online-training-robotics/)
