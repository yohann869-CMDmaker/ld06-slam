# Journal de bord : erreurs, corrections et remarques

Chronologie du travail sur le projet, avec les problèmes rencontrés, leur cause et la
correction retenue. Les détails techniques sont dans les comptes rendus ([jour 1](jour1.md),
[jour 2](jour2.md)) et dans les messages de commit.

## Lundi 2 novembre — jour 1 : comprendre, puis cartographier

### Compréhension du capteur et de la librairie LD06

Lecture de la librairie (`ld06.h`, `ld06.cpp`, `ld06crc.h`), du programme de test et du protocole constructeur :

- trame de 47 octets : `54 2C`, vitesse (°/s), angle de début, 12 × (distance mm, intensité), angle
  de fin, horodatage, CRC-8 ; CRC vérifié sur l'exemple du constructeur (0x50), table = polynôme 0x4D ;
- la librairie convertit l'angle horaire du capteur en angle antihoraire (`360 − angle`) ;
- **remarques sur la librairie** (utiles pour la synthèse) :
  - interpolation `début + (i + 0,5)·plage/12` différente de celle du constructeur
    `début + i·plage/11` : écart de ±0,43° sur la trame d'exemple (≈ 2 cm à 3 m) ;
  - les x/y calculés par la librairie ont le signe de y inversé (double inversion) : l'affichage
    Teleplot est **en miroir** ;
  - `isChecksumOk()` : affectation écrite à l'envers, la fonction renvoie faux pour toujours après
    la première erreur CRC ;
  - l'exemple de trame du document du cours ne fait que 40 octets (tronqué) ;
  - RobotControl refuse les scans de plus de 460 points (≈ 450 à 10 tours/s : marge faible).

### Développement des outils du jour 1

Choix : JavaScript (modules ES) sans dépendance, même code pour l'application web et pour Node
(tests, ligne de commande). Jeu de scans simulés à pose connue (salle 8 × 6 m) pour valider.

| Problème | Cause | Correction |
| --- | --- | --- |
| `node --test test/` ne lance aucun test | Node 22+ attend des fichiers, pas un dossier | `node --test "test/*.test.js"` |
| Grid map d'un seul scan presque entièrement « inconnue » | Seuil libre p ≤ 0,35 alors qu'une traversée donne p = 0,40 | Seuils 0,45 (libre) et 0,6 (occupé) : une observation suffit |
| Test du carton retiré qui échoue | Face du carton exactement sur une frontière de cellule | Carton décalé de 2 cm dans le test |
| Mur coupé en deux segments dans la pièce fermée | La boucle de points commence à l'angle 0°, au milieu d'un mur | Boucle fermée redémarrée sur un coin avant le split |
| Nombre d'observations d'un segment surestimé | Deux morceaux d'un même mur extraits du même scan comptaient deux fois | Liste des numéros de scans qui ont vu le segment |
| Application sans style ni script | Page servie à `/` : `style.css` et `app.js` cherchés au mauvais endroit | Redirection `/` → `/app/` |
| Calque « Déposer des scans » toujours visible | `display: grid` du CSS écrasait l'attribut `hidden` | Règle `[hidden] { display: none !important }` |
| Grid map grise et peu lisible | Rendu linéaire de p : une cellule libre vue une fois est gris moyen | Rendu par classe (libre / inconnu / occupé) nuancé par la confiance |
| Hachures loin du robot | Faisceaux espacés de 5,6 cm à 4 m : des cellules jamais traversées | Faisceaux intermédiaires (libres seulement) entre impacts voisins |
| Murs en pointillés | Un impact bruité juste derrière un mur libérait la cellule du mur | Pas de marquage libre dans les 5 derniers cm avant l'impact |

### Acquisition des scans réels (fin de journée)

| Problème | Cause | Correction |
| --- | --- | --- |
| L'application ne lisait pas la sortie du programme de test | Format Teleplot (`>lidar:x:y;…`), x/y en miroir | Lecteur Teleplot avec correction du miroir |
| Fichiers de scan de 12 points seulement | Le moniteur série de l'IDE Arduino 2 ne copie que les lignes affichées | Copier-coller abandonné : relais Python (`scripts/live.py`) |
| Port COM absent de la liste du navigateur (Web Serial) | Navigateur (Firefox) et port parfois occupé par l'IDE | Web Serial abandonné au profit du relais Python (pyserial) |
| Programme de capture : vitesse 0, aucun scan | Aucune donnée sur GPIO33 (lidar non alimenté ou débranché) | Commande de diagnostic `r` (octets bruts) ; problème matériel résolu ensuite |
| Tours avec des trous jusqu'à 74° | Le programme de test envoie en texte à 115 200 bauds, plus lentement que le lidar : trames perdues | Programme de capture à la demande (tours complets) |
| Fusion de plusieurs tours pour boucher les trous | Biais : une petite rotation du robot entre deux tours dédouble les murs | **Abandonnée** : un scan = un seul tour |
| Téléversement Arduino « port occupé », application instable | Deux relais lancés en même temps ; sous Windows, deux serveurs peuvent écouter le même port 8080 | Un seul relais ; `allow_reuse_address = False` : le second refuse de démarrer |
| Scans de 12 à 24 points en temps réel | Programme sur l'ESP32 envoyant chaque trame séparément (options du test activées) | Retour au programme `ESP32_LD06_Capture` (≈ 455 points par tour) |
| Points à 7 cm du robot | Reflets sur la structure du robot | Filtre r min porté de 5 à 15 cm |
| Objet à 23 cm devant le robot dans tous les scans | Opérateur ou ordinateur trop près | S'éloigner du robot pendant les scans |
| Carte réelle incohérente | Scans enregistrés tous en pose (0, 0, 0) | Recalage manuel, puis automatique |

## Mardi 3 novembre — fin du jour 1, début du jour 2

| Problème | Cause | Correction |
| --- | --- | --- |
| Recalage manuel long et imprécis | Pas d'outil automatique | Recalage automatique : multi-résolution + ICP, chaque scan face à tous les autres (2 passes) |
| Scan au milieu des tables mal placé (33 % de points cohérents) | Vue pauvre en murs (pieds de meubles) | Partir de la pose manuelle approximative, puis affiner localement |
| « Je ne vois pas la grid map » | Zones libres blanches sur fond presque blanc | Fond de la couleur « inconnu » quand la grid map est affichée |
| Cartes brutes peu lisibles | Faisceaux sortis par les fenêtres, poussières, murs morcelés | Reconstruction propre (`cleanup.js`) et vue style MapTool |
| Murs supprimés par le filtre « hors de la pièce » | Un mur prolongé au-delà d'un coin avait une extrémité dehors | Filtre par échantillonnage : on garde la partie au bord de la pièce |
| Fichiers MapTool inutilisables tels quels | MapTool n'accepte pas de coordonnées négatives | Coordonnées décalées en positif, dimensions indiquées en en-tête |
| Test du nettoyage vectoriel qui échoue | Le test supposait des murs à 0° alors que la direction dominante calculée était 0,44° | Test corrigé (comparaison à la direction dominante) |

### Étape 2 du jour 2 : carte de référence figée

| Problème | Cause | Correction |
| --- | --- | --- |
| Pas de carte fixe pour localiser les nouveaux scans | Les cartes « des scans visibles » sont reconstruites à chaque changement de pose | Carte de référence figée, enregistrée en JSON (`refmap.js`) et rechargée dans l'application |
| Fichier de carte lourd (85 000 cellules) | Une valeur par cellule | Compression par plages de valeurs identiques : 10 ko |
| Peu de points du scan « sur la carte » (42 %) alors que la pose est juste | La carte propre ne garde que les murs : pieds de meubles absents | Normal ; le score ne sert qu'à comparer les poses entre elles, la précision est vérifiée à part (0,6 cm, 0,25°) |

### Scans ajoutés pour améliorer la carte (export LidarScope)

| Problème | Cause | Correction |
| --- | --- | --- |
| 6 fichiers sur 11 avec 2 à 3 tours (jusqu'à 870 points) | L'export LidarScope enchaîne plusieurs rotations | `singleTour` : un seul tour complet par scan (règle : un tour, sinon les scans sont biaisés) |
| Poses des nouveaux scans dans un autre repère (tourné de −89°) | LidarScope a son propre repère | Scan commun `scan_1102_175527` réimporté dans LidarScope : ancre du changement de repère (`import-lidarscope.mjs`) |
| Deux scans de l'ancienne carte décalés d'environ 50 cm | Recalage automatique avec peu de recouvrement : murs longs parallèles, translation mal contrainte | Recalage commun des 18 scans en partant des poses manuelles : 82 % de points cohérents, rectangle central refermé |
| Recalage commun qui semblait prendre plus de 10 min | Script de préparation lancé avec une entrée standard redirigée deux fois (bloqué en attente) | Script écrit dans un fichier puis lancé : le recalage de 18 scans prend 2 s |
| Calcul direct moins bon sur la nouvelle carte (10/18) | Méthode locale (marge 25 cm), plus de segments voisins où se tromper | Multi-résolution par défaut avec la carte de référence (17/18) |

### Meubles fixes absents de la vue propre (remarque de Yohan)

| Problème | Cause | Correction |
| --- | --- | --- |
| Table du milieu, armoire et tableau invisibles dans la vue propre | Grid map : des faisceaux d'autres scans passent sous la table et effacent ses cellules (17 cellules occupées pour 8 m de contour), puis le nettoyage retire ces restes comme des poussières | Objets fixes : segments vus par au moins 2 scans, redessinés dans la grid map propre (`classifySegments`, `drawObjects`) |
| Segments des meubles effacés dans l'application | La gestion des absences (cochée par défaut) découpe les segments traversés par des faisceaux : table 8,3 m → 0,2 m | Cartes propres construites sans découpe par les absences (`cleanMaps`) ; la carte brute garde la découpe |
| Une face de l'armoire, vue par un seul scan, retirée | Règle des 2 scans | Une face qui forme un coin avec un segment gardé est gardée |
| Première règle « mur = posé sur les murs de la grid » | Un meuble gardé par la grid était classé mur (armoire à moitié rouge, table rouge en simulation) | Mur = segment qui borde l'extérieur de la salle (cellules inconnues reliées au bord de la carte) |
| Fusion du nettoyage qui additionnait les observations | `mergeSegments` additionne : deux morceaux vus par le même scan comptaient 2 | Union des scans distincts, comme dans `VectorMap.addSegment` |

Effet sur la localisation (18 scans, départ à 29 cm et 8°) : grid map 13/18 → 18/18, calcul
direct 10/18 → 16/18.

### Meubles surélevés (table sur roulettes, tableau sur deux pieds)

| Problème | Cause | Correction |
| --- | --- | --- |
| Rien ne distinguait la table et le tableau (surélevés) de l'armoire (posée au sol) | Le plan ne montrait que des segments | Part des faisceaux qui passent sous chaque meuble (`markRaised`) : pointillés et cellules grises si plus de 15 % |
| Mesure par segment : l'armoire semblait aussi surélevée (81 %) | Le robot du premier scan et de `ls09` est dans l'emprise de l'armoire et voit à travers ses faces | Scans pris depuis l'emprise d'un meuble ignorés : armoire 6 %, table 23 %, tableau 58 % |
| Faisceau compté deux fois sous la table | Il traverse le côté proche puis le côté opposé | Comptage par meuble (segments reliés par leurs coins), un faisceau compte une fois |
| Pointillés peu visibles dans la vue propre | Cellules noires continues sous le tracé | Cellules d'un meuble surélevé « occupées en partie » (p = 0,75) dessinées en gris clair |

### Scans pris en hauteur (précision de Yohan : premier scan sur l'armoire, deuxième sur la table)

| Problème | Cause | Correction |
| --- | --- | --- |
| Robot « dans » l'armoire et « sous » la table | Il était posé dessus : premier scan sur l'armoire, deuxième sur la table (sur 3 boîtes) | Scans marqués `# pris en hauteur : ...` ; `ls09`, `ls08` (mêmes vues) et `ls12` reconnus d'après leurs mesures (rien à moins de 1 m) |
| Table comptée surélevée (23 %) | Les faisceaux des scans pris en hauteur passaient au-dessus de la table, comptés « dessous » ; ils effaçaient aussi la table dans la grid map | Cartes et meubles construits avec les scans au sol : table 4 % (le lidar ne passe pas sous la table), tableau 61 % |
| Armoire classée mur, mur de droite en morceaux | Depuis le sol, le mur derrière les meubles n'est pas vu ; l'extérieur rejoint l'armoire | Les grands murs vus d'en haut ferment l'extérieur et complètent le contour de la salle |
| Table sur roulettes non montrée | L'espace sous la table est plus bas que le plan du lidar : l'information n'est pas dans les mesures | À ajouter à la main (étiquettes, carte sémantique) |

### Carte sémantique (choix de Yohan : garder la table, l'armoire et le tableau)

| Problème | Cause | Correction |
| --- | --- | --- |
| Objets détectés qui ne correspondent pas à la réalité (bureaux, meubles le long des murs) | La détection garde tout segment vu par 2 scans hors des murs | `data/reference/etiquettes.json` : table, armoire et tableau nommés avec leur zone ; les autres objets sont retirés de la carte vectorielle (`applyLabels`), la grid map garde ce qu'elle voit |
| « Sur roulettes » invisible pour le lidar | Information hors du plan de mesure | Étiquettes « Table · sur roulettes », « Armoire · posée au sol », « Tableau · sur 2 roues » sur la carte (application, figures 06 et 07, aperçu SVG) |
| Carte vectorielle moins bonne pour la localisation (15/18) | Moins de repères le long des murs encombrés | Grid map de référence par défaut (18/18) |
| Scans perdus au rechargement de la page | L'application ne garde rien | Bouton « Scans de la salle » : les 18 scans à leurs poses recalées, avec les étiquettes |
| Zones sombres et points noirs dans la vue propre, sans réalité | Dessous des bureaux invisible depuis le sol (inconnu), pieds de chaises gardés comme obstacles | Plan sémantique (`semanticPlan`) : dans la salle, seuls les murs et les 3 meubles ; l'extérieur ne passe pas par une ouverture de moins de 60 cm |
| Localisation dégradée sur ce plan (14/18) | Sans les pieds de bureaux, glissement de 47 cm le long des murs parallèles | Deux couches : plan pour l'affichage, grid map de localisation (tout ce que voit le lidar) pour le robot : 18/18 |

### Étape 3 du jour 2 : localisation globale

| Problème | Cause | Correction |
| --- | --- | --- |
| Test trop facile (18/18) | Les scans testés avaient servi à construire la carte | Banc d'essai « leave-one-out » : chaque scan sur une carte construite sans lui (`benchmark-global.mjs`) |
| Salle meublée simulée jugée ambiguë | Indicateur fondé sur le rapport des parts de points sur la carte : les murs, symétriques, dominent | Indicateur fondé sur les seuls points qui départagent les deux candidates : 3 échecs sur 3 signalés, 11 réponses fiables sur 11 justes |
| Scène de test presque symétrique | Le deuxième meuble était le symétrique du premier par rapport au centre | Étagère placée sans symétrie ; la salle vide sert de test d'ambiguïté |

Ajouts après lecture des documents du cours : localisation sur trois références et six
méthodes (dont le calcul direct inspiré de l'article « Mars Attacks! »), coins, oubli des
segments rarement vus, banc d'essai chiffré du jour 2.

## Remarques et points de vigilance

- Les résultats chiffrés de précision viennent du jeu **simulé** (pose vraie connue). Sur les
  scans réels, la référence est la pose recalée : son incertitude se mesure en recalant deux fois.
- Le sens des angles et le montage du lidar doivent être vérifiés avec un objet de position
  connue (boîte devant le robot → 0°, à gauche → 90°).
- Les seuils (segmentation, fusion, filtres) ont été réglés sur la simulation (bruit σ = 1 cm) ;
  le bruit réel mesuré sur les segments est de 6 à 12 mm.
- Le lidar ne voit pas les vitres : des faisceaux sortent de la salle par les fenêtres (retirés
  dans la carte propre).
- Robot immobile pendant un scan : la librairie ne date pas chaque point, un déplacement
  pendant le tour déforme le scan (l'article du prof compense ce mouvement avec l'odométrie).
