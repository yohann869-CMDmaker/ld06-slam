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
