# Énoncé du TP (copie de la page du module)

Copie de l'énoncé tel que publié pour le module, gardée ici comme référence pour la grille de rendu (`README.md`, section « Où trouver chaque demande de l'énoncé »).

---

Ouvert le : lundi 2 novembre 2026, 07:45
À rendre : dimanche 22 novembre 2026, 23:59

Maintenant, à vous de jouer pour exploiter les données du lidar LD06 et vous lancer dans l’aventure de la cartographie et du mapping. Le travail à faire est individuel, mais vous pouvez discuter et échanger avec vos collègues.  
Ce module a pour objectif de vous amener à comprendre, traiter et exploiter les données issues du lidar LD06, afin de développer progressivement des outils de cartographie et de localisation pour un robot mobile.

Sur votre ordinateur, vous êtes libres de choisir votre environnement de développement :  
Python, C++ avec Qt, Processing (Java), tableur Excel, ou même une application Web (HTML/CSS/JS).  
Le choix dépend de vos compétences et de vos préférences. C’est à vous de choisir vos outils pour mener à bien vos objectifs.

Dans les grandes lignes, vous devrez :

- Afficher les données du lidar sous forme de nuages de points

- Utiliser les mesures pour construire une carte d’occupation (grid map)

- Transformer les nuages de points en segments pour produire une carte vectorielle

- Mettre en œuvre un algorithme de localisation à partir d’un scan lidar sur :
  
  - une carte d’occupation
  
  - une carte vectorielle
  
  - un scan de référence

- Atteindre l’objectif final : cartographier la salle de classe et être capable de localiser le robot dans cette carte.

- Utilisez git pour le développement du projet.

---
TP – Jour 1 : Extraction, Visualisation et Cartographie
1. Extraction des données lidar

- Consultez le dossier Ressources dédié au LD06.

- Apprenez à récupérer les données brutes du lidar.

- Chaque étudiant doit extraire ses propres jeux de scans réels dans la salle (coordonnées polaires). Ces données seront réutilisées pour la suite.
2. Visualisation des nuages de points

- Développez un outil permettant d’afficher vos scans sous forme de points cartésiens. L’outil doit permettre d’afficher plusieurs scans simultanément.

- Les différents scans doivent apparaître dans des couleurs distinctes.

- Vous devez pouvoir modifier la pose d’origine de chaque scan indépendamment, pour les déplacer manuellement sur la carte.
3. Construction d’une grid map 2D

- Lisez les étapes décrites dans le document « Construction d’une grid map 2D à partir d’un scan lidar »  pour comprendre comment construire une grid map à partir d'un scan et mettre à jour une grid map existante.

- Créez un outil qui transforme un scan + pose en carte d’occupation.

- Créez un outil qui prend une carte d'occupation, un scan + pose et qui doit mettre à jour la carte d'occupation (cellules traversées + cellules d’impact).

- Expliquez clairement votre démarche (choix techniques, hypothèses, difficultés, limites) à chaque étape.
4. Extraction et manipulation de segments

- Développez un outil capable de 
  - Transformer un nuage de points en un ensemble de segments.
  - Calculer l'angle entre deux segments
  - Dire si deux segments sont " proches" aussi bien " angulairement" qu'en position
  - Fusionner deux segments " proches"
  - Calculer l'intersection de deux segments
  - Découper un segment traversé par un autre segment en conservant le plus long côté.

Pour ce découpage, exigez une intersection située sur les deux segments et strictement à l'intérieur du segment à couper. Une intersection à une extrémité ne change rien. Les segments colinéaires ne sont pas découpés par cette opération. En cas de longueurs égales à la tolérance près, conservez le côté contenant le premier point du segment. Définissez une tolérance numérique et testez ces cas limites.
5. Construction d’une carte vectorielle

- Utilisez les fonctions développées précédemment pour construire une carte vectorielle.

- Expliquez, comme pour la grid map, les étapes suivies d’après le document « Construction d’une carte vectorielle 2D à partir d’un scan lidar ».

- Vous êtes libres :
  
  - d’intégrer cette étape dans l’outil du point 3,
  
  - ou d’en créer un distinct.

Que cela soit pour la cartographie en grid map ou en vectorielle, vous êtes encouragés à ajouter des propriétés supplémentaires à vos cartes : durée de vie des cellules, filtrages, pondérations, etc.  
Mais commencez simple puis complexifiez progressivement.

---
TP – Jour 2 : Localisation (Scan Matching)

Minimum attendu : une méthode fonctionnelle pour chacun des trois types de référence, avec une évaluation chiffrée. Les listes suivantes sont des pistes au choix, pas une obligation de tout implémenter. Les autres méthodes constituent des approfondissements facultatifs.
1. Localisation sur grid map

Mettez en œuvre un ou plusieurs algorithmes de localisation fonctionnant sur une carte d’occupation. Le scan à comparer doit représenter une zone couverte par la grid map.

- Recherche exhaustive (brute force)

- Descente de gradient

- Multi-résolution

- Probabiliste ...
2. Localisation sur carte vectorielle

Mettez en œuvre un ou plusieurs algorithmes de localisation fonctionnant sur une carte vectorielle. Le scan à comparer doit représenter une zone couverte par la carte vectorielle.

- Recherche exhaustive (brute force)

- Descente de gradient

- Multi-résolution

- Probabiliste 

- Calcul direct ?
3. Localisation entre scans

Mettez en œuvre un ou plusieurs algorithmes de localisation permettant de localiser un scan par rapport à un scan de référence. Dans ce cas le scan à comparer doit être pris près de la position où a été pris le scan de référence.

Quelques exemples d’algorithmes que vous pouvez utiliser :

- Recherche exhaustive (brute force)

- Descente de gradient

- Multi-résolution

- Probabiliste

- Calcul direct ?

Comparez les méthodes sur les mêmes scans et avec les mêmes poses initiales. Mesurez l'erreur de position (m), l'erreur angulaire (degrés), le temps de calcul et le taux de réussite selon des seuils que vous expliciterez. Faites varier la pose initiale pour mesurer la sensibilité à l'initialisation. Utilisez des poses connues en simulation et indiquez l'incertitude des références mesurées sur le robot.

---
TP – Jour 3 : Intégration SLAM & Cartographie de la salle

- Finalisez les travaux des jours 1 et 2.

- Combinez vos outils pour obtenir un système capable de localisation + cartographie simultanées (début de SLAM).

- Utilisez vos outils pour produire une **carte complète de la salle de classe**.
Bonus

Si vous réussissez, essayez de déplacer le robot automatiquement pour construire la carte en temps réel.

---
Livrable

Votre livrable final doit être un dossier .zip contenant :

- Un lien vers votre dépôt Git accessible à l’évaluateur, ou une copie du dépôt incluant son historique, afin de vérifier les commits demandés.

- L’ensemble des outils que vous avez développés, accompagnés si nécessaire des instructions permettant de les tester.

- Votre conclusion personnelle sur le module, présentée sous forme de synthèse comprenant notamment :
  
  - pour chaque outil :  
    les résultats visuels obtenus, les choix que vous avez effectués, les fonctionnalités que vous avez intégrées et les méthodes que vous avez implémentées.
  
  - un court récapitulatif de ce que vous retenez du module :  
    ce que vous avez apprécié, ce que vous avez moins apprécié, et ce que vous amélioreriez si vous en aviez la possibilité.
Remarques complémentaires

- Vous pourrez (et devrez) tester vos outils afin de vérifier vos résultats. Pour cela, vous pouvez utiliser :
  
  - des données réelles récupérées via le lidar LD06,
  
  - des données artificielles générées avec mapTool (dans le dossier Tools).

- De nombreuses ressources sont disponibles : documentation, exemples, bibliothèques, compléments de cours… N’hésitez pas à les consulter.

- Le robot utilise un ESP32, un microcontrôleur doté du Wi-Fi.  
  Vous pourrez donc transmettre les données sans fil vers votre ordinateur. L’outil RobotControl, dans le dossier Tools, peut vous aider à piloter le robot sans fil et à extraire les données du lidar en temps réel.

- L’usage de Git et GitHub est imposé. N’hésitez pas à faire des commits réguliers, au moins un commit toutes les 1 h de TP.

- Vous avez le droit d’échanger des idées avec vos collègues et de vous entraider, mais le travail reste individuel.

- Une vidéo pour vous inspirer qui montre le résultat de la mise en oeuvre d'un slam  en carte vectorielle, sur un robot mecanum équipé d'un RPLidar A2 et d'un microcontrôleur PIC32 cadencé à 80Mhz: [Démonstration SLAM sur PIC 32](https://www.youtube.com/watch?v=Pq5y7eDiReU)
