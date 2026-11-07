# Prompts pour Google Stitch (maquettes des deux interfaces)

Deux prompts à coller dans Google Stitch : l'application étudiants (Labo Carto, dossier `carto-edu`)
et l'application du TP (`ld06-slam/app`). Rédigés en anglais (Stitch les suit mieux), avec une
interface demandée en français. Stitch produit des maquettes et du HTML/CSS : il faudra ensuite
rebrancher ce HTML sur notre code JavaScript (les algorithmes et le dessin du canevas ne changent pas).

Conseil : coller d'abord le prompt entier, puis demander les écrans un par un (« now the Virtual robot
screen », « now the mobile version »), et garder les mêmes couleurs d'un écran à l'autre.

## 1. Labo Carto — application pour apprendre le SLAM

```text
Design a responsive web app called "Labo Carto": an interactive, hands-on course that teaches
engineering students how a mobile robot with a 2D lidar builds a map and localizes itself (SLAM).
All interface text must be in French. No login, no backend: it is a static web page.

Audience and tone: engineering / master students and their teachers. Calm, precise, scientific,
friendly. Think "interactive textbook + lab bench", not a game, not a marketing site.

Core layout (desktop, 1440 px):
- Top bar: logo "Labo Carto" with the tagline "cartographie et localisation, pas à pas", then a
  horizontal row of lesson tabs (pill buttons, the current one filled): "Par où commencer",
  "1. Du lidar aux points", "2. Grid map", "3. ICP", "4. Filtre particulaire",
  "5. Robot virtuel et SLAM", "6. Graphe de poses", "7. Mes données".
- Main area (about 65 % width): a large canvas showing a top-down 2D view of a room or a robot
  trajectory on a light metric grid (1 m squares). Show a robot as a circle with a heading line, lidar
  points as small dots, an occupancy grid (white = free, dark = occupied, transparent = unknown),
  and colored trajectories. Under the canvas: a one-line legend made of small color chips, and a
  discreet hint "Molette : zoom · glisser le fond : déplacer la vue".
- Right panel (about 35 % width, scrollable): lesson title (h1), one-sentence summary, a
  "Réglages" card with sliders that show their value (for example "Portée du lidar 6 m"), toggles,
  dropdowns, primary and secondary buttons; a live "État" card with key metrics in big numbers;
  then the explanation: short sections with h2 headings, short paragraphs, a monospace formula
  block, and an "À essayer" checklist of 3 to 4 experiments.

Screens to design:
1. "Par où commencer" (home): a clickable roadmap diagram of the 7 lessons as connected cards with
   arrows (Lidar → points, then Grid map and Scan matching (ICP), then Localisation (particules) and
   Robot virtuel et SLAM, then Graphe de poses and Mes données). Each card shows a number, a title and
   a one-line question ("Où suis-je sur une carte connue ?"). Beside it: "Ce qu'il faut savoir avant"
   (3 bullets) and a compact table "Pourquoi un SLAM échoue" (situation / ce qui se passe / fiche).
2. "Robot virtuel et SLAM": the student drives a simulated robot with the keyboard (arrow keys or
   Z Q S D) or with an on-screen hold-to-drive D-pad (↶ ↑ ↷ ↓). The canvas compares three
   trajectories: green = vraie pose, dashed red = odométrie seule, blue = SLAM, with keyframes as blue
   dots and loop closures as orange links. Controls: scene dropdown (Salle, Boucle, Couloir), map
   dropdown (carte du SLAM / carte avec l'odométrie seule / les deux), toggles (fermeture de boucle,
   graphe de poses), sliders (portée du lidar, glissement des roues, dérive de cap, vitesse), buttons
   "Recommencer" and "Exporter la session (.zip)". The État card shows "Erreur de l'odométrie seule :
   210 cm" next to "Erreur du SLAM : 2 cm", distance travelled, keyframes, loops, and a challenge badge
   ("Défi : revenir au départ avec moins de 15 cm d'erreur" with a green "Réussi" or orange
   "Pas encore" state). Add a toast for events like "Boucle fermée : correction de 34 cm".
3. "Graphe de poses": a pose graph drawn as rubber bands. Nodes are small robot icons, edges are lines
   whose thickness and red intensity show their tension; the loop-closure edge is dashed orange; a
   faint green line is the true trajectory; the first node is black (fixed). Controls: sliders (dérive
   de cap, bruit, confiance dans la boucle σ), toggles (fermeture de boucle, fausse boucle), buttons
   "1 itération" (primary), "Optimiser (15)", "Repartir de l'odométrie", "Autre tirage". The État card
   shows the graph error Σ eᵀΩe, the mean error to the truth in cm, and a tiny sparkline or chip list of
   the error per iteration.
4. "Mes données": a drop zone "Déposez vos scans (CSV) ou une session (.zip)" with a button
   "Ouvrir des fichiers…", and a card "Session d'exemple : vrai robot, 159 scans, 57 m". A dropdown
   "Rejouer : aux poses du fichier / en relançant le SLAM", a slider "Bruit ajouté à l'odométrie".
   Under the canvas, a media-style playback bar: play/pause, "Scan suivant", "Tout", a scrubber with
   "Scan 42 / 159", and a speed control (scans/s). The État card shows the current file name, the
   pose, the SLAM state (suivi / perdu), and the gap to the recorded poses in cm.
5. A lesson in "step by step" mode (for example "2. Grid map"): buttons "Faisceau suivant",
   "Finir le scan", a small per-cell inspector on hover ("Cellule (12, 7) : l = 1.39, p = 0.80"), and
   a formula block.
6. Mobile version (390 px) of the Robot virtuel screen: the canvas on top (16:10), a sticky bottom
   D-pad, and the panel below as collapsible sections ("Réglages", "État", "Explication").

Visual system:
- Light and dark themes. Light: background #f6f5f1, panels white, ink #1d2327, muted #5f6b73,
  lines #dfe2e4, accent teal #0b7a75. Dark: background #15191c, panels #1d2327, ink #e8ecee,
  accent #3cc2bb.
- Fixed semantic colors used everywhere: green #2b8a3e = truth, red #c92a2a = odometry / errors,
  blue #1c7ed6 = SLAM estimate and keyframes, orange #e8590c = loop closures and current scan.
- Typography: Inter or system UI; monospace (JetBrains Mono) for formulas and numbers in tables.
  Generous line height, short paragraphs.
- Components: rounded 8 px cards with a thin border (no heavy shadows), pill tabs, sliders with the
  value on the right of the label, accessible focus rings, WCAG AA contrast, keyboard hints shown as
  small key caps (⌨ ↑ ↓ ← →).
- The canvas must stay the hero of every screen: never cover it with large overlays.
Deliver clean HTML + CSS (Tailwind allowed), no JavaScript framework, with a <canvas id="view">
element in the main area and empty containers for the controls and explanation that our own
JavaScript will fill.
```

## 2. LD06 Cartographie — application du TP (vrai robot)

```text
Design a desktop web tool called "LD06 Cartographie" used by a team of engineering students during a
robotics lab: they record scans from a real LD06 2D lidar on an ESP32 robot, build occupancy-grid and
vector maps, localize the robot and run SLAM live while driving it. All interface text in French.
Style: a dense but readable engineering tool, in the spirit of Foxglove Studio, VS Code or a
ground-control station. Dark theme first, light theme too.

Layout (1440 × 900 and up):
- Top status bar: app name; a connection pill for the robot link ("USB · relais live.py connecté",
  "Wi-Fi · 192.168.4.1", or red "déconnecté"); live robot pose (x, y, θ); scan rate ("5,8 scans/s");
  a SLAM state badge ("suivi" green, "relocalisé" blue, "perdu" red); keyframe and loop counters;
  buttons "Recadrer" and "Capture PNG"; theme switch.
- Left sidebar (380 px, scrollable, accordion sections with icons, each can collapse):
  1. "Scans": buttons "Ouvrir des CSV", "Coller un scan", "Scans de la salle", "Jeu simulé",
     "Trajet simulé"; a live capture block (USB or Wi-Fi) with "Ajouter ce scan (Espace)"; the list
     of scans as rows with a color swatch, visibility eye, name, and an "élevé" tag; the selected scan
     shows editable pose fields x, y, θ with steppers, "Pose du fichier", "Exporter avec cette pose";
     buttons "Recaler automatiquement" (primary) and "Enregistrer tous les scans…".
  2. "Filtre des points": min/max distance, intensity threshold.
  3. "Grid map": resolution, l_occ / l_libre / bounds, buttons (scan sélectionné → carte, mettre à
     jour, tous les scans, effacer, décroissance, filtrer isolées) and exports (CSV, PNG, ROS PGM+YAML).
  4. "Carte vectorielle": split-and-merge threshold, merge tolerance, build/update/clear, export TXT.
  5. "Carte de référence (jour 2)": load, freeze current maps, save; semantic labels (table, armoire,
     tableau).
  6. "Localisation (jour 2)": reference type (grid / vector / scan), method dropdown (force brute,
     gradient, multi-résolution, probabiliste, ICP, PL-ICP, NDT, Gauss-Newton), initial pose,
     "Localiser le scan sélectionné" (primary), "Comparer les méthodes" (opens a results table with
     error, time, success rate, sortable), "Localisation globale (pose inconnue)", "Exporter les
     résultats".
  7. "SLAM (jour 3)": "SLAM sur les scans visibles" with a progress bar; a "En direct" toggle.
  8. "Robot : pilotage et exploration": link selector (USB / Wi-Fi) with auto-detection message, robot
     IP, "Démarrer le SLAM" (primary, turns into "Arrêter le SLAM"), motor speed slider (60–255),
     "Odométrie des commandes" toggle, a 2 × 3 hold-to-drive pad (↶ Tourner, ↑ Avancer, ↷ Tourner,
     ← Glisser, ↓ Reculer, → Glisser) with keyboard key caps U I O / J K L, a big red "Arrêt (Échap)"
     button, and an "Exploration autonome" block (simulated or real robot) with coverage %.
  9. "Exports" and "Affichage" (layers: points, grid, vector map, reference, trajectory, keyframes,
     loop closures, robot; colors; point size).
- Main canvas (rest of the screen): top-down metric view with a 1 m grid, zoom/pan, the occupancy
  grid, vector segments, colored scans, the robot pose with heading, the SLAM trajectory (blue),
  keyframes, loop closures (orange). Floating mini-toolbar in a corner (zoom +/−, fit, measure tool,
  layer toggles). A small coordinate readout of the cursor at the bottom.
- Bottom drawer (collapsible): a log / console with timestamped events ("Boucle fermée : 34 cm",
  "SLAM perdu : revenir doucement", "relais USB injoignable") and a results table tab.
- Toast notifications top-right. A keyboard-shortcut overlay opened with "?".

Visual system: dark background #111518, panels #1d2327, text #e8ecee, muted #9aa5ad, accent teal
#3cc2bb; semantic colors: blue SLAM #4dabf7, orange loop/current scan #ff922b, green OK #51cf66,
red error/stop #ff6b6b. Inter / system UI, JetBrains Mono for numbers and poses. Compact 32 px
controls, 8 px radius, thin borders, clear focus states, WCAG AA. The canvas is the hero.
Deliver clean HTML + CSS (Tailwind allowed), no JavaScript framework, a <canvas id="view"> in the
main area, and keep one container per sidebar section so our existing JavaScript can be wired back.
```
