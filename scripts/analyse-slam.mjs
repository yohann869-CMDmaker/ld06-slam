// Analyse d'une session de SLAM réelle : carte obtenue, comparée à la carte de référence de la salle.
//   1. lecture des scans enregistrés par l'application (pose SLAM dans chaque fichier) ;
//   2. grid map et carte vectorielle à ces poses ;
//   3. recalage de toute la carte du SLAM sur la carte de référence (repères différents : le SLAM
//      part de (0, 0, 0)) : localisation globale du premier scan, puis ICP de l'ensemble des points ;
//   4. accord : part des cellules occupées du SLAM à moins de 10 cm d'un mur ou meuble de la
//      référence, et part des murs de la référence vus par le SLAM (à moins de 10 cm d'un point) ;
//   5. figures : carte du SLAM, et superposition sur la référence ;
//   6. filtrage 1 (chaises, objets parasites) puis filtrage 2 + reconstruction du plan de la salle
//      (src/core/reconstruct.js), comparés à la référence.
// Usage : node scripts/analyse-slam.mjs data/slam_usb_20261104 [--nom slam_usb]
//   -> docs/<nom>.md, docs/img/<nom>_*.png
import { readdir, readFile, writeFile } from 'node:fs/promises';
import { basename, join } from 'node:path';
import { fileURLToPath } from 'node:url';
import { parseScanCSV, scanToLocalXY, scanToWorldXY } from '../src/core/scan.js';
import { gridFromScan } from '../src/core/gridmap.js';
import { VectorMap } from '../src/core/vectormap.js';
import { gridReference, localize, scanReference, vectorReference } from '../src/core/localization.js';
import { globalLocalize } from '../src/core/globalloc.js';
import { dominantDirection } from '../src/core/cleanup.js';
import { parseReferenceMap } from '../src/core/refmap.js';
import { composePoses, DEG, invertPose } from '../src/core/pose.js';
import { subsample } from '../src/core/matching.js';
import { filterClutter, reconstructRoom } from '../src/core/reconstruct.js';
import { hex, Raster } from './lib/raster.mjs';

const ROOT = fileURLToPath(new URL('../', import.meta.url));
const args = process.argv.slice(2);
const dir = args.find((a) => !a.startsWith('--'));
if (!dir) {
  console.error('Usage : node scripts/analyse-slam.mjs <dossier des scans> [--nom slam_usb]');
  process.exit(1);
}
const name = args.includes('--nom') ? args[args.indexOf('--nom') + 1] : basename(dir);

// 1. scans et poses SLAM
const files = (await readdir(join(ROOT, dir))).filter((f) => /^scan_.*\.csv$/.test(f)).sort();
const scans = [];
for (const f of files) scans.push(parseScanCSV(await readFile(join(ROOT, dir, f), 'utf8'), f));
const length = scans.slice(1).reduce((a, s, k) => a + Math.hypot(s.pose.x - scans[k].pose.x, s.pose.y - scans[k].pose.y), 0);

// 2. cartes aux poses du SLAM
const grid = gridFromScan(scans[0], scans[0].pose);
for (const s of scans.slice(1)) grid.integrateScan(s, s.pose);
const vector = new VectorMap([], { absence: false });
for (const s of scans) vector.integrateScan(s, s.pose);

// 3. recalage sur la référence
const reference = parseReferenceMap(await readFile(join(ROOT, 'data/reference/carte_reference.json'), 'utf8'));
const refGrid = gridReference(reference.grid);
const refVec = vectorReference(reference.segments);
const walls = reference.segments.filter((w) => w.kind !== 'objet');
let best = null;
for (const s of scans.slice(0, 12)) {
  // le premier scan qui se localise de façon fiable donne le changement de repère
  const g = globalLocalize(refGrid, scanToLocalXY(s), { mapDominant: dominantDirection(walls) });
  if (!g.ambiguous && (!best || g.inliers > best.g.inliers)) best = { s, g };
  if (best && best.g.inliers > 0.6) break;
}
if (!best) {
  console.error('Aucun scan ne se localise de façon fiable sur la carte de référence');
  process.exit(1);
}
let T = composePoses(best.g.pose, invertPose(best.s.pose)); // repère SLAM -> repère de référence
// tous les points du SLAM, vus comme un seul « scan » pris de l'origine du repère SLAM
const cloud = subsample(scans.flatMap((s) => scanToWorldXY(s, s.pose)), 4000);
const fit = localize(refVec, cloud, T, 'icp');
T = fit.pose;
const toRef = (p) => {
  const c = Math.cos(T.theta * DEG);
  const s = Math.sin(T.theta * DEG);
  return { x: T.x + c * p.x - s * p.y, y: T.y + s * p.x + c * p.y };
};

// 4. accord avec la référence
const refField = refVec.field(0.025);
const occ = grid.occupiedCells().map(toRef);
const agree = occ.filter((c) => refField.at(c.x, c.y) <= 0.1).length / Math.max(occ.length, 1);
const slamField = scanReference(cloud.map(toRef)).field(0.025);
let seen = 0;
let total = 0;
for (const w of walls) {
  const L = Math.hypot(w.x2 - w.x1, w.y2 - w.y1);
  const n = Math.max(1, Math.ceil(L / 0.05));
  for (let k = 0; k <= n; k++) {
    total++;
    if (slamField.at(w.x1 + ((w.x2 - w.x1) * k) / n, w.y1 + ((w.y2 - w.y1) * k) / n) <= 0.1) seen++;
  }
}
const coverage = seen / total;
const dists = occ.map((c) => refField.at(c.x, c.y)).filter((d) => d <= 0.3).sort((a, b) => a - b);
const medDist = dists.length ? dists[dists.length >> 1] : NaN;

// 5. figures
const xs = reference.segments.flatMap((w) => [w.x1, w.x2]).concat(occ.map((c) => c.x));
const ys = reference.segments.flatMap((w) => [w.y1, w.y2]).concat(occ.map((c) => c.y));
const box = { minX: Math.min(...xs) - 0.5, maxX: Math.max(...xs) + 0.5, minY: Math.min(...ys) - 0.5, maxY: Math.max(...ys) + 0.5 };
{
  const img = new Raster({ minX: grid.originX, minY: grid.originY, maxX: grid.originX + grid.width * grid.res, maxY: grid.originY + grid.height * grid.res }, 45, '#fbfbf9');
  img.grid(grid);
  img.metric(1);
  for (let k = 1; k < scans.length; k++) img.line(scans[k - 1].pose.x, scans[k - 1].pose.y, scans[k].pose.x, scans[k].pose.y, hex('#1c7ed6'), 2);
  await writeFile(join(ROOT, 'docs/img', `${name}_carte.png`), img.png());
}
{
  const img = new Raster(box, 45, '#fbfbf9');
  img.metric(1);
  for (const w of reference.segments) img.line(w.x1, w.y1, w.x2, w.y2, hex('#adb5bd'), 4);
  for (const p of cloud.map(toRef)) img.dot(p.x, p.y, 1.3, hex('#1d2327'), 0.6);
  const traj = scans.map((s) => toRef(s.pose));
  for (let k = 1; k < traj.length; k++) img.line(traj[k - 1].x, traj[k - 1].y, traj[k].x, traj[k].y, hex('#1c7ed6'), 2);
  await writeFile(join(ROOT, 'docs/img', `${name}_superposition.png`), img.png());
}

// 6. filtrage 1, puis filtrage 2 + reconstruction
const clutter = filterClutter(grid);
const room = reconstructRoom(clutter.grid, vector.segments, { seeds: scans.map((sc) => sc.pose) });
const len = (w) => Math.hypot(w.x2 - w.x1, w.y2 - w.y1);
const observed = room.walls.filter((w) => w.observed).reduce((a, w) => a + len(w), 0);
const rebuilt = room.walls.filter((w) => !w.observed).reduce((a, w) => a + len(w), 0);
// écart des murs reconstruits aux murs de la référence (points tous les 5 cm, dans le repère de référence)
const wallDist = (list) => {
  const d = [];
  for (const w of list) {
    const n = Math.max(1, Math.ceil(len(w) / 0.05));
    for (let k = 0; k <= n; k++) {
      const p = toRef({ x: w.x1 + ((w.x2 - w.x1) * k) / n, y: w.y1 + ((w.y2 - w.y1) * k) / n });
      d.push(refField.at(p.x, p.y));
    }
  }
  return d.sort((a, b) => a - b);
};
const dObs = wallDist(room.walls.filter((w) => w.observed));
const dReb = wallDist(room.walls.filter((w) => !w.observed));
const med = (d) => (d.length ? d[d.length >> 1] : NaN);
const near = (d, t) => (d.length ? d.filter((x) => x <= t).length / d.length : NaN);
{
  // figure filtrage 1 : grille du SLAM, cellules retirées en rouge
  const img = new Raster({ minX: grid.originX, minY: grid.originY, maxX: grid.originX + grid.width * grid.res, maxY: grid.originY + grid.height * grid.res }, 45, '#fbfbf9');
  img.grid(clutter.grid);
  for (let j = 0; j < grid.height; j++) {
    for (let i = 0; i < grid.width; i++) {
      if (!clutter.removed[grid.index(i, j)]) continue;
      const c = grid.cellCenter(i, j);
      img.dot(c.x, c.y, 1.6, hex('#e03131'), 0.9);
    }
  }
  img.metric(1);
  await writeFile(join(ROOT, 'docs/img', `${name}_filtrage1.png`), img.png());
}
{
  // figure reconstruction : rectangles (intérieur vert pâle, complété orange), murs observés (noir),
  // reconstruits (rouge pointillé), meubles (bleu)
  const b = { minX: grid.originX, minY: grid.originY, maxX: grid.originX + grid.width * grid.res, maxY: grid.originY + grid.height * grid.res };
  const img = new Raster(b, 45, '#fbfbf9');
  const ang = (room.angle * Math.PI) / 180;
  const cs = Math.cos(ang);
  const sn = Math.sin(ang);
  for (let py = 0; py < img.height; py++) {
    for (let px = 0; px < img.width; px++) {
      const x = b.minX + (px + 0.5) / img.scale;
      const y = b.maxY - (py + 0.5) / img.scale;
      const u = cs * x + sn * y;
      const v = -sn * x + cs * y;
      const r = room.rects.find((q) => u >= q.x0 && u < q.x1 && v >= q.y0 && v < q.y1);
      if (!r || r.label === 'extérieur') continue;
      img.blend(px, py, hex(r.label === 'complété' ? '#ffd8a8' : '#d3f9d8'), 1);
    }
  }
  img.metric(1);
  for (let j = 0; j < grid.height; j++) for (let i = 0; i < grid.width; i++) if (clutter.grid.classify(grid.index(i, j)) === 1) { const c = grid.cellCenter(i, j); img.dot(c.x, c.y, 1, hex('#868e96'), 0.7); }
  for (const w of room.furniture) img.line(w.x1, w.y1, w.x2, w.y2, hex('#1c7ed6'), 3);
  for (const w of room.walls) img.line(w.x1, w.y1, w.x2, w.y2, hex(w.observed ? '#1d2327' : '#e03131'), 4, w.observed ? null : [8, 6]);
  await writeFile(join(ROOT, 'docs/img', `${name}_reconstruction.png`), img.png());
}
{
  // carte finale dans le repère de la référence, superposée
  const img = new Raster(box, 45, '#fbfbf9');
  img.metric(1);
  for (const w of reference.segments) img.line(w.x1, w.y1, w.x2, w.y2, hex('#adb5bd'), 7);
  const draw = (list, color, width, dash) => {
    for (const w of list) {
      const p = toRef({ x: w.x1, y: w.y1 });
      const q = toRef({ x: w.x2, y: w.y2 });
      img.line(p.x, p.y, q.x, q.y, hex(color), width, dash);
    }
  };
  draw(room.furniture, '#1c7ed6', 3);
  draw(room.walls.filter((w) => w.observed), '#1d2327', 3);
  draw(room.walls.filter((w) => !w.observed), '#e03131', 3, [8, 6]);
  await writeFile(join(ROOT, 'docs/img', `${name}_finale_superposition.png`), img.png());
}
{
  const img = new Raster({ minX: grid.originX, minY: grid.originY, maxX: grid.originX + grid.width * grid.res, maxY: grid.originY + grid.height * grid.res }, 45, '#fbfbf9');
  img.grid(room.grid);
  img.metric(1);
  await writeFile(join(ROOT, 'docs/img', `${name}_finale.png`), img.png());
}

{
  // trajet du robot sur la carte finale (repère de la référence) : du bleu clair (début) au bleu
  // foncé (fin), pose du robot tous les 20 scans clés, départ vert, arrivée rouge
  const img = new Raster(box, 45, '#fbfbf9');
  img.metric(1);
  const draw = (list, color, width, dash) => {
    for (const w of list) {
      const p = toRef({ x: w.x1, y: w.y1 });
      const q = toRef({ x: w.x2, y: w.y2 });
      img.line(p.x, p.y, q.x, q.y, hex(color), width, dash);
    }
  };
  draw(room.furniture, '#adb5bd', 3);
  draw(room.walls, '#1d2327', 3);
  const poseRef = (pose) => ({ ...toRef(pose), theta: pose.theta + T.theta });
  const traj = scans.map((sc) => poseRef(sc.pose));
  const mix = (t) => {
    const c0 = [165, 216, 255];
    const c1 = [24, 100, 171];
    return '#' + c0.map((v, k) => Math.round(v + (c1[k] - v) * t).toString(16).padStart(2, '0')).join('');
  };
  for (let k = 1; k < traj.length; k++) img.line(traj[k - 1].x, traj[k - 1].y, traj[k].x, traj[k].y, hex(mix(k / traj.length)), 3);
  for (let k = 20; k < traj.length - 1; k += 20) img.robot(traj[k], hex(mix(k / traj.length)), 6);
  img.robot(traj[0], hex('#2f9e44'), 8);
  img.robot(traj.at(-1), hex('#e03131'), 8);
  await writeFile(join(ROOT, 'docs/img', `${name}_trajet.png`), img.png());
}

const pct = (v) => `${(100 * v).toFixed(0)} %`;
const md = [
  `# Session de SLAM réelle : ${name}`,
  '',
  `Généré par \`node scripts/analyse-slam.mjs ${dir}\`. ${scans.length} scans clés enregistrés par le SLAM en direct,`,
  `trajet de ${length.toFixed(1)} m (somme des déplacements entre scans clés).`,
  '',
  '## Comparaison avec la carte de référence',
  '',
  'Le SLAM part de (0, 0, 0) : sa carte est d\'abord recalée sur la carte de référence (localisation',
  `globale du scan \`${best.s.name}\`, fiable, ${pct(best.g.inliers)} de ses points sur la carte, puis ICP de tous les points).`,
  `Changement de repère : (${T.x.toFixed(2)} m ; ${T.y.toFixed(2)} m ; ${T.theta.toFixed(1)}°).`,
  '',
  '| Mesure | Valeur |',
  '| --- | --- |',
  `| Cellules occupées du SLAM à moins de 10 cm d'un mur ou meuble de la référence | **${pct(agree)}** |`,
  `| Écart médian des cellules occupées proches (< 30 cm) à la référence | ${(100 * medDist).toFixed(1)} cm |`,
  `| Murs de la référence vus par le SLAM (point à moins de 10 cm) | **${pct(coverage)}** |`,
  '',
  `Les cellules hors référence (${pct(1 - agree)}) sont des objets absents de la carte de référence (chaises,`,
  'sacs, personnes, meubles déplacés) ou des erreurs du SLAM ; les murs non vus sont les zones où le robot',
  'n\'est pas allé.',
  '',
  '| Carte du SLAM (repère du SLAM, trajet en bleu) | Superposée à la référence (gris) |',
  '| --- | --- |',
  `| ![carte](img/${name}_carte.png) | ![superposition](img/${name}_superposition.png) |`,
  '',
  '## Suivi du robot',
  '',
  `Trajet du robot estimé par le SLAM, sur la carte finale : ${scans.length} scans clés, ${length.toFixed(1)} m,`,
  'du bleu clair (début) au bleu foncé (fin) ; départ en vert, arrivée en rouge, robot dessiné tous',
  'les 20 scans clés (trait = cap).',
  '',
  `![Trajet du robot](img/${name}_trajet.png)`,
  '',
  '## Filtrage 1 : chaises et objets parasites',
  '',
  'Les cellules occupées sont regroupées en objets (composantes connexes, après une dilatation d\'une',
  'cellule qui relie les morceaux d\'un même mur). Un objet de moins de 40 cm (pieds de chaises et de',
  'tables, sacs, jambes) est retiré, ainsi que toute cellule vue occupée par un seul scan.',
  '',
  `**${clutter.objects} objets retirés** (${clutter.removed.reduce((a, v) => a + v, 0)} cellules, en rouge), ${clutter.kept} gardés (murs, grands meubles).`,
  '',
  `![Filtrage 1](img/${name}_filtrage1.png)`,
  '',
  '## Filtrage 2 et reconstruction du plan',
  '',
  'Inspiré des travaux de reconstruction de plans en milieu encombré : droites des murs par',
  'histogrammes (Okorn et al., CMU, 2010), décomposition de l\'espace en cellules par ces droites et',
  'étiquetage intérieur / extérieur (Mura et al., 2014 ; Ochmann et al., 2016 et 2019), murs parallèles',
  'ou perpendiculaires (« monde Manhattan »). Détail : `src/core/reconstruct.js`.',
  '',
  `1. Direction des murs : ${room.angle.toFixed(1)}° (moyenne des segments de la carte vectorielle) ; carte redressée.`,
  `2. Droites : ${room.lines.u.length + room.lines.v.length} (pics des histogrammes des cellules occupées, au moins 1,5 m cumulé ; plus les`,
  '   limites de l\'espace vu libre, qui donnent la place d\'un mur que le lidar n\'a pas détecté ; plus les',
  '   décrochés : un court morceau de mur accroché à un mur extérieur, dont le coin est hors de la salle).',
  '3. Rectangles de l\'arrangement : intérieur s\'ils ont été vus libres (30 % de leurs cellules au moins),',
  '   dans le cadre des murs extérieurs, et atteignables depuis le trajet du robot sans traverser de mur',
  '   observé (diffusion depuis le trajet) : l\'espace vu par une porte, derrière un mur, est exclu. Un',
  '   meuble isolé (objet compact et épais, loin des murs, comme le bureau du prof) n\'arrête pas la diffusion.',
  '4. Complétion (orange) : un rectangle non vu, collé à l\'intérieur sans mur entre eux (coin',
  '   inaccessible, personnes), ou entouré d\'intérieur, devient intérieur ; une petite niche (4 m² au plus)',
  '   au bord de la salle, coupée du trajet par un obstacle (meuble, personnes), est rattachée à la salle.',
  '5. Contour : murs **observés** (points occupés le long, noir) ou **reconstruits** (rouge pointillé) ;',
  '   grands objets restants à l\'intérieur : meubles (bleu).',
  '',
  '| Mesure | Valeur |',
  '| --- | --- |',
  `| Surface de la salle reconstruite | ${room.area.toFixed(1)} m² |`,
  `| Murs observés | ${observed.toFixed(1)} m, à ${(100 * med(dObs)).toFixed(1)} cm (médiane) de la référence, ${pct(near(dObs, 0.15))} à moins de 15 cm |`,
  `| Murs reconstruits | ${rebuilt.toFixed(1)} m, à ${(100 * med(dReb)).toFixed(1)} cm (médiane) de la référence, ${pct(near(dReb, 0.15))} à moins de 15 cm |`,
  `| Meubles | ${room.furniture.length / 4} |`,
  '',
  '| Reconstruction (repère du SLAM) | Carte finale propre |',
  '| --- | --- |',
  `| ![reconstruction](img/${name}_reconstruction.png) | ![finale](img/${name}_finale.png) |`,
  '',
  `![Carte finale sur la référence](img/${name}_finale_superposition.png)`,
  '',
  '*Carte finale dans le repère de la référence : référence en gris épais, murs observés en noir,',
  'reconstruits en rouge pointillé, meubles en bleu.*',
  '',
];
await writeFile(join(ROOT, 'docs', `${name}.md`), md.join('\n'));
console.log(md.join('\n'));
