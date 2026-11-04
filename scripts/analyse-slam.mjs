// Analyse d'une session de SLAM réelle : carte obtenue, comparée à la carte de référence de la salle.
//   1. lecture des scans enregistrés par l'application (pose SLAM dans chaque fichier) ;
//   2. grid map et carte vectorielle à ces poses ;
//   3. recalage de toute la carte du SLAM sur la carte de référence (repères différents : le SLAM
//      part de (0, 0, 0)) : localisation globale du premier scan, puis ICP de l'ensemble des points ;
//   4. accord : part des cellules occupées du SLAM à moins de 10 cm d'un mur ou meuble de la
//      référence, et part des murs de la référence vus par le SLAM (à moins de 10 cm d'un point) ;
//   5. figures : carte du SLAM, et superposition sur la référence.
// Usage : node scripts/analyse-slam.mjs data/slam_usb_20261104 [--nom slam_usb]
//   -> docs/<nom>.md, docs/img/<nom>_carte.png, docs/img/<nom>_superposition.png
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
];
await writeFile(join(ROOT, 'docs', `${name}.md`), md.join('\n'));
console.log(md.join('\n'));
