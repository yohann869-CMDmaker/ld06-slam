// Construit la carte de référence de la salle (jour 2) à partir des scans réels :
//   1. lecture des scans et de leurs poses (posées à la main dans l'application) ;
//   2. recalage automatique (multi-résolution + ICP, chaque scan face aux autres) ;
//   3. grid map et carte vectorielle, puis versions propres et coins ;
//   4. enregistrement : data/reference/carte_reference.json (chargeable dans l'application),
//      data/reference/poses_recalees.csv et un aperçu docs/img/j2_carte_reference.png.
// Usage : node scripts/build-reference.mjs [dossier des scans, défaut data/real]
import { mkdir, readdir, readFile, writeFile } from 'node:fs/promises';
import { fileURLToPath } from 'node:url';
import { parseScanCSV, scanToLocalXY } from '../src/core/scan.js';
import { gridFromScan } from '../src/core/gridmap.js';
import { VectorMap } from '../src/core/vectormap.js';
import { autoAlign, consistency } from '../src/core/matching.js';
import { cleanGrid, cleanVectorMap, findCorners, segmentsNearFloor } from '../src/core/cleanup.js';
import { makeReferenceMap, referenceBounds, serializeReferenceMap } from '../src/core/refmap.js';
import { hex, Raster } from './lib/raster.mjs';

const ROOT = fileURLToPath(new URL('../', import.meta.url));
const dir = process.argv[2] ?? ROOT + 'data/real';
const files = (await readdir(dir)).filter((f) => f.startsWith('scan_') && /\.(csv|txt)$/.test(f)).sort();
if (files.length < 2) {
  console.error(`Il faut au moins 2 scans dans ${dir}`);
  process.exit(1);
}
const scans = [];
for (const f of files) scans.push(parseScanCSV(await readFile(`${dir}/${f}`, 'utf8'), f));

// 2. recalage
const items = scans.map((s) => ({ points: scanToLocalXY(s), pose: s.pose }));
const steps = autoAlign(items);
let r;
while (!(r = steps.next()).done);
const poses = r.value;
const mean = (a) => Math.round((100 * a.reduce((x, y) => x + y, 0)) / a.length);
const before = mean(consistency(items.map((i) => i.points), items.map((i) => i.pose)));
const after = mean(consistency(items.map((i) => i.points), poses));

// 3. cartes brutes puis propres (sans découpe par les faisceaux : sur données réelles, une petite
//    erreur de pose suffirait à découper des murs)
const grid = gridFromScan(scans[0], poses[0]);
scans.slice(1).forEach((s, k) => grid.integrateScan(s, poses[k + 1]));
const vm = new VectorMap([], { absence: false });
scans.forEach((s, k) => vm.integrateScan(s, poses[k]));
const gridClean = cleanGrid(grid, poses);
const vc = cleanVectorMap(vm.segments);
vc.segments = segmentsNearFloor(vc.segments, gridClean);
const corners = findCorners(vc.segments);

// 4. enregistrement
const ref = makeReferenceMap({
  name: `Salle de classe (${scans.length} scans)`,
  grid: gridClean,
  segments: vc.segments,
  corners,
  scans: scans.map((s, k) => ({ name: s.name, pose: poses[k] })),
  clean: true,
});
await mkdir(ROOT + 'data/reference', { recursive: true });
await writeFile(ROOT + 'data/reference/carte_reference.json', serializeReferenceMap(ref));
await writeFile(
  ROOT + 'data/reference/poses_recalees.csv',
  ['scan,x_m,y_m,theta_deg', ...scans.map((s, k) => `${s.name},${poses[k].x.toFixed(4)},${poses[k].y.toFixed(4)},${poses[k].theta.toFixed(2)}`)].join('\n') + '\n',
);

const b = referenceBounds(ref, 0.4);
const img = new Raster(b, 70, '#d6dbe0');
for (let j = 0; j < gridClean.height; j++) {
  for (let i = 0; i < gridClean.width; i++) {
    const c = gridClean.classify(gridClean.index(i, j));
    if (c === 0) continue;
    const col = c === -1 ? [255, 255, 255] : [134, 142, 150];
    for (let y = Math.round(img.Y(gridClean.originY + (j + 1) * gridClean.res)); y < Math.round(img.Y(gridClean.originY + j * gridClean.res)); y++) {
      for (let x = Math.round(img.X(gridClean.originX + i * gridClean.res)); x < Math.round(img.X(gridClean.originX + (i + 1) * gridClean.res)); x++) img.blend(x, y, col);
    }
  }
}
img.metric(1);
for (const s of vc.segments) img.line(s.x1, s.y1, s.x2, s.y2, hex('#1d2327'), 4);
for (const c of corners) img.dot(c.x, c.y, 5, hex('#f59e0b'));
poses.forEach((p) => img.robot(p, hex('#1c7ed6'), 5));
await mkdir(ROOT + 'docs/img', { recursive: true });
await writeFile(ROOT + 'docs/img/j2_carte_reference.png', img.png());

const st = gridClean.stats();
console.log(`${scans.length} scans recalés : ${before} % -> ${after} % de points cohérents`);
console.log(`Grid map : ${gridClean.width} x ${gridClean.height} cellules de ${gridClean.res} m (${st.free} libres, ${st.occupied} occupées)`);
console.log(`Carte vectorielle : ${vc.segments.length} segments, ${corners.length} coins, direction des murs ${vc.dominant.toFixed(1)}°`);
console.log(`Emprise : ${(b.maxX - b.minX).toFixed(1)} m x ${(b.maxY - b.minY).toFixed(1)} m`);
console.log('Écrit : data/reference/carte_reference.json, data/reference/poses_recalees.csv, docs/img/j2_carte_reference.png');
