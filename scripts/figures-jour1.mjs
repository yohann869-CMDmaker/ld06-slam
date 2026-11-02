// Figures et mesures du compte rendu du jour 1 (docs/img/), à partir du jeu simulé.
// Usage : node scripts/figures-jour1.mjs
import { mkdir, readFile, writeFile } from 'node:fs/promises';
import { fileURLToPath } from 'node:url';
import { parseScanCSV, scanToWorldXY } from '../src/core/scan.js';
import { GridMap, gridFromScan } from '../src/core/gridmap.js';
import { VectorMap } from '../src/core/vectormap.js';
import { clusterize, extractSegments } from '../src/core/segmentation.js';
import { length, pointSegmentDistance } from '../src/core/geometry.js';
import { hex, Raster } from './lib/raster.mjs';

const ROOT = fileURLToPath(new URL('../', import.meta.url));
const SIM = ROOT + 'data/sim/';
const IMG = ROOT + 'docs/img/';
const PALETTE = ['#e8590c', '#1c7ed6', '#2f9e44', '#ae3ec9', '#f08c00', '#0c8599'];
const BOUNDS = { minX: -0.4, minY: -0.4, maxX: 8.4, maxY: 6.4 };
const SCALE = 90;

const index = JSON.parse(await readFile(SIM + 'index.json', 'utf8'));
const scans = [];
for (const s of index.scans) scans.push(parseScanCSV(await readFile(SIM + s.file, 'utf8'), s.file));
const sceneA = VectorMap.fromTXT(await readFile(SIM + 'scene_A.txt', 'utf8')).segments;
const sceneB = VectorMap.fromTXT(await readFile(SIM + 'scene_B.txt', 'utf8')).segments;

await mkdir(IMG, { recursive: true });
const save = (name, r) => writeFile(IMG + name, r.png());
const reference = (r, segs) => segs.forEach((s) => r.line(s.x1, s.y1, s.x2, s.y2, hex('#5d6770'), 1.5, [6, 4]));
const points = (r, scan, color, pose = scan.pose) => scanToWorldXY(scan, pose).forEach((p) => r.dot(p.x, p.y, 1.8, hex(color)));
const vector = (r, segs) => segs.forEach((s) => {
  r.line(s.x1, s.y1, s.x2, s.y2, hex('#d6336c'), 3.5);
  r.dot(s.x1, s.y1, 3, hex('#d6336c'));
  r.dot(s.x2, s.y2, 3, hex('#d6336c'));
});
const distToScene = (scene, x, y) => Math.min(...scene.map((s) => pointSegmentDistance(s, x, y)));

// 1. Nuages de points superposés, une couleur par scan
{
  const r = new Raster(BOUNDS, SCALE);
  r.metric(1);
  reference(r, sceneA);
  scans.forEach((s, k) => points(r, s, PALETTE[k]));
  scans.forEach((s, k) => r.robot(s.pose, hex(PALETTE[k])));
  await save('j1_nuages.png', r);
}

// 2. Même chose avec la pose du scan 2 faussée (+0,30 m en x, +6°) : les murs se dédoublent
{
  const r = new Raster(BOUNDS, SCALE);
  r.metric(1);
  const wrong = { ...scans[1].pose, x: scans[1].pose.x + 0.3, theta: scans[1].pose.theta + 6 };
  points(r, scans[0], PALETTE[0]);
  points(r, scans[1], PALETTE[1], wrong);
  r.robot(scans[0].pose, hex(PALETTE[0]));
  r.robot(wrong, hex(PALETTE[1]));
  await save('j1_pose_fausse.png', r);
}

// 3. Grid map depuis un seul scan, puis mise à jour avec les 5 autres
const params = { resolution: 0.05 };
const g1 = gridFromScan(scans[0], scans[0].pose, params);
{
  const r = new Raster(BOUNDS, SCALE);
  r.grid(g1);
  r.metric(1);
  r.robot(scans[0].pose, hex(PALETTE[0]));
  await save('j1_grid_scan1.png', r);
}
const gAll = g1.clone();
for (const s of scans.slice(1)) gAll.integrateScan(s);
{
  const r = new Raster(BOUNDS, SCALE);
  r.grid(gAll);
  r.metric(1);
  scans.forEach((s, k) => r.robot(s.pose, hex(PALETTE[k])));
  await save('j1_grid_tous.png', r);
}

// 4. Même grid map sans faisceaux intermédiaires ni marge avant l'impact
const gRaw = gridFromScan(scans[0], scans[0].pose, { ...params, fillGaps: false, freeMargin: 0 });
for (const s of scans.slice(1)) gRaw.integrateScan(s);
{
  const r = new Raster(BOUNDS, SCALE);
  r.grid(gRaw);
  r.metric(1);
  await save('j1_grid_brute.png', r);
}

// 5. Segmentation du scan 1 : clusters (couleurs alternées) et segments extraits
{
  const r = new Raster(BOUNDS, SCALE);
  r.metric(1);
  reference(r, sceneA);
  const pts = scanToWorldXY(scans[0]);
  const colors = ['#1c7ed6', '#e8590c', '#2f9e44', '#ae3ec9'];
  clusterize(pts).forEach((c, k) => c.forEach((p) => r.dot(p.x, p.y, 2.2, hex(colors[k % colors.length]))));
  vector(r, extractSegments(pts).segments);
  r.robot(scans[0].pose, hex('#1d2327'));
  await save('j1_segments_scan1.png', r);
}

// 6. Carte vectorielle après les scans 1-4 (carton présent) puis après les 6 (carton retiré)
const vmap = new VectorMap();
const log = [];
for (const [k, s] of scans.entries()) {
  log.push({ scan: s.name, ...vmap.integrateScan(s), total: vmap.segments.length });
  if (k === 3) {
    const r = new Raster(BOUNDS, SCALE);
    r.metric(1);
    reference(r, sceneA);
    vector(r, vmap.segments);
    scans.slice(0, 4).forEach((q, j) => r.robot(q.pose, hex(PALETTE[j])));
    await save('j1_vecto_scans1-4.png', r);
  }
}
{
  const r = new Raster(BOUNDS, SCALE);
  r.metric(1);
  reference(r, sceneB);
  vector(r, vmap.segments);
  scans.forEach((q, j) => r.robot(q.pose, hex(PALETTE[j])));
  await save('j1_vecto_tous.png', r);
}

// ---------------------------------------------------------------- mesures
console.log('Intégration dans la carte vectorielle :');
console.table(log);

// Carte vectorielle : écart des segments à la vérité terrain (scène B), échantillonné tous les 2 cm
const errs = [];
const ghosts = [];
for (const s of vmap.segments) {
  const n = Math.max(2, Math.ceil(length(s) / 0.02));
  const d = [];
  for (let k = 0; k <= n; k++) d.push(distToScene(sceneB, s.x1 + ((s.x2 - s.x1) * k) / n, s.y1 + ((s.y2 - s.y1) * k) / n));
  const mean = d.reduce((a, b) => a + b, 0) / d.length;
  if (mean > 0.1) ghosts.push({ segment: [s.x1, s.y1, s.x2, s.y2].map((v) => v.toFixed(2)).join(', '), longueur: length(s).toFixed(2), ecart_moyen: mean.toFixed(2) });
  else errs.push(...d);
}
errs.sort((a, b) => a - b);
const totalLen = vmap.segments.reduce((a, s) => a + length(s), 0);
console.log(`\nCarte vectorielle : ${vmap.segments.length} segments, ${totalLen.toFixed(1)} m au total`);
console.log(`  écart à la vérité terrain : moyen ${(1000 * errs.reduce((a, b) => a + b, 0) / errs.length).toFixed(1)} mm, ` +
  `médian ${(1000 * errs[Math.floor(errs.length / 2)]).toFixed(1)} mm, 95 % ${(1000 * errs[Math.floor(errs.length * 0.95)]).toFixed(1)} mm, max ${(1000 * errs.at(-1)).toFixed(1)} mm`);
console.log('  segments sans correspondance (écart moyen > 10 cm) :', ghosts.length ? ghosts : 'aucun');

// Grid map : cellules occupées proches d'un mur réel, et murs réels couverts
function gridMetrics(g, name) {
  const occ = g.occupiedCells();
  const tol = g.res * Math.SQRT1_2 + 0.01; // demi-diagonale d'une cellule + bruit
  const good = occ.filter((c) => distToScene(sceneB, c.x, c.y) <= tol).length;
  const st = g.stats();
  console.log(`\nGrid map ${name} : ${g.width} x ${g.height}, occupées ${st.occupied}, libres ${st.free}, inconnues ${st.unknown}`);
  console.log(`  cellules occupées à moins de ${(tol * 100).toFixed(1)} cm d'un mur réel : ${good}/${occ.length} (${((100 * good) / occ.length).toFixed(1)} %)`);
  // Continuité des murs : échantillons tous les 2 cm sur les murs réels effectivement vus
  // (un point lidar à moins de 3 cm), recouverts par une cellule occupée à moins de tol.
  let seen = 0;
  let covered = 0;
  for (const w of sceneB) {
    const n = Math.ceil(length(w) / 0.02);
    for (let k = 0; k <= n; k++) {
      const x = w.x1 + ((w.x2 - w.x1) * k) / n;
      const y = w.y1 + ((w.y2 - w.y1) * k) / n;
      if (!allPoints.some((p) => Math.abs(p.x - x) < 0.03 && Math.abs(p.y - y) < 0.03)) continue;
      seen++;
      const [i, j] = g.worldToCell(x, y);
      let hit = false;
      for (let dj = -1; dj <= 1 && !hit; dj++) {
        for (let di = -1; di <= 1 && !hit; di++) {
          if (!g.inside(i + di, j + dj) || g.classify(g.index(i + di, j + dj)) !== 1) continue;
          const c = g.cellCenter(i + di, j + dj);
          hit = Math.hypot(c.x - x, c.y - y) <= tol;
        }
      }
      if (hit) covered++;
    }
  }
  console.log(`  murs vus couverts par une cellule occupée : ${covered}/${seen} échantillons (${((100 * covered) / seen).toFixed(1)} %)`);
}
const allPoints = scans.flatMap((s) => scanToWorldXY(s));
{
  const on = gridFromScan(scans[0], scans[0].pose, params).stats();
  const off = gridFromScan(scans[0], scans[0].pose, { ...params, fillGaps: false }).stats();
  console.log(`\nScan 1 seul : cellules libres ${off.free} sans faisceaux intermédiaires, ${on.free} avec (+${(100 * (on.free / off.free - 1)).toFixed(1)} %)`);
}
gridMetrics(gAll, 'complète');
gridMetrics(gRaw, 'sans fillGaps ni marge');

// Temps de calcul (moyenne sur 20 répétitions)
const time = (fn, n = 20) => {
  const t0 = performance.now();
  for (let k = 0; k < n; k++) fn();
  return (performance.now() - t0) / n;
};
console.log('\nTemps moyens :');
console.log(`  grid map, 1 scan (création)       ${time(() => gridFromScan(scans[0], scans[0].pose, params)).toFixed(1)} ms`);
console.log(`  grid map, 6 scans                 ${time(() => { const g = gridFromScan(scans[0], scans[0].pose, params); for (const s of scans.slice(1)) g.integrateScan(s); }).toFixed(1)} ms`);
console.log(`  extraction de segments, 1 scan    ${time(() => extractSegments(scanToWorldXY(scans[0]))).toFixed(2)} ms`);
console.log(`  carte vectorielle, 6 scans        ${time(() => { const m = new VectorMap(); for (const s of scans) m.integrateScan(s); }).toFixed(1)} ms`);
console.log(`\nFigures écrites dans ${IMG}`);
void GridMap;
