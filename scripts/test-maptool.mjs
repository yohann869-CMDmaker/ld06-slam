// Nos outils testés sur des données artificielles générées par MapTool (data/maptool, produites par
// scripts/gen-maptool.mjs avec le code de MapTool.html) : la pose vraie de chaque scan et les
// « vraies » cartes (grille et carte vectorielle exportées par MapTool) sont connues.
//   A. lecture : le format MapTool (angle_deg, r_m, x_rel_m, y_rel_m) est reconnu ;
//   B. jour 1 : grid map et carte vectorielle construites par nos outils, comparées à celles de MapTool ;
//   C. jour 2 : localisation de chaque scan sur les trois types de référence exportés par MapTool
//      (sa grille, sa carte vectorielle, un autre de ses scans), mêmes poses initiales perturbées
//      pour toutes les méthodes, mêmes seuils de réussite que le banc d'essai du jour 2.
// Usage : node scripts/test-maptool.mjs [tirages par niveau, défaut 2] -> docs/test_maptool.md, docs/img/maptool_*.png
import { readdir, readFile, writeFile } from 'node:fs/promises';
import { join } from 'node:path';
import { fileURLToPath } from 'node:url';
import { parseScanCSV, scanToLocalXY, scanToWorldXY } from '../src/core/scan.js';
import { GridMap, gridFromScan } from '../src/core/gridmap.js';
import { VectorMap } from '../src/core/vectormap.js';
import { gridReference, localize, METHODS, METHODS_BY_REFERENCE, scanReference, vectorReference } from '../src/core/localization.js';
import { poseError } from '../src/core/pose.js';
import { mulberry32 } from '../src/core/sim.js';
import { hex, Raster } from './lib/raster.mjs';

const ROOT = fileURLToPath(new URL('../', import.meta.url));
const DIR = join(ROOT, 'data/maptool');
const SUCCESS = { dist: 0.05, angle: 2 };
const LEVELS = [
  { dist: 0.05, angle: 2 },
  { dist: 0.15, angle: 5 },
  { dist: 0.3, angle: 10 },
  { dist: 0.45, angle: 18 },
];
const TRIALS = Number(process.argv[2] ?? 2);

// ------------------------------------------------------------------ A. lecture
const files = (await readdir(DIR)).filter((f) => /^scan_mt_\d+\.csv$/.test(f)).sort();
const scans = [];
for (const f of files) scans.push(parseScanCSV(await readFile(join(DIR, f), 'utf8'), f));
const truth = scans.map((s) => ({ ...s.pose }));
const mtGrid = GridMap.fromCSV(await readFile(join(DIR, 'grid_map.csv'), 'utf8'));
const mtVector = VectorMap.fromTXT(await readFile(join(DIR, 'vector_map.txt'), 'utf8'));
const formats = [...new Set(scans.map((s) => s.format))];
const valid = scans.map((s) => s.points.filter((p) => p.r > 0).length);

// ------------------------------------------------------------------ B. cartes du jour 1
const ourGrid = gridFromScan(scans[0]);
scans.slice(1).forEach((s) => ourGrid.integrateScan(s));
const ourVector = new VectorMap([], { absence: true });
scans.forEach((s) => ourVector.integrateScan(s));

const segDist = (px, py, s) => {
  const dx = s.x2 - s.x1;
  const dy = s.y2 - s.y1;
  const L = dx * dx + dy * dy || 1e-12;
  const t = Math.max(0, Math.min(1, ((px - s.x1) * dx + (py - s.y1) * dy) / L));
  return Math.hypot(px - s.x1 - t * dx, py - s.y1 - t * dy);
};
const nearest = (px, py, segs) => segs.reduce((m, s) => Math.min(m, segDist(px, py, s)), Infinity);
const sample = (segs, step = 0.05) =>
  segs.flatMap((s) => {
    const n = Math.max(1, Math.ceil(Math.hypot(s.x2 - s.x1, s.y2 - s.y1) / step));
    return Array.from({ length: n + 1 }, (_, k) => ({ x: s.x1 + ((s.x2 - s.x1) * k) / n, y: s.y1 + ((s.y2 - s.y1) * k) / n }));
  });
const median = (a) => {
  const s = [...a].sort((x, y) => x - y);
  return s.length ? s[Math.floor(s.length / 2)] : NaN;
};
const pctOf = (a, f) => (a.length ? Math.round((100 * a.filter(f).length) / a.length) : 0);

// grid map : cellules occupées des deux grilles, comparées à 1 cellule près
const occ = (g) => g.occupiedCells().map((c) => ({ x: c.x, y: c.y }));
const keyOf = (p, res) => `${Math.floor(p.x / res)},${Math.floor(p.y / res)}`;
const near1 = (set, p, res) => {
  const i = Math.floor(p.x / res);
  const j = Math.floor(p.y / res);
  for (let dj = -1; dj <= 1; dj++) for (let di = -1; di <= 1; di++) if (set.has(`${i + di},${j + dj}`)) return true;
  return false;
};
const res = mtGrid.res;
const ourOcc = occ(ourGrid);
const mtOcc = occ(mtGrid);
const mtSet = new Set(mtOcc.map((p) => keyOf(p, res)));
const ourSet = new Set(ourOcc.map((p) => keyOf(p, res)));
const gridPrecision = pctOf(ourOcc, (p) => near1(mtSet, p, res));
const gridRecall = pctOf(mtOcc, (p) => near1(ourSet, p, res));
// murs vus par au moins un scan : la grille de MapTool contient aussi les faces que le lidar ne voit jamais (dos de l'armoire, dedans du pilier)
const ourVecPts = sample(ourVector.segments);
const vecDist = ourVecPts.map((p) => nearest(p.x, p.y, mtVector.segments));
const mtVecPts = sample(mtVector.segments);
const vecCover = pctOf(mtVecPts, (p) => nearest(p.x, p.y, ourVector.segments) < 0.05);

// ------------------------------------------------------------------ C. localisation (jour 2)
const refs = (k) => {
  const others = scans.filter((_, j) => j !== k);
  const closest = others.reduce((a, b) => (poseError(a.pose, truth[k]).dist < poseError(b.pose, truth[k]).dist ? a : b));
  return { grid: gridReference(mtGrid), vector: vectorReference(mtVector.segments), scan: scanReference(scanToWorldXY(closest)) };
};
const rng = mulberry32(2026);
const inits = scans.map((_, k) =>
  LEVELS.map((lvl) =>
    Array.from({ length: TRIALS }, () => {
      const dir = rng() * 2 * Math.PI;
      const sign = rng() < 0.5 ? -1 : 1;
      return { x: truth[k].x + lvl.dist * Math.cos(dir), y: truth[k].y + lvl.dist * Math.sin(dir), theta: truth[k].theta + sign * lvl.angle };
    }),
  ),
);
const rows = [];
for (let k = 0; k < scans.length; k++) {
  const R = refs(k);
  const pts = scanToLocalXY(scans[k]);
  for (const [kind, ref] of Object.entries(R)) {
    for (const method of METHODS_BY_REFERENCE[kind]) {
      LEVELS.forEach((lvl, li) => {
        for (const init of inits[k][li]) {
          const r = localize(ref, pts, init, method);
          const e = poseError(r.pose, truth[k]);
          rows.push({ kind, method, level: li, dist: e.dist, angle: e.angle, time: r.time, ok: e.dist < SUCCESS.dist && e.angle < SUCCESS.angle });
        }
      });
    }
  }
  process.stdout.write(`scan ${k + 1}/${scans.length}\n`);
}

// ------------------------------------------------------------------ figures
{
  const b = { minX: -0.5, minY: -0.5, maxX: 8.5, maxY: 6.5 };
  const img = new Raster(b, 70, '#fbfbf9');
  img.metric(1);
  for (const s of mtVector.segments) img.line(s.x1, s.y1, s.x2, s.y2, hex('#adb5bd'), 7);
  const colors = ['#1c7ed6', '#e8590c', '#2b8a3e', '#7048e8', '#d6336c', '#0c8599', '#f59f00', '#5c940d', '#862e9c', '#c92a2a'];
  scans.forEach((s, k) => {
    for (const p of scanToWorldXY(s)) img.dot(p.x, p.y, 1.6, hex(colors[k % colors.length]), 0.8);
    img.robot(s.pose, hex(colors[k % colors.length]), 6);
  });
  await writeFile(join(ROOT, 'docs/img/maptool_scans.png'), img.png());
}
{
  const b = { minX: -0.5, minY: -0.5, maxX: 8.5, maxY: 6.5 };
  const img = new Raster(b, 70, '#fbfbf9');
  img.grid(ourGrid);
  img.metric(1);
  for (const s of mtVector.segments) img.line(s.x1, s.y1, s.x2, s.y2, hex('#adb5bd'), 6);
  for (const s of ourVector.segments) img.line(s.x1, s.y1, s.x2, s.y2, hex('#1d2327'), 3);
  await writeFile(join(ROOT, 'docs/img/maptool_cartes.png'), img.png());
}

// ------------------------------------------------------------------ compte rendu
const KIND = { grid: 'Grille exportée par MapTool', vector: 'Carte vectorielle exportée par MapTool', scan: 'Scan MapTool le plus proche' };
const mean = (a) => a.reduce((x, y) => x + y, 0) / a.length;
const ratio = (a) => `${Math.round((100 * a.filter((r) => r.ok).length) / a.length)} %`;
const lines = [
  '# Test sur des données générées par MapTool',
  '',
  'Énoncé, « Remarques complémentaires » : tester ses outils avec des données réelles **et** des données',
  'artificielles générées avec mapTool. Les données réelles sont testées aux jours 1 à 3 ; ce document',
  'teste les mêmes outils sur des données de MapTool.',
  '',
  '**Comment les données ont été produites.** `node scripts/gen-maptool.mjs` lit le fichier',
  '`RESSOURCES/Tools/MapTool/MapTool.html` fourni, en extrait ses fonctions de calcul et d\'export',
  '(`computeScan`, `raySegmentIntersection`, `rasterizeSegment`, `exportScanCSV`, `exportVectorTXT`,',
  '`exportGridCSV`…) sans les modifier, et les exécute : les fichiers de `data/maptool/` sont ceux que',
  'MapTool exporterait. Scène : salle de 8 × 6 m (murs du bord ajoutés par MapTool), table, armoire,',
  'pilier, tableau en biais ; 10 poses en tour de salle ; 450 rayons par scan ; bruit de MapTool : ±1 cm',
  'sur la distance, ±0,5° sur l\'angle (uniforme). Pose vraie dans l\'en-tête de chaque scan.',
  '',
  `Généré par \`node scripts/test-maptool.mjs\` (${TRIALS} tirages par niveau de perturbation et par scan).`,
  '',
  '## A. Lecture',
  '',
  `${scans.length} scans lus, format reconnu : **${formats.join(', ')}** (colonnes \`angle_deg,r_m,x_rel_m,y_rel_m\`),`,
  `${Math.min(...valid)} à ${Math.max(...valid)} points valides par scan, pose lue dans la ligne \`# Robot position\`.`,
  '',
  '![Scans MapTool à leurs poses](img/maptool_scans.png)',
  '',
  '*Les 10 scans MapTool affichés par notre outil à leur pose, une couleur par scan, sur la carte',
  'vectorielle de MapTool (gris épais).*',
  '',
  '## B. Cartes (jour 1) comparées aux cartes de MapTool',
  '',
  '| Mesure | Valeur |',
  '| --- | --- |',
  `| Grid map : cellules occupées de notre grille à moins d'une cellule (5 cm) d'un mur de MapTool | **${gridPrecision} %** |`,
  `| Grid map : cellules de mur de MapTool retrouvées par notre grille | ${gridRecall} % |`,
  `| Carte vectorielle : écart médian de nos segments aux segments de MapTool | **${(100 * median(vecDist)).toFixed(1)} cm** (${pctOf(vecDist, (d) => d < 0.05)} % à moins de 5 cm) |`,
  `| Carte vectorielle : longueur des murs de MapTool couverte par nos segments (à 5 cm) | ${vecCover} % |`,
  '',
  'Les murs de MapTool non retrouvés sont des faces que le lidar ne peut pas voir depuis les 10 poses :',
  'le dos de l\'armoire contre le mur, l\'intérieur du pilier et de la table, le dessous du tableau en biais.',
  '',
  '![Nos cartes sur celles de MapTool](img/maptool_cartes.png)',
  '',
  '*Notre grid map (blanc libre, noir occupé) et notre carte vectorielle (noir) sur la carte',
  'vectorielle de MapTool (gris épais).*',
  '',
  '## C. Localisation (jour 2) sur les références exportées par MapTool',
  '',
  `Pour chaque scan : poses de départ perturbées de 5 cm / 2° à 45 cm / 18° (${LEVELS.length} niveaux × ${TRIALS} tirages),`,
  'les mêmes pour toutes les méthodes. **Réussite** : moins de 5 cm **et** moins de 2° (seuils du',
  '[banc d\'essai du jour 2](jour2_benchmark.md)).',
  '',
  '| Référence | Méthode | Réussite | Erreur position médiane | Erreur angle médiane | Temps moyen |',
  '| --- | --- | --- | --- | --- | --- |',
];
for (const kind of Object.keys(KIND)) {
  for (const method of METHODS_BY_REFERENCE[kind]) {
    const r = rows.filter((x) => x.kind === kind && x.method === method);
    lines.push(`| ${KIND[kind]} | ${METHODS[method].label} | ${ratio(r)} | ${(100 * median(r.map((x) => x.dist))).toFixed(1)} cm | ${median(r.map((x) => x.angle)).toFixed(2)}° | ${mean(r.map((x) => x.time)).toFixed(0)} ms |`);
  }
}
lines.push(
  '',
  'Sur la **grille** de MapTool, l\'erreur médiane de 2 à 3 cm vient de la grille elle-même : MapTool marque',
  'un mur par la cellule de 5 cm qui le contient, sans valeur intermédiaire ; le centre de cette cellule est en',
  'moyenne à 2,5 cm du vrai mur. Sur sa **carte vectorielle** et entre ses **scans**, les meilleures méthodes',
  'retrouvent la pose au millimètre (ICP point-droite, calcul direct).',
);
lines.push('', '### Sensibilité à l\'initialisation (taux de réussite)', '', `| Référence | Méthode | ${LEVELS.map((l) => `${100 * l.dist} cm / ${l.angle}°`).join(' | ')} |`, `| --- | --- | ${LEVELS.map(() => '---').join(' | ')} |`);
for (const kind of Object.keys(KIND)) {
  for (const method of METHODS_BY_REFERENCE[kind]) {
    const cells = LEVELS.map((_, li) => ratio(rows.filter((x) => x.kind === kind && x.method === method && x.level === li)));
    lines.push(`| ${KIND[kind]} | ${METHODS[method].label} | ${cells.join(' | ')} |`);
  }
}
lines.push('');
await writeFile(join(ROOT, 'docs/test_maptool.md'), lines.join('\n'));
console.log(`docs/test_maptool.md : grille ${gridPrecision} % / ${gridRecall} %, vecteurs ${(100 * median(vecDist)).toFixed(1)} cm, ${rows.length} localisations`);
