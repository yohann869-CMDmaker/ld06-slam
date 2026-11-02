// Construit une grid map et une carte vectorielle à partir de scans (pose lue dans chaque fichier).
// Usage :
//   node scripts/build-maps.mjs [options] scan1.csv scan2.csv ...
// Options :
//   --res 0.05          résolution de la grid map (m)
//   --out out           dossier de sortie
//   --grid fichier.csv  grid map existante à mettre à jour (au lieu d'en créer une)
//   --vector fichier.txt carte vectorielle existante à mettre à jour
//   --ref fichier.txt   carte de référence dessinée en pointillés sur l'aperçu
//   --no-absence        désactive la découpe des segments traversés
//   --min-intensity N   filtre des points de faible intensité
// Sorties : grid_map.csv, grid_map.png, vector_map.txt, apercu.png
import { mkdir, readFile, writeFile } from 'node:fs/promises';
import { basename, join } from 'node:path';
import { parseScanCSV, scanToWorldXY, DEFAULT_FILTER } from '../src/core/scan.js';
import { GridMap, gridFromScan } from '../src/core/gridmap.js';
import { VectorMap } from '../src/core/vectormap.js';
import { hex, Raster } from './lib/raster.mjs';

const PALETTE = ['#e8590c', '#1c7ed6', '#2f9e44', '#ae3ec9', '#f08c00', '#0c8599', '#c2255c', '#5c940d', '#5f3dc4', '#795548'];

const args = process.argv.slice(2);
const opt = { res: 0.05, out: 'out', absence: true, minIntensity: 0 };
const files = [];
for (let k = 0; k < args.length; k++) {
  const a = args[k];
  if (a === '--res') opt.res = Number(args[++k]);
  else if (a === '--out') opt.out = args[++k];
  else if (a === '--grid') opt.grid = args[++k];
  else if (a === '--vector') opt.vector = args[++k];
  else if (a === '--ref') opt.ref = args[++k];
  else if (a === '--no-absence') opt.absence = false;
  else if (a === '--min-intensity') opt.minIntensity = Number(args[++k]);
  else files.push(a);
}
if (!files.length) {
  console.error('Usage : node scripts/build-maps.mjs [--res 0.05] [--out out] [--grid g.csv] [--vector v.txt] [--ref r.txt] scans...');
  process.exit(1);
}

const filter = { ...DEFAULT_FILTER, minIntensity: opt.minIntensity };
const scans = [];
for (const f of files) scans.push(parseScanCSV(await readFile(f, 'utf8'), basename(f)));

let grid = opt.grid ? GridMap.fromCSV(await readFile(opt.grid, 'utf8'), { resolution: opt.res }) : null;
const vector = opt.vector ? VectorMap.fromTXT(await readFile(opt.vector, 'utf8'), { absence: opt.absence }) : new VectorMap([], { absence: opt.absence });

console.log('scan                      pose (x, y, θ)            points  cellules libres/occupées   segments ext./fus./ajout/découpes');
for (const s of scans) {
  let g;
  if (!grid) {
    grid = gridFromScan(s, s.pose, { resolution: opt.res }, filter);
    g = { freeCells: '-', occupiedCells: '-' };
  } else {
    g = grid.integrateScan(s, s.pose, filter);
  }
  const v = vector.integrateScan(s, s.pose, filter);
  const pose = `(${s.pose.x.toFixed(2)}, ${s.pose.y.toFixed(2)}, ${s.pose.theta.toFixed(1)}°)`;
  console.log(
    `${s.name.padEnd(25)} ${pose.padEnd(25)} ${String(scanToWorldXY(s, s.pose, filter).length).padStart(6)}  ${`${g.freeCells}/${g.occupiedCells}`.padStart(22)}   ${v.extracted}/${v.merged}/${v.added}/${v.cuts}`,
  );
}

await mkdir(opt.out, { recursive: true });
await writeFile(join(opt.out, 'grid_map.csv'), grid.toCSV());
await writeFile(join(opt.out, 'vector_map.txt'), vector.toTXT());

const gb = { minX: grid.originX, minY: grid.originY, maxX: grid.originX + grid.width * grid.res, maxY: grid.originY + grid.height * grid.res };
const gridPng = new Raster(gb, 4 / grid.res);
gridPng.grid(grid);
await writeFile(join(opt.out, 'grid_map.png'), gridPng.png());

const view = new Raster(gb, 100);
view.grid(grid);
view.metric(1);
if (opt.ref) {
  for (const s of VectorMap.fromTXT(await readFile(opt.ref, 'utf8')).segments) view.line(s.x1, s.y1, s.x2, s.y2, hex('#5d6770'), 1.5, [6, 4]);
}
scans.forEach((s, k) => {
  for (const p of scanToWorldXY(s, s.pose, filter)) view.dot(p.x, p.y, 1.6, hex(PALETTE[k % PALETTE.length]));
});
for (const s of vector.segments) view.line(s.x1, s.y1, s.x2, s.y2, hex('#d6336c'), 3);
scans.forEach((s, k) => view.robot(s.pose, hex(PALETTE[k % PALETTE.length])));
await writeFile(join(opt.out, 'apercu.png'), view.png());

const st = grid.stats();
console.log(`\nGrid map : ${grid.width} x ${grid.height} cellules de ${grid.res} m, ${st.occupied} occupées, ${st.free} libres, ${st.unknown} inconnues`);
console.log(`Carte vectorielle : ${vector.segments.length} segments`);
console.log(`Fichiers écrits dans ${opt.out}/ : grid_map.csv, grid_map.png, vector_map.txt, apercu.png`);
