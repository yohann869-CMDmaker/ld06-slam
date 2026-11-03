// Importe les scans d'un export LidarScope dans data/real (jour 2 : scans ajoutés pour améliorer la carte).
//   1. lecture des scans (dossier scans_tp/ de l'export, format RobotControl avec pose) ;
//   2. un seul tour par scan (singleTour) : l'export enchaîne parfois 2 ou 3 rotations ;
//   3. changement de repère : LidarScope a son propre repère (origine et axes choisis par
//      l'opérateur). Un scan déjà présent dans data/real et réimporté dans LidarScope sert
//      d'ancre : T = pose dans notre repère ∘ (pose LidarScope)⁻¹, appliqué à tous les scans ;
//   4. écriture de chaque nouveau scan avec sa pose dans notre repère (pose de départ : le
//      recalage fin est fait ensuite par scripts/build-reference.mjs).
// Usage : node scripts/import-lidarscope.mjs <dossier export> [--prefix scan_1103_ls] [--out data/real]
import { readdir, readFile, stat, writeFile } from 'node:fs/promises';
import { fileURLToPath } from 'node:url';
import { parseScanCSV, serializeScanCSV, singleTour } from '../src/core/scan.js';
import { composePoses, invertPose } from '../src/core/pose.js';

const ROOT = fileURLToPath(new URL('../', import.meta.url));
const args = process.argv.slice(2);
const opt = (name, def) => (args.includes(name) ? args[args.indexOf(name) + 1] : def);
const src = args.find((a, k) => !a.startsWith('--') && !args[k - 1]?.startsWith('--'));
if (!src) {
  console.error('Usage : node scripts/import-lidarscope.mjs <dossier export> [--prefix scan_1103_ls] [--out data/real]');
  process.exit(1);
}
const prefix = opt('--prefix', 'scan_ls');
const out = opt('--out', ROOT + 'data/real');

// dossier des scans : <export>/ensemble/scans_tp, ou le dossier donné
let dir = src;
for (const sub of ['ensemble/scans_tp', 'scans_tp']) {
  if (await stat(`${src}/${sub}`).then((s) => s.isDirectory(), () => false)) {
    dir = `${src}/${sub}`;
    break;
  }
}

// scans déjà présents : reconnus par leurs 20 premières mesures
const signature = (scan) => scan.points.slice(0, 20).map((p) => `${p.angle.toFixed(2)}:${p.r.toFixed(3)}`).join(';');
const known = new Map();
for (const f of (await readdir(out)).filter((f) => f.startsWith('scan_') && f.endsWith('.csv'))) {
  const s = parseScanCSV(await readFile(`${out}/${f}`, 'utf8'), f);
  known.set(signature(s), s);
}

const scans = [];
for (const f of (await readdir(dir)).filter((f) => /\.(csv|txt)$/.test(f)).sort()) {
  scans.push(parseScanCSV(await readFile(`${dir}/${f}`, 'utf8'), f));
}
const anchors = scans.filter((s) => known.has(signature(s)));
let T = { x: 0, y: 0, theta: 0 };
if (anchors.length) {
  const a = anchors[0];
  const ours = known.get(signature(a));
  T = composePoses(ours.pose, invertPose(a.pose));
  console.log(`Ancre : ${a.name} = ${ours.name} ; changement de repère (${T.x.toFixed(3)} m, ${T.y.toFixed(3)} m, ${T.theta.toFixed(2)}°)`);
} else {
  console.warn('Aucun scan commun avec data/real : poses gardées telles quelles (repère LidarScope).');
}

let n = 0;
for (const s of scans) {
  if (anchors.includes(s)) continue;
  const one = singleTour(s);
  const pose = composePoses(T, s.pose);
  const num = s.name.match(/(\d+)\.\w+$/)?.[1] ?? String(++n).padStart(2, '0');
  const name = `${prefix}${num}.csv`;
  const comments = [
    `importé de LidarScope : ${s.name}, pose LidarScope (${s.pose.x.toFixed(4)}, ${s.pose.y.toFixed(4)}, ${s.pose.theta.toFixed(2)})`,
    `un seul tour : ${one.points.length} points gardés sur ${s.points.length}`,
  ];
  await writeFile(`${out}/${name}`, serializeScanCSV({ ...one, comments }, pose));
  console.log(`${name} : ${s.points.length} -> ${one.points.length} points, pose (${pose.x.toFixed(2)}, ${pose.y.toFixed(2)}, ${pose.theta.toFixed(1)})`);
}
