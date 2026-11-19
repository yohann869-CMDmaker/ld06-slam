// Données artificielles générées par MapTool, l'outil fourni avec le module (énoncé, « Remarques
// complémentaires » : tester ses outils avec des données réelles et des données générées avec mapTool).
//
// Le script n'imite pas MapTool : il lit le fichier MapTool.html fourni, en extrait les fonctions de
// calcul et d'export telles qu'elles sont écrites (computeScan, raySegmentIntersection, randomNoise,
// rasterizeSegment…, exportScanCSV, exportVectorTXT, exportGridCSV) et les exécute. Seuls l'affichage
// et le téléchargement sont remplacés (pas de navigateur) ; le hasard du bruit est rendu reproductible.
// Les fichiers obtenus sont donc ceux que MapTool exporterait avec la même scène et les mêmes poses.
//
// Usage : node scripts/gen-maptool.mjs [chemin/vers/MapTool.html]   -> data/maptool/
import { readFile, writeFile, mkdir } from 'node:fs/promises';
import { join } from 'node:path';
import { fileURLToPath } from 'node:url';
import { mulberry32 } from '../src/core/sim.js';

const ROOT = fileURLToPath(new URL('../', import.meta.url));
const SRC = process.argv[2] ?? join(ROOT, '../Ressources-20261102/RESSOURCES/Tools/MapTool/MapTool.html');
const OUT = join(ROOT, 'data/maptool');

const html = await readFile(SRC, 'utf8');

/** Source complète de « function nom(...) { ... } » dans le fichier, accolades équilibrées. */
function extract(name) {
  const start = html.indexOf(`function ${name}(`);
  if (start < 0) throw new Error(`MapTool.html : fonction ${name} introuvable`);
  let depth = 0;
  for (let k = html.indexOf('{', start); k < html.length; k++) {
    if (html[k] === '{') depth++;
    else if (html[k] === '}' && --depth === 0) return html.slice(start, k + 1);
  }
  throw new Error(`MapTool.html : fin de ${name} introuvable`);
}

const NAMES = ['clamp', 'randomNoise', 'allocGrid', 'clearGrid', 'cellAt', 'markCell', 'rasterizeSegment',
  'rebuildGridFromSegments', 'addSegment', 'resetSegments', 'addBorderWalls', 'raySegmentIntersection',
  'computeScan', 'exportGridCSV', 'exportVectorTXT', 'exportScanCSV'];
const code = NAMES.map(extract).join('\n');

// Environnement minimal : l'état global de MapTool, un faux document (nombre de rayons lu dans le champ
// « inRays »), le téléchargement remplacé par une capture du texte, Math.random reproductible.
const captured = [];
const rng = mulberry32(2026);
const fakeMath = Object.create(Math);
fakeMath.random = rng;
const state = {
  W: 8, H: 6, cell: 0.05, rays: 450, // salle de 8 × 6 m, cellules de 5 cm, 450 rayons comme le LD06
  addBorders: true, segments: [], gridOcc: [],
  robot: { x: 0.5, y: 0.5, thDeg: 0 },
  noiseAngle: 0.5, // degrés (bruit uniforme ±0,5°, champ « bruit angle » de MapTool)
  noiseDist: 0.01, // mètres (bruit uniforme ±1 cm, champ « bruit distance »)
};
const documentStub = { getElementById: (id) => ({ value: id === 'inRays' ? String(state.rays) : '' }) };
const downloadText = (filename, text) => captured.push({ filename, text });
const api = new Function('state', 'document', 'downloadText', 'Math', `${code}
  return { allocGrid, resetSegments, addSegment, rebuildGridFromSegments, exportScanCSV, exportVectorTXT, exportGridCSV };`)(
  state, documentStub, downloadText, fakeMath);

// Scène : murs du bord (addBorderWalls de MapTool) et meubles, sans symétrie

api.allocGrid();
api.resetSegments();
const rect = (x0, y0, x1, y1) => {
  api.addSegment(x0, y0, x1, y0);
  api.addSegment(x1, y0, x1, y1);
  api.addSegment(x1, y1, x0, y1);
  api.addSegment(x0, y1, x0, y0);
};
rect(3, 2.4, 5, 3.4); // table du milieu
rect(7.4, 3.6, 8, 5.4); // armoire contre le mur de droite
rect(1.4, 4.4, 1.7, 4.7); // pilier
api.addSegment(0.32, 1.23, 1.21, 0.31); // tableau en biais dans un coin (extrémités hors des coins de cellules : avec 0.3, 1.2 → 1.2, 0.3, à 45° pile, la rastérisation de MapTool ne s’arrête pas)
api.rebuildGridFromSegments();

// Poses du robot (x, y en m, θ en degrés) : un tour de la salle, un scan tous les 1 à 1,5 m
const POSES = [
  [1.5, 2.0, 0], [2.8, 1.2, 15], [4.2, 1.0, 35], [5.8, 1.3, 70], [6.6, 2.6, 110],
  [6.2, 4.4, 160], [4.8, 4.6, 190], [3.2, 4.5, 220], [2.0, 3.4, 260], [1.2, 2.6, 300],
];

await mkdir(OUT, { recursive: true });
const rows = ['scan,x_m,y_m,theta_deg'];
for (const [k, [x, y, th]] of POSES.entries()) {
  state.robot = { x, y, thDeg: th };
  captured.length = 0;
  api.exportScanCSV();
  const name = `scan_mt_${String(k + 1).padStart(2, '0')}.csv`;
  await writeFile(join(OUT, name), captured[0].text + '\n');
  rows.push(`${name},${x.toFixed(4)},${y.toFixed(4)},${th.toFixed(2)}`);
}
await writeFile(join(OUT, 'poses.csv'), rows.join('\n') + '\n');
captured.length = 0;
api.exportVectorTXT();
api.exportGridCSV();
for (const f of captured) await writeFile(join(OUT, f.filename), f.text + '\n');
console.log(`${POSES.length} scans MapTool, vector_map.txt et grid_map.csv dans data/maptool/`);
