// Génère le jeu de scans simulés dans data/sim/ :
// - scene_A.txt / scene_B.txt : la salle simulée au format carte vectorielle (vérité terrain)
// - scan_XX.csv : scans au format RobotControl, pose vraie dans la ligne "# Robot position"
// - index.json  : liste des scans, scène utilisée et pose vraie
// Usage : node scripts/gen-sim.mjs
import { mkdir, writeFile } from 'node:fs/promises';
import { fileURLToPath } from 'node:url';
import { rectangle, simulateScan } from '../src/core/sim.js';
import { serializeScanCSV } from '../src/core/scan.js';

const OUT = fileURLToPath(new URL('../data/sim/', import.meta.url));

// Salle de classe 8 m x 6 m, origine au coin bas-gauche.
const walls = [
  { x1: 0, y1: 0, x2: 6.5, y2: 0 }, // mur du bas, porte entre x = 6,5 et 7,4
  { x1: 7.4, y1: 0, x2: 8, y2: 0 },
  { x1: 8, y1: 0, x2: 8, y2: 6 },
  { x1: 8, y1: 6, x2: 0, y2: 6 },
  { x1: 0, y1: 6, x2: 0, y2: 0 },
];
const furniture = [
  ...rectangle(1.5, 1.5, 2.9, 2.2), // tables élèves
  ...rectangle(4.0, 1.5, 5.4, 2.2),
  ...rectangle(1.5, 3.8, 2.9, 4.5),
  ...rectangle(4.0, 3.8, 5.4, 4.5),
  ...rectangle(6.2, 4.3, 7.6, 5.0), // bureau
  ...rectangle(6.6, 2.5, 6.9, 2.8), // poteau
  ...rectangle(0.0, 4.6, 0.5, 5.8), // armoire contre le mur gauche
];
const box = rectangle(3.2, 0.6, 3.6, 1.0); // carton présent seulement dans la scène A

const scenes = {
  A: [...walls, ...furniture, ...box],
  B: [...walls, ...furniture],
};

const shots = [
  { scene: 'A', pose: { x: 3.4, y: 3.0, theta: 0 } },
  { scene: 'A', pose: { x: 5.8, y: 3.1, theta: 15 } },
  { scene: 'A', pose: { x: 7.2, y: 1.2, theta: 90 } },
  { scene: 'A', pose: { x: 0.9, y: 2.9, theta: -30 } },
  { scene: 'B', pose: { x: 1.0, y: 0.8, theta: 45 } },
  { scene: 'B', pose: { x: 5.0, y: 0.8, theta: 180 } },
];

const toTxt = (segs, title) =>
  [`# ${title}`, '# x1,y1,x2,y2 (m)', ...segs.map((s) => [s.x1, s.y1, s.x2, s.y2].map((v) => v.toFixed(4)).join(','))].join('\n') +
  '\n';

await mkdir(OUT, { recursive: true });
await writeFile(OUT + 'scene_A.txt', toTxt(scenes.A, 'Salle simulée, scène A (avec carton)'));
await writeFile(OUT + 'scene_B.txt', toTxt(scenes.B, 'Salle simulée, scène B (carton retiré)'));

const index = { description: 'Scans simulés, salle 8 m x 6 m, pose vraie connue', scans: [] };
for (const [k, shot] of shots.entries()) {
  const file = `scan_${String(k + 1).padStart(2, '0')}.csv`;
  const scan = simulateScan(scenes[shot.scene], shot.pose, { seed: 100 + k, name: file });
  scan.comments.push(`scene=${shot.scene}`);
  await writeFile(OUT + file, serializeScanCSV(scan));
  index.scans.push({ file, scene: shot.scene, truePose: shot.pose });
}
await writeFile(OUT + 'index.json', JSON.stringify(index, null, 2) + '\n');
console.log(`${shots.length} scans écrits dans ${OUT}`);
