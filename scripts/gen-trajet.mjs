// Trajet simulé du jour 3, pour essayer le SLAM dans l'application sans le robot : 75 scans pris
// tous les 30 cm le long d'une boucle de 21 m dans la salle de classe simulée. La pose écrite dans
// chaque fichier est celle de l'ODOMÉTRIE seule (commandes du robot, sans codeurs : elle dérive) ;
// la pose vraie est dans index.json. Dans l'application : « Trajet simulé », puis « SLAM sur les
// scans visibles ».
// Usage : node scripts/gen-trajet.mjs   -> data/sim_trajet/
import { mkdir, writeFile } from 'node:fs/promises';
import { fileURLToPath } from 'node:url';
import { simulateTrajectory } from '../src/core/sim.js';
import { composePoses } from '../src/core/pose.js';
import { serializeScanCSV } from '../src/core/scan.js';
import { CLASSROOM, CLASSROOM_LOOP } from './lib/scenes.mjs';

const ROOT = fileURLToPath(new URL('../', import.meta.url));
const OUT = ROOT + 'data/sim_trajet/';
const { poses, scans, odom } = simulateTrajectory(CLASSROOM, CLASSROOM_LOOP, { step: 0.3, seed: 7 });
const dead = [poses[0]];
for (let k = 1; k < poses.length; k++) dead.push(composePoses(dead[k - 1], odom[k]));
await mkdir(OUT, { recursive: true });
const index = { description: 'Trajet simulé (jour 3) : boucle de 21 m dans la salle de classe simulée, pose du fichier = odométrie seule', scans: [] };
for (let k = 0; k < scans.length; k++) {
  const file = `trajet_${String(k).padStart(2, '0')}.csv`;
  const scan = { ...scans[k], comments: [...scans[k].comments, `pose vraie : x=${poses[k].x.toFixed(4)} y=${poses[k].y.toFixed(4)} theta=${poses[k].theta.toFixed(2)}`, 'pose du fichier : odométrie seule (dérive)'] };
  await writeFile(OUT + file, serializeScanCSV(scan, dead[k]));
  index.scans.push({ file, truePose: poses[k], odomPose: dead[k] });
}
await writeFile(OUT + 'index.json', JSON.stringify(index, null, 1) + '\n');
console.log(`${scans.length} scans écrits dans data/sim_trajet/ (arrivée de l'odométrie à ${(100 * Math.hypot(dead.at(-1).x - poses.at(-1).x, dead.at(-1).y - poses.at(-1).y)).toFixed(0)} cm de la vraie pose)`);
