// Banc d'essai de l'exploration autonome (bonus du jour 3), en simulation, pose vraie connue.
// Le robot part sans carte ; à chaque pas : scan simulé, SLAM, choix de la commande par la méthode
// des frontières (src/core/explore.js), mouvement vrai entaché d'erreurs (10 % sur la distance et
// sur l'angle, 5 cm de glissement latéral et 1,5° de dérive de cap par mètre).
//   A. Salle de classe simulée (10 m × 8 m) ;
//   B. Trois pièces reliées par des portes de 1 m (14 m × 8 m) : depuis la première pièce on ne voit
//      presque rien des autres.
// Plusieurs départs par scène. Mesures : pas, distance parcourue, couverture (part du sol libre de
// la scène vue libre par la carte), collisions, erreur de trajectoire (ATE), temps par pas.
// Usage : node scripts/benchmark-explore.mjs   -> docs/jour3_explore.md et figures docs/img/j3_explo_*.png
import { mkdir, writeFile } from 'node:fs/promises';
import { fileURLToPath } from 'node:url';
import { freeCoverage, simulateExploration } from '../src/core/explore.js';
import { hex, Raster } from './lib/raster.mjs';
import { CLASSROOM, CLASSROOM_FURNITURE, ROOMS, ROOMS_FURNITURE } from './lib/scenes.mjs';

const ROOT = fileURLToPath(new URL('../', import.meta.url));

/** Points de contrôle du sol libre : grille de 10 cm, hors meubles et à 20 cm des murs. */
function floorPoints(scene, furniture, walls) {
  const xs = scene.flatMap((s) => [s.x1, s.x2]);
  const ys = scene.flatMap((s) => [s.y1, s.y2]);
  const pts = [];
  for (let x = Math.min(...xs) + 0.25; x < Math.max(...xs) - 0.2; x += 0.1) {
    for (let y = Math.min(...ys) + 0.25; y < Math.max(...ys) - 0.2; y += 0.1) {
      if (furniture.some(([a, b, c, d]) => x > a - 0.1 && x < c + 0.1 && y > b - 0.1 && y < d + 0.1)) continue;
      if (walls.some((w) => w(x, y))) continue;
      pts.push({ x, y });
    }
  }
  return pts;
}

const SCENES = [
  {
    id: 'salle',
    title: 'A. Salle de classe simulée (10 m × 8 m)',
    scene: CLASSROOM,
    floor: floorPoints(CLASSROOM, CLASSROOM_FURNITURE, [(x, y) => x + y < 1.8]), // derrière le tableau en diagonale
    starts: [{ x: 1.5, y: 2.2, theta: 0 }, { x: 8, y: 6, theta: 180 }, { x: 5, y: 1, theta: 90 }],
    options: {},
    snapshot: null,
  },
  {
    id: 'pieces',
    title: 'B. Trois pièces reliées par des portes de 1 m (14 m × 8 m)',
    scene: ROOMS,
    floor: floorPoints(ROOMS, ROOMS_FURNITURE, [(x) => Math.abs(x - 7) < 0.2, (x, y) => x > 7 && Math.abs(y - 4) < 0.2]),
    starts: [{ x: 1.5, y: 1.5, theta: 0 }, { x: 5.5, y: 6.5, theta: -90 }, { x: 12, y: 6.5, theta: 180 }],
    options: { explore: { maxRange: 20 } },
    snapshot: 25,
  },
];

/** Figure : grid map du SLAM, trajectoire vraie (gris pointillé), estimée (bleu) ; en option, frontières (vert) et chemin prévu (orange). */
function figure(scene, s, file, plan = null) {
  const xs = scene.flatMap((w) => [w.x1, w.x2]);
  const ys = scene.flatMap((w) => [w.y1, w.y2]);
  const img = new Raster({ minX: Math.min(...xs) - 0.5, maxX: Math.max(...xs) + 0.5, minY: Math.min(...ys) - 0.5, maxY: Math.max(...ys) + 0.5 }, 55, '#fbfbf9');
  img.grid(s.slam.grid);
  img.metric(1);
  const path = (poses, color, width, dash) => {
    for (let k = 1; k < poses.length; k++) img.line(poses[k - 1].x, poses[k - 1].y, poses[k].x, poses[k].y, hex(color), width, dash);
  };
  path(s.poses, '#868e96', 4, [6, 4]);
  path(s.slam.trajectory.map((t) => t.pose), '#1c7ed6', 2.5);
  if (plan && !plan.done) {
    path(plan.path, '#f08c00', 3);
    for (const f of plan.frontiers) img.dot(f.x, f.y, 4 + Math.min(6, f.size / 10), hex('#2f9e44'), 0.85);
  }
  img.robot(s.slam.last, hex('#1c7ed6'), 9);
  return writeFile(ROOT + 'docs/img/' + file, img.png());
}

await mkdir(ROOT + 'docs/img', { recursive: true });
const md = [
  "# Jour 3 — Banc d'essai de l'exploration autonome (simulation)",
  '',
  'Généré par `node scripts/benchmark-explore.mjs`. Le robot part sans carte et choisit seul ses',
  'commandes (méthode des frontières sur la grid map du SLAM). Mouvement vrai : 10 % d\'erreur sur',
  'la distance et l\'angle, 5 cm de glissement latéral et 1,5° de dérive de cap par mètre ; le SLAM',
  'ne reçoit que la commande. **Couverture** : part du sol libre de la scène (grille de 10 cm, hors',
  'meubles et murs) que la carte a vue libre. **Réussite** : couverture ≥ 95 % sans collision.',
  '',
];
for (const sc of SCENES) {
  md.push(`## ${sc.title}`, '', '| Départ | Pas | Distance | Couverture | Collisions | ATE | Erreur finale | Boucles | Temps / pas | Réussite |', '| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |');
  for (const [n, start] of sc.starts.entries()) {
    const t0 = performance.now();
    let last = null;
    for (const s of simulateExploration(sc.scene, start, { seed: 3 + n, ...sc.options })) {
      if (n === 0 && s.step === sc.snapshot) await figure(sc.scene, s, `j3_explo_${sc.id}_en_cours.png`, s.plan);
      last = s;
    }
    const time = (performance.now() - t0) / (last.step + 1);
    const est = last.slam.trajectory.map((t) => t.pose);
    const err = est.map((p, k) => Math.hypot(p.x - last.poses[k].x, p.y - last.poses[k].y));
    const ate = Math.sqrt(err.reduce((a, e) => a + e * e, 0) / err.length);
    const cov = freeCoverage(last.slam.grid, sc.floor);
    const ok = cov >= 0.95 && last.collisions === 0 && last.plan.done;
    md.push(`| (${start.x} ; ${start.y} ; ${start.theta}°) | ${last.step} | ${last.distance.toFixed(1)} m | **${(100 * cov).toFixed(1)} %** | ${last.collisions} | ${(100 * ate).toFixed(1)} cm | ${(100 * err.at(-1)).toFixed(1)} cm | ${last.slam.loops.length} | ${time.toFixed(0)} ms | ${ok ? 'oui' : 'non'} |`);
    process.stdout.write(`${sc.id} départ ${n + 1} : ${last.step} pas, couverture ${(100 * cov).toFixed(1)} %\n`);
    if (n === 0) await figure(sc.scene, last, `j3_explo_${sc.id}.png`);
  }
  md.push('');
}
md.push(
  'Figures (premier départ de chaque scène) : grid map construite par le SLAM, trajectoire vraie',
  '(gris pointillé) et estimée (bleu) ; sur la vue en cours : zones frontières (vert, taille selon',
  'leur longueur) et chemin prévu vers la plus proche (orange).',
  '',
  '| Salle de classe : carte finale | Trois pièces : carte finale |',
  '| --- | --- |',
  '| ![salle](img/j3_explo_salle.png) | ![pièces](img/j3_explo_pieces.png) |',
  '',
  '![Trois pièces : exploration en cours](img/j3_explo_pieces_en_cours.png)',
  '',
);
await writeFile(ROOT + 'docs/jour3_explore.md', md.join('\n'));
console.log(md.join('\n'));
