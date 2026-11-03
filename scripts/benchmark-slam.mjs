// Banc d'essai du SLAM (jour 3) sur trajets simulés, pose vraie connue à chaque scan.
//   A. Salle de classe simulée (10 m × 8 m, table, armoire, bureaux, pilier, tableau) : boucle de
//      21 m autour de la table, un scan tous les 30 cm ;
//   B. Couloir en boucle de 54 m autour d'un bloc central, lidar limité à 4 m, bruit 2 cm, 5 % de
//      mesures perdues : la dérive s'accumule le long des couloirs (peu de repères).
// Odométrie simulée du robot Mecanum (sans codeurs) : 10 % d'erreur sur la distance, 2° par pas,
// dérive de cap de 1,5° par mètre. Mesure : erreur de trajectoire (ATE = moyenne quadratique des
// erreurs de position), erreur maximale, erreur à l'arrivée, temps par scan.
// Usage : node scripts/benchmark-slam.mjs   -> docs/jour3_slam.md et figures docs/img/j3_*.png
import { mkdir, writeFile } from 'node:fs/promises';
import { fileURLToPath } from 'node:url';
import { simulateTrajectory } from '../src/core/sim.js';
import { Slam } from '../src/core/slam.js';
import { composePoses, normalizeAngleDeg } from '../src/core/pose.js';
import { scanToWorldXY } from '../src/core/scan.js';
import { hex, Raster } from './lib/raster.mjs';
import { CLASSROOM, CLASSROOM_LOOP, CORRIDOR, CORRIDOR_LOOP } from './lib/scenes.mjs';

const ROOT = fileURLToPath(new URL('../', import.meta.url));

const classroom = CLASSROOM;

const SCENARIOS = [
  {
    id: 'salle',
    title: 'A. Salle de classe simulée (boucle de 21 m autour de la table)',
    scene: classroom,
    path: CLASSROOM_LOOP,
    sim: { step: 0.3, seed: 7 },
    runs: [
      { label: 'SLAM sans odométrie', odom: false, options: {} },
      { label: 'SLAM avec odométrie', odom: true, options: {} },
    ],
  },
  {
    id: 'couloir',
    title: 'B. Couloir en boucle de 54 m (lidar limité à 4 m)',
    scene: CORRIDOR,
    path: CORRIDOR_LOOP,
    sim: { step: 0.3, seed: 11, scan: { maxRange: 4, noiseRange: 0.02, dropRate: 0.05 } },
    runs: [
      { label: 'SLAM sans fermeture de boucle', odom: true, options: { loopClosure: false } },
      { label: 'SLAM + fermeture de boucle (précisions fixes)', odom: true, options: { loopClosure: true, anisotropic: false } },
      { label: 'SLAM + fermeture de boucle (précision par direction)', odom: true, options: { loopClosure: true, anisotropic: true } },
    ],
  },
];

const errors = (est, truth) => est.map((p, k) => ({ d: Math.hypot(p.x - truth[k].x, p.y - truth[k].y), a: Math.abs(normalizeAngleDeg(p.theta - truth[k].theta)) }));
const summary = (e) => ({
  ate: Math.sqrt(e.reduce((s, x) => s + x.d * x.d, 0) / e.length),
  max: Math.max(...e.map((x) => x.d)),
  end: e.at(-1).d,
  endAngle: e.at(-1).a,
});

/** Figure : points des scans clés à leur pose estimée, trajectoires vraie (gris), odométrie (orange) et SLAM (bleu). */
function figure(scene, truth, dead, slam, file) {
  const xs = scene.flatMap((s) => [s.x1, s.x2]);
  const ys = scene.flatMap((s) => [s.y1, s.y2]);
  const b = { minX: Math.min(...xs) - 0.5, maxX: Math.max(...xs) + 0.5, minY: Math.min(...ys) - 0.5, maxY: Math.max(...ys) + 0.5 };
  const img = new Raster(b, scene === classroom ? 70 : 45, '#fbfbf9');
  img.metric(1);
  for (const kf of slam.keyframes) for (const p of scanToWorldXY(kf.scan, kf.pose)) img.dot(p.x, p.y, 1.3, hex('#1d2327'), 0.5);
  const path = (poses, color, width, dash) => {
    for (let k = 1; k < poses.length; k++) img.line(poses[k - 1].x, poses[k - 1].y, poses[k].x, poses[k].y, hex(color), width, dash);
  };
  path(truth, '#868e96', 4, [6, 4]);
  if (dead) path(dead, '#f08c00', 2);
  path(slam.trajectory.map((t) => t.pose), '#1c7ed6', 2.5);
  for (const l of slam.loops) img.line(slam.keyframes[l.from].pose.x, slam.keyframes[l.from].pose.y, slam.keyframes[l.to].pose.x, slam.keyframes[l.to].pose.y, hex('#e03131'), 2);
  return writeFile(ROOT + 'docs/img/' + file, img.png());
}

await mkdir(ROOT + 'docs/img', { recursive: true });
const md = [
  '# Jour 3 — Banc d\'essai du SLAM (trajets simulés)',
  '',
  'Généré par `node scripts/benchmark-slam.mjs`. Pose vraie connue à chaque scan. Odométrie simulée',
  'du robot Mecanum, qui n\'a pas de codeurs (durée des commandes) : 10 % d\'erreur sur la distance,',
  '2° par pas, dérive de cap de 1,5° par mètre. **ATE** : moyenne quadratique des erreurs de position',
  'sur toute la trajectoire.',
  '',
];
for (const sc of SCENARIOS) {
  const { poses, scans, odom } = simulateTrajectory(sc.scene, sc.path, sc.sim);
  const dead = [poses[0]];
  for (let k = 1; k < poses.length; k++) dead.push(composePoses(dead[k - 1], odom[k]));
  const len = sc.path.slice(1).reduce((a, p, k) => a + Math.hypot(p.x - sc.path[k].x, p.y - sc.path[k].y), 0);
  md.push(`## ${sc.title}`, '', `${poses.length} scans sur ${len.toFixed(0)} m.`, '', '| Méthode | ATE | Erreur max | Arrivée | Scans clés | Relocalisations | Boucles | Temps / scan |', '| --- | --- | --- | --- | --- | --- | --- | --- |');
  const d = summary(errors(dead, poses));
  md.push(`| Odométrie seule | ${(100 * d.ate).toFixed(1)} cm | ${(100 * d.max).toFixed(1)} cm | ${(100 * d.end).toFixed(1)} cm / ${d.endAngle.toFixed(1)}° | – | – | – | – |`);
  let last = null;
  for (const run of sc.runs) {
    const slam = new Slam(run.options);
    const t0 = performance.now();
    scans.forEach((s, k) => slam.addScan(s, run.odom ? odom[k] : null, k === 0 ? poses[0] : null));
    const time = (performance.now() - t0) / scans.length;
    const e = summary(errors(slam.trajectory.map((t) => t.pose), poses));
    const reloc = slam.trajectory.filter((t) => t.mode === 'relocalisé').length;
    const loops = slam.loops.map((l) => `${Math.round(100 * l.correction)} cm`).join(', ');
    md.push(`| ${run.label} | **${(100 * e.ate).toFixed(1)} cm** | ${(100 * e.max).toFixed(1)} cm | ${(100 * e.end).toFixed(1)} cm / ${e.endAngle.toFixed(1)}° | ${slam.keyframes.length} | ${reloc} | ${slam.loops.length ? `${slam.loops.length} (correction ${loops})` : '0'} | ${time.toFixed(0)} ms |`);
    process.stdout.write(`${sc.id} : ${run.label} terminé\n`);
    if (sc.id === 'couloir' && !run.options.loopClosure) await figure(sc.scene, poses, dead, slam, 'j3_slam_couloir_sans_boucle.png');
    last = slam;
  }
  await figure(sc.scene, poses, sc.id === 'salle' ? dead : null, last, `j3_slam_${sc.id}.png`);
  md.push('');
}
md.push(
  'Figures : points des scans clés à leur pose estimée (noir), trajectoire vraie (gris pointillé),',
  'odométrie seule (orange), SLAM (bleu), liens de fermeture de boucle (rouge).',
  '',
  '![Salle de classe simulée](img/j3_slam_salle.png)',
  '',
  '| Couloir sans fermeture de boucle | Couloir avec fermeture de boucle |',
  '| --- | --- |',
  '| ![sans boucle](img/j3_slam_couloir_sans_boucle.png) | ![avec boucle](img/j3_slam_couloir.png) |',
  '',
);
await writeFile(ROOT + 'docs/jour3_slam.md', md.join('\n'));
console.log(md.join('\n'));
