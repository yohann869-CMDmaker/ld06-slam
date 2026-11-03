import { test } from 'node:test';
import assert from 'node:assert/strict';
import { optimizePoseGraph } from '../src/core/posegraph.js';
import { freeSpaceConflict, relative, scanInformation, Slam } from '../src/core/slam.js';
import { composePoses, normalizeAngleDeg } from '../src/core/pose.js';
import { rectangle, simulateScan, simulateTrajectory } from '../src/core/sim.js';
import { gridFromScan } from '../src/core/gridmap.js';
import { scanToLocalXY } from '../src/core/scan.js';

const dist = (a, b) => Math.hypot(a.x - b.x, a.y - b.y);

test('graphe de poses : une fermeture de boucle répartit la dérive le long du carré', () => {
  // carré de 4 m parcouru en 16 pas de 1 m (rotation de 90° aux coins) ; chaque pas mesuré avec
  // une dérive de 2° et 3 cm ; le dernier nœud revient sur le premier (lien de boucle exact)
  const truth = [];
  let p = { x: 0, y: 0, theta: 0 };
  for (let k = 0; k < 16; k++) {
    truth.push(p);
    p = composePoses(p, { x: 1, y: 0, theta: k % 4 === 3 ? 90 : 0 });
  }
  truth.push(p);
  const edges = [];
  for (let k = 0; k + 1 < truth.length; k++) {
    const z = relative(truth[k], truth[k + 1]);
    edges.push({ from: k, to: k + 1, z: { x: z.x + 0.03, y: z.y, theta: z.theta + 2 }, sigma: { xy: 0.05, theta: 1 } });
  }
  const dead = [truth[0]];
  for (const e of edges) dead.push(composePoses(dead.at(-1), e.z));
  edges.push({ from: 0, to: truth.length - 1, z: relative(truth[0], truth.at(-1)), sigma: { xy: 0.01, theta: 0.2 } });
  const g = optimizePoseGraph(dead, edges);
  const before = Math.max(...dead.map((q, k) => dist(q, truth[k])));
  const after = Math.max(...g.poses.map((q, k) => dist(q, truth[k])));
  assert.ok(g.error < g.error0 / 100, `erreur ${g.error0} -> ${g.error}`);
  assert.ok(after < before / 3, `dérive max ${before} -> ${after}`);
  assert.ok(dist(g.poses.at(-1), truth.at(-1)) < 0.05, 'arrivée ramenée sur le départ');
  assert.ok(Math.abs(g.poses[0].x) + Math.abs(g.poses[0].y) + Math.abs(g.poses[0].theta) < 1e-9, 'premier nœud fixe');
});

test('information du recalage : couloir précis en travers, imprécis le long', () => {
  const pts = [];
  for (let x = -3; x <= 3; x += 0.05) pts.push({ x, y: 1 }, { x, y: -1 });
  pts.sort((a, b) => Math.atan2(a.y, a.x) - Math.atan2(b.y, b.x));
  const H = scanInformation(pts);
  assert.ok(H[1][1] > 100 * H[0][0], `travers ${H[1][1]} / long ${H[0][0]}`);
});

test("cohérence avec l'espace libre : un scan déplacé dans le sol vide est contradictoire", () => {
  const room = rectangle(0, 0, 6, 4);
  const pose = { x: 2, y: 2, theta: 0 };
  const scan = simulateScan(room, pose, { seed: 3 });
  const grid = gridFromScan(scan, pose);
  const pts = scanToLocalXY(scan);
  assert.ok(freeSpaceConflict(grid, pts, pose) < 0.02);
  assert.ok(freeSpaceConflict(grid, pts, { x: 1.2, y: 1.4, theta: 30 }) > 0.2);
});

test("SLAM : trajet simulé suivi au centimètre, bien mieux que l'odométrie seule", () => {
  const scene = [...rectangle(0, 0, 7, 5), ...rectangle(2.5, 2, 4, 3), ...rectangle(6.4, 0.5, 7, 1.6)];
  const path = [{ x: 1, y: 1 }, { x: 5.8, y: 1 }, { x: 5.8, y: 4 }];
  const { poses, scans, odom } = simulateTrajectory(scene, path, { step: 0.3, seed: 5 });
  const slam = new Slam();
  scans.forEach((s, k) => slam.addScan(s, odom[k], k === 0 ? poses[0] : null));
  const errs = slam.trajectory.map((t, k) => dist(t.pose, poses[k]));
  const dead = [poses[0]];
  for (let k = 1; k < poses.length; k++) dead.push(composePoses(dead[k - 1], odom[k]));
  assert.ok(Math.max(...errs) < 0.05, `erreur max ${Math.max(...errs)}`);
  assert.ok(dist(dead.at(-1), poses.at(-1)) > 0.2, "l'odométrie seule dérive");
  assert.ok(Math.abs(normalizeAngleDeg(slam.trajectory.at(-1).pose.theta - poses.at(-1).theta)) < 1.5);
  assert.ok(slam.keyframes.length >= poses.length - 2);
  assert.ok(slam.trajectory.every((t) => t.mode === 'départ' || t.mode === 'suivi'));
});
