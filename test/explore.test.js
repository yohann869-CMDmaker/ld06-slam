import { test } from 'node:test';
import assert from 'node:assert/strict';
import { blockedCells, commandRequest, freeAhead, freeCoverage, planExploration, simulateExploration, updateSpeeds } from '../src/core/explore.js';
import { rectangle, simulateScan } from '../src/core/sim.js';
import { gridFromScan } from '../src/core/gridmap.js';
import { parseRobotBinary, scanToLocalXY } from '../src/core/scan.js';

// deux pièces 4 m × 3 m reliées par une porte de 1 m (x = 4, y entre 1 et 2)
const TWO_ROOMS = [...rectangle(0, 0, 8, 3), { x1: 4, y1: 0, x2: 4, y2: 1 }, { x1: 4, y1: 2, x2: 4, y2: 3 }];

test('obstacles gonflés : les cellules à moins du rayon d\'un mur sont interdites, le centre de la pièce non', () => {
  const pose = { x: 2, y: 1.5, theta: 0 };
  const g = gridFromScan(simulateScan(TWO_ROOMS, pose), pose);
  const blocked = blockedCells(g, 0.25);
  const at = (x, y) => {
    const [i, j] = g.worldToCell(x, y);
    return blocked[g.index(i, j)];
  };
  assert.equal(at(2, 0.1), 1, 'près du mur du bas');
  assert.equal(at(2, 1.5), 0, 'centre de la pièce');
});

test('frontières : depuis la première pièce, la seconde est vue par la porte ; but = zone la plus proche', () => {
  const pose = { x: 1, y: 1.5, theta: 0 };
  const g = gridFromScan(simulateScan(TWO_ROOMS, pose), pose);
  const plan = planExploration(g, pose, scanToLocalXY(simulateScan(TWO_ROOMS, pose)));
  assert.equal(plan.done, false);
  assert.ok(plan.frontiers.some((f) => f.x > 4), 'frontière au-delà de la porte');
  assert.ok(plan.frontiers.every((f) => Math.hypot(f.x - pose.x, f.y - pose.y) >= Math.hypot(plan.goal.x - pose.x, plan.goal.y - pose.y) - 0.5), 'pas de zone nettement plus proche que le but');
  // chemin continu, sans traverser de mur
  for (let k = 1; k < plan.path.length; k++) assert.ok(Math.hypot(plan.path[k].x - plan.path[k - 1].x, plan.path[k].y - plan.path[k - 1].y) < 0.08);
  assert.ok(plan.path.every((p) => p.x > 0.2 && p.y > 0.2 && p.y < 2.8));
});

test('commande : tourner d\'abord si le but est derrière le robot', () => {
  const pose = { x: 1, y: 1.5, theta: 180 };
  const g = gridFromScan(simulateScan(TWO_ROOMS, pose), pose);
  const plan = planExploration(g, pose, []);
  assert.equal(plan.command.kind, 'rotate');
  assert.equal(Math.abs(plan.command.angle), 60, 'rotation plafonnée à maxTurn');
});

test('pièce fermée entièrement vue : exploration terminée', () => {
  const room = rectangle(0, 0, 3, 2);
  const pose = { x: 1.5, y: 1, theta: 0 };
  const g = gridFromScan(simulateScan(room, pose, { rays: 1800 }), pose);
  assert.equal(planExploration(g, pose).done, true);
});

test('sécurité : distance libre droit devant dans le scan', () => {
  const pts = [{ x: 0.8, y: 0.05 }, { x: 0.5, y: 0.4 }, { x: -0.3, y: 0 }, { x: 1.2, y: -0.1 }];
  assert.equal(freeAhead(pts, 0.2), 0.8);
  assert.equal(freeAhead([], 0.2), Infinity);
});

test('exploration simulée : deux pièces couvertes, sans collision, pose suivie au centimètre', () => {
  let last = null;
  for (const s of simulateExploration(TWO_ROOMS, { x: 1, y: 1.5, theta: 90 }, { maxSteps: 150 })) last = s;
  assert.ok(last.plan.done, 'exploration terminée');
  const floor = [];
  for (let x = 0.3; x < 7.8; x += 0.1) for (let y = 0.3; y < 2.8; y += 0.1) if (Math.abs(x - 4) > 0.2) floor.push({ x, y });
  const cov = freeCoverage(last.slam.grid, floor);
  assert.ok(cov > 0.95, `couverture ${cov}`);
  assert.equal(last.collisions, 0);
  assert.ok(Math.hypot(last.truth.x - last.slam.last.x, last.truth.y - last.slam.last.y) < 0.05);
});

test('robot réel : durée des commandes (rotation doublée, durée plafonnée à 2 s) et étalonnage', () => {
  const speeds = { linear: 0.25, angular: 90 };
  const f = commandRequest({ kind: 'forward', distance: 0.3 }, speeds);
  assert.deepEqual([f.route, f.time], ['forward', 1200]);
  const r = commandRequest({ kind: 'rotate', angle: -45 }, speeds);
  assert.deepEqual([r.route, r.time, r.duration], ['rotateright', 1000, 500]);
  const long = commandRequest({ kind: 'forward', distance: 1 }, speeds);
  assert.equal(long.time, 2000);
  assert.ok(Math.abs(long.command.distance - 0.5) < 1e-9, 'commande effective réduite');
  // le robot a fait 0,36 m au lieu de 0,30 : il est plus rapide que prévu
  const s = updateSpeeds(speeds, { kind: 'forward', distance: 0.3 }, { x: 0.36, y: 0, theta: 0 });
  assert.ok(s.linear > 0.25 && s.linear < 0.3);
  assert.deepEqual(updateSpeeds(speeds, { kind: 'forward', distance: 0.05 }, { x: 0.2, y: 0, theta: 0 }), speeds, 'petit mouvement ignoré');
});

test('scan binaire Wi-Fi (ESP32RobotControl) : pose et points décodés', () => {
  const buf = new ArrayBuffer(8 + 10);
  const v = new DataView(buf);
  v.setInt16(0, -1500, true);
  v.setInt16(2, 250, true);
  v.setUint16(4, 9000, true);
  v.setUint16(6, 2, true);
  v.setUint16(8, 1234, true);
  v.setUint16(10, 2500, true);
  v.setUint8(12, 200);
  v.setUint16(13, 35999, true);
  v.setUint16(15, 0, true);
  v.setUint8(17, 0);
  const scan = parseRobotBinary(buf);
  assert.deepEqual(scan.pose, { x: -1.5, y: 0.25, theta: 90 });
  assert.deepEqual(scan.points, [{ angle: 12.34, r: 2.5, intensity: 200 }, { angle: 359.99, r: 0, intensity: 0 }]);
  assert.throws(() => parseRobotBinary(new ArrayBuffer(12)), /taille invalide/);
});
