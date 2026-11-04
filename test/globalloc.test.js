import { test } from 'node:test';
import assert from 'node:assert/strict';
import { candidateAngles, freePositions, globalLocalize, scanDominant } from '../src/core/globalloc.js';
import { gridFromScan } from '../src/core/gridmap.js';
import { gridReference } from '../src/core/localization.js';
import { normalizeAngleDeg } from '../src/core/pose.js';
import { rectangle, simulateScan } from '../src/core/sim.js';
import { scanToLocalXY } from '../src/core/scan.js';

// Salle de 8 m × 6 m avec un meuble et une étagère placés sans symétrie (le symétrique de l'un par
// rapport au centre de la salle ne tombe pas sur l'autre)
const scene = [...rectangle(0, 0, 8, 6), ...rectangle(2, 1, 3, 1.6), ...rectangle(6, 3.5, 7.5, 3.9)];
function mapOf(sceneSegs, poses) {
  const scans = poses.map((p, k) => simulateScan(sceneSegs, p, { seed: 10 + k }));
  const grid = gridFromScan(scans[0], poses[0]);
  scans.slice(1).forEach((s, k) => grid.integrateScan(s, poses[k + 1]));
  return gridReference(grid);
}
const err = (a, b) => ({ dist: Math.hypot(a.x - b.x, a.y - b.y), angle: Math.abs(normalizeAngleDeg(a.theta - b.theta)) });

test('positions candidates : sol seulement, à distance des obstacles', () => {
  const ref = mapOf(scene, [{ x: 1, y: 1, theta: 0 }, { x: 7, y: 5, theta: 180 }, { x: 4, y: 3, theta: 45 }]);
  const pos = freePositions(ref.grid, 0.2, 0.1);
  assert.ok(pos.length > 500, `${pos.length} positions`);
  for (const p of pos) {
    assert.ok(p.x > 0.05 && p.x < 7.95 && p.y > 0.05 && p.y < 5.95, 'dans la salle');
    assert.ok(!(p.x > 2.05 && p.x < 2.95 && p.y > 1.05 && p.y < 1.55), 'pas dans le meuble');
  }
});

test('orientations candidates : 4 directions tirées des murs, à 90° près', () => {
  const s = simulateScan(scene, { x: 4, y: 3, theta: 30 }, { seed: 4 });
  const dom = scanDominant(scanToLocalXY(s));
  assert.ok(dom.support > 0.5, `${dom.support}`);
  const { angles, mode } = candidateAngles(scanToLocalXY(s), 0);
  assert.equal(mode, 'murs');
  assert.equal(angles.length, 36);
  // la vraie orientation (30°) fait partie des candidates
  assert.ok(angles.some((a) => Math.abs(normalizeAngleDeg(a - 30)) < 1.01));
  assert.equal(candidateAngles(scanToLocalXY(s), 0, { useDominant: false }).angles.length, 180);
});

test('localisation globale : pose retrouvée sans pose de départ, réponse fiable', () => {
  const ref = mapOf(scene, [{ x: 1, y: 1, theta: 0 }, { x: 7, y: 5, theta: 180 }, { x: 4, y: 3, theta: 45 }, { x: 1.5, y: 5, theta: -90 }]);
  const truth = { x: 6.1, y: 1.7, theta: 127 };
  const pts = scanToLocalXY(simulateScan(scene, truth, { seed: 99 }));
  for (const useDominant of [true, false]) {
    const r = globalLocalize(ref, pts, { mapDominant: 0, useDominant });
    const e = err(r.pose, truth);
    assert.ok(e.dist < 0.05 && e.angle < 2, `${useDominant ? 'murs' : 'tous'} : ${e.dist} m, ${e.angle}°`);
    assert.equal(r.ambiguous, false);
  }
});

test('localisation globale : salle vide symétrique signalée ambiguë', () => {
  const empty = rectangle(0, 0, 8, 6);
  const ref = mapOf(empty, [{ x: 2, y: 2, theta: 0 }, { x: 6, y: 4, theta: 90 }]);
  const pts = scanToLocalXY(simulateScan(empty, { x: 2, y: 1.5, theta: 20 }, { seed: 7 }));
  const r = globalLocalize(ref, pts, { mapDominant: 0 });
  // le symétrique par rapport au centre de la salle (6 ; 4,5 ; 200°) explique aussi bien le scan
  assert.equal(r.ambiguous, true, `rapport ${r.ratio}`);
});

test('a priori de position : une salle parfaitement symétrique devient non ambiguë', () => {
  // salle vide 8 m × 4 m : depuis (2 ; 1,5 ; 0°) le scan est identique depuis (6 ; 2,5 ; 180°)
  const room = rectangle(0, 0, 8, 4);
  const truthPose = { x: 2, y: 1.5, theta: 0 };
  const ref = mapOf(room, [{ x: 2, y: 2, theta: 0 }, { x: 6, y: 2, theta: 0 }, { x: 4, y: 1, theta: 0 }]);
  const pts = scanToLocalXY(simulateScan(room, truthPose, { seed: 3 }));
  const blind = globalLocalize(ref, pts);
  assert.equal(blind.ambiguous, true, 'sans a priori : deux poses symétriques expliquent le scan');
  const prior = { pose: { x: 2.3, y: 1.3, theta: 6 }, sigmaXY: 0.5, sigmaTheta: 15 };
  const r = globalLocalize(ref, pts, { prior });
  const e = err(r.pose, truthPose);
  assert.ok(e.dist < 0.05 && e.angle < 1, `${e.dist} m, ${e.angle}°`);
  assert.equal(r.ambiguous, false, 'le symétrique, à 4 m de la pose attendue, est écarté');
});
