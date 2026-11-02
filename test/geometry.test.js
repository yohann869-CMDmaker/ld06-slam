import { test } from 'node:test';
import assert from 'node:assert/strict';
import {
  EPS,
  angleBetweenDeg,
  areClose,
  closeness,
  cutSegment,
  intersect,
  length,
  mergeSegments,
  orientedAngleDeg,
  pointSegmentDistance,
} from '../src/core/geometry.js';

const S = (x1, y1, x2, y2) => ({ x1, y1, x2, y2 });
const close = (a, b, eps = 1e-9) => assert.ok(Math.abs(a - b) <= eps, `${a} != ${b}`);
const sameSeg = (s, x1, y1, x2, y2, eps = 1e-9) => {
  close(s.x1, x1, eps);
  close(s.y1, y1, eps);
  close(s.x2, x2, eps);
  close(s.y2, y2, eps);
};

// ---------------------------------------------------------------- angle

test('angle entre segments : droites non orientées, résultat dans [0, 90]', () => {
  close(angleBetweenDeg(S(0, 0, 1, 0), S(0, 0, 2, 0)), 0);
  close(angleBetweenDeg(S(0, 0, 1, 0), S(0, 0, 0, 1)), 90);
  close(angleBetweenDeg(S(0, 0, 1, 0), S(1, 0, 0, 0)), 0); // même mur parcouru à l'envers
  close(angleBetweenDeg(S(0, 0, 1, 0), S(0, 0, Math.cos(Math.PI / 6), Math.sin(Math.PI / 6))), 30);
  close(angleBetweenDeg(S(0, 0, 1, 0), S(0, 0, -Math.cos(Math.PI / 6), Math.sin(Math.PI / 6))), 30); // 150° -> 30°
  close(orientedAngleDeg(S(0, 0, 1, 0), S(0, 0, 0, 1)), 90);
  close(orientedAngleDeg(S(0, 0, 1, 0), S(0, 0, 0, -1)), -90);
});

// ---------------------------------------------------------------- proximité

test('segments proches : colinéaires qui se recouvrent', () => {
  assert.ok(areClose(S(0, 0, 2, 0), S(1.5, 0.01, 3, 0.02)));
});

test('segments proches : écart perpendiculaire', () => {
  assert.ok(areClose(S(0, 0, 2, 0), S(0.5, 0.05, 1.5, 0.05)));
  assert.ok(!areClose(S(0, 0, 2, 0), S(0.5, 0.2, 1.5, 0.2)));
});

test('segments proches : écart angulaire', () => {
  const tilted = (deg) => S(0, 0, Math.cos(deg * Math.PI / 180), Math.sin(deg * Math.PI / 180));
  assert.ok(areClose(S(0, 0, 2, 0), tilted(4)));
  assert.ok(!areClose(S(0, 0, 2, 0), tilted(20)));
});

test('segments proches : trou longitudinal entre deux morceaux d\'un même mur', () => {
  assert.ok(areClose(S(0, 0, 2, 0), S(2.1, 0, 3, 0)));
  assert.ok(!areClose(S(0, 0, 2, 0), S(2.5, 0, 3, 0)));
  close(closeness(S(0, 0, 2, 0), S(2.5, 0, 3, 0)).gap, 0.5);
  close(closeness(S(0, 0, 2, 0), S(-1, 0, -0.3, 0)).gap, 0.3);
});

test('segments parallèles proches mais en sens opposés : bien détectés comme proches', () => {
  assert.ok(areClose(S(0, 0, 2, 0), S(1.8, 0.03, 0.2, 0.03)));
});

// ---------------------------------------------------------------- fusion

test('fusion de deux morceaux colinéaires : union des étendues', () => {
  const m = mergeSegments(S(0, 0, 2, 0), S(1.5, 0, 3, 0));
  sameSeg(m, 0, 0, 3, 0);
  close(m.weight, 3.5);
  assert.equal(m.observations, 2);
});

test('fusion de deux parallèles décalés : droite au barycentre pondéré par la longueur', () => {
  const m = mergeSegments(S(0, 0, 3, 0), S(0, 0.04, 1, 0.04)); // poids 3 et 1 -> y = 0,01
  close(m.y1, 0.01);
  close(m.y2, 0.01);
  close(m.x1, 0);
  close(m.x2, 3);
});

test('fusion de segments de sens opposés : pas de rotation parasite à 90°', () => {
  const m = mergeSegments(S(0, 0, 2, 0), S(2, 0.02, 0, 0.02));
  close(angleBetweenDeg(m, S(0, 0, 1, 0)), 0, 1e-9);
  close(length(m), 2);
  assert.ok(m.x2 > m.x1, 'le sens du premier segment est conservé');
});

test('fusion pondérée par le nombre de points (propriété weight)', () => {
  const m = mergeSegments({ ...S(0, 0, 2, 0), weight: 90 }, { ...S(0, 0.1, 2, 0.1), weight: 10 });
  close(m.y1, 0.01);
  assert.equal(m.weight, 100);
});

// ---------------------------------------------------------------- intersection

test('intersection : croisement en X', () => {
  const r = intersect(S(0, 0, 2, 2), S(0, 2, 2, 0));
  assert.equal(r.type, 'point');
  close(r.x, 1);
  close(r.y, 1);
  close(r.t, 0.5);
  close(r.u, 0.5);
});

test('intersection : droites sécantes hors des segments', () => {
  const r = intersect(S(0, 0, 1, 0), S(2, -1, 2, 1));
  assert.equal(r.type, 'outside');
  close(r.x, 2);
});

test('intersection : parallèles, colinéaires, dégénéré', () => {
  assert.equal(intersect(S(0, 0, 1, 0), S(0, 1, 1, 1)).type, 'parallel');
  assert.deepEqual(intersect(S(0, 0, 2, 0), S(1, 0, 3, 0)), { type: 'colinear', overlap: true });
  assert.deepEqual(intersect(S(0, 0, 1, 0), S(2, 0, 3, 0)), { type: 'colinear', overlap: false });
  assert.equal(intersect(S(0, 0, 1, 0), S(0.5, 0, 0.5, 0)).type, 'degenerate');
});

test('intersection : extrémité commune (sommet en L)', () => {
  const r = intersect(S(0, 0, 1, 0), S(1, 0, 1, 1));
  assert.equal(r.type, 'point');
  close(r.t, 1);
  close(r.u, 0);
});

test('intersection : segments presque parallèles mais sécants', () => {
  const r = intersect(S(0, 0, 10, 0), S(0, -0.001, 10, 0.001));
  assert.equal(r.type, 'point');
  close(r.x, 5, 1e-9);
});

// ---------------------------------------------------------------- découpage

test('découpage : croisement franc, le plus long côté est conservé (côté P1)', () => {
  const r = cutSegment(S(0, 0, 4, 0), S(3, -1, 3, 1));
  assert.equal(r.cut, true);
  assert.equal(r.kept, 'P1');
  sameSeg(r.segment, 0, 0, 3, 0);
});

test('découpage : le plus long côté est côté P2, le sens du segment est conservé', () => {
  const r = cutSegment(S(0, 0, 4, 0), S(1, -1, 1, 1));
  assert.equal(r.kept, 'P2');
  sameSeg(r.segment, 1, 0, 4, 0);
});

test('découpage : intersection exactement à une extrémité -> aucun changement', () => {
  for (const cutter of [S(0, -1, 0, 1), S(4, -1, 4, 1)]) {
    const r = cutSegment(S(0, 0, 4, 0), cutter);
    assert.equal(r.cut, false);
    assert.equal(r.reason, 'intersection à une extrémité');
  }
});

test('découpage : intersection à moins de EPS d\'une extrémité -> aucun changement', () => {
  const r = cutSegment(S(0, 0, 4, 0), S(EPS / 2, -1, EPS / 2, 1));
  assert.equal(r.cut, false);
  assert.equal(r.reason, 'intersection à une extrémité');
});

test('découpage : intersection juste au-delà de EPS d\'une extrémité -> découpe', () => {
  const r = cutSegment(S(0, 0, 4, 0), S(3 * EPS, -1, 3 * EPS, 1));
  assert.equal(r.cut, true);
  assert.equal(r.kept, 'P2');
  close(r.segment.x1, 3 * EPS, 1e-12);
});

test('découpage : le segment coupant s\'arrête sur le segment (en T) -> découpe', () => {
  const r = cutSegment(S(0, 0, 4, 0), S(1, 0, 1, 2));
  assert.equal(r.cut, true);
  sameSeg(r.segment, 1, 0, 4, 0);
});

test('découpage : le segment coupant s\'arrête avant le segment -> aucun changement', () => {
  const r = cutSegment(S(0, 0, 4, 0), S(1, 0.001, 1, 2));
  assert.equal(r.cut, false);
  assert.equal(r.reason, 'pas d\'intersection sur les deux segments');
});

test('découpage : le segment coupant s\'arrête à moins de EPS -> considéré comme touchant', () => {
  const r = cutSegment(S(0, 0, 4, 0), S(1, EPS / 2, 1, 2));
  assert.equal(r.cut, true);
});

test('découpage : le point d\'intersection est hors du segment à couper -> aucun changement', () => {
  const r = cutSegment(S(0, 0, 4, 0), S(5, -1, 5, 1));
  assert.equal(r.cut, false);
});

test('découpage : segments colinéaires (qui se recouvrent ou non) -> aucun changement', () => {
  assert.equal(cutSegment(S(0, 0, 4, 0), S(1, 0, 2, 0)).reason, 'segments colinéaires');
  assert.equal(cutSegment(S(0, 0, 4, 0), S(-1, 0, 5, 0)).reason, 'segments colinéaires');
  assert.equal(cutSegment(S(0, 0, 4, 0), S(4, 0, 6, 0)).reason, 'segments colinéaires');
});

test('découpage : segments parallèles distincts -> aucun changement', () => {
  assert.equal(cutSegment(S(0, 0, 4, 0), S(0, 1, 4, 1)).reason, 'segments parallèles');
});

test('découpage : deux moitiés égales -> on garde le côté de P1', () => {
  const r = cutSegment(S(0, 0, 4, 0), S(2, -1, 2, 1));
  assert.equal(r.kept, 'P1');
  sameSeg(r.segment, 0, 0, 2, 0);
  // même cas avec un segment orienté dans l'autre sens : P1 est maintenant à droite
  const r2 = cutSegment(S(4, 0, 0, 0), S(2, -1, 2, 1));
  assert.equal(r2.kept, 'P1');
  sameSeg(r2.segment, 4, 0, 2, 0);
});

test('découpage : longueurs égales à EPS près -> côté de P1, même si P2 est très légèrement plus long', () => {
  // côté P1 = 2 - EPS/4, côté P2 = 2 + EPS/4 : différence EPS/2 <= EPS
  const x = 2 - EPS / 4;
  const r = cutSegment(S(0, 0, 4, 0), S(x, -1, x, 1));
  assert.equal(r.kept, 'P1');
});

test('découpage : différence de longueur supérieure à EPS -> le plus long côté', () => {
  // côté P1 = 2 - EPS, côté P2 = 2 + EPS : différence 2 EPS > EPS
  const x = 2 - EPS;
  const r = cutSegment(S(0, 0, 4, 0), S(x, -1, x, 1));
  assert.equal(r.kept, 'P2');
});

test('découpage : segment à couper ou segment coupant de longueur nulle -> aucun changement', () => {
  assert.equal(cutSegment(S(1, 1, 1, 1), S(0, 0, 2, 2)).cut, false);
  assert.equal(cutSegment(S(0, 0, 4, 0), S(1, 0, 1, 0)).cut, false);
});

test('découpage : croisement oblique, coordonnées réalistes (salle de quelques mètres)', () => {
  const target = S(1.2, 3.4, 7.9, 3.9);
  const r = cutSegment(target, S(3, 0.5, 3.5, 6.5));
  assert.equal(r.cut, true);
  assert.ok(pointSegmentDistance(target, r.point.x, r.point.y) < 1e-12);
  assert.equal(r.kept, 'P2');
});

test('découpage : les propriétés du segment (poids, observations) sont conservées', () => {
  const r = cutSegment({ ...S(0, 0, 4, 0), weight: 50, observations: 3 }, S(1, -1, 1, 1));
  assert.equal(r.segment.weight, 50);
  assert.equal(r.segment.observations, 3);
});
