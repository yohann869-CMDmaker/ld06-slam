import { test } from 'node:test';
import assert from 'node:assert/strict';
import {
  composePoses,
  invertPose,
  localToWorld,
  normalizeAngleDeg,
  poseError,
  worldToLocal,
} from '../src/core/pose.js';

const close = (a, b, eps = 1e-9) => assert.ok(Math.abs(a - b) <= eps, `${a} != ${b}`);

test('normalizeAngleDeg ramène dans ]-180, 180]', () => {
  assert.equal(normalizeAngleDeg(190), -170);
  assert.equal(normalizeAngleDeg(-180), 180);
  assert.equal(normalizeAngleDeg(540), 180);
  assert.equal(normalizeAngleDeg(-30), -30);
});

test('exemple du cours : point à 2 m sur la droite, robot en (1,1) orienté à 90°', () => {
  // Le LD06 donne 90° en sens horaire, soit -90° en convention maths : point local (0, -2).
  const p = localToWorld({ x: 1, y: 1, theta: 90 }, 0, -2);
  close(p.x, 3);
  close(p.y, 1);
});

test('worldToLocal est l\'inverse de localToWorld', () => {
  const pose = { x: 0.7, y: -1.3, theta: 37 };
  const w = localToWorld(pose, 1.2, 0.4);
  const l = worldToLocal(pose, w.x, w.y);
  close(l.x, 1.2);
  close(l.y, 0.4);
});

test('composePoses avec invertPose donne l\'identité', () => {
  const p = { x: 2, y: -1, theta: 123 };
  const id = composePoses(p, invertPose(p));
  close(id.x, 0);
  close(id.y, 0);
  close(id.theta, 0);
});

test('poseError gère le passage par ±180°', () => {
  const e = poseError({ x: 0, y: 0, theta: 179 }, { x: 3, y: 4, theta: -179 });
  close(e.dist, 5);
  close(e.angle, 2);
});
