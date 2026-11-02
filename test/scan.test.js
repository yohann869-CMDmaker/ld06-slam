import { test } from 'node:test';
import assert from 'node:assert/strict';
import { parseScanCSV, scanToLocalXY, scanToWorldXY, serializeScanCSV, validPoints } from '../src/core/scan.js';

const close = (a, b, eps = 1e-6) => assert.ok(Math.abs(a - b) <= eps, `${a} != ${b}`);

const ROBOTCONTROL = `# Robot position: x=1.0000m, y=2.0000m, theta=90.00deg
angle_deg,distance_m,intensity
0.000,1.0000,200
90.000,2.0000,180
180.000,0.0000,0
270.000,0.5000,20
`;

const MAPTOOL = `# Robot position: x=0.5000m, y=0.2500m, theta=-45.00deg
angle_deg,r_m,x_rel_m,y_rel_m
0.000,1.0000,1.0000,0.0000
90.000,1.0000,0.0000,1.0000
180.000,0,0,0
`;

const LD06_SERIAL = `N,Angle(°),Distance(mm),Intensity,x(mm),y(mm)
0,10.00,1500,210,1477,-260
1,11.00,1510,205,1482,-288

0,10.50,1600,200,1573,-292
`;

test('format RobotControl : pose, colonnes et intensité', () => {
  const s = parseScanCSV(ROBOTCONTROL, 'rc');
  assert.equal(s.format, 'robotcontrol');
  assert.deepEqual(s.pose, { x: 1, y: 2, theta: 90 });
  assert.equal(s.poseFound, true);
  assert.equal(s.points.length, 4);
  assert.deepEqual(s.points[1], { angle: 90, r: 2, intensity: 180 });
});

test('format MapTool : r_m en mètres, intensité absente', () => {
  const s = parseScanCSV(MAPTOOL);
  assert.equal(s.format, 'maptool');
  assert.deepEqual(s.pose, { x: 0.5, y: 0.25, theta: -45 });
  assert.equal(s.points[0].intensity, null);
  assert.equal(s.points[2].r, 0);
});

test('capture série de la librairie LD06 : mm -> m et premier scan seulement', () => {
  const s = parseScanCSV(LD06_SERIAL);
  assert.equal(s.format, 'ld06-lib');
  assert.equal(s.points.length, 2);
  close(s.points[0].r, 1.5);
  assert.equal(s.poseFound, false);
});

test('bloc envoyé par firmware/ESP32_LD06_Capture : lu comme un CSV RobotControl sans pose', () => {
  const block = [
    '# LD06 scan n=3 speed=3602deg/s crc_errors=0',
    'angle_deg,distance_m,intensity',
    '359.62,1.234,212',
    '358.80,0.000,0',
    '357.98,1.240,208',
    '# end',
  ].join('\n');
  const s = parseScanCSV(block, 'usb');
  assert.equal(s.format, 'robotcontrol');
  assert.equal(s.poseFound, false);
  assert.deepEqual(s.pose, { x: 0, y: 0, theta: 0 });
  assert.equal(s.points.length, 3);
  assert.equal(validPoints(s).length, 2);
  assert.equal(s.comments[0], 'LD06 scan n=3 speed=3602deg/s crc_errors=0');
  // relu après enregistrement avec une pose recalée à la main
  const back = parseScanCSV(serializeScanCSV(s, { x: 0.42, y: -0.1, theta: 12.5 }));
  assert.deepEqual(back.pose, { x: 0.42, y: -0.1, theta: 12.5 });
  assert.deepEqual(back.points, s.points);
});

test('validPoints rejette r = 0 et les intensités trop faibles', () => {
  const s = parseScanCSV(ROBOTCONTROL);
  assert.equal(validPoints(s).length, 3);
  assert.equal(validPoints(s, { minIntensity: 100 }).length, 2);
  assert.equal(validPoints(s, { maxRange: 1.5 }).length, 2);
});

test('scanToLocalXY : angle 90° antihoraire -> point sur +Y', () => {
  const s = parseScanCSV(ROBOTCONTROL);
  const pts = scanToLocalXY(s);
  const p90 = pts.find((p) => p.angle === 90);
  close(p90.x, 0);
  close(p90.y, 2);
});

test('flipAngles inverse le sens de rotation (angles bruts horaires)', () => {
  const s = parseScanCSV(ROBOTCONTROL);
  const p = scanToLocalXY(s, undefined, { flipAngles: true }).find((q) => q.r === 2);
  close(p.y, -2);
});

test('scanToWorldXY applique la pose du fichier', () => {
  const s = parseScanCSV(ROBOTCONTROL);
  // point à 1 m devant le robot, robot en (1, 2) orienté vers +Y -> (1, 3)
  const p = scanToWorldXY(s).find((q) => q.r === 1);
  close(p.x, 1);
  close(p.y, 3);
});

test('serializeScanCSV puis parseScanCSV redonne le même scan', () => {
  const s = parseScanCSV(ROBOTCONTROL, 'a');
  const back = parseScanCSV(serializeScanCSV(s, { x: -1, y: 0.5, theta: 10 }));
  assert.deepEqual(back.pose, { x: -1, y: 0.5, theta: 10 });
  assert.deepEqual(back.points, s.points);
});
