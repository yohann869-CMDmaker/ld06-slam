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

/** Ligne Teleplot telle que l'écrit la librairie (mm, y en miroir) pour une liste de points {angle, r}. */
const teleplotLine = (points) =>
  '>lidar:' +
  points
    .map((p) => `${Math.round(p.r * 1000 * Math.cos((p.angle * Math.PI) / 180))}:${-Math.round(p.r * 1000 * Math.sin((p.angle * Math.PI) / 180))}`)
    .join(';') +
  ';|xy';

test('Teleplot : mm -> m et correction du miroir de la librairie', () => {
  const s = parseScanCSV('>lidar:1000:500;0:0;|xy\n', 'tp');
  assert.equal(s.format, 'teleplot');
  assert.equal(s.points.length, 2);
  // la librairie écrit y = -d·sin(angle) : (1000, 500) correspond au point (1 ; -0,5)
  const [p] = scanToLocalXY(s);
  close(p.x, 1);
  close(p.y, -0.5);
  close(p.angle, 360 - (Math.atan2(0.5, 1) * 180) / Math.PI);
  assert.equal(validPoints(s).length, 1, '0:0 = pas d\'impact');
});

test('Teleplot : la dernière ligne coupée par le copier-coller est ignorée', () => {
  const s = parseScanCSV('>lidar:1000:0;0:-1000;|xy\r\n>lidar:2000:0;0:-20', 'tp');
  assert.equal(s.points.length, 2);
  close(s.points[0].r, 1);
});

test('Teleplot : les tours pris depuis la même position sont fusionnés, pas ceux d\'avant un déplacement', async () => {
  const { rectangle, simulateScan } = await import('../src/core/sim.js');
  const room = rectangle(0, 0, 5, 4);
  const here = { x: 2, y: 1.5, theta: 0 };
  const hit = (p) => p.r > 0;
  // tour pris avant le déplacement du robot
  const before = simulateScan(room, { x: 3.2, y: 2.5, theta: 30 }, { seed: 1 }).points.filter(hit);
  // trois tours depuis la position actuelle, chacun avec un secteur de 90° perdu, à un endroit différent
  const lossy = (seed, lo) => simulateScan(room, here, { seed }).points.filter((p) => hit(p) && !(p.angle >= lo && p.angle < lo + 90));
  const tours = [lossy(2, 0), lossy(3, 120), lossy(4, 240)];
  const text = [before, ...tours].map(teleplotLine).join('\n') + '\n>lidar:12:3';
  const s = parseScanCSV(text, 'tp');
  assert.equal(s.points.length, tours.reduce((n, t) => n + t.length, 0));
  assert.match(s.comments[0], /3 tour\(s\) fusionné\(s\) sur 4/);
  // l'union couvre tout le tour : plus aucun secteur vide de plus de 5°
  const angles = s.points.map((p) => p.angle).sort((a, b) => a - b);
  const gaps = angles.slice(1).map((a, k) => a - angles[k]).concat(angles[0] + 360 - angles.at(-1));
  assert.ok(Math.max(...gaps) < 5, `plus grand trou ${Math.max(...gaps).toFixed(1)}°`);
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
