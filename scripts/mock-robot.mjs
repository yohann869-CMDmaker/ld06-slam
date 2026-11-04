// Faux robot : imite les routes HTTP du programme du robot (ESP32RobotControl du module,
// firmware/ESP32_RobotWifi) dans une scène simulée, pour essayer le pilotage et l'exploration
// sans le robot.
//   /scan (binaire, pose + points), /forward, /backward, /left, /right, /rotateleft, /rotateright
//   (?time=ms ; durée divisée par 2 pour les rotations, comme le vrai programme), /stop,
//   /speed?v=0..255, /pose, /ping.
// Mouvement continu, comme le vrai robot : une commande fixe la vitesse et repousse l'arrêt
// automatique de `time` ms ; la position avance avec le temps (et pendant les scans). Vitesses
// vraies différentes des valeurs par défaut de l'appli (pour voir l'étalonnage), proportionnelles à
// /speed, avec du glissement ; le robot s'arrête avant les murs.
// Scènes : « reelle » (défaut) = NOTRE salle de classe, murs et meubles de la carte de référence
// (data/reference/carte_reference.json), départ à la pose du scan scan_1102_180026 ; « salle » et
// « pieces » = scènes simulées du banc d'essai. --chaises N ajoute N chaises (4 pieds de 3 cm) au
// hasard, comme lors du premier essai réel ; --bruit σ (m) règle le bruit du lidar (défaut 1,5 cm).
// Usage : node scripts/mock-robot.mjs [--scene reelle|salle|pieces] [--chaises 0] [--port 8090]
//         puis, dans l'appli : liaison Wi-Fi, IP du robot localhost:8090
import { createServer } from 'node:http';
import { readFileSync } from 'node:fs';
import { fileURLToPath } from 'node:url';
import { composePoses, DEG } from '../src/core/pose.js';
import { gaussian, mulberry32, rayCast, rectangle, simulateScan } from '../src/core/sim.js';
import { CLASSROOM, ROOMS } from './lib/scenes.mjs';

const arg = (name, def) => {
  const k = process.argv.indexOf(`--${name}`);
  return k > 0 ? process.argv[k + 1] : def;
};
const sceneName = arg('scene', 'reelle');
const port = Number(arg('port', 8090));
const noise = Number(arg('bruit', 0.015));

/** Notre salle : segments de la carte de référence, et une pose de départ prise au sol. */
function realRoom() {
  const ref = JSON.parse(readFileSync(fileURLToPath(new URL('../data/reference/carte_reference.json', import.meta.url)), 'utf8'));
  const segments = ref.segments.map(([x1, y1, x2, y2]) => ({ x1, y1, x2, y2 }));
  const s = ref.scans.find((k) => k.name === 'scan_1102_180026.csv') ?? ref.scans.find((k) => !k.en_hauteur);
  return { segments, start: { x: s.pose[0], y: s.pose[1], theta: s.pose[2] } };
}

const SCENES = {
  reelle: realRoom,
  salle: () => ({ segments: CLASSROOM, start: { x: 1.5, y: 2.2, theta: 0 } }),
  pieces: () => ({ segments: ROOMS, start: { x: 1.5, y: 1.5, theta: 0 } }),
};
if (!SCENES[sceneName]) {
  console.error(`Scène inconnue : ${sceneName} (reelle, salle ou pieces)`);
  process.exit(1);
}
const built = SCENES[sceneName]();
const scene = [...built.segments];

// chaises : 4 pieds de 3 cm aux coins d'un carré de 40 cm, posées au hasard dans la boîte de la
// scène, à plus de 60 cm du départ (le robot doit pouvoir démarrer)
const nChaises = Number(arg('chaises', 0));
const placeRng = mulberry32(42);
{
  const xs = built.segments.flatMap((w) => [w.x1, w.x2]);
  const ys = built.segments.flatMap((w) => [w.y1, w.y2]);
  for (let k = 0; k < nChaises; k++) {
    const cx = Math.min(...xs) + 0.5 + placeRng() * (Math.max(...xs) - Math.min(...xs) - 1);
    const cy = Math.min(...ys) + 0.5 + placeRng() * (Math.max(...ys) - Math.min(...ys) - 1);
    if (Math.hypot(cx - built.start.x, cy - built.start.y) < 0.6) continue;
    for (const [dx, dy] of [[-0.2, -0.2], [0.2, -0.2], [-0.2, 0.2], [0.2, 0.2]]) scene.push(...rectangle(cx + dx, cy + dy, cx + dx + 0.03, cy + dy + 0.03));
  }
}
const BASE = { linear: 0.32, angular: 75, lateral: 0.27 }; // m/s, °/s, m/s à la vitesse 180
const RADIUS = 0.17;
const rng = mulberry32(5);
let truth = { ...built.start };
let pwm = 180;
let motion = { vx: 0, vy: 0, w: 0, until: 0 }; // vitesses dans le repère du robot (m/s, °/s)
let last = Date.now();
let seed = 1;
const slip = 1 + 0.08 * gaussian(rng); // glissement propre à ce robot

/** Fait avancer la vraie pose jusqu'à maintenant, par pas de 20 ms, sans traverser les murs. */
function advance(now = Date.now()) {
  const end = Math.min(now, motion.until);
  while (last < end) {
    const dt = Math.min(20, end - last) / 1000;
    const dx = motion.vx * dt * slip;
    const dy = motion.vy * dt * slip + 0.03 * Math.abs(motion.vx * dt) * gaussian(rng);
    const dir = Math.atan2(dy, dx) + truth.theta * DEG;
    const len = Math.hypot(dx, dy);
    const hit = len > 0 ? rayCast(scene, truth.x, truth.y, Math.cos(dir), Math.sin(dir)) : null;
    if (hit && hit.t < len + RADIUS) {
      motion = { vx: 0, vy: 0, w: 0, until: 0 };
      console.log('  (arrêté avant un mur)');
      break;
    }
    truth = composePoses(truth, { x: dx, y: dy, theta: motion.w * dt * slip + 1.5 * Math.abs(motion.vx * dt) });
    last += dt * 1000;
  }
  last = now;
}

function drive(vx, vy, w, ms) {
  advance();
  const k = pwm / 180;
  motion = { vx: vx * k, vy: vy * k, w: w * k, until: Date.now() + Math.min(ms, 2000) };
}

function scanBinary() {
  advance();
  const scan = simulateScan(scene, truth, { seed: seed++, noiseRange: noise });
  const pts = scan.points;
  const buf = Buffer.alloc(8 + 5 * pts.length);
  buf.writeUInt16LE(pts.length, 6); // pose laissée à 0 : le vrai robot ne connaît pas sa pose
  pts.forEach((p, k) => {
    buf.writeUInt16LE(Math.round(p.angle * 100) % 36000, 8 + 5 * k);
    buf.writeUInt16LE(Math.round(p.r * 1000), 10 + 5 * k);
    buf.writeUInt8(p.intensity, 12 + 5 * k);
  });
  return buf;
}

createServer((req, res) => {
  const url = new URL(req.url, 'http://robot');
  const time = Number(url.searchParams.get('time') ?? 300);
  const cors = { 'Access-Control-Allow-Origin': '*' };
  const ok = () => {
    res.writeHead(200, { ...cors, 'Content-Type': 'text/plain' });
    res.end('OK');
  };
  const routes = {
    '/forward': () => drive(BASE.linear, 0, 0, time),
    '/backward': () => drive(-BASE.linear, 0, 0, time),
    '/left': () => drive(0, BASE.lateral, 0, time),
    '/right': () => drive(0, -BASE.lateral, 0, time),
    '/rotateleft': () => drive(0, 0, BASE.angular, time / 2),
    '/rotateright': () => drive(0, 0, -BASE.angular, time / 2),
  };
  if (url.pathname === '/scan') {
    res.writeHead(200, { ...cors, 'Content-Type': 'application/octet-stream' });
    res.end(scanBinary());
    return;
  }
  if (routes[url.pathname]) {
    routes[url.pathname]();
    return ok();
  }
  if (url.pathname === '/stop') {
    advance();
    motion.until = 0;
    console.log(`/stop -> vraie pose (${truth.x.toFixed(2)} ; ${truth.y.toFixed(2)} ; ${truth.theta.toFixed(1)}°)`);
    return ok();
  }
  if (url.pathname === '/speed') {
    pwm = Math.max(0, Math.min(255, Number(url.searchParams.get('v') ?? pwm)));
    console.log(`/speed ${pwm}`);
    return ok();
  }
  if (['/pose', '/ping'].includes(url.pathname)) return ok();
  res.writeHead(404, cors);
  res.end();
}).listen(port, () => {
  console.log(`Faux robot sur http://localhost:${port} : scène « ${sceneName} »${nChaises ? `, ${nChaises} chaises` : ''}, bruit ${(100 * noise).toFixed(1)} cm`);
  console.log(`Départ : (${truth.x.toFixed(2)} ; ${truth.y.toFixed(2)} ; ${truth.theta.toFixed(1)}°). Dans l'appli : liaison Wi-Fi, IP du robot = localhost:${port}`);
});
