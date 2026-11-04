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
// Usage : node scripts/mock-robot.mjs [--scene pieces|salle] [--port 8090]
//         puis, dans l'appli, IP du robot : localhost:8090
import { createServer } from 'node:http';
import { composePoses, DEG } from '../src/core/pose.js';
import { gaussian, mulberry32, rayCast, simulateScan } from '../src/core/sim.js';
import { CLASSROOM, ROOMS } from './lib/scenes.mjs';

const arg = (name, def) => {
  const k = process.argv.indexOf(`--${name}`);
  return k > 0 ? process.argv[k + 1] : def;
};
const scene = arg('scene', 'pieces') === 'salle' ? CLASSROOM : ROOMS;
const port = Number(arg('port', 8090));
const BASE = { linear: 0.32, angular: 75, lateral: 0.27 }; // m/s, °/s, m/s à la vitesse 180
const RADIUS = 0.17;
const rng = mulberry32(5);
let truth = scene === ROOMS ? { x: 1.5, y: 1.5, theta: 0 } : { x: 1.5, y: 2.2, theta: 0 };
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
  const scan = simulateScan(scene, truth, { seed: seed++ });
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
}).listen(port, () => console.log(`Faux robot sur http://localhost:${port} (scène ${scene === ROOMS ? 'trois pièces' : 'salle'}) : IP du robot dans l'appli = localhost:${port}`));
