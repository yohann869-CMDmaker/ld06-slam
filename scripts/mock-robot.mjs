// Faux robot : imite les routes HTTP du programme ESP32RobotControl dans une scène simulée, pour
// essayer « Explorer avec le robot (Wi-Fi) » sans le robot.
//   /scan (binaire, pose + points), /forward, /backward, /left, /right, /rotateleft, /rotateright
//   (?time=ms ; durée divisée par 2 pour les rotations, comme le vrai programme), /stop, /pose, /ping.
// Le mouvement suit les vitesses vraies du faux robot (différentes des valeurs par défaut de
// l'appli, pour voir l'étalonnage), avec du glissement, et ne traverse pas les murs.
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
const SPEED = { linear: 0.32, angular: 75 }; // m/s, °/s : vitesses vraies du faux robot
const RADIUS = 0.17;
const rng = mulberry32(5);
let truth = scene === ROOMS ? { x: 1.5, y: 1.5, theta: 0 } : { x: 1.5, y: 2.2, theta: 0 };
let busyUntil = 0;
let seed = 1;

/** Mouvement vrai pendant `ms` dans le repère du robot (dx, dy, dθ par seconde), arrêté avant un mur. */
function move(vx, vy, w, ms) {
  const t = Math.min(ms, 2000) / 1000;
  let dx = vx * t * (1 + 0.08 * gaussian(rng));
  const dy = vy * t * (1 + 0.08 * gaussian(rng)) + 0.04 * Math.abs(vx * t) * gaussian(rng);
  const dth = w * t * (1 + 0.08 * gaussian(rng)) + 1.5 * Math.abs(vx * t);
  const dir = Math.atan2(dy, dx) + truth.theta * DEG;
  const len = Math.hypot(dx, dy);
  const hit = len > 0 ? rayCast(scene, truth.x, truth.y, Math.cos(dir), Math.sin(dir)) : null;
  const scale = hit && hit.t < len + RADIUS ? Math.max(0, hit.t - RADIUS) / len : 1;
  if (scale < 1) console.log('  (arrêté avant un mur)');
  dx *= scale;
  truth = composePoses(truth, { x: dx, y: dy * scale, theta: dth });
  busyUntil = Date.now() + t * 1000;
}

function scanBinary() {
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
    '/forward': () => move(SPEED.linear, 0, 0, time),
    '/backward': () => move(-SPEED.linear, 0, 0, time),
    '/left': () => move(0, SPEED.linear, 0, time),
    '/right': () => move(0, -SPEED.linear, 0, time),
    '/rotateleft': () => move(0, 0, SPEED.angular, time / 2),
    '/rotateright': () => move(0, 0, -SPEED.angular, time / 2),
  };
  if (url.pathname === '/scan') {
    // pendant un mouvement, le vrai robot publierait un scan déformé : on attend la fin
    setTimeout(() => {
      res.writeHead(200, { ...cors, 'Content-Type': 'application/octet-stream' });
      res.end(scanBinary());
    }, Math.max(0, busyUntil - Date.now()));
    return;
  }
  if (routes[url.pathname]) {
    routes[url.pathname]();
    console.log(`${url.pathname} ${time} ms -> vraie pose (${truth.x.toFixed(2)} ; ${truth.y.toFixed(2)} ; ${truth.theta.toFixed(1)}°)`);
    return ok();
  }
  if (['/stop', '/pose', '/ping'].includes(url.pathname)) return ok();
  res.writeHead(404, cors);
  res.end();
}).listen(port, () => console.log(`Faux robot sur http://localhost:${port} (scène ${scene === ROOMS ? 'trois pièces' : 'salle'}) : IP du robot dans l'appli = localhost:${port}`));
