// Exploration autonome (bonus du jour 3) : le robot choisit seul où aller pour agrandir la carte.
//
// Méthode des frontières (Yamauchi, 1997) sur la grid map construite par le SLAM :
//   1. obstacles gonflés du rayon du robot plus une marge : le robot devient un point ;
//   2. parcours en largeur depuis le robot sur les cellules libres non gonflées : distance de
//      chaque cellule atteignable et chemin le plus court ;
//   3. cellules frontières : libres, atteignables, avec une voisine inconnue (bord de ce qu'on a
//      vu) ; regroupées en zones, les petites zones (bruit) sont ignorées ;
//   4. but : la cellule la plus proche de la zone frontière la plus proche ;
//   5. commande : le point le plus loin du chemin encore en vue directe (au plus `maxStep`) ;
//      tourner vers lui si l'écart dépasse `turnTol`, sinon avancer, sans s'approcher de l'obstacle
//      le plus proche droit devant dans le dernier scan (sécurité si la carte est en retard).
// Plus aucune frontière atteignable : la carte est finie.
//
// Commandes simples (tourner sur place, avancer) : ce sont celles du robot Mecanum (/rotateleft,
// /forward...), et le lidar reste dans l'axe de la marche.

import { composePoses, DEG, normalizeAngleDeg } from './pose.js';
import { rayCast, simulateScan, gaussian, mulberry32 } from './sim.js';
import { scanToLocalXY } from './scan.js';
import { Slam } from './slam.js';

export const DEFAULT_EXPLORE = Object.freeze({
  robotRadius: 0.17, // m : demi-diagonale du robot (châssis Mecanum ~ 24 × 22 cm)
  clearance: 0.08, // m : marge en plus autour des obstacles
  minFrontier: 5, // cellules : une zone frontière plus petite est du bruit
  maxStep: 0.4, // m : avance maximale par commande (un scan au plus tous les 40 cm)
  maxTurn: 60, // degrés : rotation maximale par commande
  turnTol: 15, // degrés : au-delà, tourner avant d'avancer
  goalReached: 0.25, // m : frontières plus proches ignorées (déjà sous le robot, rien de plus à voir)
  maxRange: 12, // m : frontières plus loin du départ ignorées (faisceaux sortis par les vitres)
  startFree: 0.3, // m : autour du robot, on peut passer même si la carte n'est pas sûre (dégagement)
  maxTries: 12, // commandes vers un même but sans l'atteindre : but abandonné
});

const N4 = [
  [1, 0],
  [-1, 0],
  [0, 1],
  [0, -1],
];
const N8 = [...N4, [1, 1], [1, -1], [-1, 1], [-1, -1]];

/** Cellules interdites au centre du robot : à moins de `radius` d'une cellule occupée. */
export function blockedCells(grid, radius) {
  const blocked = new Uint8Array(grid.width * grid.height);
  const rc = radius / grid.res;
  const offsets = [];
  for (let dj = -Math.ceil(rc); dj <= Math.ceil(rc); dj++) {
    for (let di = -Math.ceil(rc); di <= Math.ceil(rc); di++) if (Math.hypot(di, dj) <= rc) offsets.push([di, dj]);
  }
  for (let j = 0; j < grid.height; j++) {
    for (let i = 0; i < grid.width; i++) {
      if (grid.classify(grid.index(i, j)) !== 1) continue;
      for (const [di, dj] of offsets) if (grid.inside(i + di, j + dj)) blocked[grid.index(i + di, j + dj)] = 1;
    }
  }
  return blocked;
}

/**
 * Où aller ensuite. grid : grid map du SLAM, pose : pose estimée du robot, points : dernier scan
 * dans le repère du robot (sécurité devant), home : pose de départ (limite `maxRange`),
 * avoid : buts abandonnés [{x, y}].
 * Retourne { done, reason, goal, path [{x, y}], frontiers [{x, y, size}], command } ; command :
 * { kind: 'rotate', angle (degrés, + = à gauche) } ou { kind: 'forward', distance (m) }.
 */
export function planExploration(grid, pose, points = [], home = pose, options = {}, avoid = []) {
  const o = { ...DEFAULT_EXPLORE, ...options };
  const W = grid.width;
  const H = grid.height;
  const res = grid.res;
  const blocked = blockedCells(grid, o.robotRadius + o.clearance);
  const [i0, j0] = grid.worldToCell(pose.x, pose.y);
  if (!grid.inside(i0, j0)) return { done: true, reason: 'robot hors de la carte', path: [], frontiers: [] };
  const nearStart = (i, j) => Math.hypot(i - i0, j - j0) * res <= o.startFree;
  const passable = (i, j) => {
    if (!grid.inside(i, j)) return false;
    const c = grid.classify(grid.index(i, j));
    if (nearStart(i, j)) return c !== 1;
    return c === -1 && !blocked[grid.index(i, j)];
  };

  // 2. parcours en largeur (8 voisins, sans couper les coins)
  const dist = new Int32Array(W * H).fill(-1);
  const parent = new Int32Array(W * H).fill(-1);
  const queue = [grid.index(i0, j0)];
  dist[queue[0]] = 0;
  for (let q = 0; q < queue.length; q++) {
    const k = queue[q];
    const i = k % W;
    const j = (k - i) / W;
    for (const [di, dj] of N8) {
      const a = i + di;
      const b = j + dj;
      if (!passable(a, b) || dist[grid.index(a, b)] >= 0) continue;
      if (di && dj && !(passable(i + di, j) && passable(i, j + dj))) continue;
      const n = grid.index(a, b);
      dist[n] = dist[k] + 1;
      parent[n] = k;
      queue.push(n);
    }
  }

  // 3. frontières atteignables, regroupées en zones
  const isFrontier = new Uint8Array(W * H);
  for (const k of queue) {
    const i = k % W;
    const j = (k - i) / W;
    if (grid.classify(k) !== -1) continue;
    const c = grid.cellCenter(i, j);
    if (Math.hypot(c.x - home.x, c.y - home.y) > o.maxRange) continue;
    if (Math.hypot(c.x - pose.x, c.y - pose.y) < o.goalReached) continue;
    if (avoid.some((p) => Math.hypot(c.x - p.x, c.y - p.y) < 0.4)) continue;
    if (N4.some(([di, dj]) => !grid.inside(i + di, j + dj) || grid.classify(grid.index(i + di, j + dj)) === 0)) isFrontier[k] = 1;
  }
  const zones = [];
  const seen = new Uint8Array(W * H);
  for (const k of queue) {
    if (!isFrontier[k] || seen[k]) continue;
    const cells = [k];
    seen[k] = 1;
    for (let q = 0; q < cells.length; q++) {
      const i = cells[q] % W;
      const j = (cells[q] - i) / W;
      for (const [di, dj] of N8) {
        if (!grid.inside(i + di, j + dj)) continue;
        const n = grid.index(i + di, j + dj);
        if (isFrontier[n] && !seen[n]) {
          seen[n] = 1;
          cells.push(n);
        }
      }
    }
    if (cells.length < o.minFrontier) continue;
    const goal = cells.reduce((a, b) => (dist[b] < dist[a] ? b : a));
    const gi = goal % W;
    zones.push({ goal, size: cells.length, cost: dist[goal], center: grid.cellCenter(gi, (goal - gi) / W) });
  }
  const frontiers = zones.map((z) => ({ ...z.center, size: z.size }));
  if (!zones.length) return { done: true, reason: 'plus de frontière atteignable : carte terminée', path: [], frontiers };

  // 4. zone la plus proche ; chemin du robot au but
  const zone = zones.reduce((a, b) => (b.cost < a.cost ? b : a));
  const cellsPath = [];
  for (let k = zone.goal; k >= 0; k = parent[k]) cellsPath.push(k);
  cellsPath.reverse();
  const path = cellsPath.map((k) => grid.cellCenter(k % W, (k - (k % W)) / W));
  const goal = path.at(-1);

  // 5. point visé : le plus loin du chemin en vue directe, à moins de maxStep
  const visible = (k) => {
    const ti = k % W;
    const tj = (k - ti) / W;
    const n = Math.max(Math.abs(ti - i0), Math.abs(tj - j0));
    for (let s = 1; s <= n; s++) {
      if (!passable(Math.round(i0 + ((ti - i0) * s) / n), Math.round(j0 + ((tj - j0) * s) / n))) return false;
    }
    return true;
  };
  let target = path[Math.min(1, path.length - 1)];
  for (let s = 1; s < cellsPath.length; s++) {
    if (Math.hypot(path[s].x - pose.x, path[s].y - pose.y) > o.maxStep) break;
    if (visible(cellsPath[s])) target = path[s];
  }
  const bearing = normalizeAngleDeg(Math.atan2(target.y - pose.y, target.x - pose.x) / DEG - pose.theta);
  let command;
  if (Math.abs(bearing) > o.turnTol) {
    command = { kind: 'rotate', angle: Math.max(-o.maxTurn, Math.min(o.maxTurn, bearing)) };
  } else {
    const free = freeAhead(points, o.robotRadius) - o.robotRadius - o.clearance;
    const d = Math.min(Math.hypot(target.x - pose.x, target.y - pose.y), free);
    // obstacle juste devant, pas encore dans la carte : se tourner pour le voir et le contourner
    command = d < 0.05 ? { kind: 'rotate', angle: bearing >= 0 ? 30 : -30 } : { kind: 'forward', distance: d };
  }
  return { done: false, reason: '', goal, path, frontiers, command };
}

/** Distance libre droit devant dans le scan (repère du robot) : premier point dans le couloir de demi-largeur halfWidth. */
export function freeAhead(points, halfWidth) {
  let d = Infinity;
  for (const p of points) if (p.x > 0 && Math.abs(p.y) <= halfWidth && p.x < d) d = p.x;
  return d;
}

/**
 * Exploration pas à pas : garde la pose de départ, le but en cours et les buts abandonnés (un but
 * vers lequel le robot n'avance plus : frontière qu'on ne peut pas voir de plus près, vitre...).
 */
export class Explorer {
  constructor(options = {}) {
    this.o = { ...DEFAULT_EXPLORE, ...options };
    this.home = null;
    this.avoid = [];
    this.goal = null;
    this.tries = 0;
  }

  next(grid, pose, points) {
    if (!this.home) this.home = { ...pose };
    const plan = planExploration(grid, pose, points, this.home, this.o, this.avoid);
    if (plan.done) return plan;
    if (this.goal && Math.hypot(plan.goal.x - this.goal.x, plan.goal.y - this.goal.y) < 0.3) this.tries++;
    else {
      this.goal = plan.goal;
      this.tries = 1;
    }
    if (this.tries > this.o.maxTries) {
      this.avoid.push(this.goal);
      this.goal = null;
      return this.next(grid, pose, points);
    }
    return plan;
  }
}

/** Déplacement commandé (repère du robot) correspondant à une commande : sert d'odométrie au SLAM. */
export function commandMotion(command) {
  return command.kind === 'rotate' ? { x: 0, y: 0, theta: command.angle } : { x: command.distance, y: 0, theta: 0 };
}

/**
 * Exploration simulée dans une scène de segments : à chaque pas, scan simulé à la pose vraie,
 * SLAM, choix de la commande, puis mouvement vrai entaché d'erreurs (roues Mecanum qui glissent :
 * `trans` en part de la distance, `slip` dérive latérale par mètre, `rot` en part de l'angle,
 * `bias` dérive de cap en degrés par mètre). Un mouvement qui heurterait un mur s'arrête avant
 * (compté dans `collisions`).
 * Générateur : produit { step, truth, poses (vraies), slam, result, plan, collisions, distance }.
 */
export function* simulateExploration(scene, start, options = {}) {
  const o = { maxSteps: 400, seed: 3, slam: {}, explore: {}, scan: {}, ...options };
  const m = { trans: 0.1, slip: 0.05, rot: 0.1, bias: 1.5, ...options.motion };
  const rng = mulberry32(o.seed);
  const slam = new Slam(o.slam);
  const explorer = new Explorer(o.explore);
  const radius = explorer.o.robotRadius;
  let truth = { ...start };
  const poses = [];
  let odom = null;
  let collisions = 0;
  let distance = 0;
  for (let step = 0; step < o.maxSteps; step++) {
    const scan = simulateScan(scene, truth, { seed: o.seed * 1000 + step, ...o.scan, name: `explo_${String(step).padStart(3, '0')}` });
    const result = slam.addScan(scan, odom, step === 0 ? start : null);
    poses.push({ ...truth });
    const plan = explorer.next(slam.grid, slam.last, scanToLocalXY(scan));
    yield { step, truth: { ...truth }, poses, slam, result, plan, collisions, distance };
    if (plan.done) return;
    const c = plan.command;
    odom = commandMotion(c);
    if (c.kind === 'rotate') {
      truth = composePoses(truth, { x: 0, y: 0, theta: c.angle * (1 + m.rot * gaussian(rng)) });
    } else {
      let d = c.distance * (1 + m.trans * gaussian(rng));
      const a = truth.theta * DEG;
      const hit = rayCast(scene, truth.x, truth.y, Math.cos(a), Math.sin(a));
      if (hit && hit.t < d + radius) {
        d = Math.max(0, hit.t - radius);
        collisions++;
      }
      truth = composePoses(truth, { x: d, y: m.slip * d * gaussian(rng), theta: m.bias * d });
      distance += d;
    }
  }
}

/** Part des points de contrôle (sol libre de la scène) que la carte a vus libres. */
export function freeCoverage(grid, points) {
  let n = 0;
  for (const p of points) {
    const [i, j] = grid.worldToCell(p.x, p.y);
    if (grid.inside(i, j) && grid.classify(grid.index(i, j)) === -1) n++;
  }
  return n / Math.max(points.length, 1);
}

/**
 * Vitesses du robot réel, pour traduire une commande en durée (le robot Mecanum n'a pas de
 * codeurs : il roule pendant une durée donnée). Valeurs de départ à mesurer ; updateSpeeds les
 * corrige au fil de l'exploration d'après le déplacement mesuré par le SLAM.
 */
export const DEFAULT_SPEEDS = Object.freeze({ linear: 0.25, angular: 90 }); // m/s, degrés/s

const MAX_MOVE_MS = 2000; // limite du programme ESP32RobotControl

/**
 * Requête HTTP pour une commande : { route, time (paramètre envoyé), duration (ms de mouvement),
 * command (commande effective, réduite si la durée a été plafonnée) }. Le programme divise par 2
 * la durée des rotations : on envoie le double.
 */
export function commandRequest(command, speeds = DEFAULT_SPEEDS) {
  if (command.kind === 'rotate') {
    const duration = Math.min(MAX_MOVE_MS, (1000 * Math.abs(command.angle)) / speeds.angular);
    const angle = Math.sign(command.angle) * (duration / 1000) * speeds.angular;
    return { route: command.angle >= 0 ? 'rotateleft' : 'rotateright', time: Math.round(2 * duration), duration, command: { kind: 'rotate', angle } };
  }
  const duration = Math.min(MAX_MOVE_MS, (1000 * command.distance) / speeds.linear);
  return { route: 'forward', time: Math.round(duration), duration, command: { kind: 'forward', distance: (duration / 1000) * speeds.linear } };
}

/**
 * Étalonnage en ligne : compare le mouvement commandé au mouvement mesuré par le SLAM
 * (déplacement relatif entre deux scans bien localisés) et corrige la vitesse correspondante
 * (rapport borné à [0,5 ; 2], lissé par `rate`). Les petits mouvements sont ignorés (trop bruités).
 */
export function updateSpeeds(speeds, command, measured, rate = 0.3) {
  const out = { ...speeds };
  if (command.kind === 'rotate') {
    if (Math.abs(command.angle) < 15) return out;
    const ratio = Math.min(2, Math.max(0.5, Math.abs(measured.theta) / Math.abs(command.angle)));
    out.angular = speeds.angular * ratio ** rate;
  } else {
    if (command.distance < 0.1) return out;
    const ratio = Math.min(2, Math.max(0.5, Math.hypot(measured.x, measured.y) / command.distance));
    out.linear = speeds.linear * ratio ** rate;
  }
  return out;
}
