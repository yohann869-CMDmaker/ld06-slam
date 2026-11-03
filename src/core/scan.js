// Lecture, filtrage et conversion des scans lidar.
//
// Un scan est représenté par :
//   { name, pose: {x, y, theta}, points: [{angle, r, intensity}], format, comments }
// - angle : degrés, repère du robot, convention mathématique (antihoraire)
// - r     : distance en mètres, 0 = pas d'impact
// - intensity : 0..255, ou null si le format ne la fournit pas
//
// Formats reconnus (détectés d'après la ligne d'en-tête) :
// - RobotControl : angle_deg,distance_m,intensity
// - MapTool      : angle_deg,r_m,x_rel_m,y_rel_m
// - Librairie LD06 (printScanCSV, capture du moniteur série) : N,Angle(°),Distance(mm),Intensity,x(mm),y(mm)
// - Teleplot (programme de test ESP32_LD06_Test) : >lidar:x:y;x:y;...|xy, x et y en mm
// Dans les trois premiers cas l'angle est déjà antihoraire : la librairie LD06 a converti
// l'angle horaire du capteur avec 360 - angle.

import { DEG, transformPoints } from './pose.js';

const POSE_RE = /x\s*=\s*(-?\d+(?:\.\d+)?)\s*m?\s*,\s*y\s*=\s*(-?\d+(?:\.\d+)?)\s*m?\s*,\s*theta\s*=\s*(-?\d+(?:\.\d+)?)/i;

/** Filtre par défaut des points aberrants. */
export const DEFAULT_FILTER = Object.freeze({
  minRange: 0.15, // m : en dessous, reflets sur le robot lui-même (vus jusqu'à 10 cm sur le robot réel)
  maxRange: 12, // m : portée max du LD06
  minIntensity: 0, // 0..255 : à régler d'après les données réelles
});

function detectColumns(header) {
  const cols = header.split(',').map((c) => c.trim().toLowerCase());
  const find = (pred) => cols.findIndex(pred);
  const angle = find((c) => c.startsWith('angle'));
  let dist = find((c) => c === 'distance_m' || c === 'r_m');
  let distScale = 1;
  if (dist < 0) {
    dist = find((c) => c.startsWith('distance'));
    distScale = dist >= 0 && isMillimetres(cols[dist]) ? 0.001 : 1;
  }
  const intensity = find((c) => c.startsWith('intensity'));
  const xRel = find((c) => c === 'x_rel_m');
  const yRel = find((c) => c === 'y_rel_m');
  const nCol = find((c) => c === 'n');
  let format = 'inconnu';
  if (xRel >= 0) format = 'maptool';
  else if (cols.includes('distance_m')) format = 'robotcontrol';
  else if (nCol >= 0) format = 'ld06-lib';
  return { angle, dist, distScale, intensity, xRel, yRel, nCol, format };
}

function isMillimetres(col) {
  return col.includes('mm');
}

/** Une ligne Teleplot "x:y;x:y;..." (mm, repère en miroir de la librairie) -> points {angle, r}. */
function decodeTeleplot(body) {
  const points = [];
  for (const pair of body.split(';')) {
    const [xs, ys] = pair.split(':');
    if (!xs || ys === undefined || xs.trim() === '') continue;
    const x = Number(xs) / 1000;
    const y = -Number(ys) / 1000; // correction du miroir de la librairie
    if (!Number.isFinite(x) || !Number.isFinite(y)) continue;
    const r = Math.hypot(x, y);
    points.push({ angle: r > 0 ? (((Math.atan2(y, x) / DEG) % 360) + 360) % 360 : 0, r, intensity: null });
  }
  return points;
}

/**
 * Écart médian (m) entre les points de a et leur plus proche voisin dans b : faible si les
 * deux tours ont été pris depuis la même position, grand si le robot a bougé entre les deux.
 */
function medianNeighbourDistance(a, b) {
  const xy = (pts) => pts.filter((p) => p.r > 0).map((p) => [p.r * Math.cos(p.angle * DEG), p.r * Math.sin(p.angle * DEG)]);
  const pa = xy(a);
  const pb = xy(b);
  if (!pa.length || !pb.length) return Infinity;
  const d = pa.map(([x, y]) => Math.min(...pb.map(([u, v]) => Math.hypot(x - u, y - v)))).sort((p, q) => p - q);
  return d[Math.floor(d.length / 2)];
}

/**
 * Lit la sortie Teleplot du programme de test (une ligne >lidar:...|xy par tour).
 *
 * - La librairie calcule y = -d·sin(angle) alors que l'angle est déjà antihoraire
 *   (ld06.cpp, l.164) : le nuage Teleplot est en miroir. On rétablit y = +d·sin(angle),
 *   ce qui redonne exactement les points de l'export RobotControl.
 * - Par défaut seule la dernière ligne complète est gardée (la dernière ligne est souvent
 *   coupée : seules les lignes terminées par |xy sont lues).
 * - Le programme de test envoie chaque tour en texte à 115200 bauds, plus lentement que le
 *   lidar ne produit ses données : des trames sont perdues et chaque tour a des trous.
 *   Avec mergeLines > 1, les dernières lignes sont fusionnées tant que l'écart médian au
 *   dernier tour reste sous `sameViewMax`. Désactivé par défaut : une petite rotation du robot
 *   entre deux tours garde un écart médian faible mais dédouble les murs lointains.
 */
export function parseTeleplot(text, name = 'scan', options = {}) {
  const { mergeLines = 1, sameViewMax = 0.05 } = options;
  const lines = [...text.matchAll(/>\s*([\w-]+)\s*:([^|>]*)\|xy/g)];
  const scan = { name, pose: { x: 0, y: 0, theta: 0 }, poseFound: false, points: [], format: 'teleplot', comments: [] };
  if (!lines.length) return scan;
  const last = decodeTeleplot(lines.at(-1)[2]);
  scan.points = [...last];
  let used = 1;
  for (let k = lines.length - 2; k >= 0 && used < mergeLines; k--) {
    const pts = decodeTeleplot(lines[k][2]);
    if (medianNeighbourDistance(pts, last) > sameViewMax) break; // le robot a bougé
    scan.points.push(...pts);
    used++;
  }
  scan.mergedLines = used;
  scan.comments.push(
    `teleplot ${lines.at(-1)[1]} : ${used} tour(s) fusionné(s) sur ${lines.length} ligne(s) complète(s), miroir de la librairie corrigé`,
  );
  return scan;
}

/**
 * Lit un scan CSV (ou une sortie Teleplot, détectée automatiquement). Si le texte
 * contient plusieurs scans (capture série de la librairie LD06, où N repart à 0),
 * seul le premier est conservé.
 */
export function parseScanCSV(text, name = 'scan') {
  if (/\|xy/.test(text)) return parseTeleplot(text, name);
  const lines = text.split(/\r?\n/);
  const comments = [];
  let pose = { x: 0, y: 0, theta: 0 };
  let poseFound = false;
  let cols = null;
  const points = [];
  let lastN = -1;

  for (const raw of lines) {
    const line = raw.trim();
    if (!line) {
      if (points.length && cols && cols.format === 'ld06-lib') break; // fin du premier scan
      continue;
    }
    if (line.startsWith('#')) {
      comments.push(line.slice(1).trim());
      const m = POSE_RE.exec(line);
      if (m && !poseFound) {
        pose = { x: Number(m[1]), y: Number(m[2]), theta: Number(m[3]) };
        poseFound = true;
      }
      continue;
    }
    if (!cols && /[a-z]/i.test(line)) {
      cols = detectColumns(line);
      continue;
    }
    if (!cols) cols = { angle: 0, dist: 1, distScale: 1, intensity: 2, xRel: -1, yRel: -1, nCol: -1, format: 'sans en-tête' };

    const v = line.split(',').map((s) => Number(s.trim()));
    if (cols.nCol >= 0) {
      const n = v[cols.nCol];
      if (n <= lastN) break; // un nouveau scan commence
      lastN = n;
    }
    let angle = v[cols.angle];
    const r = v[cols.dist] * cols.distScale;
    if (!Number.isFinite(angle) || !Number.isFinite(r)) continue;
    // MapTool applique le bruit d'angle sur x_rel/y_rel mais exporte angle_deg sans bruit :
    // on reprend l'angle mesuré depuis x_rel/y_rel.
    if (cols.xRel >= 0 && r > 0 && Number.isFinite(v[cols.xRel]) && Number.isFinite(v[cols.yRel])) {
      angle = Math.atan2(v[cols.yRel], v[cols.xRel]) / DEG;
    }
    angle = ((angle % 360) + 360) % 360;
    const intensity = cols.intensity >= 0 && Number.isFinite(v[cols.intensity]) ? v[cols.intensity] : null;
    points.push({ angle, r, intensity });
  }

  return { name, pose, poseFound, points, format: cols ? cols.format : 'vide', comments };
}

/** Points valides : impact présent, distance dans [minRange, maxRange], intensité suffisante. */
export function validPoints(scan, filter = DEFAULT_FILTER) {
  const f = { ...DEFAULT_FILTER, ...filter };
  return scan.points.filter(
    (p) => p.r > 0 && p.r >= f.minRange && p.r <= f.maxRange && (p.intensity === null || p.intensity >= f.minIntensity),
  );
}

/**
 * Un seul tour du lidar. Certains fichiers (export LidarScope, RobotControl) enchaînent
 * plusieurs rotations : les points d'un même angle s'y retrouvent deux ou trois fois et les
 * tours se superposent mal dès que quelque chose bouge. On garde la plus longue suite de
 * points, dans l'ordre d'acquisition, qui tient dans 360° de rotation : un tour complet, sans
 * doublon d'angle. Si des trames ont été perdues dans un tour, un autre tour plus complet gagne.
 */
export function singleTour(scan) {
  const pts = scan.points;
  if (pts.length < 2) return scan;
  // écart d'angle entre deux points consécutifs, ramené dans [-180, 180[
  const steps = [];
  for (let k = 1; k < pts.length; k++) steps.push(((pts[k].angle - pts[k - 1].angle + 540) % 360) - 180);
  const dir = Math.sign(steps.reduce((s, d) => s + d, 0)) || 1; // sens de rotation dans le fichier
  // rotation cumulée depuis le premier point (un retour en arrière ne compte pas)
  const turn = [0];
  for (const d of steps) turn.push(turn.at(-1) + Math.max(0, dir * d));
  let best = [0, pts.length];
  let bestLen = -1;
  for (let i = 0, j = 0; i < pts.length; i++) {
    while (j < pts.length && turn[j] - turn[i] < 360) j++;
    if (j - i > bestLen) {
      best = [i, j];
      bestLen = j - i;
    }
    if (j === pts.length) break;
  }
  if (bestLen === pts.length) return scan;
  return { ...scan, points: pts.slice(best[0], best[1]), tours: turn.at(-1) / 360 };
}

/**
 * Coordonnées cartésiennes dans le repère du robot, triées par angle croissant.
 * options.flipAngles : à activer pour des angles bruts du capteur (sens horaire).
 */
export function scanToLocalXY(scan, filter = DEFAULT_FILTER, options = {}) {
  const sign = options.flipAngles ? -1 : 1;
  return validPoints(scan, filter)
    .map((p) => {
      const a = (((sign * p.angle) % 360) + 360) % 360;
      return { x: p.r * Math.cos(a * DEG), y: p.r * Math.sin(a * DEG), r: p.r, angle: a, intensity: p.intensity };
    })
    .sort((a, b) => a.angle - b.angle);
}

/** Coordonnées dans le repère monde pour une pose donnée (par défaut celle du scan). */
export function scanToWorldXY(scan, pose = scan.pose, filter = DEFAULT_FILTER, options = {}) {
  return transformPoints(scanToLocalXY(scan, filter, options), pose);
}

/** Écrit un scan au format RobotControl (relisible par parseScanCSV). */
export function serializeScanCSV(scan, pose = scan.pose) {
  const lines = [
    `# Robot position: x=${pose.x.toFixed(4)}m, y=${pose.y.toFixed(4)}m, theta=${pose.theta.toFixed(2)}deg`,
  ];
  for (const c of scan.comments || []) {
    if (!POSE_RE.test(c)) lines.push(`# ${c}`);
  }
  lines.push('angle_deg,distance_m,intensity');
  for (const p of scan.points) {
    lines.push(`${p.angle.toFixed(3)},${p.r.toFixed(4)},${p.intensity ?? 0}`);
  }
  return lines.join('\n') + '\n';
}

/** Boîte englobante {minX, minY, maxX, maxY} d'une liste de points. */
export function bounds(points) {
  let minX = Infinity;
  let minY = Infinity;
  let maxX = -Infinity;
  let maxY = -Infinity;
  for (const p of points) {
    if (p.x < minX) minX = p.x;
    if (p.y < minY) minY = p.y;
    if (p.x > maxX) maxX = p.x;
    if (p.y > maxY) maxY = p.y;
  }
  return { minX, minY, maxX, maxY };
}
