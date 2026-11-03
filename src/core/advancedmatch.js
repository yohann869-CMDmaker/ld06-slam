// Méthodes de localisation ajoutées après l'état de l'art (docs/etat_de_l_art.md) :
//   - PL-ICP : ICP point-droite (Censi, ICRA 2008) ;
//   - NDT : Normal Distributions Transform (Biber & Straßer, IROS 2003) ;
//   - Gauss-Newton sur la carte, à la manière de Hector SLAM (Kohlbrecher et al., SSRR 2011).
// Même interface que les méthodes de localization.js : (ref, points locaux, pose initiale,
// options) -> { pose, ... }. Les trois sont locales (affinage autour de la pose initiale).
//
// Toutes linéarisent le mouvement d'un point du scan. Pour la pose (x, y, θ) et un point local p,
// le point monde est w = R(θ)·p + (x, y). Une petite correction (dx, dy, dθ) le déplace de
//   ∂w/∂x = (1, 0),   ∂w/∂y = (0, 1),   ∂w/∂θ = (−(wy − y), wx − x)        (θ en radians).

import { DEG, normalizeAngleDeg } from './pose.js';
import { PointHash } from './matching.js';

const rot = (pose) => [Math.cos(pose.theta * DEG), Math.sin(pose.theta * DEG)];

/** Résout le système 3 × 3 A·d = b (A rangée ligne par ligne) ; null s'il est singulier. */
export function solve3(A, b) {
  const det = (m) => m[0] * (m[4] * m[8] - m[5] * m[7]) - m[1] * (m[3] * m[8] - m[5] * m[6]) + m[2] * (m[3] * m[7] - m[4] * m[6]);
  const D = det(A);
  if (!Number.isFinite(D) || Math.abs(D) < 1e-12) return null;
  return [0, 1, 2].map((col) => {
    const M = [...A];
    for (let r = 0; r < 3; r++) M[3 * r + col] = b[r];
    return det(M) / D;
  });
}

const apply = (pose, d) => ({ x: pose.x + d[0], y: pose.y + d[1], theta: normalizeAngleDeg(pose.theta + d[2] / DEG) });

// ------------------------------------------------------------------ PL-ICP

/**
 * Normales des points de référence (gardées dans la référence) : pour chaque point, analyse en
 * composantes principales de ses voisins à moins de `radius` ; la normale est la direction de
 * plus faible dispersion. `ok` = le voisinage est bien rectiligne (un mur, pas un coin ni un
 * point isolé).
 */
export function orientedPoints(points, radius = 0.08) {
  const hash = new Map();
  const key = (i, j) => i * 100003 + j;
  points.forEach((p, k) => {
    const h = key(Math.floor(p.x / radius), Math.floor(p.y / radius));
    if (!hash.has(h)) hash.set(h, []);
    hash.get(h).push(k);
  });
  return points.map((p) => {
    const ci = Math.floor(p.x / radius);
    const cj = Math.floor(p.y / radius);
    let n = 0;
    let mx = 0;
    let my = 0;
    const nb = [];
    for (let di = -1; di <= 1; di++) {
      for (let dj = -1; dj <= 1; dj++) {
        for (const k of hash.get(key(ci + di, cj + dj)) ?? []) {
          const q = points[k];
          if ((q.x - p.x) ** 2 + (q.y - p.y) ** 2 > radius * radius) continue;
          nb.push(q);
          mx += q.x;
          my += q.y;
          n++;
        }
      }
    }
    if (n < 3) return { x: p.x, y: p.y, nx: 0, ny: 0, ok: false };
    mx /= n;
    my /= n;
    let sxx = 0;
    let syy = 0;
    let sxy = 0;
    for (const q of nb) {
      sxx += (q.x - mx) ** 2;
      syy += (q.y - my) ** 2;
      sxy += (q.x - mx) * (q.y - my);
    }
    const phi = 0.5 * Math.atan2(2 * sxy, sxx - syy); // direction principale
    const tr = sxx + syy;
    const disc = Math.sqrt(((sxx - syy) / 2) ** 2 + sxy * sxy);
    const flat = tr > 0 ? (tr / 2 - disc) / (tr / 2 + disc) : 1; // petite valeur propre / grande
    return { x: p.x, y: p.y, nx: -Math.sin(phi), ny: Math.cos(phi), ok: flat < 0.1 };
  });
}

/**
 * PL-ICP (Censi 2008). Comme l'ICP, chaque point du scan est associé au point de référence le
 * plus proche, mais l'erreur est la distance du point à la DROITE locale de la référence (normale
 * n au point q) : r = n · (w − q), au lieu de la distance au point q. Un point qui glisse le long
 * d'un mur ne coûte rien : c'est le bon modèle pour des murs échantillonnés.
 * Minimiser Σ (r + J·d)² donne (Σ JᵀJ) d = −Σ Jᵀ r, avec J = [nx, ny, n · ∂w/∂θ] : système 3 × 3,
 * résolu directement (comme le calcul direct, mais sur n'importe quelle référence).
 * + Converge en quelques itérations, très précis. − Moins robuste que l'ICP point à point quand
 *   la pose initiale est loin (Censi le note aussi) : on commence avec une grande porte d'association.
 */
export function plicp(ref, pts, init, o = {}) {
  ref.oriented ??= orientedPoints(ref.points);
  ref.orientedHash ??= new PointHash(ref.oriented.filter((q) => q.ok), 0.3);
  const hash = ref.orientedHash;
  let pose = { ...init };
  let iterations = 0;
  const gates = o.gates ?? [0.3, 0.15, 0.08, 0.04];
  for (const gate of gates) {
    for (let it = 0; it < 8; it++) {
      iterations++;
      const [c, s] = rot(pose);
      const A = [0, 0, 0, 0, 0, 0, 0, 0, 0];
      const b = [0, 0, 0];
      let used = 0;
      for (const p of pts) {
        const wx = pose.x + c * p.x - s * p.y;
        const wy = pose.y + s * p.x + c * p.y;
        const q = hash.nearest(wx, wy, gate);
        if (!q) continue;
        const r = q.nx * (wx - q.x) + q.ny * (wy - q.y);
        const J = [q.nx, q.ny, q.nx * -(wy - pose.y) + q.ny * (wx - pose.x)];
        for (let i = 0; i < 3; i++) {
          b[i] -= J[i] * r;
          for (let j = 0; j < 3; j++) A[3 * i + j] += J[i] * J[j];
        }
        used++;
      }
      if (used < 10) break;
      const d = solve3(A, b);
      if (!d) break;
      pose = apply(pose, d);
      if (Math.hypot(d[0], d[1]) < 1e-5 && Math.abs(d[2]) < 1e-6) break;
    }
  }
  return { pose, iterations };
}

// ------------------------------------------------------------------ NDT

/**
 * Grille NDT : pour chaque cellule de côté `size` contenant au moins 3 points de référence, la
 * moyenne μ et la covariance Σ de ces points (Σ régularisée : un mur parfaitement droit donnerait
 * une covariance plate, non inversible). On garde l'inverse C = Σ⁻¹. Comme Biber, on superpose 4
 * grilles décalées d'une demi-cellule, pour lisser les effets de bord des cellules.
 */
export function ndtGrids(points, size) {
  const grids = [];
  for (const [ox, oy] of [[0, 0], [0.5, 0], [0, 0.5], [0.5, 0.5]]) {
    const cells = new Map();
    for (const p of points) {
      const k = `${Math.floor(p.x / size - ox)},${Math.floor(p.y / size - oy)}`;
      if (!cells.has(k)) cells.set(k, []);
      cells.get(k).push(p);
    }
    const out = new Map();
    for (const [k, pts] of cells) {
      if (pts.length < 3) continue;
      const mx = pts.reduce((a, p) => a + p.x, 0) / pts.length;
      const my = pts.reduce((a, p) => a + p.y, 0) / pts.length;
      let sxx = 0;
      let syy = 0;
      let sxy = 0;
      for (const p of pts) {
        sxx += (p.x - mx) ** 2;
        syy += (p.y - my) ** 2;
        sxy += (p.x - mx) * (p.y - my);
      }
      sxx /= pts.length;
      syy /= pts.length;
      sxy /= pts.length;
      // régularisation : plus petite valeur propre au moins 1 % de la plus grande (et 1 cm²)
      const eps = Math.max(0.01 * (sxx + syy), 1e-4);
      sxx += eps;
      syy += eps;
      const det = sxx * syy - sxy * sxy;
      out.set(k, { mx, my, cxx: syy / det, cyy: sxx / det, cxy: -sxy / det });
    }
    grids.push({ ox, oy, cells: out });
  }
  return { size, grids };
}

/**
 * NDT (Biber & Straßer 2003). La référence devient une somme de gaussiennes (une par cellule) :
 * une densité continue, dérivable, sans correspondances point à point. Score d'une pose :
 *   f = −Σ_points Σ_grilles exp(−½ qᵀ C q),   q = w − μ de la cellule où tombe le point w.
 * Méthode de Newton : gradient g et hessienne H analytiques (formules de Biber), pas Δ = −H⁻¹ g,
 * raccourci de moitié tant que le score ne s'améliore pas. Cellules de 1 m puis 0,5 m puis 25 cm :
 * les grandes cellules rattrapent une grosse erreur, les petites donnent la précision.
 */
export function ndt(ref, pts, init, o = {}) {
  ref.ndt ??= new Map();
  let pose = { ...init };
  let iterations = 0;
  for (const size of o.sizes ?? [1, 0.5, 0.25]) {
    if (!ref.ndt.has(size)) ref.ndt.set(size, ndtGrids(ref.points, size));
    const model = ref.ndt.get(size);
    const evaluate = (p, derivatives) => {
      const [c, s] = rot(p);
      let f = 0;
      const g = [0, 0, 0];
      const H = [0, 0, 0, 0, 0, 0, 0, 0, 0];
      for (const pt of pts) {
        const wx = p.x + c * pt.x - s * pt.y;
        const wy = p.y + s * pt.x + c * pt.y;
        // dérivées de w : a0 = (1,0), a1 = (0,1), a2 = ∂w/∂θ ; dérivée seconde b22 = ∂²w/∂θ²
        const a2x = -(wy - p.y);
        const a2y = wx - p.x;
        for (const grid of model.grids) {
          const cell = grid.cells.get(`${Math.floor(wx / size - grid.ox)},${Math.floor(wy / size - grid.oy)}`);
          if (!cell) continue;
          const qx = wx - cell.mx;
          const qy = wy - cell.my;
          // Cq
          const cqx = cell.cxx * qx + cell.cxy * qy;
          const cqy = cell.cxy * qx + cell.cyy * qy;
          const e = Math.exp(-0.5 * (qx * cqx + qy * cqy));
          f -= e;
          if (!derivatives) continue;
          // qᵀC aᵢ
          const u = [cqx, cqy, cqx * a2x + cqy * a2y];
          for (let i = 0; i < 3; i++) g[i] += e * u[i];
          // aⱼᵀ C aᵢ
          const ca2x = cell.cxx * a2x + cell.cxy * a2y;
          const ca2y = cell.cxy * a2x + cell.cyy * a2y;
          const M = [cell.cxx, cell.cxy, ca2x, cell.cxy, cell.cyy, ca2y, ca2x, ca2y, a2x * ca2x + a2y * ca2y];
          const qCb22 = cqx * -(wx - p.x) + cqy * -(wy - p.y);
          for (let i = 0; i < 3; i++) {
            for (let j = 0; j < 3; j++) H[3 * i + j] += e * (-u[i] * u[j] + M[3 * i + j] + (i === 2 && j === 2 ? qCb22 : 0));
          }
        }
      }
      return { f, g, H };
    };
    // Loin du minimum, la hessienne n'est pas définie positive : le pas de Newton peut remonter.
    // Levenberg-Marquardt : (H + λ·diag) d = −g ; λ grandit tant que le score ne baisse pas (le pas
    // tend alors vers une petite descente de gradient), et diminue après un pas réussi.
    let lambda = 1e-3;
    for (let it = 0; it < (o.iterations ?? 20); it++) {
      iterations++;
      const { f, g, H } = evaluate(pose, true);
      if (f === 0) break; // aucun point dans une cellule : rien à optimiser à cette échelle
      const diag = [Math.abs(H[0]), Math.abs(H[4]), Math.abs(H[8])].map((v) => v || 1);
      let improved = false;
      let d = null;
      for (let tries = 0; tries < 12; tries++) {
        const Hd = [...H];
        for (let i = 0; i < 3; i++) Hd[4 * i] += lambda * diag[i];
        d = solve3(Hd, g.map((v) => -v));
        if (d) {
          // pas plafonné à 20 cm et 5° (le modèle n'est valable que près de la pose)
          const scale = Math.min(1, 0.2 / (Math.hypot(d[0], d[1]) || 1), (5 * DEG) / (Math.abs(d[2]) || 1));
          d = d.map((v) => v * scale);
          const next = apply(pose, d);
          if (evaluate(next, false).f < f) {
            pose = next;
            improved = true;
            lambda = Math.max(1e-6, lambda / 3);
            break;
          }
        }
        lambda *= 10;
      }
      if (!improved || (Math.hypot(d[0], d[1]) < 1e-5 && Math.abs(d[2]) < 1e-6)) break;
    }
  }
  return { pose, iterations };
}

// ------------------------------------------------------------------ Gauss-Newton (Hector)

/**
 * Gauss-Newton sur la carte, à la manière de Hector SLAM : chaque point du scan doit tomber sur
 * la référence, c'est-à-dire là où le champ de distance D vaut 0. Résidu : rᵢ = D(wᵢ) ; sa
 * pente ∇D se lit dans le champ interpolé (différences centrées), d'où la ligne de jacobienne
 * Jᵢ = [∂D/∂x, ∂D/∂y, ∇D · ∂w/∂θ]. Pas de Gauss-Newton : (Σ JᵀJ) d = −Σ Jᵀ r. Les points plus
 * loin que la porte (meubles absents de la carte, personnes) sont ignorés. Comme Hector, on
 * travaille du grossier au fin : champs à 10 cm, 5 cm puis 2,5 cm, portes de 50, 20 puis 10 cm.
 * + Pas de correspondances explicites, quelques millisecondes. − Local, comme la descente de
 *   gradient, mais avec un pas calculé (pas de pas à régler).
 */
export function gaussNewton(ref, pts, init, o = {}) {
  let pose = { ...init };
  let iterations = 0;
  for (const [res, gate] of o.levels ?? [[0.1, 0.5], [0.05, 0.2], [0.025, 0.1]]) {
    const field = ref.field(res);
    const h = res / 2;
    for (let it = 0; it < 10; it++) {
      iterations++;
      const [c, s] = rot(pose);
      const A = [0, 0, 0, 0, 0, 0, 0, 0, 0];
      const b = [0, 0, 0];
      let used = 0;
      for (const p of pts) {
        const wx = pose.x + c * p.x - s * p.y;
        const wy = pose.y + s * p.x + c * p.y;
        const r = field.at(wx, wy);
        if (r > gate) continue;
        const gx = (field.at(wx + h, wy) - field.at(wx - h, wy)) / (2 * h);
        const gy = (field.at(wx, wy + h) - field.at(wx, wy - h)) / (2 * h);
        const J = [gx, gy, gx * -(wy - pose.y) + gy * (wx - pose.x)];
        for (let i = 0; i < 3; i++) {
          b[i] -= J[i] * r;
          for (let j = 0; j < 3; j++) A[3 * i + j] += J[i] * J[j];
        }
        used++;
      }
      if (used < 10) break;
      // amortissement de Levenberg-Marquardt léger : le champ n'est pas dérivable sur la référence
      const lambda = 1e-2 * (A[0] + A[4]) + 1e-9;
      A[0] += lambda;
      A[4] += lambda;
      A[8] += lambda;
      const d = solve3(A, b);
      if (!d) break;
      pose = apply(pose, d);
      if (Math.hypot(d[0], d[1]) < 1e-5 && Math.abs(d[2]) < 1e-6) break;
    }
  }
  return { pose, iterations };
}
