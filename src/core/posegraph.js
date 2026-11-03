// Graphe de poses (jour 3, Graph-SLAM du document « Localisation ») : corriger la dérive.
//
// Nœuds : les poses des scans clés (x, y, θ). Liens : un déplacement mesuré z entre deux nœuds
// (« le scan j est à z du scan i »), avec sa précision. Deux sortes de liens :
//   - suite : entre deux scans clés consécutifs (suivi du robot) ; leurs petites erreurs s'ajoutent
//     au fil du trajet : c'est la dérive ;
//   - boucle : quand le robot revient dans une zone déjà vue, le nouveau scan est recalé sur un
//     ancien : ce lien relie deux nœuds éloignés dans la suite et révèle la dérive accumulée.
// On cherche les poses qui respectent au mieux tous les liens à la fois : moindres carrés
//   min Σ eᵢⱼᵀ Ωᵢⱼ eᵢⱼ,   eᵢⱼ = (déplacement prédit par les poses i et j) − zᵢⱼ,
// résolus par Gauss-Newton : on linéarise les erreurs autour des poses courantes, on résout le
// système normal H·Δ = −b (H = Σ Jᵀ Ω J, creux mais traité ici en dense : quelques centaines de
// nœuds au plus), on applique Δ, et on recommence. Le premier nœud est fixé (il définit le repère).
// La dérive se répartit ainsi le long de la boucle au lieu de se concentrer à l'arrivée.

import { DEG } from './pose.js';

const wrap = (a) => Math.atan2(Math.sin(a), Math.cos(a));

/**
 * poses : [{ x, y, theta (degrés) }] ; edges : [{ from, to, z: { x, y, theta (degrés) }, sigma: { xy (m), theta (degrés) } }]
 * ou, à la place de sigma, info : matrice d'information 3 × 3 (Ω, unités m et radians) qui peut être
 * anisotrope (précis en travers d'un couloir, imprécis le long).
 * Retourne { poses (corrigées), error0, error, iterations } : error = Σ eᵀΩe avant / après.
 */
export function optimizePoseGraph(poses, edges, { iterations = 15, fixed = 0, tolerance = 1e-6 } = {}) {
  const n = poses.length;
  const x = poses.flatMap((p) => [p.x, p.y, p.theta * DEG]);
  const totalError = () => {
    let e2 = 0;
    for (const ed of edges) {
      const e = edgeError(x, ed);
      const W = weights(ed);
      for (let r = 0; r < 3; r++) for (let c = 0; c < 3; c++) e2 += e[r] * W[r][c] * e[c];
    }
    return e2;
  };
  const error0 = totalError();
  let it = 0;
  for (; it < iterations; it++) {
    const N = 3 * n;
    const H = new Float64Array(N * N);
    const b = new Float64Array(N);
    for (const ed of edges) {
      const { e, A, B } = linearize(x, ed);
      const W = weights(ed);
      // WA = Ω·A, WB = Ω·B, We = Ω·e
      const WA = [0, 1, 2].map((k) => [0, 1, 2].map((c) => W[k][0] * A[0][c] + W[k][1] * A[1][c] + W[k][2] * A[2][c]));
      const WB = [0, 1, 2].map((k) => [0, 1, 2].map((c) => W[k][0] * B[0][c] + W[k][1] * B[1][c] + W[k][2] * B[2][c]));
      const We = [0, 1, 2].map((k) => W[k][0] * e[0] + W[k][1] * e[1] + W[k][2] * e[2]);
      const i = 3 * ed.from;
      const j = 3 * ed.to;
      // H += Jᵀ Ω J et b += Jᵀ Ω e, avec J = [A | B] (Ω diagonale)
      for (let r = 0; r < 3; r++) {
        for (let c = 0; c < 3; c++) {
          let hii = 0;
          let hij = 0;
          let hjj = 0;
          for (let k = 0; k < 3; k++) {
            hii += A[k][r] * WA[k][c];
            hij += A[k][r] * WB[k][c];
            hjj += B[k][r] * WB[k][c];
          }
          H[(i + r) * N + i + c] += hii;
          H[(i + r) * N + j + c] += hij;
          H[(j + c) * N + i + r] += hij;
          H[(j + r) * N + j + c] += hjj;
        }
        let bi = 0;
        let bj = 0;
        for (let k = 0; k < 3; k++) {
          bi += A[k][r] * We[k];
          bj += B[k][r] * We[k];
        }
        b[i + r] += bi;
        b[j + r] += bj;
      }
    }
    // nœud fixé : contrainte très forte (le repère ne bouge pas)
    for (let r = 0; r < 3; r++) H[(3 * fixed + r) * N + 3 * fixed + r] += 1e12;
    const dx = solveSPD(H, b.map((v) => -v), N);
    let step = 0;
    for (let k = 0; k < N; k++) {
      x[k] += dx[k];
      step = Math.max(step, Math.abs(dx[k]));
    }
    for (let k = 2; k < N; k += 3) x[k] = wrap(x[k]);
    if (step < tolerance) {
      it++;
      break;
    }
  }
  const out = [];
  for (let k = 0; k < n; k++) out.push({ x: x[3 * k], y: x[3 * k + 1], theta: x[3 * k + 2] / DEG });
  return { poses: out, error0, error: totalError(), iterations: it };
}

/** Ω : matrice d'information du lien (info), sinon diagonale 1 / σ² (σ en m et en radians). */
function weights(ed) {
  if (ed.info) return ed.info;
  const s = ed.sigma ?? { xy: 0.05, theta: 1 };
  return [
    [1 / (s.xy * s.xy), 0, 0],
    [0, 1 / (s.xy * s.xy), 0],
    [0, 0, 1 / (s.theta * DEG) ** 2],
  ];
}

/** Erreur du lien : déplacement prédit par les poses (dans le repère de i) moins le déplacement mesuré, exprimé dans le repère de z. */
function edgeError(x, ed) {
  const [xi, yi, ti] = [x[3 * ed.from], x[3 * ed.from + 1], x[3 * ed.from + 2]];
  const [xj, yj, tj] = [x[3 * ed.to], x[3 * ed.to + 1], x[3 * ed.to + 2]];
  const ci = Math.cos(ti);
  const si = Math.sin(ti);
  // déplacement prédit de i vers j, dans le repère de i
  const px = ci * (xj - xi) + si * (yj - yi);
  const py = -si * (xj - xi) + ci * (yj - yi);
  const tz = ed.z.theta * DEG;
  const cz = Math.cos(tz);
  const sz = Math.sin(tz);
  const ex = px - ed.z.x;
  const ey = py - ed.z.y;
  return [cz * ex + sz * ey, -sz * ex + cz * ey, wrap(tj - ti - tz)];
}

/** Erreur et jacobiennes A = ∂e/∂xᵢ, B = ∂e/∂xⱼ (forme classique du Graph-SLAM 2D). */
function linearize(x, ed) {
  const [xi, yi, ti] = [x[3 * ed.from], x[3 * ed.from + 1], x[3 * ed.from + 2]];
  const [xj, yj] = [x[3 * ed.to], x[3 * ed.to + 1]];
  const ci = Math.cos(ti);
  const si = Math.sin(ti);
  const tz = ed.z.theta * DEG;
  const cz = Math.cos(tz);
  const sz = Math.sin(tz);
  const dx = xj - xi;
  const dy = yj - yi;
  // Rzᵀ Riᵀ et Rzᵀ dRiᵀ/dθ
  const RziT = [
    [cz * ci - sz * si, cz * si + sz * ci],
    [-sz * ci - cz * si, -sz * si + cz * ci],
  ];
  const dRi = [
    [-si * dx + ci * dy, -ci * dx - si * dy],
  ];
  const dRx = cz * dRi[0][0] + sz * dRi[0][1];
  const dRy = -sz * dRi[0][0] + cz * dRi[0][1];
  const A = [
    [-RziT[0][0], -RziT[0][1], dRx],
    [-RziT[1][0], -RziT[1][1], dRy],
    [0, 0, -1],
  ];
  const B = [
    [RziT[0][0], RziT[0][1], 0],
    [RziT[1][0], RziT[1][1], 0],
    [0, 0, 1],
  ];
  return { e: edgeError(x, ed), A, B };
}

/** Résout H·x = b pour H symétrique définie positive (Cholesky dense). */
function solveSPD(H, b, N) {
  const L = new Float64Array(N * N);
  for (let i = 0; i < N; i++) {
    for (let j = 0; j <= i; j++) {
      let s = H[i * N + j];
      for (let k = 0; k < j; k++) s -= L[i * N + k] * L[j * N + k];
      if (i === j) L[i * N + i] = Math.sqrt(Math.max(s, 1e-12));
      else L[i * N + j] = s / L[j * N + j];
    }
  }
  const y = new Float64Array(N);
  for (let i = 0; i < N; i++) {
    let s = b[i];
    for (let k = 0; k < i; k++) s -= L[i * N + k] * y[k];
    y[i] = s / L[i * N + i];
  }
  const out = new Float64Array(N);
  for (let i = N - 1; i >= 0; i--) {
    let s = y[i];
    for (let k = i + 1; k < N; k++) s -= L[k * N + i] * out[k];
    out[i] = s / L[i * N + i];
  }
  return out;
}
