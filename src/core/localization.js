// Localisation d'un scan sur une référence (jour 2, document « Scan matching » du cours).
//
// Problème : on connaît une référence (grid map, carte vectorielle ou scan de référence) et
// un scan exprimé dans le repère du robot. On cherche la pose (x, y, θ) du robot telle que
// les points du scan, ramenés dans le repère monde, tombent sur la référence.
//
// Les étapes du cours se retrouvent dans chaque méthode :
//   1. transformation : points locaux -> repère monde pour une pose candidate ;
//   2. correspondance : distance de chaque point à la référence ;
//   3. score : qualité de l'alignement pour cette pose ;
//   4. optimisation : chercher la pose qui maximise le score (ou minimise l'erreur).
//
// Méthodes implémentées (choisies dans l'application, comparées par scripts/benchmark-jour2.mjs) :
//   - force brute, multi-résolution, descente de gradient, filtre particulaire : optimisent le
//     même score, calculé sur un « champ de distance » (voir DistanceField dans matching.js) ;
//   - ICP : correspondances point à point puis transformation en forme close ;
//   - calcul direct : moindres carrés point-droite sur une carte vectorielle, généralisation de
//     l'algorithme de localisation absolue de l'article « Mars Attacks! » (Coupe de Robotique 2024).
// Toutes partent d'une pose initiale et cherchent dans une fenêtre autour d'elle (WINDOW).

import { DEG, normalizeAngleDeg } from './pose.js';
import { DistanceField, exhaustive, icp, subsample, topCandidates } from './matching.js';
import { length } from './geometry.js';
import { gaussian, mulberry32 } from './sim.js';
import { gaussNewton, ndt, plicp } from './advancedmatch.js';

// ------------------------------------------------------------------ références

/**
 * Une référence = des points dans le repère monde + des champs de distance calculés à la
 * demande pour chaque résolution (puis gardés en mémoire : on en a besoin à chaque pose testée).
 */
function makeReference(kind, points, extra = {}) {
  const fields = new Map();
  return {
    kind,
    points,
    ...extra,
    field(res = 0.025) {
      if (!fields.has(res)) fields.set(res, new DistanceField(points, { resolution: res, margin: 1.5, maxDist: 1 }));
      return fields.get(res);
    },
  };
}

/** Grid map : chaque cellule occupée devient un point (son centre). */
export function gridReference(grid) {
  return makeReference('grid', grid.occupiedCells(), { grid });
}

/**
 * Carte vectorielle : on garde les segments (utilisés tels quels par le calcul direct) et on
 * les échantillonne tous les 2 cm pour les méthodes qui travaillent sur des points.
 */
export function vectorReference(segments, step = 0.02) {
  const points = [];
  for (const s of segments) {
    const n = Math.max(1, Math.ceil(length(s) / step));
    for (let k = 0; k <= n; k++) points.push({ x: s.x1 + ((s.x2 - s.x1) * k) / n, y: s.y1 + ((s.y2 - s.y1) * k) / n });
  }
  return makeReference('vector', points, { segments });
}

/** Scan de référence : ses points, déjà placés dans le repère monde. */
export function scanReference(worldPoints) {
  return makeReference('scan', worldPoints);
}

// ------------------------------------------------------------------ score

// Étape 1 du cours : P_monde = R(θ)·P_local + (x, y)
const toWorld = (p, pose, c = Math.cos(pose.theta * DEG), s = Math.sin(pose.theta * DEG)) => ({
  x: pose.x + c * p.x - s * p.y,
  y: pose.y + s * p.x + c * p.y,
});

/**
 * Score d'une pose (étapes 2 et 3 du cours) : pour chaque point du scan transformé, d = distance
 * au point de référence le plus proche, lue dans le champ de distance (interpolée, donc
 * continue). Chaque point rapporte exp(−d² / 2σ²) : 1 s'il tombe pile sur la référence,
 * environ 0 s'il en est loin. Le score est la moyenne, entre 0 et 1.
 * σ règle la tolérance : grand σ = score lisse, facile à optimiser mais peu précis ; petit σ =
 * précis mais avec des « trous » entre les maxima.
 * Une gaussienne plutôt qu'un comptage de points sur des cellules occupées : un point à 1 cm
 * d'un mur compte presque autant qu'un point dessus, ce qui rend le score robuste au bruit.
 */
export function scorePose(field, pts, pose, sigma) {
  const c = Math.cos(pose.theta * DEG);
  const s = Math.sin(pose.theta * DEG);
  const k = 1 / (2 * sigma * sigma);
  let sum = 0;
  for (const p of pts) {
    const d = field.at(pose.x + c * p.x - s * p.y, pose.y + s * p.x + c * p.y);
    sum += Math.exp(-d * d * k);
  }
  return sum / pts.length;
}

/** Indicateur de réussite : proportion des points du scan à moins de `tol` (5 cm) de la référence. */
export function inlierRatio(ref, pts, pose, tol = 0.05) {
  const f = ref.field(0.025);
  const c = Math.cos(pose.theta * DEG);
  const s = Math.sin(pose.theta * DEG);
  let n = 0;
  for (const p of pts) if (f.at(pose.x + c * p.x - s * p.y, pose.y + s * p.x + c * p.y) <= tol) n++;
  return n / pts.length;
}

// ------------------------------------------------------------------ méthodes (étape 4 : optimisation)

/** Fenêtre de recherche autour de la pose initiale : ±50 cm en x et en y, ±20° en θ. */
export const WINDOW = Object.freeze({ xyRange: 0.5, thRange: 20 });

/**
 * FORCE BRUTE (cours §8.1)
 *   1. découper la fenêtre en une grille de poses : pas de 2 cm en x et y, 0,5° en θ
 *      (51 × 51 × 81 ≈ 210 000 poses) ;
 *   2. calculer le score de chaque pose (σ = 5 cm, champ de distance à 2,5 cm) ;
 *   3. garder la meilleure.
 * + Simple, ne peut pas « rester coincé » : trouve le maximum de la fenêtre.
 * − Lent (≈ 0,4 s), précision limitée au pas de la grille (environ ±1 cm, ±0,25°).
 * Le scan est réduit à 200 points pour accélérer (un mur reste bien représenté).
 */
function bruteForce(ref, pts, init, o) {
  const field = ref.field(0.025);
  const res = exhaustive(field, field.scoreTable(0.05), subsample(pts, 200), init, {
    xyRange: o.xyRange,
    xyStep: 0.02,
    thRange: o.thRange,
    thStep: 0.5,
  });
  return { pose: topCandidates(res, 1)[0].pose };
}

/**
 * MULTI-RÉSOLUTION (cours §8.2)
 *   1. niveau grossier : champ à 10 cm, σ = 15 cm (score lissé : les bons alignements se voient
 *      de loin), grille de 10 cm et 2°, scan réduit à 120 points ;
 *   2. garder les 3 meilleurs candidats distincts (et pas seulement le premier : le niveau
 *      grossier peut se tromper de peu entre deux maxima voisins) ;
 *   3. niveau fin autour de chacun : champ à 2,5 cm, σ = 4 cm, grille de 1 cm et 0,25° sur
 *      ±12 cm et ±2,5° ;
 *   4. garder le meilleur score fin.
 * + Robuste comme la force brute, 2 à 3 fois plus rapide, plus précise (pas final de 1 cm).
 */
function multiResolution(ref, pts, init, o) {
  const coarse = ref.field(0.1);
  const cands = topCandidates(
    exhaustive(coarse, coarse.scoreTable(0.15), subsample(pts, 120), init, { xyRange: o.xyRange, xyStep: 0.1, thRange: o.thRange, thStep: 2 }),
    3,
  );
  const fine = ref.field(0.025);
  const table = fine.scoreTable(0.04);
  let best = null;
  for (const c of cands) {
    const [r] = topCandidates(exhaustive(fine, table, pts, c.pose, { xyRange: 0.12, xyStep: 0.01, thRange: 2.5, thStep: 0.25 }), 1);
    if (!best || r.score > best.score) best = r;
  }
  return { pose: best.pose };
}

/**
 * DESCENTE DE GRADIENT (cours §8.3) — ici une montée, puisqu'on maximise le score.
 *   1. gradient numérique : pour x, y et θ, on évalue le score un peu avant et un peu après
 *      (différences centrées, h = 2 mm et 0,1°) ; la pente indique dans quel sens bouger ;
 *   2. on fait un pas dans le sens de la pente (séparément pour la position et l'angle) ;
 *   3. pas adaptatif : si le score augmente, on accepte et on allonge le pas (×1,2), sinon on
 *      refuse et on le raccourcit (×0,5) ; arrêt quand le pas devient minuscule ;
 *   4. on recommence avec σ = 15 cm, puis 8 cm, puis 4 cm : le premier score, très lisse, guide
 *      de loin vers le bon alignement ; les suivants affinent la précision.
 * + Très rapide (≈ 5 ms : quelques centaines d'évaluations au lieu de 210 000).
 * − Méthode locale : peut s'arrêter sur un maximum local si la pose initiale est trop loin.
 */
function gradient(ref, pts, init, o) {
  const field = ref.field(0.025);
  const sample = subsample(pts, 250);
  let pose = { ...init };
  let evaluations = 0;
  for (const sigma of [0.15, 0.08, 0.04]) {
    let step = { xy: 0.05, th: 2 };
    let score = scorePose(field, sample, pose, sigma);
    for (let it = 0; it < (o.iterations ?? 60); it++) {
      // 1. pente du score selon x, y et θ
      const h = { xy: 0.002, th: 0.1 };
      const g = {
        x: (scorePose(field, sample, { ...pose, x: pose.x + h.xy }, sigma) - scorePose(field, sample, { ...pose, x: pose.x - h.xy }, sigma)) / (2 * h.xy),
        y: (scorePose(field, sample, { ...pose, y: pose.y + h.xy }, sigma) - scorePose(field, sample, { ...pose, y: pose.y - h.xy }, sigma)) / (2 * h.xy),
        t: (scorePose(field, sample, { ...pose, theta: pose.theta + h.th }, sigma) - scorePose(field, sample, { ...pose, theta: pose.theta - h.th }, sigma)) / (2 * h.th),
      };
      evaluations += 6;
      // 2. pas dans le sens de la pente (direction normalisée : la longueur du pas est step)
      const gxy = Math.hypot(g.x, g.y) || 1;
      const gt = Math.abs(g.t) || 1;
      const next = { x: pose.x + (step.xy * g.x) / gxy, y: pose.y + (step.xy * g.y) / gxy, theta: pose.theta + (step.th * g.t) / gt };
      const s = scorePose(field, sample, next, sigma);
      evaluations++;
      // 3. pas adaptatif
      if (s > score) {
        pose = next;
        score = s;
        step = { xy: step.xy * 1.2, th: step.th * 1.2 };
      } else {
        step = { xy: step.xy * 0.5, th: step.th * 0.5 };
        if (step.xy < 2e-4 && step.th < 0.01) break;
      }
    }
  }
  return { pose: { ...pose, theta: normalizeAngleDeg(pose.theta) }, evaluations };
}

/**
 * FILTRE PARTICULAIRE (cours §8.4, localisation probabiliste / Monte Carlo)
 * La pose est représentée par un nuage d'hypothèses (« particules ») au lieu d'une seule valeur.
 *   1. tirer N = 400 particules autour de la pose initiale (écart-type : la moitié de la fenêtre) ;
 *   2. poids de chaque particule = vraisemblance du scan s'il avait été pris de là
 *      (score^8 : on accentue l'écart entre bonnes et mauvaises hypothèses) ;
 *   3. rééchantillonnage systématique : on retire N particules proportionnellement à leur
 *      poids (les bonnes sont dupliquées, les mauvaises disparaissent) ;
 *   4. on agite chaque particule d'un bruit qui diminue à chaque itération (×0,7, principe
 *      du recuit simulé) et σ du score diminue aussi ; on recommence 15 fois ;
 *   5. résultat : la meilleure particule rencontrée.
 * + Explore plusieurs hypothèses à la fois : robuste aux minima locaux et aux ambiguïtés.
 * − Précision limitée (≈ 3 cm) par le nombre de particules ; plusieurs réglages à choisir.
 *   En pratique, on l'utilise pour trouver la zone, puis on affine avec ICP ou le gradient.
 */
function particles(ref, pts, init, o) {
  const field = ref.field(0.05);
  const sample = subsample(pts, 100);
  const N = o.particles ?? 400;
  const rand = mulberry32(o.seed ?? 12345); // tirages reproductibles
  const gauss = () => gaussian(rand);
  let sx = o.xyRange / 2;
  let st = o.thRange / 2;
  // 1. nuage initial (la particule 0 est la pose initiale elle-même)
  let ps = Array.from({ length: N }, (_, k) =>
    k === 0 ? { ...init } : { x: init.x + gauss() * sx, y: init.y + gauss() * sx, theta: init.theta + gauss() * st },
  );
  let best = null;
  for (let it = 0; it < (o.iterations ?? 15); it++) {
    // 2. poids = vraisemblance
    const sigma = Math.max(0.04, 0.2 * 0.8 ** it);
    const w = ps.map((p) => scorePose(field, sample, p, sigma));
    w.forEach((v, k) => {
      if (!best || v > best.score) best = { score: v, pose: { ...ps[k] } };
    });
    const weights = w.map((v) => v ** 8);
    const total = weights.reduce((a, b) => a + b, 0) || 1;
    // 3. rééchantillonnage systématique : N pointeurs régulièrement espacés de 1/N sur la
    //    somme cumulée des poids (une seule valeur aléatoire : moins de variance qu'un tirage à N valeurs)
    const out = [];
    let u = rand() / N;
    let cum = weights[0] / total;
    let k = 0;
    for (let m = 0; m < N; m++) {
      while (u > cum && k < N - 1) cum += weights[++k] / total;
      out.push(ps[k]);
      u += 1 / N;
    }
    // 4. bruit décroissant (la meilleure particule est conservée telle quelle)
    sx *= 0.7;
    st *= 0.7;
    ps = out.map((p, m) => (m === 0 ? { ...best.pose } : { x: p.x + gauss() * sx, y: p.y + gauss() * sx, theta: p.theta + gauss() * st }));
  }
  return { pose: { ...best.pose, theta: normalizeAngleDeg(best.pose.theta) }, evaluations: N * (o.iterations ?? 15) };
}

/**
 * ICP — Iterative Closest Point (cours §6 et §8.3). Détail du calcul dans icp() (matching.js) :
 *   1. chaque point du scan est associé au point de référence le plus proche (s'il est à moins
 *      d'une distance qui diminue de 30 à 5 cm au fil des itérations) ;
 *   2. la rotation et la translation qui superposent au mieux ces couples se calculent
 *      directement (barycentres + angle donné par un arc tangente) ;
 *   3. on applique, on recommence jusqu'à ce que la pose ne bouge plus.
 * + Très précis (≈ 1 mm) et rapide. − Local : a besoin d'une pose initiale assez juste.
 */
function icpMethod(ref, pts, init) {
  const r = icp(ref.points, pts, init, { maxDist: 0.3, minDist: 0.05, iterations: 50 });
  return { pose: r.pose };
}

/**
 * CALCUL DIRECT sur carte vectorielle (cours §8.5 « calcul direct ? »).
 *
 * Idée (article « Mars Attacks! ») : si l'on sait à quel mur appartient chaque point, l'erreur
 * de pose se calcule directement par une régression, sans tester de poses. L'article le fait
 * sur une table rectangulaire connue : points proches de chaque bord (boîtes de marge), droite
 * de régression par paire de bords parallèles, (x, y, θ) déduits de l'ordonnée à l'origine et
 * de la pente. Ici, les murs sont des segments quelconques.
 *
 * Pour chaque point du scan, transformé avec la pose courante w = (wx, wy) :
 *   - on cherche le segment le plus proche, avec n sa normale unitaire, et la distance signée
 *     du point à la droite du segment : r = n · (w − a) (a : une extrémité du segment) ;
 *   - si |r| dépasse la marge, le point est ignoré (meuble, personne, point aberrant).
 * Une petite correction (dx, dy, dθ) de la pose déplace le point de
 *   (dx, dy) + dθ · (−(wy − y), wx − x)        (rotation autour du robot, au premier ordre)
 * et la distance devient   r' ≈ r + nx·dx + ny·dy + dθ·[ nx·(−(wy − y)) + ny·(wx − x) ] = r + J·d.
 * On cherche d qui rend toutes les distances r' les plus petites possibles (moindres carrés) :
 *   minimiser Σ (r + J·d)²   <=>   (Σ JᵀJ) d = −Σ Jᵀ r     (système linéaire 3 × 3)
 * résolu directement (règle de Cramer). On applique d, on refait les associations (4 fois),
 * puis on resserre la marge : 25 cm, puis 10 cm, puis 4 cm. C'est la proposition « passes
 * successives à marge décroissante » de l'article : une grande marge rattrape une grosse
 * erreur, une petite marge écarte les points qui n'appartiennent pas aux murs.
 * + Très rapide et très précis (≈ 1 mm), pas de grille de recherche.
 * − Local (associations fausses si la pose initiale est trop loin) ; système singulier si un
 *   seul mur est visible (la position le long du mur est alors indéterminée).
 */
function direct(ref, pts, init, o) {
  if (!ref.segments) throw new Error('Le calcul direct demande une carte vectorielle');
  // pour chaque segment : longueur L, direction unitaire u, normale unitaire n
  const segs = ref.segments.map((s) => {
    const L = length(s);
    const ux = (s.x2 - s.x1) / L;
    const uy = (s.y2 - s.y1) / L;
    return { ...s, L, ux, uy, nx: -uy, ny: ux };
  });
  let pose = { ...init };
  let used = 0;
  for (const margin of o.margins ?? [0.25, 0.1, 0.04]) {
    for (let it = 0; it < 4; it++) {
      const c = Math.cos(pose.theta * DEG);
      const s = Math.sin(pose.theta * DEG);
      // matrice A = Σ JᵀJ (3 × 3, rangée ligne par ligne) et second membre b = −Σ Jᵀ r
      const A = [0, 0, 0, 0, 0, 0, 0, 0, 0];
      const b = [0, 0, 0];
      used = 0;
      for (const p of pts) {
        const w = toWorld(p, pose, c, s);
        // association : segment le plus proche (projection dans le segment, à la marge près)
        let bestSeg = null;
        let bestD = margin;
        let bestR = 0;
        for (const g of segs) {
          const t = (w.x - g.x1) * g.ux + (w.y - g.y1) * g.uy; // position le long du segment
          if (t < -margin || t > g.L + margin) continue;
          const r = (w.x - g.x1) * g.nx + (w.y - g.y1) * g.ny; // distance signée à la droite
          if (Math.abs(r) < bestD) {
            bestD = Math.abs(r);
            bestSeg = g;
            bestR = r;
          }
        }
        if (!bestSeg) continue;
        // ligne J de la jacobienne : dérivée de r par rapport à (dx, dy, dθ)
        const J = [bestSeg.nx, bestSeg.ny, bestSeg.nx * -(w.y - pose.y) + bestSeg.ny * (w.x - pose.x)];
        for (let i = 0; i < 3; i++) {
          b[i] -= J[i] * bestR;
          for (let j = 0; j < 3; j++) A[3 * i + j] += J[i] * J[j];
        }
        used++;
      }
      if (used < 10) break; // trop peu de points sur les murs : on garde la pose
      const d = solve3(A, b);
      if (!d) break; // système singulier
      pose = { x: pose.x + d[0], y: pose.y + d[1], theta: normalizeAngleDeg(pose.theta + d[2] / DEG) };
      if (Math.hypot(d[0], d[1]) < 1e-5 && Math.abs(d[2]) < 1e-6) break; // convergé
    }
  }
  return { pose, used };
}

/** Résolution d'un système 3 × 3 par la règle de Cramer ; null s'il est singulier (un seul mur vu, par exemple). */
function solve3(A, b) {
  const det = (m) => m[0] * (m[4] * m[8] - m[5] * m[7]) - m[1] * (m[3] * m[8] - m[5] * m[6]) + m[2] * (m[3] * m[7] - m[4] * m[6]);
  const D = det(A);
  if (Math.abs(D) < 1e-12) return null;
  return [0, 1, 2].map((col) => {
    const M = [...A];
    for (let r = 0; r < 3; r++) M[3 * r + col] = b[r];
    return det(M) / D;
  });
}

export const METHODS = Object.freeze({
  brute: { label: 'Force brute', run: bruteForce },
  multires: { label: 'Multi-résolution', run: multiResolution },
  gradient: { label: 'Descente de gradient', run: gradient },
  particles: { label: 'Filtre particulaire (probabiliste)', run: particles },
  icp: { label: 'ICP', run: icpMethod },
  direct: { label: 'Calcul direct (moindres carrés point-droite)', run: direct },
  // ajoutées après l'état de l'art (advancedmatch.js, docs/etat_de_l_art.md)
  plicp: { label: 'PL-ICP (ICP point-droite, Censi 2008)', run: (ref, pts, init, o) => plicp(ref, pts, init, o) },
  ndt: { label: 'NDT (Biber 2003)', run: (ref, pts, init, o) => ndt(ref, pts, init, o) },
  gaussnewton: { label: 'Gauss-Newton sur la carte (Hector 2011)', run: (ref, pts, init, o) => gaussNewton(ref, pts, init, o) },
});

/** Méthodes proposées pour chaque type de référence (le calcul direct a besoin de segments). */
export const METHODS_BY_REFERENCE = Object.freeze({
  grid: ['brute', 'multires', 'gradient', 'particles', 'icp', 'plicp', 'ndt', 'gaussnewton'],
  vector: ['direct', 'brute', 'multires', 'gradient', 'particles', 'icp', 'plicp', 'ndt', 'gaussnewton'],
  scan: ['icp', 'brute', 'multires', 'gradient', 'particles', 'plicp', 'ndt', 'gaussnewton'],
});

/**
 * Localise un scan (points dans le repère du robot) sur une référence en partant de `init`.
 * Retourne { pose, time (ms), inliers (part des points à moins de 5 cm de la référence) }.
 */
export function localize(ref, localPts, init, method, options = {}) {
  const o = { ...WINDOW, ...options };
  const t0 = performance.now();
  const r = METHODS[method].run(ref, localPts, init, o);
  const time = performance.now() - t0;
  return { ...r, method, time, inliers: inlierRatio(ref, localPts, r.pose) };
}
