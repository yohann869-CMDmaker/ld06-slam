// SLAM (jour 3) : localisation et cartographie simultanées, à partir des outils des jours 1 et 2.
//
// Pour chaque nouveau scan :
//   1. prédiction : pose du scan précédent, composée avec le mouvement commandé s'il est connu
//      (le robot Mecanum n'a pas de codeurs : c'est la durée des commandes, très approximative) ;
//      sinon (robot poussé à la main) modèle à vitesse constante : le robot continue comme il
//      bougeait entre les deux scans précédents (il faut alors l'heure de chaque scan) ;
//   2. suivi sur la CARTE LOCALE (les `localKeyframes` derniers scans clés) : multi-résolution dans
//      une fenêtre autour de la prédiction, puis ICP. Une carte locale plutôt que toute la carte :
//      au retour dans une zone déjà vue, le suivi ne « saute » pas vers l'ancienne carte (ce saut
//      fausserait la chaîne des déplacements) ; c'est la fermeture de boucle qui s'en charge ;
//      si le scan ne colle pas (moins de `minInliers` de ses points sur la carte), recherche élargie
//      (±60°, `wideWindow`) autour de la prédiction et de la pose précédente, puis relocalisation
//      globale dans toute la carte (globalloc.js) ;
//   3. cartographie : le scan est ajouté à la carte s'il apporte du nouveau (« scan clé » : au
//      moins `keyDist` ou `keyAngle` depuis le dernier scan clé) et si sa pose est sûre ;
//   4. graphe de poses : chaque scan clé est relié au précédent par le déplacement suivi ;
//   5. fermeture de boucle : le nouveau scan clé est recalé sur les anciens scans clés proches
//      (hors carte locale). Si l'un d'eux colle (au moins `loopMinInliers` des points), un lien
//      « boucle » est ajouté et le graphe est optimisé (posegraph.js) : toute la trajectoire est
//      corrigée, la dérive répartie le long de la boucle, et la carte reconstruite.

import { scanToLocalXY } from './scan.js';
import { gridFromScan } from './gridmap.js';
import { VectorMap } from './vectormap.js';
import { composePoses, DEG, invertPose, normalizeAngleDeg } from './pose.js';
import { localize, scanReference } from './localization.js';
import { globalLocalize } from './globalloc.js';
import { dominantDirection } from './cleanup.js';
import { optimizePoseGraph } from './posegraph.js';

export const DEFAULT_SLAM = Object.freeze({
  window: { xyRange: 0.5, thRange: 20 }, // recherche locale autour de la prédiction
  minInliers: 0.5, // part des points du scan sur la carte pour accepter le suivi
  fastTracking: true, // suivi par Gauss-Newton d'abord, multi-résolution en secours
  fastInliers: 0.7, // part des points sur la carte pour se contenter du suivi rapide
  motionModel: true, // sans odométrie : prédiction à vitesse constante (si l'heure des scans est donnée)
  wideWindow: { xyRange: 0.6, thRange: 60 }, // recherche élargie avant de se déclarer perdu (null : aucune)
  relocalize: true, // relocalisation globale si le suivi échoue
  relocEvery: 1, // scans minimum entre deux tentatives de relocalisation (coûteuse : 0,5 à 1 s)
  priorXY: 1, // m : écart-type de l'a priori de position quand le mouvement est connu (odométrie)
  priorTheta: 20, // degrés : idem pour l'orientation
  keyDist: 0.2, // m : nouveau scan clé après ce déplacement
  keyAngle: 10, // degrés : ou cette rotation
  localKeyframes: 15, // taille de la carte locale du suivi (en scans clés)
  loopClosure: true,
  loopRadius: 3, // m : anciens scans clés candidats à moins de cette distance de la pose estimée
  loopWindow: { xyRange: 1, thRange: 20 }, // recherche autour de la pose estimée (dérive tolérée)
  loopMinInliers: 0.6, // part des points du nouveau scan sur l'ancien pour accepter la boucle
  loopEvery: 5, // scans clés minimum entre deux fermetures de boucle
  loopFullSearch: true, // recherche large si le recalage rapide échoue (coûteuse ; false en direct)
  anisotropic: true, // précision des liens par direction, d'après les normales des points (sinon σ fixes)
  sigmaPoint: 0.03, // m : bruit supposé d'un point (bruit du lidar + erreurs de recalage)
  sigmaTrack: { xy: 0.05, theta: 1 }, // précision d'un lien de suivi si anisotropic = false (m, degrés)
  sigmaLoop: { xy: 0.03, theta: 0.5 }, // précision d'un lien de boucle si anisotropic = false
});

/**
 * Information (inverse de la covariance) d'un recalage point-droite, d'après la géométrie du scan :
 * chaque point de normale n (droite locale ajustée sur ses voisins) contraint la pose dans la
 * direction n seulement : Jᵢ = [nx, ny, p × n] et Ω = Σ JᵢJᵢᵀ / σ². Dans un couloir, toutes les
 * normales sont en travers : Ω est grande en travers et presque nulle le long ; le graphe saura
 * que c'est le long du couloir que la dérive s'accumule. Les points voisins étant corrélés, on
 * divise par 4 ; un petit a priori (50 cm, 20°) garde Ω inversible.
 */
export function scanInformation(points, sigma = DEFAULT_SLAM.sigmaPoint) {
  const H = [
    [1 / 0.25, 0, 0],
    [0, 1 / 0.25, 0],
    [0, 0, 1 / (20 * DEG) ** 2],
  ];
  const k2 = 1 / (4 * sigma * sigma);
  for (let i = 2; i + 2 < points.length; i++) {
    const nb = points.slice(i - 2, i + 3).filter((q) => Math.hypot(q.x - points[i].x, q.y - points[i].y) < 0.15);
    if (nb.length < 4) continue;
    // direction principale des voisins (ACP 2 × 2) ; la normale lui est perpendiculaire
    const mx = nb.reduce((a, q) => a + q.x, 0) / nb.length;
    const my = nb.reduce((a, q) => a + q.y, 0) / nb.length;
    let sxx = 0;
    let syy = 0;
    let sxy = 0;
    for (const q of nb) {
      sxx += (q.x - mx) ** 2;
      syy += (q.y - my) ** 2;
      sxy += (q.x - mx) * (q.y - my);
    }
    const phi = 0.5 * Math.atan2(2 * sxy, sxx - syy);
    const nx = -Math.sin(phi);
    const ny = Math.cos(phi);
    const J = [nx, ny, points[i].x * ny - points[i].y * nx];
    for (let r = 0; r < 3; r++) for (let c = 0; c < 3; c++) H[r][c] += J[r] * J[c] * k2;
  }
  return H;
}

const toWorld = (pts, pose) => {
  const c = Math.cos(pose.theta * DEG);
  const s = Math.sin(pose.theta * DEG);
  return pts.map((p) => ({ x: pose.x + c * p.x - s * p.y, y: pose.y + s * p.x + c * p.y }));
};

/** Déplacement de a vers b exprimé dans le repère de a : a⁻¹ ∘ b. */
export function relative(a, b) {
  const d = composePoses(invertPose(a), b);
  return { x: d.x, y: d.y, theta: normalizeAngleDeg(d.theta) };
}

export class Slam {
  constructor(options = {}) {
    this.o = {
      ...DEFAULT_SLAM,
      ...options,
      window: { ...DEFAULT_SLAM.window, ...options.window },
      loopWindow: { ...DEFAULT_SLAM.loopWindow, ...options.loopWindow },
    };
    this.keyframes = []; // [{ index, scan, points (locaux), pose }]
    this.trajectory = []; // une entrée par scan reçu : { index, pose, mode, inliers, keyframe, anchor, rel }
    this.edges = []; // graphe de poses : { from, to, z, sigma, kind: 'suite' | 'boucle' }
    this.loops = []; // fermetures de boucle : { from, to, inliers, correction (m), error0, error }
    this.grid = null; // grid map de tous les scans clés
    this.vector = new VectorMap([], { absence: false });
    this.last = null; // dernière pose estimée
    this.lastLoop = -Infinity;
    this.lastReloc = -Infinity;
    this.velocity = null; // vitesse estimée (repère du robot, m/s et degrés/s) pour la prédiction
    this.lastTime = null;
    this.localRef = null;
    this.globalRef = null;
  }

  /** Carte locale du suivi : points des derniers scans clés. */
  localReference() {
    if (!this.localRef) this.localRef = scanReference(this.keyframes.slice(-this.o.localKeyframes).flatMap((kf) => toWorld(kf.points, kf.pose)));
    return this.localRef;
  }

  /** Carte complète (relocalisation) : points de tous les scans clés, avec la grid map pour les positions candidates. */
  globalReference() {
    if (!this.globalRef) {
      this.globalRef = scanReference(this.keyframes.flatMap((kf) => toWorld(kf.points, kf.pose)));
      this.globalRef.grid = this.grid;
    }
    return this.globalRef;
  }

  addKeyframe(scan, points, pose, index) {
    const k = this.keyframes.length;
    if (k) {
      const link = this.o.anisotropic ? { info: scanInformation(points, this.o.sigmaPoint) } : { sigma: this.o.sigmaTrack };
      this.edges.push({ from: k - 1, to: k, z: relative(this.keyframes[k - 1].pose, pose), ...link, kind: 'suite' });
    }
    this.keyframes.push({ index, scan, points, pose: { ...pose } });
    if (!this.grid) this.grid = gridFromScan(scan, pose);
    else this.grid.integrateScan(scan, pose);
    this.vector.integrateScan(scan, pose);
    this.localRef = null;
    this.globalRef = null;
    return k;
  }

  /**
   * Ajoute un scan. odom : déplacement commandé depuis le scan précédent, dans le repère du robot
   * ({ x, y, theta }, facultatif). first : pose du premier scan (défaut : celle du fichier, ou 0).
   * time : heure du scan en ms (facultatif) ; sans odométrie, sert au modèle à vitesse constante.
   * Retourne { pose, mode ('départ' | 'suivi' | 'relocalisé' | 'perdu'), inliers, keyframe, loop, time }.
   */
  addScan(scan, odom = null, first = null, time = null) {
    const t0 = performance.now();
    const index = this.trajectory.length;
    const points = scanToLocalXY(scan);
    if (!this.keyframes.length) {
      const pose = { ...(first ?? scan.pose ?? { x: 0, y: 0, theta: 0 }) };
      const k = this.addKeyframe(scan, points, pose, index);
      this.last = pose;
      this.lastTime = time;
      const r = { index, pose, mode: 'départ', inliers: 1, keyframe: true, anchor: k, rel: { x: 0, y: 0, theta: 0 }, loop: null, time: performance.now() - t0 };
      this.trajectory.push(r);
      return r;
    }
    // 1. prédiction ; 2. suivi sur la carte locale
    const dt = time != null && this.lastTime != null ? Math.min(1, Math.max(0, (time - this.lastTime) / 1000)) : null;
    let predicted = { ...this.last };
    if (odom) predicted = composePoses(this.last, odom);
    else if (this.o.motionModel && this.velocity && dt) {
      const v = this.velocity;
      predicted = composePoses(this.last, { x: v.x * dt, y: v.y * dt, theta: v.theta * dt });
    }
    const ref = this.localReference();
    // suivi rapide d'abord (Gauss-Newton sur la carte locale, ~1 ms, à la manière de Hector SLAM) ;
    // la recherche multi-résolution (~100 ms) seulement s'il ne colle pas assez
    // départs : la prédiction et, si elle en diffère (vitesse constante), la pose précédente ; on
    // garde le recalage qui colle le mieux (une prédiction fausse au début d'un virage ne coûte rien)
    const starts = [predicted];
    if (!odom && (Math.hypot(predicted.x - this.last.x, predicted.y - this.last.y) > 0.02 || Math.abs(normalizeAngleDeg(predicted.theta - this.last.theta)) > 2)) starts.push({ ...this.last });
    let m = null;
    if (this.o.fastTracking) {
      for (const st of starts) {
        const g = localize(ref, points, st, 'gaussnewton');
        if (!m || g.inliers > m.inliers) m = g;
      }
    }
    if (!m || m.inliers < this.o.fastInliers) {
      for (const st of starts) {
        const mr = localize(ref, points, st, 'multires', this.o.window);
        if (!m || mr.inliers >= m.inliers) m = mr;
        if (m.inliers >= this.o.fastInliers) break;
      }
    }
    const i = localize(ref, points, m.pose, 'icp');
    let best = i.inliers >= m.inliers ? i : m;
    // recherche élargie (rotation rapide, prédiction fausse) avant de se déclarer perdu : autour de la
    // prédiction et, si elle en diffère, autour de la pose précédente
    if (best.inliers < this.o.minInliers && this.o.wideWindow) {
      const starts = [predicted];
      if (Math.hypot(predicted.x - this.last.x, predicted.y - this.last.y) > 0.05 || Math.abs(normalizeAngleDeg(predicted.theta - this.last.theta)) > 5) starts.push(this.last);
      for (const start of starts) {
        const w = localize(ref, points, start, 'multires', this.o.wideWindow);
        const wi = localize(ref, points, w.pose, 'icp');
        const cand = wi.inliers >= w.inliers ? wi : w;
        if (cand.inliers > best.inliers) best = cand;
        if (best.inliers >= this.o.minInliers) break;
      }
    }
    let mode = 'suivi';
    if (best.inliers < this.o.minInliers && this.o.relocalize && index - this.lastReloc >= this.o.relocEvery) {
      this.lastReloc = index;
      // avec un mouvement connu, la relocalisation cherche autour de la pose prédite (a priori)
      const g = this.relocalize(points, odom ? { pose: predicted, sigmaXY: this.o.priorXY, sigmaTheta: this.o.priorTheta } : null);
      // relocalisation acceptée seulement si elle est fiable ET colle à la carte (sinon : symétrique
      // de la salle, ou zone pas encore cartographiée)
      if (g && !g.ambiguous && g.inliers >= this.o.minInliers && g.inliers > best.inliers && freeSpaceConflict(this.grid, points, g.pose) <= 0.05) {
        best = g;
        mode = 'relocalisé';
      }
    }
    if (best.inliers < this.o.minInliers && mode === 'suivi') mode = 'perdu';
    // perdu : avec un mouvement connu, la pose prédite (odométrie) vaut mieux qu'un recalage raté,
    // et la suite repart de là au lieu d'une pose fausse
    let pose = mode === 'perdu' && odom ? predicted : best.pose;
    // 3. cartographie : scan clé s'il a assez bougé et si sa pose est sûre
    const d = relative(this.keyframes[this.keyframes.length - 1].pose, pose);
    const keyframe = mode !== 'perdu' && (Math.hypot(d.x, d.y) >= this.o.keyDist || Math.abs(d.theta) >= this.o.keyAngle);
    let loop = null;
    let anchor = this.keyframes.length - 1;
    if (keyframe) {
      anchor = this.addKeyframe(scan, points, pose, index);
      // 5. fermeture de boucle
      if (this.o.loopClosure && anchor - this.lastLoop >= this.o.loopEvery) loop = this.closeLoop(anchor);
      pose = this.keyframes[anchor].pose;
    }
    // vitesse pour la prédiction suivante (mouvement mesuré par le lidar) ; oubliée si perdu
    if (dt && mode !== 'perdu') {
      const d = relative(this.last, pose);
      const clamp = (v, m) => Math.max(-m, Math.min(m, v));
      this.velocity = { x: clamp(d.x / dt, 1), y: clamp(d.y / dt, 1), theta: clamp(d.theta / dt, 180) };
    } else if (mode === 'perdu') this.velocity = null;
    this.lastTime = time;
    this.last = pose;
    const r = { index, pose, mode, inliers: best.inliers, keyframe, anchor, rel: relative(this.keyframes[anchor].pose, pose), loop, time: performance.now() - t0 };
    this.trajectory.push(r);
    return r;
  }

  relocalize(points, prior = null) {
    const walls = this.vector.segments;
    return globalLocalize(this.globalReference(), points, { mapDominant: walls.length ? dominantDirection(walls) : undefined, prior: prior ?? undefined });
  }

  /** Cherche une boucle pour le scan clé k ; si trouvée : lien, optimisation du graphe, carte reconstruite. */
  closeLoop(k) {
    const kf = this.keyframes[k];
    const candidates = this.keyframes
      .map((old, j) => ({ j, d: Math.hypot(old.pose.x - kf.pose.x, old.pose.y - kf.pose.y) }))
      .filter((c) => c.j < k - this.o.localKeyframes && c.d <= this.o.loopRadius)
      .sort((a, b) => a.d - b.d)
      .slice(0, 2);
    for (const { j } of candidates) {
      const old = this.keyframes[j];
      const ref = scanReference(toWorld(old.points, old.pose));
      // recalage rapide d'abord (Gauss-Newton puis ICP, quelques ms) ; la recherche large
      // (multi-résolution ±1 m, ±20°, plusieurs centaines de ms) seulement s'il échoue et si permise
      const fast = localize(ref, kf.points, kf.pose, 'gaussnewton');
      let best = localize(ref, kf.points, fast.pose, 'icp');
      if (fast.inliers > best.inliers) best = fast;
      if (best.inliers < this.o.loopMinInliers && this.o.loopFullSearch) {
        const m = localize(ref, kf.points, kf.pose, 'multires', this.o.loopWindow);
        const r = localize(ref, kf.points, m.pose, 'icp');
        best = r.inliers >= m.inliers ? r : m;
      }
      if (best.inliers < this.o.loopMinInliers) continue;
      const link = this.o.anisotropic ? { info: scanInformation(kf.points, this.o.sigmaPoint) } : { sigma: this.o.sigmaLoop };
      this.edges.push({ from: j, to: k, z: relative(old.pose, best.pose), ...link, kind: 'boucle' });
      const before = this.keyframes.map((x) => ({ ...x.pose }));
      const g = optimizePoseGraph(before, this.edges);
      g.poses.forEach((p, n) => (this.keyframes[n].pose = p));
      // les scans non clés suivent le scan clé auquel ils sont rattachés
      for (const t of this.trajectory) t.pose = composePoses(this.keyframes[t.anchor].pose, t.rel);
      this.rebuild();
      this.lastLoop = k;
      const loop = { from: j, to: k, inliers: best.inliers, correction: Math.hypot(before[k].x - this.keyframes[k].pose.x, before[k].y - this.keyframes[k].pose.y), error0: g.error0, error: g.error };
      this.loops.push(loop);
      return loop;
    }
    return null;
  }

  /** Recalcule les cartes après une correction des poses des scans clés. */
  rebuild() {
    this.grid = null;
    this.vector = new VectorMap([], { absence: false });
    for (const kf of this.keyframes) {
      if (!this.grid) this.grid = gridFromScan(kf.scan, kf.pose);
      else this.grid.integrateScan(kf.scan, kf.pose);
      this.vector.integrateScan(kf.scan, kf.pose);
    }
    this.localRef = null;
    this.globalRef = null;
  }
}

/**
 * Cohérence avec l'espace libre : part des points du scan, placé à `pose`, qui tombent dans des
 * cellules que la carte a vues libres (classées libres). Un point lidar marque un obstacle :
 * il ne peut pas être là où la carte a vu du sol vide. À une pose fausse (symétrique de la salle,
 * zone pas encore cartographiée), beaucoup de points y tombent ; à la bonne pose presque aucun.
 * Valable pour des scans pris à la même hauteur (un trajet du robot au sol) : un scan pris en
 * hauteur voit « vide » au-dessus des meubles que les scans au sol rencontrent.
 */
export function freeSpaceConflict(grid, points, pose) {
  const c = Math.cos(pose.theta * DEG);
  const s = Math.sin(pose.theta * DEG);
  let n = 0;
  for (const p of points) {
    const [i, j] = grid.worldToCell(pose.x + c * p.x - s * p.y, pose.y + s * p.x + c * p.y);
    if (grid.inside(i, j) && grid.classify(grid.index(i, j)) === -1) n++;
  }
  return n / Math.max(points.length, 1);
}
