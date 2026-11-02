// Poses 2D et changements de repère.
//
// Convention (identique à MapTool et RobotControl) :
//   pose = { x, y, theta } avec x, y en mètres et theta en degrés,
//   axe X vers la droite, axe Y vers le haut, angles positifs antihoraires, 0° vers +X.

export const DEG = Math.PI / 180;

/** Ramène un angle en degrés dans ]-180, 180]. */
export function normalizeAngleDeg(a) {
  let r = a % 360;
  if (r <= -180) r += 360;
  if (r > 180) r -= 360;
  return r;
}

/** Ramène un angle en radians dans ]-PI, PI]. */
export function normalizeAngleRad(a) {
  let r = a % (2 * Math.PI);
  if (r <= -Math.PI) r += 2 * Math.PI;
  if (r > Math.PI) r -= 2 * Math.PI;
  return r;
}

export function makePose(x = 0, y = 0, theta = 0) {
  return { x, y, theta };
}

/**
 * Repère local du robot -> repère monde : P_monde = R(theta) * P_local + T(x, y).
 */
export function localToWorld(pose, lx, ly) {
  const c = Math.cos(pose.theta * DEG);
  const s = Math.sin(pose.theta * DEG);
  return { x: pose.x + c * lx - s * ly, y: pose.y + s * lx + c * ly };
}

/** Repère monde -> repère local du robot (transformation inverse). */
export function worldToLocal(pose, wx, wy) {
  const c = Math.cos(pose.theta * DEG);
  const s = Math.sin(pose.theta * DEG);
  const dx = wx - pose.x;
  const dy = wy - pose.y;
  return { x: c * dx + s * dy, y: -s * dx + c * dy };
}

/** Applique une pose à une liste de points locaux {x, y}. */
export function transformPoints(points, pose) {
  const c = Math.cos(pose.theta * DEG);
  const s = Math.sin(pose.theta * DEG);
  return points.map((p) => ({ ...p, x: pose.x + c * p.x - s * p.y, y: pose.y + s * p.x + c * p.y }));
}

/** Composition a ⊕ b : b est exprimée dans le repère de a, le résultat dans le repère monde. */
export function composePoses(a, b) {
  const p = localToWorld(a, b.x, b.y);
  return { x: p.x, y: p.y, theta: normalizeAngleDeg(a.theta + b.theta) };
}

/** Pose inverse : composePoses(p, invertPose(p)) = identité. */
export function invertPose(p) {
  const c = Math.cos(p.theta * DEG);
  const s = Math.sin(p.theta * DEG);
  return { x: -c * p.x - s * p.y, y: s * p.x - c * p.y, theta: normalizeAngleDeg(-p.theta) };
}

/** Écart entre deux poses : distance (m) et différence d'angle absolue (degrés). */
export function poseError(estimate, truth) {
  return {
    dist: Math.hypot(estimate.x - truth.x, estimate.y - truth.y),
    angle: Math.abs(normalizeAngleDeg(estimate.theta - truth.theta)),
  };
}
