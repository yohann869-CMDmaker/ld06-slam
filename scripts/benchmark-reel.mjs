// Banc d'essai du jour 2 sur les données RÉELLES (même protocole que scripts/benchmark-jour2.mjs).
//
// A. Scans de la carte (13 scans pris au sol), « leave-one-out » : chaque scan est localisé sur des
//    cartes construites SANS lui (grid map de localisation, carte vectorielle, scan de référence le
//    plus proche). Pose de référence = pose recalée (data/reference/poses_recalees.csv). Mêmes
//    4 niveaux de perturbation de la pose initiale et même critère de réussite que la simulation.
//    Incertitude des poses de référence : déplacement d'un scan quand on le relocalise, depuis sa
//    pose recalée, sur une carte construite sans lui (désaccord entre carte et recalage commun).
// B. Jeu de test indépendant (data/test, 16 scans LidarScope du 2 novembre, jamais utilisés pour la
//    carte) : localisation globale sur la carte de référence, comparaison aux poses posées à la main
//    dans LidarScope (après recalage du repère LidarScope), puis convergence des méthodes locales
//    depuis des poses perturbées autour de la pose trouvée.
// Usage : node scripts/benchmark-reel.mjs [tirages par niveau, défaut 2]   -> docs/jour2_reel.md et .csv
import { existsSync } from 'node:fs';
import { readdir, readFile, writeFile } from 'node:fs/promises';
import { fileURLToPath } from 'node:url';
import { looksElevated, parseScanCSV, scanToLocalXY, scanToWorldXY } from '../src/core/scan.js';
import { gridFromScan } from '../src/core/gridmap.js';
import { VectorMap } from '../src/core/vectormap.js';
import { cleanMaps, cleanVectorMap, dominantDirection, parseLabels } from '../src/core/cleanup.js';
import { gridReference, localize, METHODS, METHODS_BY_REFERENCE, scanReference, vectorReference } from '../src/core/localization.js';
import { globalLocalize } from '../src/core/globalloc.js';
import { parseReferenceMap } from '../src/core/refmap.js';
import { composePoses, DEG, invertPose, normalizeAngleDeg, poseError } from '../src/core/pose.js';
import { mulberry32 } from '../src/core/sim.js';

const ROOT = fileURLToPath(new URL('../', import.meta.url));
const SUCCESS = { dist: 0.05, angle: 2 };
const LEVELS = [
  { dist: 0.05, angle: 2 },
  { dist: 0.15, angle: 5 },
  { dist: 0.3, angle: 10 },
  { dist: 0.45, angle: 18 },
];
const TRIALS = Number(process.argv[2] ?? 2);
const KIND = { grid: 'Grid map de localisation', vector: 'Carte vectorielle', scan: 'Scan de référence' };
const rng = mulberry32(2026);
const perturb = (p, lvl) => {
  const dir = rng() * 2 * Math.PI;
  const sign = rng() < 0.5 ? -1 : 1;
  return { x: p.x + lvl.dist * Math.cos(dir), y: p.y + lvl.dist * Math.sin(dir), theta: p.theta + sign * lvl.angle };
};
const median = (a) => {
  const s = [...a].sort((x, y) => x - y);
  return s.length ? s[Math.floor(s.length / 2)] : NaN;
};
const mean = (a) => a.reduce((x, y) => x + y, 0) / Math.max(a.length, 1);
const pct = (a) => (a.length ? `${Math.round((100 * a.filter((r) => r.ok).length) / a.length)} %` : '–');

// ------------------------------------------------------------------ données
const truthRows = (await readFile(ROOT + 'data/reference/poses_recalees.csv', 'utf8')).trim().split(/\r?\n/).slice(1).map((l) => l.split(','));
const scans = [];
const poses = [];
for (const [name, x, y, t] of truthRows) {
  scans.push(parseScanCSV(await readFile(ROOT + 'data/real/' + name, 'utf8'), name));
  poses.push({ x: Number(x), y: Number(y), theta: Number(t) });
}
const labelsFile = ROOT + 'data/reference/etiquettes.json';
const labels = existsSync(labelsFile) ? parseLabels(await readFile(labelsFile, 'utf8')) : [];
const all = scans.map((_, k) => k);
const floor = all.filter((k) => !scans[k].elevated);

/** Les trois références construites avec les scans `use` (au sol) ; `wallsFrom` : murs vus par ces scans-là. */
function references(use, wallsFrom, near) {
  const grid = gridFromScan(scans[use[0]], poses[use[0]]);
  use.slice(1).forEach((k) => grid.integrateScan(scans[k], poses[k]));
  const vm = new VectorMap([], { absence: false });
  use.forEach((k) => vm.integrateScan(scans[k], poses[k]));
  const vmAll = new VectorMap([], { absence: false });
  wallsFrom.forEach((k) => vmAll.integrateScan(scans[k], poses[k]));
  const views = use.map((k) => ({ pose: poses[k], points: scanToWorldXY(scans[k], poses[k]) }));
  const c = cleanMaps(grid, vm.segments, use.map((k) => poses[k]), { views, barriers: cleanVectorMap(vmAll.segments).segments, labels });
  const nearest = use.reduce((a, b) => (Math.hypot(poses[a].x - near.x, poses[a].y - near.y) <= Math.hypot(poses[b].x - near.x, poses[b].y - near.y) ? a : b));
  return {
    refs: { grid: gridReference(c.gridLoc), vector: vectorReference(c.segments), scan: scanReference(scanToWorldXY(scans[nearest], poses[nearest])) },
    mapDominant: dominantDirection(c.segments.filter((s) => s.kind !== 'objet')),
    nearest: scans[nearest].name,
  };
}

function runMethods(refs, pts, truth, tag, trials) {
  const out = [];
  for (const [kind, ref] of Object.entries(refs)) {
    for (const method of METHODS_BY_REFERENCE[kind]) {
      LEVELS.forEach((lvl, li) => {
        for (let t = 0; t < trials; t++) {
          const r = localize(ref, pts, perturb(truth, lvl), method);
          const e = poseError(r.pose, truth);
          out.push({ ...tag, kind, method, level: li, dist: e.dist, angle: e.angle, time: r.time, ok: e.dist < SUCCESS.dist && e.angle < SUCCESS.angle });
        }
      });
    }
  }
  return out;
}

// ------------------------------------------------------------------ A. scans de la carte, leave-one-out
const t0 = performance.now();
const rowsA = [];
const uncertainty = [];
for (const k of floor) {
  const { refs } = references(floor.filter((j) => j !== k), all.filter((j) => j !== k), poses[k]);
  const pts = scanToLocalXY(scans[k]);
  rowsA.push(...runMethods(refs, pts, poses[k], { scan: scans[k].name }, TRIALS));
  // incertitude : relocalisation depuis la pose recalée, sur la carte construite sans le scan
  const m = localize(refs.grid, pts, poses[k], 'multires');
  const r = localize(refs.grid, pts, m.pose, 'icp');
  const best = r.inliers >= m.inliers ? r : m;
  const manual = parseScanCSV(await readFile(ROOT + 'data/real/' + scans[k].name, 'utf8'), scans[k].name).pose;
  uncertainty.push({ scan: scans[k].name, ...poseError(best.pose, poses[k]), manual: poseError(manual, poses[k]) });
  process.stdout.write(`A ${scans[k].name} terminé (${((performance.now() - t0) / 1000).toFixed(0)} s)\n`);
}

// ------------------------------------------------------------------ B. jeu de test indépendant
const map = parseReferenceMap(await readFile(ROOT + 'data/reference/carte_reference.json', 'utf8'));
const fullRefs = { grid: gridReference(map.grid), vector: vectorReference(map.segments) };
const mapDominant = dominantDirection(map.segments.filter((s) => s.kind !== 'objet'));
const testFiles = existsSync(ROOT + 'data/test') ? (await readdir(ROOT + 'data/test')).filter((f) => f.startsWith('scan_') && f.endsWith('.csv')).sort() : [];
const tests = [];
for (const f of testFiles) {
  const s = parseScanCSV(await readFile(ROOT + 'data/test/' + f, 'utf8'), f);
  const pts = scanToLocalXY(s);
  const g = globalLocalize(fullRefs.grid, pts, { mapDominant });
  tests.push({ name: f, scan: s, pts, ls: s.pose, global: g });
}
// repère LidarScope du 2 novembre -> notre repère, estimé sur les réponses fiables (médianes, robuste)
const sure = tests.filter((t) => !t.global.ambiguous);
const th = median(sure.map((t) => normalizeAngleDeg(composePoses(t.global.pose, invertPose(t.ls)).theta)));
const c = Math.cos(th * DEG);
const s = Math.sin(th * DEG);
const T = {
  x: median(sure.map((t) => t.global.pose.x - (c * t.ls.x - s * t.ls.y))),
  y: median(sure.map((t) => t.global.pose.y - (s * t.ls.x + c * t.ls.y))),
  theta: th,
};
for (const t of tests) t.manual = poseError(t.global.pose, composePoses(T, t.ls));
// méthodes locales autour de la pose trouvée (réponses fiables seulement)
const rowsB = [];
for (const t of sure) {
  const near = floor.reduce((a, b) => (Math.hypot(poses[a].x - t.global.pose.x, poses[a].y - t.global.pose.y) <= Math.hypot(poses[b].x - t.global.pose.x, poses[b].y - t.global.pose.y) ? a : b));
  const refs = { ...fullRefs, scan: scanReference(scanToWorldXY(scans[near], poses[near])) };
  rowsB.push(...runMethods(refs, t.pts, t.global.pose, { scan: t.name }, 1));
  process.stdout.write(`B ${t.name} terminé (${((performance.now() - t0) / 1000).toFixed(0)} s)\n`);
}

// ------------------------------------------------------------------ rapport
const cm = (v) => `${(100 * v).toFixed(1)} cm`;
const md = [
  '# Jour 2 — Banc d\'essai sur données réelles',
  '',
  'Généré par `node scripts/benchmark-reel.mjs`. Même protocole que le banc d\'essai simulé',
  `([jour2_benchmark.md](jour2_benchmark.md)) : 4 niveaux de perturbation de la pose initiale, ${TRIALS} tirages par niveau,`,
  `fenêtre ±50 cm / ±20° ; **réussite** : moins de ${SUCCESS.dist * 100} cm **et** moins de ${SUCCESS.angle}°.`,
  '',
  `## A. Scans de la carte (${floor.length} scans pris au sol, chacun sur une carte construite sans lui)`,
  '',
  'Pose de référence : pose recalée par le recalage commun des 18 scans. Références : grid map de',
  'localisation et carte vectorielle propre (murs et meubles nommés) des autres scans au sol, scan',
  'de référence = scan au sol le plus proche.',
  '',
  '| Référence | Méthode | Réussite | Erreur position médiane | Erreur angle médiane | Temps moyen |',
  '| --- | --- | --- | --- | --- | --- |',
];
const methodTable = (rows) => {
  for (const kind of Object.keys(KIND)) {
    for (const method of METHODS_BY_REFERENCE[kind]) {
      const r = rows.filter((x) => x.kind === kind && x.method === method);
      if (!r.length) continue;
      md.push(`| ${KIND[kind]} | ${METHODS[method].label} | ${pct(r)} | ${cm(median(r.map((x) => x.dist)))} | ${median(r.map((x) => x.angle)).toFixed(2)}° | ${mean(r.map((x) => x.time)).toFixed(0)} ms |`);
    }
  }
};
const levelTable = (rows) => {
  md.push(`| Référence | Méthode | ${LEVELS.map((l) => `${l.dist * 100} cm / ${l.angle}°`).join(' | ')} |`);
  md.push(`| --- | --- | ${LEVELS.map(() => '---').join(' | ')} |`);
  for (const kind of Object.keys(KIND)) {
    for (const method of METHODS_BY_REFERENCE[kind]) {
      const cells = LEVELS.map((_, li) => pct(rows.filter((x) => x.kind === kind && x.method === method && x.level === li)));
      if (cells.every((x) => x === '–')) continue;
      md.push(`| ${KIND[kind]} | ${METHODS[method].label} | ${cells.join(' | ')} |`);
    }
  }
};
methodTable(rowsA);
md.push('', '### Sensibilité à l\'initialisation (taux de réussite)', '');
levelTable(rowsA);
md.push(
  '',
  '### Incertitude des poses de référence réelles',
  '',
  'Chaque scan est relocalisé (multi-résolution puis ICP), **depuis sa pose recalée**, sur la grid map',
  'construite sans lui : le déplacement mesure le désaccord entre cette carte indépendante et le recalage',
  'commun, donc l\'incertitude des poses de référence. La pose manuelle (posée à la main, sans mesure)',
  'est donnée pour comparaison.',
  '',
  '| Scan | Désaccord carte sans le scan | Pose manuelle / pose recalée |',
  '| --- | --- | --- |',
  ...uncertainty.map((u) => `| ${u.scan} | ${cm(u.dist)} / ${u.angle.toFixed(2)}° | ${cm(u.manual.dist)} / ${u.manual.angle.toFixed(1)}° |`),
  `| **médiane** | **${cm(median(uncertainty.map((u) => u.dist)))} / ${median(uncertainty.map((u) => u.angle)).toFixed(2)}°** | ${cm(median(uncertainty.map((u) => u.manual.dist)))} / ${median(uncertainty.map((u) => u.manual.angle)).toFixed(1)}° |`,
  `| maximum | ${cm(Math.max(...uncertainty.map((u) => u.dist)))} / ${Math.max(...uncertainty.map((u) => u.angle)).toFixed(2)}° | ${cm(Math.max(...uncertainty.map((u) => u.manual.dist)))} / ${Math.max(...uncertainty.map((u) => u.manual.angle)).toFixed(1)}° |`,
  '',
  `## B. Jeu de test indépendant (${tests.length} scans du 2 novembre, \`data/test/\`)`,
  '',
  'Scans jamais utilisés pour la carte. **Localisation globale** (sans pose de départ) sur la carte de',
  'référence, puis comparaison aux poses posées à la main dans LidarScope, ramenées dans notre repère',
  `(changement de repère estimé sur les réponses fiables : (${T.x.toFixed(3)} m ; ${T.y.toFixed(3)} m ; ${T.theta.toFixed(2)}°)).`,
  'Un placement à la main est approximatif (de l\'ordre de 20 à 30 cm) : cette comparaison vérifie que',
  'la bonne place est trouvée, pas la précision au centimètre.',
  '',
  '| Scan | Pris | Pose trouvée | Points sur la carte | Confiance (points 1re / 2e) | Écart à la pose LidarScope | Temps |',
  '| --- | --- | --- | --- | --- | --- | --- |',
  ...tests.map((t) => {
    const g = t.global;
    return `| ${t.name} | ${t.scan.elevated ? 'en hauteur' : looksElevated(t.scan) ? 'en hauteur ? (détecté)' : 'au sol'} | (${g.pose.x.toFixed(2)} ; ${g.pose.y.toFixed(2)} ; ${g.pose.theta.toFixed(1)}°) | ${Math.round(100 * g.inliers)} % | ${g.ambiguous ? '**ambiguë**' : 'fiable'} (${g.evidence ? `${g.evidence.best} / ${g.evidence.second}` : '–'}) | ${cm(t.manual.dist)} / ${t.manual.angle.toFixed(1)}° | ${Math.round(g.time)} ms |`;
  }),
  '',
  `Réponses fiables : ${sure.length}/${tests.length}, toutes à moins de ${cm(Math.max(...sure.map((t) => t.manual.dist)))} et ${Math.max(...sure.map((t) => t.manual.angle)).toFixed(1)}° de la pose LidarScope (médiane ${cm(median(sure.map((t) => t.manual.dist)))}). Réponses ambiguës : ${tests.length - sure.length}, dont ${tests.filter((t) => t.global.ambiguous && t.manual.dist > 1).length} à plus de 1 m de la pose LidarScope.`,
  '',
  '### Méthodes locales autour de la pose trouvée (réponses fiables)',
  '',
  'Pose de départ perturbée autour de la pose trouvée par la localisation globale ; réussite = retour à',
  'moins de 5 cm et 2° de cette pose (convergence et répétabilité, faute de pose vraie mesurée).',
  '',
  '| Référence | Méthode | Réussite | Écart médian | Écart angle médian | Temps moyen |',
  '| --- | --- | --- | --- | --- | --- |',
);
methodTable(rowsB);
md.push('', '#### Sensibilité à l\'initialisation (taux de réussite)', '');
levelTable(rowsB);
md.push('', `${rowsA.length + rowsB.length + tests.length} localisations en ${((performance.now() - t0) / 1000).toFixed(0)} s. Données brutes : \`jour2_reel.csv\`.`, '');
await writeFile(ROOT + 'docs/jour2_reel.md', md.join('\n'));
await writeFile(
  ROOT + 'docs/jour2_reel.csv',
  ['partie,scan,reference,methode,niveau,erreur_m,erreur_deg,temps_ms,reussite',
    ...rowsA.map((r) => ['A', r.scan, r.kind, r.method, r.level, r.dist.toFixed(5), r.angle.toFixed(4), r.time.toFixed(1), r.ok ? 1 : 0].join(',')),
    ...rowsB.map((r) => ['B', r.scan, r.kind, r.method, r.level, r.dist.toFixed(5), r.angle.toFixed(4), r.time.toFixed(1), r.ok ? 1 : 0].join(','))].join('\n') + '\n',
);
console.log('Écrit : docs/jour2_reel.md et docs/jour2_reel.csv');
