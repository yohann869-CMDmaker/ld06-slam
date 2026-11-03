// Banc d'essai de la localisation globale (jour 2, étape 3) : retrouver la pose d'un scan SANS
// pose de départ. Chaque scan est localisé sur une carte construite SANS lui (« leave-one-out ») :
//   - jeu simulé (data/sim) : pose vraie connue exactement ;
//   - scans réels de la salle (data/real) : pose de référence = pose recalée
//     (data/reference/poses_recalees.csv, incertitude de l'ordre de 2 à 5 cm) ; cartes faites avec
//     les scans au sol, comme la carte de référence.
// Deux variantes : orientations tirées des directions des murs (4 × 9 angles) ou tous les angles
// (180). Mesures : erreur de position et d'angle, réussite (< 5 cm et < 2°), temps, confiance.
// Usage : node scripts/benchmark-global.mjs   -> docs/jour2_global.md et .csv
import { readFile, writeFile } from 'node:fs/promises';
import { existsSync } from 'node:fs';
import { fileURLToPath } from 'node:url';
import { parseScanCSV, scanToLocalXY, scanToWorldXY } from '../src/core/scan.js';
import { gridFromScan } from '../src/core/gridmap.js';
import { VectorMap } from '../src/core/vectormap.js';
import { cleanMaps, cleanVectorMap, dominantDirection, parseLabels } from '../src/core/cleanup.js';
import { gridReference } from '../src/core/localization.js';
import { globalLocalize } from '../src/core/globalloc.js';
import { normalizeAngleDeg } from '../src/core/pose.js';

const ROOT = fileURLToPath(new URL('../', import.meta.url));
const SUCCESS = { dist: 0.05, angle: 2 };
const VARIANTS = [
  { name: 'murs', label: 'orientations des murs (4 × 9 angles)', useDominant: true },
  { name: 'tous', label: 'tous les angles (180)', useDominant: false },
];

/** Cartes construites avec les scans d'indices `use` ; retourne la référence grid map et la direction des murs. */
function buildMaps(scans, poses, use, all, labels) {
  const grid = gridFromScan(scans[use[0]], poses[use[0]]);
  use.slice(1).forEach((k) => grid.integrateScan(scans[k], poses[k]));
  const vm = new VectorMap([], { absence: false });
  use.forEach((k) => vm.integrateScan(scans[k], poses[k]));
  const vmAll = new VectorMap([], { absence: false });
  all.forEach((k) => vmAll.integrateScan(scans[k], poses[k]));
  const views = use.map((k) => ({ pose: poses[k], points: scanToWorldXY(scans[k], poses[k]) }));
  const c = cleanMaps(grid, vm.segments, use.map((k) => poses[k]), { views, barriers: cleanVectorMap(vmAll.segments).segments, labels });
  return { ref: gridReference(c.gridLoc), mapDominant: dominantDirection(c.segments.filter((s) => s.kind !== 'objet')) };
}

function run(dataset, scans, poses, isTest, mapFor) {
  const rows = [];
  scans.forEach((s, k) => {
    if (!isTest(k)) return;
    const { ref, mapDominant } = mapFor(k);
    const pts = scanToLocalXY(s);
    for (const v of VARIANTS) {
      const r = globalLocalize(ref, pts, { mapDominant, useDominant: v.useDominant });
      const dist = Math.hypot(r.pose.x - poses[k].x, r.pose.y - poses[k].y);
      const angle = Math.abs(normalizeAngleDeg(r.pose.theta - poses[k].theta));
      rows.push({
        dataset,
        scan: s.name,
        elevated: Boolean(s.elevated),
        variant: v.name,
        mode: r.mode,
        dist,
        angle,
        ok: dist < SUCCESS.dist && angle < SUCCESS.angle,
        inliers: r.inliers,
        ratio: r.ratio,
        evidence: r.evidence,
        ambiguous: r.ambiguous,
        poses: r.positions * r.angles,
        time: r.time,
      });
    }
  });
  return rows;
}

// ------------------------------------------------------------------ jeu simulé
const index = JSON.parse(await readFile(ROOT + 'data/sim/index.json', 'utf8'));
const simScans = [];
for (const s of index.scans) simScans.push(parseScanCSV(await readFile(ROOT + 'data/sim/' + s.file, 'utf8'), s.file));
const simPoses = index.scans.map((s) => s.truePose);
const simAll = simScans.map((_, k) => k);
const rows = run('simulé', simScans, simPoses, () => true, (k) => buildMaps(simScans, simPoses, simAll.filter((j) => j !== k), simAll.filter((j) => j !== k), []));

// ------------------------------------------------------------------ salle réelle
const truth = (await readFile(ROOT + 'data/reference/poses_recalees.csv', 'utf8')).trim().split(/\r?\n/).slice(1).map((l) => l.split(','));
const realScans = [];
const realPoses = [];
for (const [name, x, y, t] of truth) {
  realScans.push(parseScanCSV(await readFile(ROOT + 'data/real/' + name, 'utf8'), name));
  realPoses.push({ x: Number(x), y: Number(y), theta: Number(t) });
}
const labelsFile = ROOT + 'data/reference/etiquettes.json';
const labels = existsSync(labelsFile) ? parseLabels(await readFile(labelsFile, 'utf8')) : [];
const realAll = realScans.map((_, k) => k);
rows.push(
  ...run('salle', realScans, realPoses, () => true, (k) =>
    buildMaps(realScans, realPoses, realAll.filter((j) => j !== k && !realScans[j].elevated), realAll.filter((j) => j !== k), labels),
  ),
);

// ------------------------------------------------------------------ résultats
const pct = (a, b) => `${a}/${b} (${Math.round((100 * a) / Math.max(b, 1))} %)`;
const med = (a) => {
  const s = [...a].sort((u, v) => u - v);
  return s.length ? s[Math.floor(s.length / 2)] : NaN;
};
const lines = [
  '# Localisation globale : banc d\'essai',
  '',
  'Généré par `node scripts/benchmark-global.mjs`. Chaque scan est localisé **sans pose de départ**',
  'sur une carte construite **sans lui** (grid map de localisation des autres scans). Réussite :',
  `moins de ${SUCCESS.dist * 100} cm et moins de ${SUCCESS.angle}°. Scans réels : pose de référence = pose recalée`,
  '(incertitude de 2 à 5 cm) ; cartes faites avec les scans pris au sol.',
  '',
  '| Jeu | Scans | Variante | Réussite | Erreur médiane (réussites) | Poses testées | Temps moyen | Ambiguës signalées |',
  '| --- | --- | --- | --- | --- | --- | --- | --- |',
];
for (const dataset of ['simulé', 'salle']) {
  for (const group of dataset === 'salle' ? ['au sol', 'en hauteur'] : ['tous']) {
    for (const v of VARIANTS) {
      const r = rows.filter((x) => x.dataset === dataset && x.variant === v.name && (group === 'tous' || x.elevated === (group === 'en hauteur')));
      if (!r.length) continue;
      const okRows = r.filter((x) => x.ok);
      lines.push(
        `| ${dataset} | ${group} (${r.length}) | ${v.label} | ${pct(okRows.length, r.length)} | ${okRows.length ? `${(100 * med(okRows.map((x) => x.dist))).toFixed(1)} cm / ${med(okRows.map((x) => x.angle)).toFixed(2)}°` : '–'} | ${Math.round(med(r.map((x) => x.poses))).toLocaleString('fr-FR')} | ${Math.round(r.reduce((a, x) => a + x.time, 0) / r.length)} ms | ${r.filter((x) => x.ambiguous).length} |`,
      );
    }
  }
}
// Valeur de l'indicateur de confiance : une réponse « fiable » est-elle juste ? un échec est-il signalé ?
lines.push('', '## Confiance', '', '| Jeu | Variante | Réponses fiables : justes | Réponses ambiguës : justes | Échecs signalés ambigus |', '| --- | --- | --- | --- | --- |');
for (const dataset of ['simulé', 'salle']) {
  for (const v of VARIANTS) {
    const r = rows.filter((x) => x.dataset === dataset && x.variant === v.name);
    const sure = r.filter((x) => !x.ambiguous);
    const doubt = r.filter((x) => x.ambiguous);
    const fails = r.filter((x) => !x.ok);
    lines.push(
      `| ${dataset} | ${v.label} | ${sure.filter((x) => x.ok).length}/${sure.length} | ${doubt.filter((x) => x.ok).length}/${doubt.length} | ${fails.filter((x) => x.ambiguous).length}/${fails.length} |`,
    );
  }
}
lines.push(
  '',
  '## Détail par scan',
  '',
  "Confiance : points qui départagent la 1re et la 2e candidate (sur la carte pour l'une et pas",
  "pour l'autre). Ambiguë si la 2e en a au moins moitié autant que la 1re, ou si moins de 8 points départagent.",
  '',
  '| Jeu | Scan | Variante | Erreur | Points sur la carte | Points 1re / 2e | Temps |',
  '| --- | --- | --- | --- | --- | --- | --- |',
);
for (const x of rows) {
  lines.push(
    `| ${x.dataset} | ${x.scan}${x.elevated ? ' (en hauteur)' : ''} | ${x.variant} | ${x.ok ? '' : '**échec** '}${(100 * x.dist).toFixed(1)} cm / ${x.angle.toFixed(1)}° | ${Math.round(100 * x.inliers)} % | ${x.evidence ? `${x.evidence.best} / ${x.evidence.second}` : '–'}${x.ambiguous ? ' (ambiguë)' : ''} | ${Math.round(x.time)} ms |`,
  );
}
await writeFile(ROOT + 'docs/jour2_global.md', lines.join('\n') + '\n');
await writeFile(
  ROOT + 'docs/jour2_global.csv',
  ['jeu,scan,en_hauteur,variante,erreur_m,erreur_deg,reussite,points_sur_carte,ratio_2e,ambigue,poses_testees,temps_ms',
    ...rows.map((x) => [x.dataset, x.scan, x.elevated, x.variant, x.dist.toFixed(4), x.angle.toFixed(2), x.ok, x.inliers.toFixed(3), x.ratio.toFixed(3), x.ambiguous, x.poses, x.time.toFixed(0)].join(','))].join('\n') + '\n',
);
console.log(lines.slice(7, 7 + 2 + 6).join('\n'));
console.log('Écrit : docs/jour2_global.md et docs/jour2_global.csv');
