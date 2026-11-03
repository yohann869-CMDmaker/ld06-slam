// Banc d'essai du jour 2 : chaque méthode de localisation, sur chaque type de référence, à
// partir des mêmes scans simulés (pose vraie connue) et des mêmes poses initiales perturbées.
// Mesures : erreur de position (m), erreur angulaire (°), temps de calcul, taux de réussite
// (seuils ci-dessous), et sensibilité à l'initialisation (4 niveaux de perturbation).
// Usage : node scripts/benchmark-jour2.mjs   -> docs/jour2_benchmark.md et .csv
import { readFile, writeFile } from 'node:fs/promises';
import { fileURLToPath } from 'node:url';
import { parseScanCSV, scanToLocalXY, scanToWorldXY } from '../src/core/scan.js';
import { gridFromScan } from '../src/core/gridmap.js';
import { VectorMap } from '../src/core/vectormap.js';
import { gridReference, localize, METHODS, METHODS_BY_REFERENCE, scanReference, vectorReference } from '../src/core/localization.js';
import { DEG, poseError } from '../src/core/pose.js';
import { mulberry32 } from '../src/core/sim.js';

const ROOT = fileURLToPath(new URL('../', import.meta.url));
const SUCCESS = { dist: 0.05, angle: 2 }; // réussite : moins de 5 cm et moins de 2°
const LEVELS = [
  { dist: 0.05, angle: 2 },
  { dist: 0.15, angle: 5 },
  { dist: 0.3, angle: 10 },
  { dist: 0.45, angle: 18 },
]; // perturbations de la pose initiale (dans la fenêtre de recherche ±50 cm, ±20°)
const TRIALS = Number(process.argv[2] ?? 3); // tirages par niveau et par scan

const index = JSON.parse(await readFile(ROOT + 'data/sim/index.json', 'utf8'));
const scans = [];
for (const s of index.scans) scans.push(parseScanCSV(await readFile(ROOT + 'data/sim/' + s.file, 'utf8'), s.file));
const truth = index.scans.map((s) => s.truePose);

// Références pour le scan k : cartes construites avec les autres scans (pose vraie), scan de référence = le plus proche.
function references(k) {
  const others = scans.filter((_, j) => j !== k);
  const grid = gridFromScan(others[0]);
  others.slice(1).forEach((s) => grid.integrateScan(s));
  const vm = new VectorMap([], { absence: false });
  others.forEach((s) => vm.integrateScan(s));
  const nearest = others.reduce((a, b) => (poseError(a.pose, truth[k]).dist < poseError(b.pose, truth[k]).dist ? a : b));
  return { grid: gridReference(grid), vector: vectorReference(vm.segments), scan: scanReference(scanToWorldXY(nearest)) };
}

// Poses initiales : mêmes tirages pour toutes les méthodes (graine fixe).
const rng = mulberry32(2026);
const inits = scans.map((_, k) =>
  LEVELS.map((lvl) =>
    Array.from({ length: TRIALS }, () => {
      const dir = rng() * 2 * Math.PI;
      const sign = rng() < 0.5 ? -1 : 1;
      return { x: truth[k].x + lvl.dist * Math.cos(dir), y: truth[k].y + lvl.dist * Math.sin(dir), theta: truth[k].theta + sign * lvl.angle };
    }),
  ),
);

const rows = [];
const t0 = performance.now();
for (let k = 0; k < scans.length; k++) {
  const refs = references(k);
  const pts = scanToLocalXY(scans[k]);
  for (const [kind, ref] of Object.entries(refs)) {
    for (const method of METHODS_BY_REFERENCE[kind]) {
      LEVELS.forEach((lvl, li) => {
        for (const init of inits[k][li]) {
          const r = localize(ref, pts, init, method);
          const e = poseError(r.pose, truth[k]);
          rows.push({ scan: k + 1, kind, method, level: li, dist: e.dist, angle: e.angle, time: r.time, ok: e.dist < SUCCESS.dist && e.angle < SUCCESS.angle });
        }
      });
    }
    process.stdout.write(`scan ${k + 1} : ${kind} terminé\n`);
  }
}

// ------------------------------------------------------------------ synthèse
const KIND = { grid: 'Grid map', vector: 'Carte vectorielle', scan: 'Scan de référence' };
const median = (a) => {
  const s = [...a].sort((x, y) => x - y);
  return s[Math.floor(s.length / 2)];
};
const mean = (a) => a.reduce((x, y) => x + y, 0) / a.length;
const pct = (a) => `${Math.round((100 * a.filter((r) => r.ok).length) / a.length)} %`;

const md = [
  '# Jour 2 — Banc d\'essai de la localisation',
  '',
  `Jeu simulé (salle 8 m × 6 m, ${scans.length} scans, bruit σ = 1 cm, pose vraie connue). Pour chaque scan, les`,
  'cartes de référence sont construites avec **les autres scans** à leur pose vraie ; le scan de référence est le scan',
  `le plus proche. Poses initiales : ${LEVELS.length} niveaux de perturbation × ${TRIALS} tirages par scan, **identiques pour toutes`,
  'les méthodes**. Fenêtre de recherche : ±50 cm, ±20°.',
  '',
  `**Réussite** : erreur de position < ${SUCCESS.dist * 100} cm **et** erreur angulaire < ${SUCCESS.angle}°.`,
  '',
  '## Résultats par méthode',
  '',
  '| Référence | Méthode | Réussite | Erreur position médiane | Erreur angle médiane | Temps moyen |',
  '| --- | --- | --- | --- | --- | --- |',
];
for (const kind of Object.keys(KIND)) {
  for (const method of METHODS_BY_REFERENCE[kind]) {
    const r = rows.filter((x) => x.kind === kind && x.method === method);
    md.push(`| ${KIND[kind]} | ${METHODS[method].label} | ${pct(r)} | ${(median(r.map((x) => x.dist)) * 1000).toFixed(1)} mm | ${median(r.map((x) => x.angle)).toFixed(2)}° | ${mean(r.map((x) => x.time)).toFixed(0)} ms |`);
  }
}
md.push('', '## Sensibilité à l\'initialisation (taux de réussite)', '');
md.push(`| Référence | Méthode | ${LEVELS.map((l) => `${l.dist * 100} cm / ${l.angle}°`).join(' | ')} |`);
md.push(`| --- | --- | ${LEVELS.map(() => '---').join(' | ')} |`);
for (const kind of Object.keys(KIND)) {
  for (const method of METHODS_BY_REFERENCE[kind]) {
    const cells = LEVELS.map((_, li) => pct(rows.filter((x) => x.kind === kind && x.method === method && x.level === li)));
    md.push(`| ${KIND[kind]} | ${METHODS[method].label} | ${cells.join(' | ')} |`);
  }
}
md.push(
  '',
  `${rows.length} localisations en ${((performance.now() - t0) / 1000).toFixed(0)} s (Node.js). Données brutes : \`jour2_benchmark.csv\`.`,
  'Régénérer : `node scripts/benchmark-jour2.mjs [tirages par niveau]`.',
  '',
);
await writeFile(ROOT + 'docs/jour2_benchmark.md', md.join('\n'));
await writeFile(
  ROOT + 'docs/jour2_benchmark.csv',
  ['scan,reference,methode,niveau,erreur_m,erreur_deg,temps_ms,reussite', ...rows.map((r) => [r.scan, r.kind, r.method, r.level, r.dist.toFixed(5), r.angle.toFixed(4), r.time.toFixed(1), r.ok ? 1 : 0].join(','))].join('\n') + '\n',
);
console.log(md.join('\n'));
void DEG;
