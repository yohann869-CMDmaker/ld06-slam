// Banc d'essai : que gagne-t-on à connaître à peu près la position du robot ?
// Données réelles : les 18 scans de data/real. Chaque fichier porte une pose APPROXIMATIVE (notée à la
// main ou venue de LidarScope, à 5-25 cm près) ; la référence est la pose recalée
// (data/reference/poses_recalees.csv, ± 2 cm).
//   A. Localisation globale de chaque scan sur une grid map construite SANS lui (scans au sol à leur
//      pose recalée), sans a priori puis avec la pose approximative comme a priori (σ = 50 cm, 15°).
//   B. SLAM sur les 18 scans dans l'ordre de prise : sans a priori (jour 3), puis avec, comme
//      odométrie, le déplacement entre les poses approximatives de deux scans successifs ; enfin sans
//      les scans pris en hauteur. Erreur de chaque pose estimée, scans perdus ou placés loin (> 1 m),
//      accord de la carte obtenue avec la carte de référence.
//   C. SLAM hors ligne : graphe de poses sur toutes les paires de scans proches (Lu & Milios 1997).
// Usage : node scripts/benchmark-prior.mjs   -> docs/jour3_apriori.md, figures docs/img/j3_reel_*.png
import { readdir, readFile, writeFile } from 'node:fs/promises';
import { fileURLToPath } from 'node:url';
import { parseScanCSV, scanToLocalXY, scanToWorldXY } from '../src/core/scan.js';
import { gridFromScan } from '../src/core/gridmap.js';
import { gridReference, localize, scanReference, vectorReference } from '../src/core/localization.js';
import { globalLocalize } from '../src/core/globalloc.js';
import { dominantDirection } from '../src/core/cleanup.js';
import { parseReferenceMap } from '../src/core/refmap.js';
import { normalizeAngleDeg } from '../src/core/pose.js';
import { relative, scanInformation, Slam } from '../src/core/slam.js';
import { optimizePoseGraph } from '../src/core/posegraph.js';
import { hex, Raster } from './lib/raster.mjs';

const ROOT = fileURLToPath(new URL('../', import.meta.url));
const files = (await readdir(ROOT + 'data/real')).filter((f) => /^scan_.*\.csv$/.test(f)).sort();
const truth = new Map(
  (await readFile(ROOT + 'data/reference/poses_recalees.csv', 'utf8'))
    .trim()
    .split(/\r?\n/)
    .slice(1)
    .map((l) => l.split(','))
    .map(([f, x, y, t]) => [f, { x: Number(x), y: Number(y), theta: Number(t) }]),
);
const scans = [];
for (const f of files) {
  const scan = parseScanCSV(await readFile(ROOT + 'data/real/' + f, 'utf8'), f);
  scans.push({ name: f, scan, approx: { ...scan.pose }, truth: truth.get(f), elevated: scan.elevated, pts: scanToLocalXY(scan) });
}
const reference = parseReferenceMap(await readFile(ROOT + 'data/reference/carte_reference.json', 'utf8'));
const err = (p, q) => ({ d: Math.hypot(p.x - q.x, p.y - q.y), a: Math.abs(normalizeAngleDeg(p.theta - q.theta)) });
const cm = (v) => `${(100 * v).toFixed(1)} cm`;
const median = (v) => {
  const s = [...v].sort((a, b) => a - b);
  return s.length ? (s[(s.length - 1) >> 1] + s[s.length >> 1]) / 2 : NaN;
};
const md = [
  '# Connaître (à peu près) la position du robot : ce que ça change',
  '',
  'Généré par `node scripts/benchmark-prior.mjs`. Données réelles : les 18 scans de la salle. Chaque',
  'fichier porte une **pose approximative** (notée à la main ou venue de LidarScope) ; la référence',
  'est la pose recalée (± 2 cm). Écart des poses approximatives à la référence :',
];
const approxErr = scans.map((s) => err(s.approx, s.truth));
md.push(`médiane ${cm(median(approxErr.map((e) => e.d)))} / ${median(approxErr.map((e) => e.a)).toFixed(1)}°, maximum ${cm(Math.max(...approxErr.map((e) => e.d)))} / ${Math.max(...approxErr.map((e) => e.a)).toFixed(1)}°.`, '');

// ------------------------------------------------------------------ A. localisation globale
md.push(
  '## A. Localisation globale, sans et avec a priori',
  '',
  'Chaque scan est cherché dans toute la salle sur une grid map construite **sans lui** (scans au sol à',
  'leur pose recalée). A priori : la pose approximative du fichier, σ = 50 cm et 15°. **Réussite** :',
  'erreur < 10 cm et < 2° par rapport à la pose recalée.',
  '',
  '| Scan | Sans a priori : erreur | Verdict | Avec a priori : erreur | Verdict |',
  '| --- | --- | --- | --- | --- |',
);
const tally = { sans: { ok: 0, faux: 0, time: 0 }, avec: { ok: 0, faux: 0, time: 0 } };
for (const s of scans) {
  const others = scans.filter((o) => o !== s && !o.elevated);
  let grid = null;
  for (const o of others) {
    if (!grid) grid = gridFromScan(o.scan, o.truth);
    else grid.integrateScan(o.scan, o.truth);
  }
  const ref = gridReference(grid);
  const walls = reference.segments.filter((w) => w.kind !== 'objet');
  const base = { mapDominant: dominantDirection(walls) };
  const cells = [];
  for (const [key, opts] of [['sans', base], ['avec', { ...base, prior: { pose: s.approx, sigmaXY: 0.5, sigmaTheta: 15 } }]]) {
    const g = globalLocalize(ref, s.pts, opts);
    const e = err(g.pose, s.truth);
    const ok = e.d < 0.1 && e.a < 2;
    tally[key].time += g.time;
    if (ok) tally[key].ok++;
    else if (!g.ambiguous) tally[key].faux++;
    cells.push(`${ok ? '' : '**'}${cm(e.d)} / ${e.a.toFixed(1)}°${ok ? '' : '**'}`, g.ambiguous ? 'ambiguë' : 'fiable');
  }
  md.push(`| ${s.name}${s.elevated ? ' (en hauteur)' : ''} | ${cells.join(' | ')} |`);
  process.stdout.write(`A ${s.name} : ${cells.join(' ; ')}\n`);
}
md.push(
  '',
  `**Sans a priori** : ${tally.sans.ok} / ${scans.length} réussis, ${tally.sans.faux} faux annoncés fiables, ${(tally.sans.time / scans.length).toFixed(0)} ms par scan.`,
  `**Avec a priori** : ${tally.avec.ok} / ${scans.length} réussis, ${tally.avec.faux} faux annoncés fiables, ${(tally.avec.time / scans.length).toFixed(0)} ms par scan.`,
  '',
);

// ------------------------------------------------------------------ B. SLAM
md.push(
  '## B. SLAM sur les scans réels',
  '',
  "Les scans dans l'ordre de prise ; le premier est placé à sa pose recalée (pour comparer dans le même",
  'repère), les autres poses viennent du SLAM. Odométrie (variantes 2 et 3) : déplacement entre les',
  'poses approximatives de deux scans successifs, comme le donnerait la durée des commandes du robot.',
  "**Placé loin** : à plus de 1 m de la pose recalée (symétrique de la salle, par exemple). **Accord avec",
  'la référence** : part des cellules occupées de la carte du SLAM à moins de 10 cm d\'un mur ou meuble',
  'de la carte de référence.',
  '',
  '| Variante | Scans | Perdus | Placés loin | Erreur médiane / max (scans placés) | Erreur médiane / max (tous, perdus à la pose prédite) | Accord avec la référence | Temps |',
  '| --- | --- | --- | --- | --- | --- | --- | --- |',
);
const refField = vectorReference(reference.segments).field(0.05);
const variants = [
  { id: 'sans', label: 'Sans a priori (jour 3)', odom: false, floorOnly: false },
  { id: 'avec', label: 'Avec odométrie (poses approximatives)', odom: true, floorOnly: false },
  { id: 'sol', label: 'Avec odométrie, scans au sol seulement', odom: true, floorOnly: true },
];
for (const v of variants) {
  const list = v.floorOnly ? scans.filter((s) => !s.elevated) : scans;
  const slam = new Slam();
  const t0 = performance.now();
  list.forEach((s, k) => slam.addScan(s.scan, v.odom && k ? relative(list[k - 1].approx, s.approx) : null, k ? null : s.truth));
  const time = (performance.now() - t0) / list.length;
  const rows = slam.trajectory.map((t, k) => ({ ...err(t.pose, list[k].truth), mode: t.mode }));
  const kept = rows.filter((r, k) => k && r.mode !== 'perdu');
  const all = rows.slice(1);
  const pair = (e) => (e.length ? `${cm(median(e.map((r) => r.d)))} / ${cm(Math.max(...e.map((r) => r.d)))}` : '–');
  const far = kept.filter((r) => r.d > 1).length;
  const occ = slam.grid.occupiedCells();
  const agree = occ.filter((c) => refField.at(c.x, c.y) <= 0.1).length / Math.max(occ.length, 1);
  md.push(`| ${v.label} | ${list.length} | ${all.length - kept.length} | ${far} | ${pair(kept)} | ${pair(all)} | ${(100 * agree).toFixed(0)} % | ${time.toFixed(0)} ms |`);
  process.stdout.write(`B ${v.label} : ${rows.map((r) => `${r.mode[0]}${(100 * r.d).toFixed(0)}`).join(' ')}\n`);
  // figure : carte de référence (gris), points des scans clés à leur pose SLAM, trajectoire
  const b = { minX: -11.2, maxX: 1.9, minY: -7.8, maxY: 6.6 };
  const img = new Raster(b, 45, '#fbfbf9');
  img.metric(1);
  for (const w of reference.segments) img.line(w.x1, w.y1, w.x2, w.y2, hex('#adb5bd'), 3);
  for (const kf of slam.keyframes) for (const p of scanToWorldXY(kf.scan, kf.pose)) img.dot(p.x, p.y, 1.4, hex('#1d2327'), 0.55);
  slam.trajectory.forEach((t, k) => {
    const c = t.mode === 'perdu' ? '#e03131' : rows[k].d > 1 ? '#f08c00' : '#1c7ed6';
    img.robot(t.pose, hex(c), 6);
    img.line(t.pose.x, t.pose.y, list[k].truth.x, list[k].truth.y, hex('#e03131'), 1.5);
  });
  await writeFile(ROOT + `docs/img/j3_reel_${v.id}.png`, img.png());
}
md.push(
  '',
  'Figures : murs et meubles de la carte de référence (gris), points des scans gardés à leur pose',
  'SLAM (noir), robots : bleu = bien placé, orange = placé loin, rouge = perdu (traits rouges : écart',
  'à la pose recalée).',
  '',
  '| Sans a priori | Avec odométrie | Avec odométrie, au sol |',
  '| --- | --- | --- |',
  '| ![sans](img/j3_reel_sans.png) | ![avec](img/j3_reel_avec.png) | ![sol](img/j3_reel_sol.png) |',
  '',
);

// ------------------------------------------------------------------ C. graphe de poses hors ligne
md.push(
  '## C. SLAM hors ligne : graphe de poses à partir des poses approximatives (Lu & Milios 1997)',
  '',
  'Tous les scans d\'un coup au lieu d\'un par un. Pour chaque paire de scans à moins de 6 m l\'un de',
  "l'autre (d'après les poses approximatives), le second est recalé sur le premier (multi-résolution",
  '±50 cm / ±15° autour du déplacement approximatif, puis ICP) ; si au moins 35 % de ses points collent,',
  "la paire devient un lien du graphe (déplacement mesuré, précision d'après les normales des points).",
  "Chaque scan est aussi relié au premier par sa **pose approximative** (lien peu précis : 15 cm, 2°) :",
  "c'est l'a priori de position, qui tient les scans sans voisin assez proche. Le graphe est optimisé",
  "(Gauss-Newton), le premier scan fixé à sa pose recalée. Le recalage de la carte de référence",
  '(`autoAlign`) suit la même idée, scan par scan face à tous les autres.',
  '',
  '| Liens | Paires testées | Erreur médiane / max (avant optimisation : poses approximatives) | Erreur médiane / max (après) | Temps |',
  '| --- | --- | --- | --- | --- |',
);
{
  const t0 = performance.now();
  const world = (s, pose) => scanToWorldXY(s.scan, pose);
  const edges = [];
  let tested = 0;
  for (let i = 0; i < scans.length; i++) {
    const refI = scanReference(world(scans[i], { x: 0, y: 0, theta: 0 }));
    for (let j = i + 1; j < scans.length; j++) {
      const z0 = relative(scans[i].approx, scans[j].approx);
      if (Math.hypot(z0.x, z0.y) > 6) continue;
      tested++;
      const m = localize(refI, scans[j].pts, z0, 'multires', { xyRange: 0.5, thRange: 15 });
      const r = localize(refI, scans[j].pts, m.pose, 'icp');
      const best = r.inliers >= m.inliers ? r : m;
      if (best.inliers < 0.35) continue;
      edges.push({ from: i, to: j, z: best.pose, info: scanInformation(scans[j].pts), kind: 'paire' });
    }
  }
  const pairEdges = edges.length;
  // a priori de position : chaque scan relié au premier par le déplacement entre poses approximatives
  for (let k = 1; k < scans.length; k++) edges.push({ from: 0, to: k, z: relative(scans[0].approx, scans[k].approx), sigma: { xy: 0.15, theta: 2 }, kind: 'a priori' });
  const init = scans.map((s, k) => (k ? { ...s.approx } : { ...s.truth }));
  const g = optimizePoseGraph(init, edges);
  const before = init.map((p, k) => err(p, scans[k].truth));
  const after = g.poses.map((p, k) => err(p, scans[k].truth));
  const fmt = (e) => `${cm(median(e.map((x) => x.d)))} / ${cm(Math.max(...e.map((x) => x.d)))}`;
  md.push(`| ${pairEdges} recalages + ${scans.length - 1} a priori | ${tested} | ${fmt(before)} | **${fmt(after)}** | ${((performance.now() - t0) / 1000).toFixed(1)} s |`, '');
  process.stdout.write(`C : ${pairEdges} liens, avant ${fmt(before)}, après ${fmt(after)}\n`);
  const b = { minX: -11.2, maxX: 1.9, minY: -7.8, maxY: 6.6 };
  const img = new Raster(b, 45, '#fbfbf9');
  img.metric(1);
  for (const w of reference.segments) img.line(w.x1, w.y1, w.x2, w.y2, hex('#adb5bd'), 3);
  for (const [k, s] of scans.entries()) if (!s.elevated) for (const p of world(s, g.poses[k])) img.dot(p.x, p.y, 1.4, hex('#1d2327'), 0.55);
  for (const e of edges.filter((x) => x.kind === 'paire')) img.line(g.poses[e.from].x, g.poses[e.from].y, g.poses[e.to].x, g.poses[e.to].y, hex('#1c7ed6'), 1, null, 0.35);
  g.poses.forEach((p) => img.robot(p, hex('#1c7ed6'), 6));
  await writeFile(ROOT + 'docs/img/j3_reel_graphe.png', img.png());
  md.push('![Graphe de poses hors ligne](img/j3_reel_graphe.png)', '', '*Points des scans au sol à leur pose optimisée (noir), carte de référence (gris), liens du graphe (bleu).*', '');
}

await writeFile(ROOT + 'docs/jour3_apriori.md', md.join('\n'));
console.log(md.join('\n'));
