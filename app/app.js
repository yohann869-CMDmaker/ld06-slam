// Application web : affichage de plusieurs scans, poses modifiables,
// construction et mise à jour d'une grid map et d'une carte vectorielle.
import { composePoses, DEG, normalizeAngleDeg, poseError, transformPoints } from '../src/core/pose.js';
import { looksElevated, parseRobotBinary, parseScanCSV, scanToLocalXY, serializeScanCSV, setElevated, validPoints } from '../src/core/scan.js';
import { commandMotion, commandRequest, Explorer, simulateExploration, updateSpeeds } from '../src/core/explore.js';
import { GridMap, gridFromScan, occupancyRGB, UNKNOWN_RGB } from '../src/core/gridmap.js';
import { VectorMap } from '../src/core/vectormap.js';
import { autoAlign, consistency, matchScan } from '../src/core/matching.js';
import { cleanMaps, cleanVectorMap, dominantDirection, isPartlyOccupied, labelPositions, parseLabels } from '../src/core/cleanup.js';
import { globalLocalize } from '../src/core/globalloc.js';
import { relative, Slam } from '../src/core/slam.js';
import { gridReference, localize, METHODS, METHODS_BY_REFERENCE, scanReference, vectorReference } from '../src/core/localization.js';
import { dayOneExports, drawLabels, MAPTOOL, maptoolColor } from './figures.js';
import { makeReferenceMap, parseReferenceMap, referenceBounds, serializeReferenceMap } from '../src/core/refmap.js';
import { extractSegments } from '../src/core/segmentation.js';
import { transformSegment } from '../src/core/geometry.js';

const PALETTE = ['#e8590c', '#1c7ed6', '#2f9e44', '#ae3ec9', '#f08c00', '#0c8599', '#c2255c', '#5c940d', '#5f3dc4', '#795548'];
const $ = (id) => document.getElementById(id);
const canvas = $('view');
const ctx = canvas.getContext('2d');

const state = {
  scans: [],
  selected: null,
  nextId: 1,
  grid: null,
  gridImage: null,
  vector: null,
  reference: null,
  labels: null, // carte sémantique de la salle : meubles nommés (data/reference/etiquettes.json)
  locLog: [], // résultats de localisation de la séance (bouton « Exporter les résultats »)
  slam: null, // SLAM en direct (jour 3)
  slamTrack: null, // trajectoire estimée par le SLAM (dessinée en bleu)
  slamLivePose: null, // pose courante du robot estimée par le SLAM en direct
  slamBusy: false,
  view: { x: 4, y: 3, scale: 70 }, // centre de la vue (m) et pixels par mètre
  mouse: null,
};

// ------------------------------------------------------------------ utilitaires

function filterOpts() {
  return { minRange: Number($('fMin').value), maxRange: Number($('fMax').value), minIntensity: Number($('fInt').value) };
}

function toast(msg) {
  const t = $('toast');
  t.textContent = msg;
  t.hidden = false;
  clearTimeout(toast.timer);
  toast.timer = setTimeout(() => (t.hidden = true), 2600);
}

function download(name, content, type = 'text/plain') {
  const blob = content instanceof Blob ? content : new Blob([content], { type });
  const a = document.createElement('a');
  a.href = URL.createObjectURL(blob);
  a.download = name;
  a.click();
  setTimeout(() => URL.revokeObjectURL(a.href), 1000);
}

const readFile = (file) => file.text();

// ------------------------------------------------------------------ scans

function addScan(scan) {
  const id = state.nextId++;
  const entry = {
    id,
    scan,
    name: scan.name,
    pose: { ...scan.pose },
    filePose: { ...scan.pose },
    color: PALETTE[(id - 1) % PALETTE.length],
    visible: true,
    flip: false,
    cache: null,
    looksHigh: looksElevated(scan), // indice : rien à moins de 1 m, portée médiane ≥ 4 m
  };
  state.scans.push(entry);
  return entry;
}

/** Points et segments dans le repère du robot (indépendants de la pose, donc mis en cache). */
function local(entry) {
  const f = filterOpts();
  const key = `${f.minRange}|${f.maxRange}|${f.minIntensity}|${entry.flip}`;
  if (!entry.cache || entry.cache.key !== key) {
    const pts = scanToLocalXY(entry.scan, f, { flipAngles: entry.flip });
    entry.cache = { key, pts, segs: extractSegments(pts).segments };
  }
  return entry.cache;
}

const worldPoints = (e) => transformPoints(local(e).pts, e.pose);
const worldSegments = (e) => local(e).segs.map((s) => transformSegment(s, e.pose));
const integrateOpts = (e) => [e.scan, e.pose, filterOpts(), { flipAngles: e.flip }];

function select(entry) {
  state.selected = entry;
  state.globalCandidates = null;
  renderScanList();
  renderPoseEditor();
  draw();
}

function removeScan(entry) {
  state.scans = state.scans.filter((e) => e !== entry);
  if (state.selected === entry) state.selected = null;
  renderScanList();
  renderPoseEditor();
  poseChanged();
}

function renderScanList() {
  const ul = $('scanList');
  ul.replaceChildren();
  $('scanEmpty').hidden = state.scans.length > 0;
  for (const e of state.scans) {
    const li = document.createElement('li');
    if (e === state.selected) li.className = 'selected';
    const vis = Object.assign(document.createElement('input'), { type: 'checkbox', checked: e.visible, title: 'Afficher' });
    vis.onclick = (ev) => ev.stopPropagation();
    vis.onchange = () => {
      e.visible = vis.checked;
      poseChanged();
    };
    const col = Object.assign(document.createElement('input'), { type: 'color', value: e.color, title: 'Couleur' });
    col.onclick = (ev) => ev.stopPropagation();
    col.oninput = () => {
      e.color = col.value;
      draw();
    };
    const label = document.createElement('div');
    const c = local(e);
    const high = e.scan.elevated ? ' · <strong>en hauteur</strong>' : e.looksHigh ? ' · <span title="Rien à moins de 1 m : le robot était-il posé sur un meuble ? (case dans la pose)">en hauteur ?</span>' : '';
    label.innerHTML = `<div class="name"></div><div class="meta">${c.pts.length} pts · ${c.segs.length} segments · ${e.scan.format}${high}</div>`;
    label.firstChild.textContent = e.name;
    const rm = Object.assign(document.createElement('button'), { className: 'remove', textContent: '×', title: 'Retirer' });
    rm.onclick = (ev) => {
      ev.stopPropagation();
      removeScan(e);
    };
    li.append(vis, col, label, rm);
    li.onclick = () => select(e);
    ul.append(li);
  }
}

function renderPoseEditor() {
  const e = state.selected;
  $('poseEditor').hidden = !e;
  if (!e) return;
  $('poseName').textContent = e.name;
  if (document.activeElement !== $('poseX')) $('poseX').value = e.pose.x.toFixed(3);
  if (document.activeElement !== $('poseY')) $('poseY').value = e.pose.y.toFixed(3);
  if (document.activeElement !== $('poseT')) $('poseT').value = e.pose.theta.toFixed(2);
  $('poseFlip').checked = e.flip;
  $('poseHigh').checked = Boolean(e.scan.elevated);
}

let rebuildTimer = null;
/**
 * Une pose, la visibilité ou la liste des scans a changé : redessin, et reconstruction
 * automatique de la grid map et de la carte vectorielle (case « Cartes automatiques »).
 */
function poseChanged({ final = true } = {}) {
  renderPoseEditor();
  draw();
  if ($('autoRebuild').checked && final) {
    clearTimeout(rebuildTimer);
    rebuildTimer = setTimeout(() => {
      if (!state.scans.some((e) => e.visible)) return;
      buildGridAll(true);
      buildVectorAll(true);
    }, 150);
  }
}

const tick = () => new Promise((r) => setTimeout(r, 0));
const visibleItems = () => state.scans.filter((e) => e.visible).map((e) => ({ entry: e, points: local(e).pts, pose: e.pose }));

/** Bouton « Recaler automatiquement » : place puis affine tous les scans visibles (le premier est la référence). */
async function runAutoAlign() {
  const items = visibleItems();
  if (items.length < 2) return toast('Il faut au moins deux scans visibles');
  $('btnAlign').disabled = true;
  const before = consistency(items.map((i) => i.points), items.map((i) => i.pose));
  const t0 = performance.now();
  const steps = autoAlign(items);
  let r;
  while (!(r = steps.next()).done) {
    const { phase, k, total } = r.value;
    toast(`Recalage automatique : ${phase} du scan ${k + 1}/${total}…`);
    await tick();
  }
  r.value.forEach((pose, k) => (items[k].entry.pose = pose));
  const after = consistency(items.map((i) => i.points), r.value);
  const mean = (a) => Math.round((100 * a.reduce((x, y) => x + y, 0)) / a.length);
  $('btnAlign').disabled = false;
  poseChanged();
  if (!$('autoRebuild').checked) {
    buildGridAll(true);
    buildVectorAll(true);
  }
  toast(`Recalage terminé en ${((performance.now() - t0) / 1000).toFixed(1)} s : ${mean(before)} % → ${mean(after)} % des points collent aux autres scans`);
}

/** Recalage d'un scan qui vient d'être capturé, sur les scans déjà présents (fenêtre large). */
function alignNewScan(entry) {
  const others = state.scans.filter((e) => e !== entry && e.visible);
  if (!others.length) return;
  const ref = others.flatMap((e) => worldPoints(e));
  const r = matchScan(ref, local(entry).pts, entry.pose);
  entry.pose = r.pose;
  return r;
}

async function loadFiles(files) {
  let n = 0;
  for (const file of files) {
    const text = await readFile(file);
    const kind = classify(text);
    if (kind === 'grid') {
      setGrid(GridMap.fromCSV(text, { resolution: Number($('gridRes').value) }));
      toast(`Grid map importée : ${file.name}`);
    } else if (kind === 'vector') {
      state.reference = VectorMap.fromTXT(text);
      $('showRef').checked = true;
      toast(`Carte de référence : ${file.name} (${state.reference.segments.length} segments)`);
    } else {
      const scan = parseScanCSV(text, file.name);
      if (!scan.points.length) {
        toast(`${file.name} : aucun point lu`);
        continue;
      }
      addScan(scan);
      n++;
    }
  }
  if (n) {
    renderScanList();
    fitView();
    poseChanged();
  }
  draw();
}

/** Devine le type d'un fichier déposé : grid map, carte vectorielle ou scan. */
function classify(text) {
  if (/^#\s*(gridmap|grid map)/im.test(text)) return 'grid';
  const data = text.split(/\r?\n/).map((l) => l.trim()).filter((l) => l && !l.startsWith('#'));
  if (data.length && data.every((l) => /^-?[\d.]+(\s*,\s*-?[\d.]+){3}$/.test(l))) return 'vector';
  return 'scan';
}

async function loadSimulated() {
  state.labels = null; // les étiquettes de la salle ne valent pas pour la scène simulée
  try {
    const index = await (await fetch('../data/sim/index.json')).json();
    for (const s of index.scans) {
      const text = await (await fetch(`../data/sim/${s.file}`)).text();
      addScan(parseScanCSV(text, s.file));
    }
    state.reference = VectorMap.fromTXT(await (await fetch('../data/sim/scene_A.txt')).text());
    $('showRef').checked = false;
    renderScanList();
    fitView();
    poseChanged();
    toast(`${index.scans.length} scans simulés chargés (référence : scene_A.txt, masquée)`);
  } catch (err) {
    toast(`Chargement impossible : ${err.message}`);
  }
}

/**
 * Trajet simulé du jour 3 (data/sim_trajet, scripts/gen-trajet.mjs) : 75 scans le long d'une boucle
 * de 21 m dans une salle de classe simulée, placés selon l'odométrie seule (qui dérive). Bouton
 * « SLAM sur les scans visibles » ensuite : la carte se remet d'aplomb.
 */
async function loadSimTrajet() {
  state.labels = null;
  try {
    const index = await (await fetch('../data/sim_trajet/index.json', { cache: 'no-store' })).json();
    for (const s of index.scans) addScan(parseScanCSV(await (await fetch(`../data/sim_trajet/${s.file}`)).text(), s.file));
    renderScanList();
    fitView();
    poseChanged();
    toast(`${index.scans.length} scans du trajet simulé, placés selon l'odométrie seule : lancer « SLAM sur les scans visibles »`);
  } catch (err) {
    toast(`Chargement impossible : ${err.message} (node scripts/gen-trajet.mjs)`);
  }
}

/**
 * Les scans réels de la salle (data/real) à leurs poses recalées (data/reference/poses_recalees.csv,
 * écrit par scripts/build-reference.mjs) : de quoi tout retrouver après un rechargement de la page.
 * Les scans déjà dans la liste ne sont pas rechargés.
 */
/**
 * Carte sémantique de la salle (data/reference/etiquettes.json) : table, armoire et tableau nommés
 * à la main. Les objets fixes détectés hors de ces meubles sont retirés des cartes propres.
 * Chargée avec les scans de la salle (zones dans le repère de la carte de référence).
 */
async function loadLabels() {
  try {
    const res = await fetch('../data/reference/etiquettes.json', { cache: 'no-store' });
    state.labels = res.ok ? parseLabels(await res.text()) : null;
  } catch {
    state.labels = null;
  }
}

async function loadRoomScans() {
  try {
    const res = await fetch('../data/reference/poses_recalees.csv', { cache: 'no-store' });
    if (!res.ok) throw new Error('data/reference/poses_recalees.csv introuvable (node scripts/build-reference.mjs)');
    const rows = (await res.text()).trim().split(/\r?\n/).slice(1).map((l) => l.split(','));
    await loadLabels();
    const have = new Set(state.scans.map((e) => e.name));
    let n = 0;
    for (const [name, x, y, theta] of rows) {
      if (have.has(name)) continue;
      const r = await fetch(`../data/real/${name}`, { cache: 'no-store' });
      if (!r.ok) continue;
      const entry = addScan(parseScanCSV(await r.text(), name));
      entry.pose = { x: Number(x), y: Number(y), theta: Number(theta) }; // « Pose du fichier » : pose manuelle
      n++;
    }
    renderScanList();
    fitView();
    poseChanged();
    const high = state.scans.filter((e) => e.scan.elevated).length;
    toast(`${n} scans de la salle chargés à leurs poses recalées${high ? ` (${high} pris en hauteur)` : ''}`);
  } catch (err) {
    toast(`Chargement impossible : ${err.message}`);
  }
}

// ------------------------------------------------------------------ robot en temps réel
// Le relais scripts/live.py lit l'ESP32 sur le port USB et diffuse chaque nouveau scan
// sur /live (Server-Sent Events). Le scan courant est dessiné en continu à la pose du scan
// sélectionné ; « Ajouter ce scan » (ou Espace) le fige dans la liste.

const live = { source: null, text: null, scan: null, time: 0, status: null };

function liveStatus(msg) {
  $('liveStat').textContent = msg;
}

async function startLive() {
  try {
    const res = await fetch('/live/status', { cache: 'no-store' });
    if (!res.ok) throw new Error();
  } catch {
    liveStatus('Pas de robot : lancer « python scripts/live.py » (au lieu de node serve.mjs) puis recharger la page.');
    return;
  }
  live.source = new EventSource('/live');
  live.source.addEventListener('status', (ev) => {
    live.status = JSON.parse(ev.data);
    renderLiveStatus();
  });
  live.source.addEventListener('scan', (ev) => {
    const d = JSON.parse(ev.data);
    live.text = d.text;
    live.scan = parseScanCSV(d.text, 'temps réel');
    const firstScan = live.time === 0;
    live.time = Date.now();
    $('liveAdd').disabled = false;
    renderLiveStatus();
    if ($('slamLive').checked) slamLiveStep();
    if (firstScan && !state.scans.length) fitView();
    else draw();
  });
  live.source.onerror = () => liveStatus('Relais live.py injoignable (arrêté ?), nouvel essai…');
}

function renderLiveStatus() {
  const s = live.status;
  let msg = s ? s.message : 'Connexion au relais…';
  if (live.scan && s?.connected) {
    const n = validPoints(live.scan, filterOpts()).length;
    const tours = live.scan.mergedLines > 1 ? `, ${live.scan.mergedLines} tours fusionnés` : '';
    msg += ` · ${n} points${tours}`;
  }
  liveStatus(msg);
}

/** Pose à laquelle le scan en temps réel est dessiné : celle du scan sélectionné (point de départ du recalage). */
function livePose() {
  if ($('slamLive').checked && state.slamLivePose) return state.slamLivePose;
  return state.selected ? state.selected.pose : { x: 0, y: 0, theta: 0 };
}

/** Fige le scan en temps réel : il rejoint la liste avec sa propre couleur. */
function liveCapture() {
  if (!live.text) return toast('Aucun scan reçu du robot pour l\'instant');
  const scan = parseScanCSV(live.text, captureName());
  const entry = addLiveScan(scan);
  const n = validPoints(scan, filterOpts()).length;
  const r = $('liveAlign').checked ? alignNewScan(entry) : null;
  poseChanged();
  toast(r ? `${scan.name} ajouté et recalé : ${n} points, ${Math.round(r.inliers * 100)} % collent aux scans précédents` : `${scan.name} ajouté : ${n} points`);
}

/**
 * Ajoute un scan acquis à l'instant (temps réel ou collage). Le robot a peu bougé
 * depuis le scan précédent : le nouveau part de la pose du scan sélectionné, il ne
 * reste qu'à le recaler. Le premier scan reste en (0, 0, 0) et définit le repère.
 */
function addLiveScan(scan) {
  const first = state.scans.length === 0;
  const ref = state.selected;
  const entry = addScan(scan);
  if (ref) entry.pose = { ...ref.pose };
  select(entry);
  if (first) fitView();
  return entry;
}

/** Texte collé (moniteur série, Teleplot ou CSV) -> nouveau scan. */
function addPastedText(text) {
  const scan = parseScanCSV(text, captureName());
  const n = validPoints(scan, filterOpts()).length;
  if (n < 10) {
    toast('Aucun scan reconnu dans le texte collé (lignes >lidar:…|xy ou CSV attendues)');
    return;
  }
  addLiveScan(scan);
  const tours = scan.mergedLines > 1 ? ` (${scan.mergedLines} tours fusionnés)` : '';
  toast(`${scan.name} ajouté : ${n} points valides${tours}`);
}

/** Nom de fichier horodaté : pas d'écrasement d'une séance de TP à l'autre. */
function captureName() {
  const d = new Date();
  const p = (n) => String(n).padStart(2, '0');
  return `scan_${p(d.getMonth() + 1)}${p(d.getDate())}_${p(d.getHours())}${p(d.getMinutes())}${p(d.getSeconds())}.csv`;
}


/** Enregistre chaque scan avec sa pose actuelle, plus un récapitulatif poses.csv. */
async function saveAll() {
  if (!state.scans.length) return toast('Aucun scan à enregistrer');
  const files = state.scans.map((e) => ({ name: e.name.replace(/\.(csv|txt)$/i, '') + '.csv', text: serializeScanCSV(e.scan, e.pose) }));
  files.push({
    name: 'poses.csv',
    text: ['scan,x_m,y_m,theta_deg', ...state.scans.map((e) => `${e.name},${e.pose.x.toFixed(4)},${e.pose.y.toFixed(4)},${e.pose.theta.toFixed(2)}`)].join('\n') + '\n',
  });
  await saveFiles(files);
}

/**
 * Écrit des fichiers { name, text | blob } : dans un dossier choisi (Chrome, Edge) ou en
 * téléchargements successifs (Firefox).
 */
async function saveFiles(files) {
  if (!window.showDirectoryPicker) {
    for (const f of files) {
      download(f.name, f.blob ?? f.text, 'text/plain');
      await new Promise((r) => setTimeout(r, 150)); // Firefox ignore des téléchargements trop rapprochés
    }
    toast(`${files.length} fichiers téléchargés`);
    return;
  }
  try {
    const dir = await window.showDirectoryPicker({ mode: 'readwrite' });
    for (const f of files) {
      const w = await (await dir.getFileHandle(f.name, { create: true })).createWritable();
      await w.write(f.blob ?? f.text);
      await w.close();
    }
    toast(`${files.length} fichiers enregistrés dans « ${dir.name} »`);
  } catch (err) {
    if (err.name !== 'AbortError') toast(`Enregistrement impossible : ${err.message}`);
  }
}

// ------------------------------------------------------------------ grid map

function gridParams() {
  return { resolution: Number($('gridRes').value) };
}

function setGrid(g) {
  state.grid = g;
  state.gridImage = null;
  if (g) {
    const s = g.stats();
    $('gridStat').textContent = `${g.width} × ${g.height} cellules de ${g.res * 100} cm · ${g.scanCount} scan(s) · occupées ${s.occupied}, libres ${s.free}, inconnues ${s.unknown}`;
  } else {
    $('gridStat').textContent = '';
  }
  refreshClean();
  draw();
}

function gridImage() {
  if (state.gridImage || !state.grid) return state.gridImage;
  const g = state.grid;
  const off = document.createElement('canvas');
  off.width = g.width;
  off.height = g.height;
  const octx = off.getContext('2d');
  const img = octx.createImageData(g.width, g.height);
  for (let j = 0; j < g.height; j++) {
    for (let i = 0; i < g.width; i++) {
      const o = 4 * ((g.height - 1 - j) * g.width + i); // ligne 0 de l'image = haut de la carte
      img.data.set(occupancyRGB(g, g.index(i, j)), o);
      img.data[o + 3] = 255;
    }
  }
  octx.putImageData(img, 0, 0);
  state.gridImage = off;
  return off;
}

function requireSelection() {
  if (!state.selected) toast('Sélectionner un scan dans la liste');
  return state.selected;
}

function buildGridAll(silent = false) {
  const vis = state.scans.filter((e) => e.visible);
  if (!vis.length) return silent || toast('Aucun scan visible');
  const t0 = performance.now();
  const [first, ...rest] = vis;
  const g = gridFromScan(first.scan, first.pose, gridParams(), filterOpts(), { flipAngles: first.flip });
  for (const e of rest) g.integrateScan(...integrateOpts(e));
  setGrid(g);
  if (!silent) toast(`Grid map : ${vis.length} scans intégrés en ${(performance.now() - t0).toFixed(0)} ms`);
}

// ------------------------------------------------------------------ carte vectorielle

function vecParams() {
  return { absence: $('vecAbsence').checked };
}

function setVector(m, info = '') {
  state.vector = m;
  $('vecStat').textContent = m ? `${m.segments.length} segments · ${m.scanCount} scan(s)${info ? ' · ' + info : ''}` : '';
  refreshClean();
  draw();
}

function buildVectorAll(silent = false) {
  const vis = state.scans.filter((e) => e.visible);
  if (!vis.length) return silent || toast('Aucun scan visible');
  const m = new VectorMap([], vecParams());
  let cuts = 0;
  for (const e of vis) cuts += m.integrateScan(...integrateOpts(e)).cuts;
  setVector(m, `${cuts} découpes`);
  if (!silent) toast(`Carte vectorielle : ${m.segments.length} segments`);
}

// ------------------------------------------------------------------ vue

function resize() {
  const dpr = window.devicePixelRatio || 1;
  const r = canvas.getBoundingClientRect();
  canvas.width = Math.round(r.width * dpr);
  canvas.height = Math.round(r.height * dpr);
  draw();
}

const W = () => canvas.width / (window.devicePixelRatio || 1);
const H = () => canvas.height / (window.devicePixelRatio || 1);
const sx = (x) => W() / 2 + (x - state.view.x) * state.view.scale;
const sy = (y) => H() / 2 - (y - state.view.y) * state.view.scale;
const wx = (px) => state.view.x + (px - W() / 2) / state.view.scale;
const wy = (py) => state.view.y - (py - H() / 2) / state.view.scale;

function fitView() {
  const xs = [];
  const ys = [];
  const pointSets = state.scans.filter((s) => s.visible).map(worldPoints);
  if (!pointSets.length && live.scan) pointSets.push(livePoints());
  if (!pointSets.length && refmap.map) {
    const b = referenceBounds(refmap.map);
    pointSets.push([{ x: b.minX, y: b.minY }, { x: b.maxX, y: b.maxY }]);
  }
  for (const pts of pointSets) {
    for (const p of pts) {
      xs.push(p.x);
      ys.push(p.y);
    }
  }
  for (const e of state.scans.filter((s) => s.visible)) {
    xs.push(e.pose.x);
    ys.push(e.pose.y);
  }
  if (!xs.length) return draw();
  const minX = Math.min(...xs);
  const maxX = Math.max(...xs);
  const minY = Math.min(...ys);
  const maxY = Math.max(...ys);
  state.view.x = (minX + maxX) / 2;
  state.view.y = (minY + maxY) / 2;
  state.view.scale = 0.9 * Math.min(W() / Math.max(0.5, maxX - minX), H() / Math.max(0.5, maxY - minY));
  draw();
}

let frame = 0;
function draw() {
  if (!frame) frame = requestAnimationFrame(render);
}

function render() {
  frame = 0;
  const dpr = window.devicePixelRatio || 1;
  ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
  // Avec une grid map affichée, le fond prend la couleur « inconnu » : les zones libres
  // (blanches) ressortent au lieu de se confondre avec un fond blanc.
  ctx.fillStyle = $('showGrid').checked && state.grid ? `rgb(${UNKNOWN_RGB.join(',')})` : '#fbfbf9';
  ctx.fillRect(0, 0, W(), H());
  const s = state.view.scale;

  const showRef = $('showRefMap').checked && refmap.map;
  // Vue propre (style MapTool) : fond ardoise, sol blanc, murs noirs, segments rouges.
  const clean = $('showClean').checked && (state.gridClean || showRef);
  if (clean) {
    ctx.fillStyle = MAPTOOL.background;
    ctx.fillRect(0, 0, W(), H());
    ctx.imageSmoothingEnabled = false;
    if (showRef) {
      const g = refmap.map.plan ?? refmap.map.grid;
      refmap.cleanImage ??= cleanGridImage(g);
      ctx.globalAlpha = state.gridClean ? 0.55 : 1;
      ctx.drawImage(refmap.cleanImage, sx(g.originX), sy(g.originY + g.height * g.res), g.width * g.res * s, g.height * g.res * s);
      ctx.globalAlpha = 1;
    }
    if (state.gridClean) {
      const g = state.gridClean;
      state.gridCleanImage ??= cleanGridImage(g);
      ctx.drawImage(state.gridCleanImage, sx(g.originX), sy(g.originY + g.height * g.res), g.width * g.res * s, g.height * g.res * s);
    }
    if ($('showMetric').checked) drawMetricGrid();
    if (showRef && !state.vectorClean) strokeCleanSegments(refmap.map.segments);
    if (state.vectorClean) strokeCleanSegments(state.vectorClean.segments);
    drawCorners();
    drawMapLabels();
    drawGlobalCandidates();
    drawSlamTrack();
    for (const e of state.scans.filter((v) => v.visible)) drawRobot(e, e === state.selected);
    drawLive(s);
    renderStatus();
    return;
  }

  if (showRef) {
    // carte de référence en fond : sol blanc, murs gris, segments gris foncé
    const g = refmap.map.plan ?? refmap.map.grid;
    refmap.image ??= referenceImage(g);
    ctx.imageSmoothingEnabled = false;
    ctx.drawImage(refmap.image, sx(g.originX), sy(g.originY + g.height * g.res), g.width * g.res * s, g.height * g.res * s);
    ctx.strokeStyle = '#495057';
    ctx.lineWidth = 2;
    ctx.lineCap = 'round';
    strokeSegments(refmap.map.segments);
  }

  if ($('showGrid').checked && state.grid) {
    const g = state.grid;
    ctx.imageSmoothingEnabled = false;
    ctx.globalAlpha = showRef ? 0.6 : 0.9;
    ctx.drawImage(gridImage(), sx(g.originX), sy(g.originY + g.height * g.res), g.width * g.res * s, g.height * g.res * s);
    ctx.globalAlpha = 1;
  }

  if ($('showMetric').checked) drawMetricGrid();

  if ($('showRef').checked && state.reference) {
    ctx.save();
    ctx.setLineDash([6, 4]);
    ctx.strokeStyle = '#8a949c';
    ctx.lineWidth = 1.2;
    strokeSegments(state.reference.segments);
    ctx.restore();
  }

  if ($('showVector').checked && state.vector) {
    ctx.strokeStyle = '#d6336c';
    ctx.lineWidth = 3;
    ctx.lineCap = 'round';
    strokeSegments(state.vector.segments);
    ctx.fillStyle = '#d6336c';
    for (const seg of state.vector.segments) {
      dot(seg.x1, seg.y1, 3);
      dot(seg.x2, seg.y2, 3);
    }
  }

  const visible = state.scans.filter((e) => e.visible);
  const sel = state.selected;

  if ($('showRays').checked && sel && sel.visible) {
    ctx.strokeStyle = hexAlpha(sel.color, 0.18);
    ctx.lineWidth = 1;
    ctx.beginPath();
    for (const p of worldPoints(sel)) {
      ctx.moveTo(sx(sel.pose.x), sy(sel.pose.y));
      ctx.lineTo(sx(p.x), sy(p.y));
    }
    ctx.stroke();
  }

  if ($('showSegments').checked) {
    for (const e of visible) {
      ctx.strokeStyle = e.color;
      ctx.lineWidth = 2;
      strokeSegments(worldSegments(e));
    }
  }

  if ($('showPoints').checked) {
    const size = Math.max(2, Math.min(4, s / 40));
    for (const e of visible) {
      ctx.fillStyle = e.color;
      for (const p of worldPoints(e)) ctx.fillRect(sx(p.x) - size / 2, sy(p.y) - size / 2, size, size);
    }
  }

  for (const e of visible) drawRobot(e, e === sel);
  drawMapLabels();
  drawGlobalCandidates();
  drawSlamTrack();
  drawExplo();
  drawLive(s);
  renderStatus();
}

/** Trajectoire estimée par le SLAM : ligne bleue, scans clés en points, pose courante en cercle. */
function drawSlamTrack() {
  const t = state.slamTrack;
  if (!t?.length) return;
  ctx.save();
  ctx.strokeStyle = '#1c7ed6';
  ctx.lineWidth = 2.5;
  ctx.lineJoin = 'round';
  ctx.beginPath();
  t.forEach((p, k) => (k ? ctx.lineTo(sx(p.x), sy(p.y)) : ctx.moveTo(sx(p.x), sy(p.y))));
  ctx.stroke();
  ctx.fillStyle = '#1c7ed6';
  for (const p of t) {
    ctx.beginPath();
    ctx.arc(sx(p.x), sy(p.y), 2.5, 0, 2 * Math.PI);
    ctx.fill();
  }
  const last = state.slamLivePose ?? t.at(-1);
  ctx.strokeStyle = '#1c7ed6';
  ctx.lineWidth = 2;
  ctx.beginPath();
  ctx.arc(sx(last.x), sy(last.y), 9, 0, 2 * Math.PI);
  ctx.stroke();
  ctx.restore();
}

/** Candidates de la localisation globale : cercles numérotés (1 = pose retenue), orientés. */
function drawGlobalCandidates() {
  const cands = state.globalCandidates;
  if (!cands) return;
  ctx.save();
  ctx.textAlign = 'center';
  ctx.textBaseline = 'middle';
  ctx.font = '600 11px system-ui, sans-serif';
  cands.forEach((p, k) => {
    const x = sx(p.x);
    const y = sy(p.y);
    const a = p.theta * DEG;
    ctx.strokeStyle = k === 0 ? '#2b8a3e' : '#e8590c';
    ctx.lineWidth = 2;
    ctx.setLineDash(k === 0 ? [] : [4, 3]);
    ctx.beginPath();
    ctx.arc(x, y, 14, 0, 2 * Math.PI);
    ctx.moveTo(x + 14 * Math.cos(a), y - 14 * Math.sin(a));
    ctx.lineTo(x + 24 * Math.cos(a), y - 24 * Math.sin(a));
    ctx.stroke();
    ctx.setLineDash([]);
    ctx.fillStyle = 'rgba(255, 255, 255, 0.9)';
    ctx.beginPath();
    ctx.arc(x, y, 8, 0, 2 * Math.PI);
    ctx.fill();
    ctx.fillStyle = ctx.strokeStyle;
    ctx.fillText(String(k + 1), x, y + 0.5);
  });
  ctx.restore();
}

/** Étiquettes de la carte sémantique (case « Étiquettes ») : sur la carte propre, sinon sur la carte de référence. */
function drawMapLabels() {
  if (!$('showLabels').checked) return;
  const ref = $('showRefMap').checked && refmap.map;
  const labels = state.labels ?? (ref ? refmap.map.labels : null);
  const segments = state.vectorClean?.segments ?? (ref ? refmap.map.segments : []);
  if (!labels?.length) return;
  drawLabels(ctx, sx, sy, labelPositions(segments, labels));
}

/** Scan en temps réel : en noir, à la pose du scan sélectionné (cyan en vue propre, comme le scan de MapTool). */
function drawLive(s) {
  if (!$('showLive').checked || !live.scan) return;
  const stale = Date.now() - live.time > 3000; // plus de nouveau scan depuis 3 s
  const clean = $('showClean').checked && state.gridClean;
  ctx.fillStyle = clean ? (stale ? 'rgba(6,182,212,0.4)' : '#06b6d4') : stale ? 'rgba(29,35,39,0.3)' : '#1d2327';
  const size = Math.max(2.5, Math.min(4.5, s / 35));
  for (const p of livePoints()) ctx.fillRect(sx(p.x) - size / 2, sy(p.y) - size / 2, size, size);
  drawRobot({ pose: livePose(), color: '#ffffff' }, true);
}

/** Image de la grid map propre aux couleurs MapTool (inconnu transparent). */
function cleanGridImage(g) {
  const off = document.createElement('canvas');
  off.width = g.width;
  off.height = g.height;
  const octx = off.getContext('2d');
  const img = octx.createImageData(g.width, g.height);
  const color = maptoolColor(g);
  for (let j = 0; j < g.height; j++) {
    for (let i = 0; i < g.width; i++) {
      const rgb = color(g.index(i, j));
      if (!rgb) continue;
      const o = 4 * ((g.height - 1 - j) * g.width + i);
      img.data.set(rgb, o);
      img.data[o + 3] = 255;
    }
  }
  octx.putImageData(img, 0, 0);
  return off;
}

/**
 * Versions propres des cartes (recalculées à chaque reconstruction) : murs et objets fixes
 * (table, armoire...), voir cleanMaps. La carte vectorielle propre part de segments SANS
 * découpe par les absences : la découpe efface les meubles que des faisceaux traversent
 * (sous une table, entre ses pieds) avant qu'ils aient pu être reconnus.
 */
function refreshClean() {
  state.gridCleanImage = null;
  state.gridClean = null;
  state.gridLoc = null;
  state.vectorClean = null;
  state.corners = null;
  if (!state.grid) return;
  const vis = state.scans.filter((e) => e.visible);
  // Scans pris en hauteur (robot posé sur un meuble) : ils passent au-dessus des meubles et les
  // effaceraient. Les cartes propres sont faites avec les scans au sol ; les scans en hauteur
  // ne donnent que la frontière de la salle (les grands murs qu'ils voient par-dessus les meubles).
  const floor = vis.filter((e) => !e.scan.elevated);
  const high = floor.length > 0 && floor.length < vis.length;
  const base = high ? floor : vis;
  let grid = state.grid;
  if (high) {
    grid = gridFromScan(floor[0].scan, floor[0].pose, gridParams(), filterOpts(), { flipAngles: floor[0].flip });
    for (const e of floor.slice(1)) grid.integrateScan(...integrateOpts(e));
  }
  const noAbsence = (list) => {
    const m = new VectorMap([], { ...vecParams(), absence: false });
    for (const e of list) m.integrateScan(...integrateOpts(e));
    return m.segments;
  };
  let segments = [];
  if (state.vector) segments = state.vector.params.absence || high ? noAbsence(base) : state.vector.segments;
  const c = cleanMaps(grid, segments, base.map((e) => e.pose), {
    vector: { minObservations: Math.max(1, Number($('vecMinObs').value) || 1) },
    views: base.map((e) => ({ pose: e.pose, points: worldPoints(e) })), // meubles surélevés
    barriers: high && state.vector ? cleanVectorMap(noAbsence(vis)).segments : [],
    labels: state.labels, // carte sémantique : seuls les meubles nommés restent des objets
  });
  state.gridClean = c.grid; // plan (affichage)
  state.gridLoc = c.gridLoc; // grid map de localisation (figée avec la carte de référence)
  if (state.vector) {
    state.vectorClean = { segments: c.segments, dominant: c.dominant };
    state.corners = c.corners;
  }
}

/** Segments propres : murs en rouge (comme MapTool), objets fixes (meubles) en violet. */
function strokeCleanSegments(segs) {
  ctx.lineWidth = 3;
  ctx.lineCap = 'round';
  ctx.strokeStyle = MAPTOOL.segment;
  strokeSegments(segs.filter((s) => s.kind !== 'objet'));
  ctx.strokeStyle = MAPTOOL.object;
  strokeSegments(segs.filter((s) => s.kind === 'objet' && !s.raised));
  // meubles surélevés (le lidar passe dessous) : pointillés, comme sur un plan d'architecte
  ctx.setLineDash([9, 6]);
  ctx.lineCap = 'butt';
  strokeSegments(segs.filter((s) => s.kind === 'objet' && s.raised));
  ctx.setLineDash([]);
}

/** Coins (points d'intérêt) de la carte vectorielle propre : losanges orange, comme le tracé en cours de MapTool. */
function drawCorners() {
  if (!state.corners) return;
  ctx.fillStyle = '#f59e0b';
  ctx.strokeStyle = '#1e293b';
  ctx.lineWidth = 1.5;
  for (const c of state.corners) {
    const x = sx(c.x);
    const y = sy(c.y);
    ctx.beginPath();
    ctx.moveTo(x, y - 6);
    ctx.lineTo(x + 6, y);
    ctx.lineTo(x, y + 6);
    ctx.lineTo(x - 6, y);
    ctx.closePath();
    ctx.fill();
    ctx.stroke();
  }
}

// ------------------------------------------------------------------ carte de référence (jour 2)
// La carte de la salle construite au jour 1, figée : on y localise les nouveaux scans.

const refmap = { map: null, image: null, cleanImage: null, gridRef: null, vectorRef: null };

function setReferenceMap(map) {
  Object.assign(refmap, { map, image: null, cleanImage: null, gridRef: null, vectorRef: null });
  const d = new Date(map.date);
  const when = `${d.toLocaleDateString('fr-FR')} ${d.toLocaleTimeString('fr-FR', { hour: '2-digit', minute: '2-digit' })}`;
  $('refStat').textContent = `${map.name} · ${map.segments.length} segments, ${map.corners.length} coins, grid ${map.grid.width} × ${map.grid.height} (${Math.round(map.grid.res * 100)} cm) · ${map.clean ? 'version propre' : 'version brute'} · ${when}`;
  $('showRefMap').checked = true;
  // grid map de référence + multi-résolution : les 18 scans réels relocalisés à moins de 5 cm et
  // 2° depuis un départ à 29 cm et 8° (carte vectorielle : 15/18, elle n'a plus que les murs et
  // les trois meubles nommés)
  $('locRef').value = 'refgrid';
  fillMethods();
  $('locMethod').value = 'multires';
  if (!state.scans.length) fitView();
  draw();
}

/** Image de la carte de référence pour la vue normale : sol blanc, occupé gris, inconnu transparent. */
function referenceImage(g) {
  const off = document.createElement('canvas');
  off.width = g.width;
  off.height = g.height;
  const octx = off.getContext('2d');
  const img = octx.createImageData(g.width, g.height);
  for (let j = 0; j < g.height; j++) {
    for (let i = 0; i < g.width; i++) {
      const c = g.classify(g.index(i, j));
      if (c === 0) continue;
      const o = 4 * ((g.height - 1 - j) * g.width + i);
      img.data.set(c === -1 ? [255, 255, 255, 255] : isPartlyOccupied(g, g.index(i, j)) ? [193, 198, 204, 255] : [134, 142, 150, 255], o);
    }
  }
  octx.putImageData(img, 0, 0);
  return off;
}

async function loadProjectReference() {
  try {
    const res = await fetch('../data/reference/carte_reference.json', { cache: 'no-store' });
    if (!res.ok) throw new Error('fichier data/reference/carte_reference.json introuvable (node scripts/build-reference.mjs)');
    setReferenceMap(parseReferenceMap(await res.text()));
    toast('Carte de la salle chargée comme référence');
  } catch (err) {
    toast(`Chargement impossible : ${err.message}`);
  }
}

/** Les cartes actuelles (scans visibles) deviennent la référence, en version propre ou brute. */
function freezeReference() {
  const vis = state.scans.filter((e) => e.visible);
  if (!vis.length) return toast('Aucun scan visible pour construire la carte');
  if (!state.grid) buildGridAll(true);
  if (!state.vector) buildVectorAll(true);
  refreshClean();
  const clean = $('refClean').checked;
  setReferenceMap(
    makeReferenceMap({
      name: `Carte figée (${vis.length} scans)`,
      grid: clean ? state.gridLoc ?? state.gridClean : state.grid,
      plan: clean ? state.gridClean : null,
      segments: clean ? state.vectorClean.segments : state.vector.segments,
      corners: clean ? state.corners ?? [] : [],
      scans: vis.map((e) => ({ name: e.name, pose: e.pose, elevated: e.scan.elevated })),
      clean,
    }),
  );
  toast('Cartes figées comme référence : pense à « Enregistrer… »');
}

function saveReference() {
  if (!refmap.map) return toast('Aucune carte de référence à enregistrer');
  download('carte_reference.json', serializeReferenceMap(refmap.map), 'application/json');
}

// ------------------------------------------------------------------ localisation (jour 2)

/** Type de référence (grid, vector, scan) correspondant au choix de la liste. */
function locKind() {
  const v = $('locRef').value;
  return { imported: 'vector', refvector: 'vector', refgrid: 'grid' }[v] ?? v;
}

/** Méthodes proposées selon la référence choisie. */
function fillMethods() {
  const kind = locKind();
  const sel = $('locMethod');
  const prev = sel.value;
  sel.replaceChildren(...METHODS_BY_REFERENCE[kind].map((m) => new Option(METHODS[m].label, m)));
  if (METHODS_BY_REFERENCE[kind].includes(prev)) sel.value = prev;
}

/**
 * Référence de localisation pour le scan `target` : grid map ou carte vectorielle construites
 * avec les autres scans visibles (ou avec tous, case décochée), carte importée, ou scan précédent.
 */
function locReference(target) {
  const kind = $('locRef').value;
  if (kind === 'refgrid' || kind === 'refvector') {
    if (!refmap.map) throw new Error('Aucune carte de référence : « Charger la carte de la salle » ou « Figer les cartes actuelles »');
    if (kind === 'refgrid') return (refmap.gridRef ??= gridReference(refmap.map.grid));
    return (refmap.vectorRef ??= vectorReference(refmap.map.segments));
  }
  if (kind === 'imported') {
    if (!state.reference) throw new Error('Aucune carte importée : bouton « Référence… » de la carte vectorielle');
    return vectorReference(state.reference.segments);
  }
  const visible = state.scans.filter((e) => e.visible);
  if (kind === 'scan') {
    const k = visible.indexOf(target);
    const ref = k > 0 ? visible[k - 1] : visible.find((e) => e !== target);
    if (!ref) throw new Error('Il faut un autre scan visible comme référence');
    return scanReference(worldPoints(ref));
  }
  const used = $('locExclude').checked ? visible.filter((e) => e !== target) : visible;
  if (!used.length) throw new Error("Il faut d'autres scans visibles pour construire la carte");
  if (kind === 'grid') {
    const [first, ...rest] = used;
    const g = gridFromScan(first.scan, first.pose, gridParams(), filterOpts(), { flipAngles: first.flip });
    for (const e of rest) g.integrateScan(...integrateOpts(e));
    return gridReference(g);
  }
  const m = new VectorMap([], { ...vecParams(), absence: false });
  for (const e of used) m.integrateScan(...integrateOpts(e));
  return vectorReference(m.segments);
}

/**
 * Journal des localisations de la séance : une ligne par résultat (localiser, comparer, globale),
 * exporté en CSV par « Exporter les résultats » pour le compte rendu.
 */
function logLocalization(entry, method, init, r, extra = {}) {
  const ref = entry.scan.poseFound ? entry.filePose : null;
  const moved = poseError(r.pose, init);
  const err = ref ? poseError(r.pose, ref) : null;
  state.locLog.push({
    date: new Date().toLocaleString('fr-FR'),
    scan: entry.name,
    reference: $('locRef').selectedOptions[0]?.textContent ?? $('locRef').value,
    method,
    init,
    pose: r.pose,
    moved,
    err,
    inliers: r.inliers,
    time: r.time,
    ...extra,
  });
  $('locLogStat').textContent = `${state.locLog.length} résultat(s) enregistré(s) dans la séance`;
}

function exportLocLog() {
  if (!state.locLog.length) return toast('Aucun résultat : localiser d\'abord un scan');
  const f = (v, d = 4) => (Number.isFinite(v) ? v.toFixed(d) : '');
  const q = (t) => `"${String(t).replace(/"/g, '""')}"`;
  const lines = [
    'date,scan,reference,methode,depart_x_m,depart_y_m,depart_theta_deg,pose_x_m,pose_y_m,pose_theta_deg,correction_cm,correction_deg,ecart_pose_fichier_cm,ecart_pose_fichier_deg,points_sur_carte,temps_ms,confiance',
    ...state.locLog.map((l) =>
      [
        q(l.date), q(l.scan), q(l.reference), q(l.method),
        f(l.init.x), f(l.init.y), f(l.init.theta, 2), f(l.pose.x), f(l.pose.y), f(l.pose.theta, 2),
        f(l.moved.dist * 100, 1), f(l.moved.angle, 2), l.err ? f(l.err.dist * 100, 1) : '', l.err ? f(l.err.angle, 2) : '',
        f(l.inliers, 3), f(l.time, 0), q(l.confidence ?? ''),
      ].join(','),
    ),
  ];
  download('resultats_localisation.csv', lines.join('\n') + '\n', 'text/csv');
}

/** Tableau des résultats : déplacement depuis la pose initiale, écart à la pose du fichier, temps. */
function showLocResults(results, init, entry) {
  const ref = entry.scan.poseFound ? entry.filePose : null;
  const best = results.reduce((a, b) => (b.inliers > a.inliers ? b : a));
  const rows = results.map((r) => {
    const moved = poseError(r.pose, init);
    const err = ref ? poseError(r.pose, ref) : null;
    return `<tr class="${r === best && results.length > 1 ? 'best' : ''}"><td>${METHODS[r.method].label}</td>
      <td>${(moved.dist * 100).toFixed(1)} cm / ${moved.angle.toFixed(1)}°</td>
      ${ref ? `<td>${(err.dist * 100).toFixed(1)} cm / ${err.angle.toFixed(2)}°</td>` : ''}
      <td>${Math.round(r.inliers * 100)} %</td><td>${r.time.toFixed(0)} ms</td></tr>`;
  });
  $('locResult').innerHTML = `<table class="loc-table"><tr><th>Méthode</th><th>Correction</th>${ref ? '<th>Écart pose fichier</th>' : ''}<th>Points sur la carte</th><th>Temps</th></tr>${rows.join('')}</table>`;
}

function runLocalize() {
  const e = requireSelection();
  if (!e) return;
  let ref;
  try {
    ref = locReference(e);
  } catch (err) {
    return toast(err.message);
  }
  const init = { ...e.pose };
  state.globalCandidates = null;
  const r = localize(ref, local(e).pts, init, $('locMethod').value);
  e.pose = r.pose;
  poseChanged();
  showLocResults([r], init, e);
  logLocalization(e, METHODS[r.method].label, init, r);
  toast(`${e.name} localisé (${METHODS[r.method].label}) : ${Math.round(r.inliers * 100)} % des points sur la carte`);
}

/**
 * Localisation globale (étape 3) : pose du scan sélectionné retrouvée SANS pose de départ, dans
 * toute la salle (src/core/globalloc.js). Positions candidates : sol de la grid map de la
 * référence ; orientations : tirées de la direction des murs. Les trois meilleures candidates
 * sont dessinées (1 = retenue) ; si la deuxième est presque aussi bonne, la réponse est ambiguë.
 */
async function runGlobal() {
  const e = requireSelection();
  if (!e) return;
  let ref;
  try {
    ref = locReference(e);
  } catch (err) {
    return toast(err.message);
  }
  const grid = ref.grid ?? refmap.map?.grid ?? state.gridLoc ?? state.grid;
  if (!grid) return toast('Il faut une grid map : choisir « Carte de référence : grid map »');
  const walls = (refmap.map?.segments ?? state.vectorClean?.segments ?? []).filter((s) => s.kind !== 'objet');
  toast('Localisation globale : recherche dans toute la salle…');
  await tick();
  const init = { ...e.pose };
  const prior = $('globalPrior').checked ? { pose: init, sigmaXY: 0.5, sigmaTheta: 15 } : undefined;
  const r = globalLocalize(ref, local(e).pts, { grid, mapDominant: walls.length ? dominantDirection(walls) : undefined, prior });
  e.pose = r.pose;
  poseChanged();
  state.globalCandidates = r.candidates.slice(0, 3).map((c) => c.pose);
  logLocalization(e, 'Localisation globale', init, r, {
    confidence: `${r.ambiguous ? 'ambiguë' : 'fiable'} (${r.evidence?.best ?? 0} / ${r.evidence?.second ?? 0} points)`,
  });
  draw();
  const filePose = e.scan.poseFound ? e.filePose : null;
  const rows = r.candidates.slice(0, 3).map((c, k) => {
    const err = filePose ? poseError(c.pose, filePose) : null;
    return `<tr class="${k === 0 ? 'best' : ''}"><td>${k + 1}</td><td>(${c.pose.x.toFixed(2)} ; ${c.pose.y.toFixed(2)} ; ${c.pose.theta.toFixed(1)}°)</td>
      <td>${Math.round(c.inliers * 100)} %</td>${err ? `<td>${(err.dist * 100).toFixed(1)} cm / ${err.angle.toFixed(1)}°</td>` : ''}</tr>`;
  });
  const ev = r.evidence ?? { best: 0, second: 0 };
  const verdict = r.ambiguous
    ? `<strong>Ambiguë</strong> : ${ev.best} points pour la 1<sup>re</sup> candidate contre ${ev.second} pour la 2<sup>e</sup> (points qui les départagent) : salle presque symétrique depuis ce point de vue, à vérifier.`
    : `<strong>Fiable</strong> : ${ev.best} points pour la 1<sup>re</sup> candidate contre ${ev.second} pour la 2<sup>e</sup> (points qui les départagent).`;
  $('locResult').innerHTML = `<table class="loc-table"><tr><th>#</th><th>Pose</th><th>Points sur la carte</th>${filePose ? '<th>Écart pose fichier</th>' : ''}</tr>${rows.join('')}</table>
    <p>${verdict}</p><p class="hint">${r.positions} positions × ${r.angles} orientations (${r.mode === 'murs' ? 'tirées des murs' : 'tous les angles'}), ${Math.round(r.time)} ms. ${$('globalPrior').checked ? `A priori : pose de départ à ±50 cm, 15° (trouvée à ${(poseError(r.pose, init).dist * 100).toFixed(0)} cm de celle-ci).` : `Pose de départ ignorée (était à ${(poseError(r.pose, init).dist * 100).toFixed(0)} cm).`}</p>`;
  toast(`${e.name} : localisation globale ${r.ambiguous ? 'ambiguë' : 'fiable'}, ${Math.round(r.inliers * 100)} % des points sur la carte`);
}

// ------------------------------------------------------------------ SLAM (jour 3)

/**
 * SLAM sur les scans visibles, dans l'ordre de la liste (src/core/slam.js) : le premier scan fixe
 * le repère à sa pose actuelle, chaque suivant est localisé sur la carte déjà construite (suivi
 * local, relocalisation globale si besoin), ajouté s'il apporte du nouveau, et les boucles sont
 * fermées. Les poses manuelles des autres scans sont ignorées, sauf avec « Poses des scans comme
 * odométrie » : leur déplacement sert alors de prédiction et d'a priori. Fait pour des scans pris
 * le long d'un trajet (tous les 20 à 30 cm, au sol).
 */
async function runSlamScans() {
  const vis = state.scans.filter((e) => e.visible);
  if (vis.length < 2) return toast('Il faut au moins 2 scans visibles, dans l\'ordre du trajet');
  const slam = new Slam();
  const t0 = performance.now();
  // odométrie : déplacement entre les poses actuelles (approximatives) de deux scans successifs
  const approx = vis.map((e) => ({ ...e.pose }));
  const useOdom = $('slamOdom').checked;
  for (const [k, e] of vis.entries()) {
    const r = slam.addScan(e.scan, useOdom && k ? relative(approx[k - 1], approx[k]) : null, k === 0 ? e.pose : null);
    // après une fermeture de boucle, toute la trajectoire est corrigée
    slam.trajectory.forEach((t, i) => (vis[i].pose = { ...t.pose }));
    state.slamTrack = slam.trajectory.map((t) => t.pose);
    $('slamStat').textContent = `${k + 1}/${vis.length} · ${e.name} : ${r.mode} (${Math.round(100 * r.inliers)} % des points sur la carte)${r.loop ? ` · boucle fermée (correction ${Math.round(100 * r.loop.correction)} cm)` : ''}`;
    draw();
    await tick();
  }
  poseChanged();
  const count = (m) => slam.trajectory.filter((t) => t.mode === m).length;
  const lost = slam.trajectory.filter((t) => t.mode === 'perdu').map((t) => vis[t.index].name);
  $('slamStat').textContent = `SLAM terminé en ${((performance.now() - t0) / 1000).toFixed(1)} s : ${slam.keyframes.length} scans clés, ${count('suivi')} suivis, ${count('relocalisé')} relocalisés, ${slam.loops.length} boucle(s) fermée(s)${lost.length ? ` · perdus (pose douteuse) : ${lost.join(', ')}` : ''}`;
  toast(lost.length ? `${lost.length} scan(s) perdu(s) : trop loin du précédent ou trop différents` : 'SLAM terminé');
}

/**
 * SLAM en direct (robot en USB, relais live.py) : chaque scan reçu est localisé sur la carte en
 * cours ; quand le robot a assez bougé (20 cm ou 10°), le scan rejoint la liste à sa pose estimée
 * et la carte grandit. Il suffit de pousser ou de piloter doucement le robot.
 */
function slamLiveStep() {
  if (state.slamBusy || !live.scan) return;
  state.slamBusy = true;
  try {
    state.slam ??= new Slam();
    const slam = state.slam;
    const first = slam.keyframes.length ? null : state.selected?.pose ?? { x: 0, y: 0, theta: 0 };
    const prev = state.slamLivePose;
    // pilotage continu en USB : les touches maintenues donnent l'odométrie
    const driving = teleop.running && viaUsb();
    const r = slam.addScan(live.scan, driving ? takeDriveOdom() : null, first);
    state.slamLivePose = r.pose;
    if (driving) {
      afterDriveScan(r, slam);
    }
    // déplacement depuis le scan précédent, mesuré par le lidar : il doit rester dans la fenêtre de
    // suivi (±50 cm, ±20°) ; au-delà de 35 cm ou 15° par scan, on prévient
    const step = prev ? relative(prev, r.pose) : null;
    const stepCm = step ? 100 * Math.hypot(step.x, step.y) : 0;
    const fast = step && (stepCm > 35 || Math.abs(step.theta) > 15);
    if (r.keyframe) {
      const entry = addScan(parseScanCSV(live.text, captureName()));
      entry.pose = { ...r.pose };
      entry.slamIndex = r.index;
      select(entry);
    }
    if (r.loop) for (const e of state.scans) if (e.slamIndex != null) e.pose = { ...slam.trajectory[e.slamIndex].pose };
    state.slamTrack = slam.keyframes.map((kf) => kf.pose);
    if (r.keyframe || r.loop) poseChanged();
    const kf = slam.keyframes.at(-1).pose;
    const sinceKf = 100 * Math.hypot(r.pose.x - kf.x, r.pose.y - kf.y);
    $('slamStat').innerHTML = `SLAM en direct : pose ${fmtPose(r.pose)} · dernier pas ${step ? `${stepCm.toFixed(0)} cm / ${Math.abs(step.theta).toFixed(0)}°` : '–'}${fast ? ' · <strong class="warn">trop vite, ralentir</strong>' : ''} · ${sinceKf.toFixed(0)} cm depuis le dernier scan clé<br>${slam.keyframes.length} scans clés · dernier scan ${r.mode} (${Math.round(100 * r.inliers)} % sur la carte, ${Math.round(r.time)} ms)${slam.loops.length ? ` · ${slam.loops.length} boucle(s)` : ''}`;
    draw();
  } finally {
    state.slamBusy = false;
  }
}

function resetSlam() {
  Object.assign(state, { slam: null, slamTrack: null, slamLivePose: null });
  Object.assign(explo, { plan: null, truth: null, scene: null });
  Object.assign(wifi, { odom: null, lastCmd: null, prevPose: null });
  Object.assign(teleop, { odom: { ...ZERO }, calib: null, lost: false });
  for (const e of state.scans) delete e.slamIndex;
  $('slamStat').textContent = 'SLAM réinitialisé : le prochain scan en direct fixe le repère (à la pose du scan sélectionné).';
  draw();
}

// ------------------------------------------------------------------ exploration autonome (bonus du jour 3)
// src/core/explore.js : à chaque pas, scan → SLAM → frontière la plus proche → commande.
// En simulation (scènes de scripts/lib/scenes.mjs) ou avec le vrai robot en Wi-Fi
// (programme ESP32RobotControl : routes /scan, /forward, /rotateleft, /rotateright, /stop).

const explo = { running: false, mode: null, plan: null, scene: null, truth: null };
const sleep = (ms) => new Promise((r) => setTimeout(r, ms));

const EXPLO_SCENES = {
  salle: { name: 'CLASSROOM', start: { x: 1.5, y: 2.2, theta: 0 }, options: {} },
  pieces: { name: 'ROOMS', start: { x: 1.5, y: 1.5, theta: 0 }, options: { explore: { maxRange: 20 } } },
};

function exploStat(msg) {
  $('exploStat').textContent = msg;
}

/** Scène simulée (murs vrais en gris), chemin prévu (orange), zones frontières (vert), robot vrai (gris). */
function drawExplo() {
  if (!explo.scene && !explo.plan) return;
  ctx.save();
  if (explo.scene) {
    ctx.strokeStyle = 'rgba(29,35,39,0.35)';
    ctx.lineWidth = 1.5;
    ctx.setLineDash([5, 4]);
    strokeSegments(explo.scene);
    ctx.setLineDash([]);
  }
  const plan = explo.plan;
  if (plan && !plan.done) {
    ctx.strokeStyle = '#f08c00';
    ctx.lineWidth = 3;
    ctx.beginPath();
    plan.path.forEach((p, k) => (k ? ctx.lineTo(sx(p.x), sy(p.y)) : ctx.moveTo(sx(p.x), sy(p.y))));
    ctx.stroke();
    ctx.fillStyle = 'rgba(47,158,68,0.85)';
    for (const f of plan.frontiers) {
      ctx.beginPath();
      ctx.arc(sx(f.x), sy(f.y), 4 + Math.min(6, f.size / 10), 0, 2 * Math.PI);
      ctx.fill();
    }
  }
  if (explo.truth) drawRobot({ pose: explo.truth, color: '#adb5bd' }, false);
  ctx.restore();
}

function viewBox(segments) {
  const xs = segments.flatMap((s) => [s.x1, s.x2]);
  const ys = segments.flatMap((s) => [s.y1, s.y2]);
  state.view.x = (Math.min(...xs) + Math.max(...xs)) / 2;
  state.view.y = (Math.min(...ys) + Math.max(...ys)) / 2;
  state.view.scale = 0.9 * Math.min(W() / (Math.max(...xs) - Math.min(...xs) + 1), H() / (Math.max(...ys) - Math.min(...ys) + 1));
}

/** Scans clés de l'exploration ajoutés à la liste, à leur pose estimée (cartes, exports). */
function keepKeyframes(slam) {
  for (const kf of slam.keyframes) {
    const entry = addScan(kf.scan);
    entry.pose = { ...kf.pose };
    entry.exploSim = true;
  }
  renderScanList();
  poseChanged();
}

/** Exploration simulée, animée pas à pas. */
async function runExploSim() {
  if (explo.running) return stopExplo();
  const sc = EXPLO_SCENES[$('exploScene').value];
  const scenes = await import('../scripts/lib/scenes.mjs');
  // les scans d'une simulation précédente (autre scène) ne doivent pas se superposer à celle-ci
  state.scans = state.scans.filter((e) => !e.exploSim);
  if (state.selected?.exploSim) state.selected = null;
  renderScanList();
  setVector(null);
  resetSlam();
  Object.assign(explo, { running: true, mode: 'sim', plan: null, truth: null, scene: scenes[sc.name] });
  $('btnExploSim').textContent = 'Arrêter la simulation';
  viewBox(explo.scene);
  const t0 = performance.now();
  let last = null;
  for (const s of simulateExploration(explo.scene, sc.start, sc.options)) {
    if (!explo.running) break;
    last = s;
    Object.assign(explo, { plan: s.plan, truth: s.truth });
    state.slamTrack = s.slam.trajectory.map((t) => t.pose);
    state.slamLivePose = s.slam.last;
    const err = Math.hypot(s.truth.x - s.slam.last.x, s.truth.y - s.slam.last.y);
    exploStat(`Pas ${s.step} · ${s.distance.toFixed(1)} m parcourus · ${s.slam.keyframes.length} scans clés · ${s.plan.frontiers.length} zone(s) frontière · erreur de pose ${(100 * err).toFixed(1)} cm${s.collisions ? ` · ${s.collisions} collision(s)` : ''}`);
    setGrid(s.slam.grid);
    await tick();
  }
  explo.running = false;
  $('btnExploSim').textContent = 'Exploration simulée';
  if (last?.plan.done) {
    exploStat(`${$('exploStat').textContent} · terminé en ${((performance.now() - t0) / 1000).toFixed(0)} s : ${last.plan.reason}`);
    keepKeyframes(last.slam);
  }
  draw();
}

/** Requête au robot (route du programme ESP32RobotControl), délai de 3 s. */
async function robotGet(path, binary = false) {
  const ctrl = new AbortController();
  const timer = setTimeout(() => ctrl.abort(), 3000);
  try {
    const res = await fetch(`http://${$('robotIp').value.trim()}${path}`, { signal: ctrl.signal, cache: 'no-store' });
    if (!res.ok) throw new Error(`robot : ${path} → HTTP ${res.status}`);
    return binary ? await res.arrayBuffer() : await res.text();
  } catch (err) {
    throw err.name === 'AbortError' || err instanceof TypeError ? new Error(`robot injoignable (${path}) : Wi-Fi DON MIGUEL3 connecté ? IP ?`) : err;
  } finally {
    clearTimeout(timer);
  }
}

// Robot en Wi-Fi : un pas = (commande →) scan → SLAM. Le déplacement commandé sert d'odométrie
// (prédiction du SLAM), et le déplacement mesuré par le SLAM corrige les vitesses du robot
// (étalonnage : le robot n'a pas de codeurs, seule la durée des commandes est connue).
const wifi = { odom: null, lastCmd: null, prevPose: null, busy: false, speeds: null };

function robotSpeeds() {
  wifi.speeds = { linear: Number($('robotLin').value), angular: Number($('robotAng').value), lateral: wifi.speeds?.lateral ?? 0.2 };
  return wifi.speeds;
}

/** Scan Wi-Fi, SLAM avec l'odométrie de la dernière commande, étalonnage, scans clés dans la liste. */
async function wifiScanStep() {
  const scan = parseRobotBinary(await robotGet('/scan', true), captureName());
  const slam = (state.slam ??= new Slam());
  const first = slam.keyframes.length ? null : state.selected?.pose ?? { x: 0, y: 0, theta: 0 };
  const r = slam.addScan(scan, wifi.odom, first);
  let measured = null;
  if (wifi.lastCmd && wifi.prevPose) {
    measured = relative(wifi.prevPose, r.pose);
    if (r.mode === 'suivi' && r.inliers >= 0.7) {
      const sp = updateSpeeds(robotSpeeds(), wifi.lastCmd, measured);
      wifi.speeds = sp;
      $('robotLin').value = sp.linear.toFixed(3);
      $('robotAng').value = sp.angular.toFixed(0);
    }
  }
  wifi.odom = null;
  if (r.keyframe) {
    const entry = addScan(scan);
    entry.pose = { ...r.pose };
    entry.slamIndex = r.index;
    select(entry);
  }
  if (r.loop) for (const e of state.scans) if (e.slamIndex != null) e.pose = { ...slam.trajectory[e.slamIndex].pose };
  if (r.keyframe || r.loop) poseChanged();
  state.slamTrack = slam.keyframes.map((kf) => kf.pose);
  state.slamLivePose = r.pose;
  return { scan, r, slam, measured };
}

/** Envoie une commande au robot, garde son déplacement comme odométrie, attend la fin du mouvement. */
async function wifiMove(command) {
  const req = commandRequest(command, robotSpeeds());
  await robotGet(`/${req.route}?time=${req.time}`);
  wifi.lastCmd = req.command;
  wifi.odom = commandMotion(req.command);
  wifi.prevPose = state.slamLivePose ? { ...state.slamLivePose } : null;
  await sleep(req.duration + 400); // fin du mouvement, puis quelques tours de lidar robot arrêté
  return req;
}

const fmtPose = (p) => `(${p.x.toFixed(2)} ; ${p.y.toFixed(2)} ; ${p.theta.toFixed(0)}°)`;
// ------------------------------------------------------------------ pilotage continu (Wi-Fi)
// Le robot roule TANT QUE la touche (ou le bouton) est maintenue : l'appli renvoie la commande
// toutes les 150 ms, et le robot s'arrête seul 0,4 s après la dernière (touche lâchée, Wi-Fi
// perdu). C'est l'utilisateur qui guide : pas d'arrêt automatique devant un obstacle ni quand le SLAM
// se perd (un avertissement seulement). Pendant ce temps le SLAM tourne en continu (4 à 5 scans par seconde, robot en mouvement) :
// entre deux scans le robot ne bouge que de quelques centimètres, le suivi ne décroche pas.
// Odométrie : vitesse estimée × durée d'appui, intégrée entre deux scans ; elle s'étalonne sur des
// segments d'au moins 15 cm ou 25° d'un même mouvement, d'après le déplacement mesuré par le SLAM.

// route : commande Wi-Fi (ESP32_RobotWifi) ; usb : lettre envoyée par live.py (ESP32_RobotUSB)
const DRIVE = {
  avance: { route: 'forward', usb: 'F', kind: 'forward', sign: 1 },
  recul: { route: 'backward', usb: 'B', kind: 'forward', sign: -1 },
  glisseG: { route: 'left', usb: 'G', kind: 'strafe', sign: 1 },
  glisseD: { route: 'right', usb: 'D', kind: 'strafe', sign: -1 },
  gauche: { route: 'rotateleft', usb: 'Q', kind: 'rotate', sign: 1 },
  droite: { route: 'rotateright', usb: 'E', kind: 'rotate', sign: -1 },
};

/** Liaison avec le robot : 'usb' (câble, relais live.py) ou 'wifi'. */
const viaUsb = () => $('robotLink').value === 'usb';

/** Ordre moteur par le câble USB : le relais live.py le transmet à ESP32_RobotUSB. */
async function usbCmd(c) {
  let res;
  try {
    res = await fetch(`/live/cmd?c=${encodeURIComponent(c)}`, { cache: 'no-store' });
  } catch {
    throw new Error('relais USB injoignable : lancer « python scripts/live.py » et ouvrir l\'appli depuis son adresse');
  }
  if (res.status === 404) throw new Error('relais USB absent : lancer « python scripts/live.py » (au lieu de node serve.mjs)');
  if (!res.ok) throw new Error(`robot USB : ${await res.text()}`);
}

const robotStop = () => (viaUsb() ? usbCmd('X') : robotGet('/stop'));
const robotSetSpeed = (v) => (viaUsb() ? usbCmd(`V${v}`) : robotGet(`/speed?v=${v}`));
const KEEPALIVE_MS = 150; // renvoi de la commande pendant l'appui
const HOLD_MS = 400; // le robot s'arrête seul si plus rien n'arrive pendant ce temps
const ZERO = { x: 0, y: 0, theta: 0 };
const teleop = { running: false, active: null, since: 0, timer: null, odom: { ...ZERO }, calib: null, lost: false, rate: 0, pwm: 140, count: 0, t0: 0 };

/** Déplacement (repère du robot) d'un mouvement tenu pendant dt secondes, avec les vitesses estimées. */
function motionDelta(mv, dt) {
  const sp = wifi.speeds ?? robotSpeeds();
  if (mv.kind === 'rotate') return { x: 0, y: 0, theta: mv.sign * sp.angular * dt };
  if (mv.kind === 'strafe') return { x: 0, y: mv.sign * sp.lateral * dt, theta: 0 };
  return { x: mv.sign * sp.linear * dt, y: 0, theta: 0 };
}

/** Ajoute à l'odométrie (et au segment d'étalonnage) le mouvement tenu depuis le dernier appel. */
function integrateDrive(now = performance.now()) {
  const mv = teleop.active;
  if (mv) {
    const dt = (now - teleop.since) / 1000;
    teleop.odom = composePoses(teleop.odom, motionDelta(mv, dt));
    if (teleop.calib?.kind === mv.kind) {
      const sp = wifi.speeds ?? robotSpeeds();
      teleop.calib.amount += mv.sign * dt * (mv.kind === 'rotate' ? sp.angular : mv.kind === 'strafe' ? sp.lateral : sp.linear);
    }
  }
  teleop.since = now;
}

function sendDrive() {
  const mv = teleop.active;
  if (!mv) return;
  const req = viaUsb() ? usbCmd(mv.usb) : robotGet(`/${mv.route}?time=${mv.kind === 'rotate' ? 2 * HOLD_MS : HOLD_MS}`);
  req.catch((err) => {
    releaseDrive();
    toast(err.message);
  });
}

function pressDrive(id) {
  if (teleop.active?.id === id || explo.running) return;
  const mv = { ...DRIVE[id], id };
  if (!teleop.running) startTeleop();
  integrateDrive();
  // nouveau segment d'étalonnage si le type de mouvement change
  if (teleop.calib?.kind !== mv.kind) teleop.calib = { kind: mv.kind, amount: 0, start: state.slamLivePose ? { ...state.slamLivePose } : null };
  clearInterval(teleop.timer);
  teleop.active = mv;
  teleop.since = performance.now();
  sendDrive();
  teleop.timer = setInterval(sendDrive, KEEPALIVE_MS);
  for (const b of document.querySelectorAll('.drive button')) b.classList.toggle('on', b.id === `drive_${id}`);
}

function releaseDrive(id = null) {
  if (!teleop.active || (id && teleop.active.id !== id)) return;
  integrateDrive();
  clearInterval(teleop.timer);
  teleop.active = null;
  robotStop().catch(() => {});
  for (const b of document.querySelectorAll('.drive button')) b.classList.remove('on');
}

/** Odométrie des touches accumulée depuis le scan précédent (null si rien n'a bougé ou si décochée). */
function takeDriveOdom() {
  integrateDrive();
  const odom = teleop.odom;
  teleop.odom = { ...ZERO };
  const moved = Math.hypot(odom.x, odom.y) > 0 || odom.theta !== 0;
  return $('teleopOdom').checked && moved ? odom : null;
}

/** Après chaque scan du pilotage continu (Wi-Fi ou USB) : étalonnage, sécurité, état. */
function afterDriveScan(r, slam) {
  // étalonnage sur un segment assez long d'un même mouvement, scan bien recalé
  const cal = teleop.calib;
  if (cal?.start && r.mode === 'suivi' && r.inliers >= 0.7 && Math.abs(cal.amount) >= (cal.kind === 'rotate' ? 25 : 0.15)) {
    const command = cal.kind === 'rotate' ? { kind: 'rotate', angle: cal.amount } : { kind: cal.kind, distance: cal.amount };
    const sp = updateSpeeds(robotSpeeds(), command, relative(cal.start, r.pose));
    wifi.speeds = sp;
    $('robotLin').value = sp.linear.toFixed(3);
    $('robotAng').value = sp.angular.toFixed(0);
    teleop.calib = { kind: cal.kind, amount: 0, start: { ...r.pose } };
  } else if (cal && !cal.start) cal.start = { ...r.pose };
  // c'est l'utilisateur qui guide le robot : pas d'arrêt automatique, un simple avertissement
  if (r.mode === 'perdu' && !teleop.lost) toast('SLAM perdu : revenir doucement vers la zone connue, ou ralentir');
  teleop.lost = r.mode === 'perdu';
  teleop.count++;
  const now = performance.now();
  if (now - teleop.t0 > 2000) {
    teleop.rate = (1000 * teleop.count) / (now - teleop.t0);
    teleop.count = 0;
    teleop.t0 = now;
  }
  const sp = wifi.speeds ?? robotSpeeds();
  exploStat(`Pose ${fmtPose(r.pose)} · ${r.mode} (${Math.round(100 * r.inliers)} % sur la carte, ${Math.round(r.time)} ms) · ${teleop.rate.toFixed(1)} scans/s · ${slam.keyframes.length} scans clés · vitesses ${(100 * sp.linear).toFixed(0)} cm/s, ${sp.angular.toFixed(0)}°/s, latéral ${(100 * sp.lateral).toFixed(0)} cm/s`);
}

/** Boucle du SLAM en Wi-Fi : scans en continu, odométrie des commandes (en USB : slamLiveStep). */
async function teleopLoop() {
  while (teleop.running && !viaUsb()) {
    try {
      wifi.odom = takeDriveOdom();
      wifi.lastCmd = null; // l'étalonnage du pilotage continu se fait par segments
      const { r, slam } = await wifiScanStep();
      afterDriveScan(r, slam);
      draw();
    } catch (err) {
      releaseDrive();
      exploStat(`Arrêt : ${err.message}`);
      toast(err.message);
      stopTeleop();
    }
    await sleep(60);
  }
}

async function startTeleop() {
  if (teleop.running || explo.running) return;
  teleop.running = true;
  Object.assign(teleop, { count: 0, t0: performance.now() });
  $('btnTeleop').textContent = 'Arrêter le SLAM';
  robotSpeeds();
  if (viaUsb()) {
    // en USB, les scans arrivent du relais live.py : le SLAM en direct les traite (slamLiveStep)
    if (!live.source) startLive();
    $('slamLive').checked = true;
  }
  try {
    await robotSetSpeed(teleop.pwm);
  } catch (err) {
    toast(err.message);
  }
  if (!viaUsb()) teleopLoop();
}

function stopTeleop() {
  releaseDrive();
  teleop.running = false;
  if (viaUsb()) $('slamLive').checked = false;
  $('btnTeleop').textContent = 'Démarrer le SLAM';
}

/** Curseur de vitesse : nouvelle consigne au robot ; les vitesses estimées suivent la même proportion. */
function setDrivePwm(v) {
  const old = teleop.pwm;
  teleop.pwm = v;
  $('driveSpeedVal').textContent = String(v);
  if (wifi.speeds && old) {
    const k = v / old;
    wifi.speeds = { linear: wifi.speeds.linear * k, angular: wifi.speeds.angular * k, lateral: wifi.speeds.lateral * k };
    $('robotLin').value = wifi.speeds.linear.toFixed(3);
    $('robotAng').value = wifi.speeds.angular.toFixed(0);
  }
  robotSetSpeed(v).catch((err) => toast(err.message));
}

/**
 * Exploration avec le vrai robot : scan Wi-Fi → SLAM (avec le mouvement commandé comme
 * prédiction) → commande → attente de la fin du mouvement et d'un tour de lidar neuf. S'arrête si
 * le SLAM perd le robot (sécurité), sur « Arrêt » ou Échap, ou quand la carte est finie.
 */
async function runExploRobot() {
  if (explo.running) return stopExplo();
  if (wifi.busy) return;
  if (teleop.running) return toast('Arrêter d\'abord le SLAM (pilotage continu)');
  if (viaUsb()) return toast('Exploration autonome : en Wi-Fi seulement (choisir « Liaison : Wi-Fi »)');
  Object.assign(explo, { running: true, mode: 'robot', plan: null, truth: null, scene: null });
  const explorer = new Explorer();
  let steps = 0;
  try {
    while (explo.running) {
      const { scan, r, slam } = await wifiScanStep();
      if (r.mode === 'perdu') throw new Error('SLAM perdu : le scan ne colle plus à la carte, exploration arrêtée');
      const plan = explorer.next(slam.grid, slam.last, scanToLocalXY(scan, filterOpts()));
      explo.plan = plan;
      draw();
      if (plan.done) {
        exploStat(`Exploration terminée après ${steps} pas : ${plan.reason}.`);
        break;
      }
      exploStat(`Pas ${++steps} · ${slam.keyframes.length} scans clés · ${r.mode} (${Math.round(100 * r.inliers)} %) · pose ${fmtPose(r.pose)} · ${plan.frontiers.length} zone(s) frontière`);
      if (!explo.running) break;
      await wifiMove(plan.command);
    }
  } catch (err) {
    robotGet('/stop').catch(() => {});
    exploStat(`Arrêt : ${err.message}`);
    toast(err.message);
  } finally {
    explo.running = false;
    draw();
  }
}

function stopExplo() {
  releaseDrive();
  if (explo.mode === 'robot' || teleop.running) robotStop().catch((err) => toast(err.message));
  explo.running = false;
  exploStat('Exploration arrêtée.');
}

/** Toutes les méthodes depuis la même pose initiale ; la pose du scan n'est pas modifiée. */
async function runCompare() {
  const e = requireSelection();
  if (!e) return;
  let ref;
  try {
    ref = locReference(e);
  } catch (err) {
    return toast(err.message);
  }
  const kind = locKind();
  const init = { ...e.pose };
  const results = [];
  for (const m of METHODS_BY_REFERENCE[kind]) {
    toast(`Comparaison : ${METHODS[m].label}…`);
    await tick();
    results.push(localize(ref, local(e).pts, init, m));
    logLocalization(e, `${METHODS[m].label} (comparaison)`, init, results.at(-1));
  }
  showLocResults(results, init, e);
  toast('Comparaison terminée (pose du scan inchangée)');
}

/** Bouton « Exporter tout le jour 1 » : figures brutes et propres, cartes, fichiers MapTool, poses. */
async function exportDayOne() {
  const vis = state.scans.filter((e) => e.visible);
  if (!vis.length) return toast('Aucun scan visible à exporter');
  if (!state.grid) buildGridAll(true);
  if (!state.vector) buildVectorAll(true);
  refreshClean();
  toast('Préparation des figures…');
  await tick();
  const files = await dayOneExports({
    scans: vis.map((e) => ({ name: e.name, color: e.color, pose: e.pose, points: worldPoints(e), segments: worldSegments(e) })),
    grid: state.grid,
    gridClean: state.gridClean,
    vector: state.vector,
    vectorClean: state.vectorClean,
    corners: state.corners ?? [],
    labels: state.labels ?? [],
  });
  await saveFiles(files);
}

/** Points du scan en temps réel, dans le repère monde (à la pose du scan sélectionné). */
function livePoints() {
  return live.scan ? transformPoints(scanToLocalXY(live.scan, filterOpts()), livePose()) : [];
}

function drawMetricGrid() {
  const s = state.view.scale;
  const step = s > 25 ? 1 : 5;
  const x0 = Math.floor(wx(0) / step) * step;
  const x1 = wx(W());
  const y0 = Math.floor(wy(H()) / step) * step;
  const y1 = wy(0);
  ctx.lineWidth = 1;
  for (let x = x0; x <= x1; x += step) {
    ctx.strokeStyle = x === 0 ? 'rgba(29,35,39,0.35)' : 'rgba(29,35,39,0.07)';
    ctx.beginPath();
    ctx.moveTo(Math.round(sx(x)) + 0.5, 0);
    ctx.lineTo(Math.round(sx(x)) + 0.5, H());
    ctx.stroke();
  }
  for (let y = y0; y <= y1; y += step) {
    ctx.strokeStyle = y === 0 ? 'rgba(29,35,39,0.35)' : 'rgba(29,35,39,0.07)';
    ctx.beginPath();
    ctx.moveTo(0, Math.round(sy(y)) + 0.5);
    ctx.lineTo(W(), Math.round(sy(y)) + 0.5);
    ctx.stroke();
  }
}

function strokeSegments(segs) {
  ctx.beginPath();
  for (const g of segs) {
    ctx.moveTo(sx(g.x1), sy(g.y1));
    ctx.lineTo(sx(g.x2), sy(g.y2));
  }
  ctx.stroke();
}

function dot(x, y, r) {
  ctx.beginPath();
  ctx.arc(sx(x), sy(y), r, 0, 2 * Math.PI);
  ctx.fill();
}

function drawRobot(e, selected) {
  const { x, y, theta } = e.pose;
  const a = theta * DEG;
  const size = selected ? 13 : 10;
  ctx.save();
  ctx.translate(sx(x), sy(y));
  ctx.rotate(-a);
  ctx.beginPath();
  ctx.moveTo(size, 0);
  ctx.lineTo(-size * 0.6, size * 0.6);
  ctx.lineTo(-size * 0.6, -size * 0.6);
  ctx.closePath();
  ctx.fillStyle = e.color;
  ctx.fill();
  ctx.lineWidth = selected ? 2.5 : 1;
  ctx.strokeStyle = selected ? '#1d2327' : '#ffffff';
  ctx.stroke();
  ctx.restore();
}

function hexAlpha(hex, a) {
  const n = parseInt(hex.slice(1), 16);
  return `rgba(${n >> 16}, ${(n >> 8) & 255}, ${n & 255}, ${a})`;
}

function renderStatus() {
  const m = state.mouse;
  let txt = `échelle ${state.view.scale.toFixed(0)} px/m`;
  if (m) {
    txt = `x = ${m.x.toFixed(3)} m · y = ${m.y.toFixed(3)} m · ` + txt;
    const g = state.grid;
    if (g) {
      const [i, j] = g.worldToCell(m.x, m.y);
      if (g.inside(i, j)) txt += ` · cellule (${i}, ${j}) p = ${g.probability(g.index(i, j)).toFixed(2)}`;
    }
  }
  $('status').textContent = txt;
}

// ------------------------------------------------------------------ souris et clavier

let drag = null;

function eventWorld(ev) {
  const r = canvas.getBoundingClientRect();
  const px = ev.clientX - r.left;
  const py = ev.clientY - r.top;
  return { px, py, x: wx(px), y: wy(py) };
}

/** Scan visible dont un point (ou le robot) est à moins de `tol` pixels. */
function hitScan(px, py, tol = 7) {
  const order = [state.selected, ...state.scans.filter((e) => e !== state.selected)].filter((e) => e && e.visible);
  for (const e of order) {
    if (Math.hypot(sx(e.pose.x) - px, sy(e.pose.y) - py) < 14) return e;
    for (const p of worldPoints(e)) if (Math.hypot(sx(p.x) - px, sy(p.y) - py) < tol) return e;
  }
  return null;
}

canvas.addEventListener('mousedown', (ev) => {
  const m = eventWorld(ev);
  const sel = state.selected;
  if (ev.shiftKey && sel) {
    drag = { mode: 'rotate', entry: sel, a0: Math.atan2(m.y - sel.pose.y, m.x - sel.pose.x), t0: sel.pose.theta };
  } else {
    const hit = hitScan(m.px, m.py);
    if (hit) {
      if (hit !== sel) select(hit);
      drag = { mode: 'move', entry: hit, dx: hit.pose.x - m.x, dy: hit.pose.y - m.y };
    } else {
      drag = { mode: 'pan', px: m.px, py: m.py, vx: state.view.x, vy: state.view.y };
    }
  }
  canvas.classList.add('dragging');
});

window.addEventListener('mousemove', (ev) => {
  const m = eventWorld(ev);
  state.mouse = ev.target === canvas ? m : null;
  if (!drag) return renderStatus();
  if (drag.mode === 'pan') {
    state.view.x = drag.vx - (m.px - drag.px) / state.view.scale;
    state.view.y = drag.vy + (m.py - drag.py) / state.view.scale;
    draw();
  } else if (drag.mode === 'move') {
    drag.entry.pose.x = m.x + drag.dx;
    drag.entry.pose.y = m.y + drag.dy;
    poseChanged({ final: false });
  } else {
    const a = Math.atan2(m.y - drag.entry.pose.y, m.x - drag.entry.pose.x);
    drag.entry.pose.theta = normalizeAngleDeg(drag.t0 + (a - drag.a0) / DEG);
    poseChanged({ final: false });
  }
});

window.addEventListener('mouseup', () => {
  if (drag && drag.mode !== 'pan') poseChanged();
  drag = null;
  canvas.classList.remove('dragging');
});

canvas.addEventListener('wheel', (ev) => {
  ev.preventDefault();
  const m = eventWorld(ev);
  const f = Math.exp(-ev.deltaY * 0.0015);
  state.view.scale = Math.min(2000, Math.max(5, state.view.scale * f));
  // garder le point sous le curseur fixe
  state.view.x = m.x - (m.px - W() / 2) / state.view.scale;
  state.view.y = m.y + (m.py - H() / 2) / state.view.scale;
  draw();
}, { passive: false });

const DRIVE_KEYS = { i: 'avance', k: 'recul', j: 'glisseG', l: 'glisseD', u: 'gauche', o: 'droite' };

window.addEventListener('keyup', (ev) => {
  const id = DRIVE_KEYS[ev.key.toLowerCase()];
  if (id) releaseDrive(id);
});
window.addEventListener('blur', () => releaseDrive()); // fenêtre quittée : on lâche tout

window.addEventListener('keydown', (ev) => {
  if (ev.key === 'Escape' && (explo.running || teleop.active)) {
    stopExplo();
    return;
  }
  if (['INPUT', 'SELECT', 'TEXTAREA'].includes(document.activeElement?.tagName) || $('pasteDialog').open) return;
  // pilotage au clavier, mêmes touches que RobotControl : I/K avancer/reculer, J/L glisser, U/O
  // tourner ; le robot roule tant que la touche est maintenue (les répétitions sont ignorées)
  if ($('driveKeys').checked && DRIVE_KEYS[ev.key.toLowerCase()]) {
    ev.preventDefault();
    if (!ev.repeat) pressDrive(DRIVE_KEYS[ev.key.toLowerCase()]);
    return;
  }
  if (ev.key === ' ' && live.scan) {
    ev.preventDefault();
    liveCapture();
    return;
  }
  const e = state.selected;
  if (!e) return;
  const d = ev.shiftKey ? 0.1 : 0.01;
  const r = ev.shiftKey ? 5 : 0.5;
  const k = ev.key.toLowerCase();
  if (ev.key === 'ArrowLeft') e.pose.x -= d;
  else if (ev.key === 'ArrowRight') e.pose.x += d;
  else if (ev.key === 'ArrowUp') e.pose.y += d;
  else if (ev.key === 'ArrowDown') e.pose.y -= d;
  else if (k === 'q' || k === 'a') e.pose.theta = normalizeAngleDeg(e.pose.theta + r);
  else if (k === 'e') e.pose.theta = normalizeAngleDeg(e.pose.theta - r);
  else return;
  ev.preventDefault();
  poseChanged();
});

// ------------------------------------------------------------------ glisser-déposer

let dragDepth = 0;
window.addEventListener('dragenter', (ev) => {
  ev.preventDefault();
  dragDepth++;
  $('drop').hidden = false;
});
window.addEventListener('dragleave', () => {
  if (--dragDepth <= 0) $('drop').hidden = true;
});
window.addEventListener('dragover', (ev) => ev.preventDefault());
window.addEventListener('drop', (ev) => {
  ev.preventDefault();
  dragDepth = 0;
  $('drop').hidden = true;
  loadFiles([...ev.dataTransfer.files]);
});

// ------------------------------------------------------------------ boutons

$('fileScans').onchange = (ev) => {
  loadFiles([...ev.target.files]);
  ev.target.value = '';
};
$('btnSim').onclick = loadSimulated;
$('btnRoom').onclick = loadRoomScans;
$('btnTrajet').onclick = loadSimTrajet;
$('liveAdd').onclick = liveCapture;
$('btnAlign').onclick = runAutoAlign;
$('btnExportDay1').onclick = exportDayOne;
$('btnRefProject').onclick = loadProjectReference;
$('btnRefFreeze').onclick = freezeReference;
$('btnRefSave').onclick = saveReference;
$('refLoad').onchange = async (ev) => {
  const f = ev.target.files[0];
  ev.target.value = '';
  if (!f) return;
  try {
    setReferenceMap(parseReferenceMap(await readFile(f)));
    toast(`Carte de référence chargée : ${f.name}`);
  } catch (err) {
    toast(err.message);
  }
};
$('showRefMap').addEventListener('change', draw);
$('locRef').onchange = fillMethods;
$('btnLocalize').onclick = runLocalize;
$('btnCompare').onclick = runCompare;
$('btnGlobal').onclick = runGlobal;
$('btnLocExport').onclick = exportLocLog;
$('btnSlam').onclick = runSlamScans;
$('btnSlamReset').onclick = resetSlam;
$('btnExploSim').onclick = runExploSim;
$('btnExploRobot').onclick = runExploRobot;
$('btnExploStop').onclick = stopExplo;
for (const id of Object.keys(DRIVE)) {
  const b = $(`drive_${id}`);
  b.addEventListener('pointerdown', (ev) => {
    b.setPointerCapture(ev.pointerId);
    pressDrive(id);
  });
  for (const type of ['pointerup', 'pointercancel', 'lostpointercapture']) b.addEventListener(type, () => releaseDrive(id));
}
$('btnTeleop').onclick = () => (teleop.running ? stopTeleop() : startTeleop());
$('driveSpeed').addEventListener('change', () => setDrivePwm(Number($('driveSpeed').value)));
$('driveSpeed').addEventListener('input', () => ($('driveSpeedVal').textContent = $('driveSpeed').value));
$('slamLive').onchange = () => {
  $('slamStat').textContent = $('slamLive').checked
    ? 'SLAM en direct : déplacer doucement le robot ; un scan clé est ajouté tous les 20 cm ou 10°.'
    : 'SLAM en direct arrêté.';
  draw();
};
$('vecMinObs').onchange = () => {
  refreshClean();
  draw();
};
$('btnSaveAll').onclick = saveAll;
$('btnPaste').onclick = () => {
  $('pasteText').value = '';
  $('pasteDialog').showModal();
  $('pasteText').focus();
};
$('pasteOk').onclick = (ev) => {
  ev.preventDefault();
  $('pasteDialog').close();
  if ($('pasteText').value.trim()) addPastedText($('pasteText').value);
};
// Ctrl+V n'importe où hors des champs de saisie : ajoute directement le scan collé.
document.addEventListener('paste', (ev) => {
  if (['INPUT', 'SELECT', 'TEXTAREA'].includes(document.activeElement?.tagName) || $('pasteDialog').open) return;
  const text = ev.clipboardData?.getData('text');
  if (!text) return;
  ev.preventDefault();
  addPastedText(text);
});
$('btnFit').onclick = fitView;
$('btnShot').onclick = () => canvas.toBlob((b) => download('capture.png', b));

for (const id of ['poseX', 'poseY', 'poseT']) {
  $(id).addEventListener('input', () => {
    const e = state.selected;
    if (!e) return;
    const v = Number($(id).value);
    if (!Number.isFinite(v)) return;
    if (id === 'poseX') e.pose.x = v;
    else if (id === 'poseY') e.pose.y = v;
    else e.pose.theta = v;
    poseChanged();
  });
}
$('poseFlip').onchange = () => {
  if (!state.selected) return;
  state.selected.flip = $('poseFlip').checked;
  renderScanList();
  poseChanged();
};
$('poseHigh').onchange = () => {
  if (!state.selected) return;
  // enregistré dans le fichier du scan (« # pris en hauteur : ... ») à l'export
  state.selected.scan = setElevated(state.selected.scan, $('poseHigh').checked);
  renderScanList();
  poseChanged();
};
$('btnPoseReset').onclick = () => {
  if (!state.selected) return;
  state.selected.pose = { ...state.selected.filePose };
  poseChanged();
};
$('btnScanExport').onclick = () => {
  const e = requireSelection();
  if (e) download(e.name.replace(/\.(csv|txt)$/i, '') + '_pose.csv', serializeScanCSV(e.scan, e.pose), 'text/csv');
};

for (const id of ['fMin', 'fMax', 'fInt']) {
  $(id).addEventListener('change', () => {
    renderScanList();
    poseChanged();
  });
}

$('gridFromSel').onclick = () => {
  const e = requireSelection();
  if (!e) return;
  setGrid(gridFromScan(e.scan, e.pose, gridParams(), filterOpts(), { flipAngles: e.flip }));
  toast(`Grid map créée depuis ${e.name}`);
};
$('gridUpdateSel').onclick = () => {
  const e = requireSelection();
  if (!e) return;
  if (!state.grid) return $('gridFromSel').onclick();
  const st = state.grid.integrateScan(...integrateOpts(e));
  setGrid(state.grid);
  toast(`${e.name} intégré : ${st.freeCells} cellules libérées, ${st.occupiedCells} cellules d'impact`);
};
$('gridAll').onclick = () => buildGridAll();
$('gridClear').onclick = () => setGrid(null);
$('gridDecay').onclick = () => {
  if (!state.grid) return;
  const n = state.grid.decay(0.7, 1);
  setGrid(state.grid);
  toast(`Décroissance appliquée à ${n} cellules`);
};
$('gridIsolated').onclick = () => {
  if (!state.grid) return;
  const n = state.grid.removeIsolated();
  setGrid(state.grid);
  toast(`${n} cellules isolées retirées`);
};
$('gridExport').onclick = () => state.grid && download('grid_map.csv', state.grid.toCSV(), 'text/csv');
$('gridExportPng').onclick = () => state.grid && gridImage().toBlob((b) => download('grid_map.png', b));
$('gridExportRos').onclick = () => {
  if (!state.grid) return;
  const { pgm, yaml } = state.grid.toROS('grid_map');
  download('grid_map.pgm', new Blob([pgm], { type: 'image/x-portable-graymap' }));
  setTimeout(() => download('grid_map.yaml', yaml, 'text/yaml'), 300);
};
$('gridImport').onchange = async (ev) => {
  const f = ev.target.files[0];
  ev.target.value = '';
  if (!f) return;
  try {
    setGrid(GridMap.fromCSV(await readFile(f), gridParams()));
  } catch (err) {
    toast(err.message);
  }
};
$('gridRes').onchange = () => state.grid && buildGridAll();

$('vecFromSel').onclick = () => {
  const e = requireSelection();
  if (!e) return;
  const m = new VectorMap([], vecParams());
  m.integrateScan(...integrateOpts(e));
  setVector(m);
};
$('vecUpdateSel').onclick = () => {
  const e = requireSelection();
  if (!e) return;
  if (!state.vector) return $('vecFromSel').onclick();
  state.vector.params.absence = $('vecAbsence').checked;
  const st = state.vector.integrateScan(...integrateOpts(e));
  setVector(state.vector, `dernier scan : ${st.extracted} extraits, ${st.merged} fusions, ${st.added} ajouts, ${st.cuts} découpes`);
};
$('vecAll').onclick = () => buildVectorAll();
$('vecClear').onclick = () => setVector(null);
$('vecExport').onclick = () => state.vector && download('vector_map.txt', state.vector.toTXT());
$('vecImport').onchange = async (ev) => {
  const f = ev.target.files[0];
  ev.target.value = '';
  if (f) setVector(VectorMap.fromTXT(await readFile(f), vecParams()), 'importée');
};
$('refImport').onchange = async (ev) => {
  const f = ev.target.files[0];
  ev.target.value = '';
  if (!f) return;
  state.reference = VectorMap.fromTXT(await readFile(f));
  $('showRef').checked = true;
  draw();
};

for (const id of ['showPoints', 'showRays', 'showSegments', 'showGrid', 'showVector', 'showRef', 'showMetric', 'showLive', 'showClean', 'showLabels']) {
  $(id).addEventListener('change', draw);
}

window.addEventListener('resize', resize);
resize();
renderScanList();
fillMethods();
startLive();
