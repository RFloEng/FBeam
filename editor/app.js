// FBeam car editor prototype: three.js view + Pyodide running the real Python core.
// JavaScript only draws and edits the model; every engineering number comes from Python.
//
// Map of this file (sections in order):
//   Python engine     boot() loads Pyodide and FILES; change() / evaluate() / analyseSuspension() / redraw()
//   3D                scene, groups, axis helpers, drawScene() and the chassis drawing (tubes, jbeam, ghosts)
//   picking and tools screen-space picking, pointer events, onClick() per workspace
//   model edits       joints and tubes, mirroring (Symmetric), tools, keyboard, undo
//   panels            left lists, chassis inspector, budget bar, issues, timing
//   header actions    build + checker, save / open, SVJ export and import, SVJ glTF meshes, view options
//   components        mass boxes, full-car CG, driveline drawing and panel
//   body, aero        shell mesh, wing / splitter and their panels
//   suspension        corners over travel, pickup picking, charts, car dynamics (steering, roll)
//   window.fbeam      debug hook for automated UI tests
//
// State (module globals) and who writes it:
//   model            the design (nodes, members, suspension, components, aero, body, driveline, ...);
//                    the only thing saved, undone and sent to Python. Written inside change() callbacks,
//                    by loadExample / open / SVJ import, and by evaluate() for derived body and ballast boxes.
//   result           core.evaluate(): members, node masses, totals, CoG, catalogue, envelope (evaluate)
//   susp, drive      core.suspension() (corners, layouts, issues) and core.driveline() (analyseSuspension);
//                    susp.stale marks results older than the model (only re-run where they are shown)
//   mass, aeroRes, bodyRes   core.masses / aero / body (evaluate); *Error holds the Python message
//   rigid            core.rigidity(), on demand only (Compute rigidity button); rigid.stale after edits
//   svjBundle        meshes and bindings of the last SVJ import (kept in memory, not in the model)
//   workspace, tool  UI mode; selected (chassis tube / joint), selPoint + suspCorner (suspension),
//                    selComp (components), selAero (aero); pendingA, hoverPoint, anchor (drawing tools)
//
// Data flow: an edit calls change(fn) -> fn mutates model -> evaluate() sends JSON.stringify(model)
// to the fbeam_editor_core JSON functions -> result, susp, drive, mass, aeroRes, bodyRes -> redraw(),
// which rebuilds the 3D groups (draw*) and the panels from those results. Nothing is drawn from
// numbers JavaScript computes itself, except interpolation of Python's curves over wheel travel.
//
// Undo: change() pushes JSON.stringify(model) on undoStack (last 100) before fn runs; undo() pops it
// and re-evaluates. If Python raises in evaluate(), the last state is popped back automatically.
// The Tube tool pushes its own snapshot (one per click) because it chains several edits.
// Edits that return a new model from Python (layout switch, set_driveline, fill ballast) also go
// through change(), so they are undoable too.
//
// Adding a workspace: a data-space button in index.html; a left container if it needs a list
// (shown in drawLists); a THREE.Group added to the scene and drawn from drawScene(); a branch in
// drawLists, drawInspector, drawBudget, liveIssues, hint and onClick; and, if it needs Python
// results, a core.* call in evaluate(). Adding a panel field: render it in the workspace's
// inspector HTML, then bind its onchange to change(() => { model... = v; }) after setting innerHTML.
//
// Axes: model points are BeamNG / FBeam metres (X left, Y rear, Z up, origin mid-wheelbase on the
// ground); v3() maps them to three.js (Y up). Suspension points are left-side; the right is mirrored.
import * as THREE from 'three';
import { OrbitControls } from 'three/addons/controls/OrbitControls.js';
import { GLTFLoader } from 'three/addons/loaders/GLTFLoader.js';
import { ZipReader, BlobReader, TextWriter } from 'zipjs';   // reads single entries of the game's large vehicle zips

// repo files copied into Pyodide's file system under /fbeam (add new Python modules here)
const FILES = [
  'fbeam_checker/__init__.py', 'fbeam_checker/jbeam.py', 'fbeam_checker/sections.py',
  'fbeam_checker/submission.py', 'fbeam_checker/checks.py', 'rules/fbeam_v1.json',
  'examples/cage/cage_calc.py', 'fbeam_checker/frame.py', 'fbeam_checker/bending.py', 'fbeam_checker/pickups.py', 'fbeam_checker/buckling.py', 'editor/fbeam_frame.py', 'editor/fbeam_suspension.py',
  'editor/fbeam_svj.py', 'editor/fbeam_gltf.py', 'editor/fbeam_mass.py', 'editor/fbeam_dynamics.py', 'editor/fbeam_aero.py', 'editor/fbeam_vehicle.py', 'editor/fbeam_body.py', 'editor/fbeam_driveline.py', 'editor/fbeam_editor_core.py', 'editor/fbeam_beamng.py',
];
const REPO = new URL('../', import.meta.url);
// tube colours by failure mode (m.governs) and by role (m.role)
const COL = { yield: 0x2f6fdf, buckling: 0xe0782a, sel: 0x111111, frame: 0x7d8590, main: 0xcf222e, other: 0x8250df };
const SNAP = 0.01;   // m, grid for placed joints
const TYRE = { r: 0.297, w: 0.150 };   // spec tyre 165/70R14, drawn only (SPEC_TYRE in fbeam_suspension.py)

const $ = (id) => document.getElementById(id);
let py, core, vehpy, model, result = null, rigid = null, selected = null, drive = null;   // vehpy: fbeam_beamng (base car)
let tool = 'select', pendingA = null, hoverPoint = null, anchor = [0, 0, 0.2];
let workspace = 'chassis', susp = null, selPoint = null, suspCorner = 'front';
let mass = null, massError = null, selComp = null;
let aeroRes = null, aeroError = null, selAero = null;
let bodyRes = null, bodyError = null;
let svjBundle = null;      // meshes and bindings of the last imported SVJ (kept in memory, not in the model)
const undoStack = [];      // JSON snapshots of model, newest last (see change / undo)
const timing = {};         // ms per Python step, shown in the footer by drawTiming

// ---------- Python engine ----------
// Start-up, once: load Pyodide, copy FILES into its file system, import the core, load the full example.
async function boot() {
  const t0 = performance.now();
  py = await loadPyodide();
  timing.pyodide = performance.now() - t0;
  const t1 = performance.now();
  for (const f of FILES) {
    const res = await fetch(new URL(f, REPO), { cache: 'no-store' });   // always the current rules and code
    if (!res.ok) throw new Error(`cannot load ${f} (${res.status}); serve the repo root over http`);
    const parts = ('fbeam/' + f).split('/');
    for (let i = 1; i < parts.length; i++) {
      try { py.FS.mkdir('/' + parts.slice(0, i).join('/')); } catch (e) { /* exists */ }
    }
    py.FS.writeFile('/' + parts.join('/'), await res.text());
  }
  py.runPython("import sys; sys.path.insert(0, '/fbeam/editor')");
  core = py.pyimport('fbeam_editor_core');
  vehpy = py.pyimport('fbeam_beamng');
  timing.files = performance.now() - t1;
  loadExample('full');
  $('loading').remove();
  $('build').disabled = false;
  fitCamera();
}

// Replace the model with an example from the core (or an empty one) and clear undo; boot and the Example menu.
function loadExample(which) {
  const fn = { full: 'full_chassis_model', cage: 'cage_model' }[which];
  model = fn ? JSON.parse(py.runPython(`import json, fbeam_editor_core as c; json.dumps(c.${fn}())`))
             : { nodes: {}, members: [] };
  selected = null; pendingA = null; undoStack.length = 0;
  evaluate();
}

function change(fn) {                 // every model edit goes through here (undo + re-evaluate)
  undoStack.push(JSON.stringify(model));
  if (undoStack.length > 100) undoStack.shift();
  fn();
  evaluate();
}

// Run Python on the current model and refresh every derived result, then redraw. Called by change(),
// undo() and loads. Suspension and driveline (slow) only run in the workspaces that show them.
function evaluate() {
  const t = performance.now();
  try {
    result = JSON.parse(core.evaluate(JSON.stringify(model)));
  } catch (e) {
    showIssues([{ level: 'ERROR', rule: 'editor', message: pyError(e) }]);
    if (undoStack.length) { model = JSON.parse(undoStack.pop()); result = JSON.parse(core.evaluate(JSON.stringify(model))); }
    else return;
  }
  timing.evaluate = performance.now() - t;
  if (workspace === 'suspension' || workspace === 'components' || !susp) analyseSuspension(); else susp.stale = true;
  try { mass = JSON.parse(core.masses(JSON.stringify(model))); massError = null; } catch (e) { mass = null; massError = pyError(e); }
  try { aeroRes = (model.aero || []).length ? JSON.parse(core.aero(JSON.stringify(model))) : null; aeroError = null; }
  catch (e) { aeroRes = null; aeroError = pyError(e); }
  try { bodyRes = model.body ? JSON.parse(core.body(JSON.stringify(model))) : null; bodyError = null; }
  catch (e) { bodyRes = null; bodyError = pyError(e); }
  if (mass?.derived_body) {                               // body panel mass follows the shell (6.2-6.9)
    const d = mass.derived_body, c = (model.components || []).find((x) => x.id === d.id);
    if (c) Object.assign(c, { mass: d.mass, position: d.position, size: d.size, note: d.note, estimate: false });
    else (model.components = model.components || []).push({ id: d.id, kind: 'body', mass: d.mass, position: d.position, size: d.size, note: d.note, estimate: false });
  }
  for (const [id, d] of Object.entries(mass?.derived_ballast || {})) {      // ballast size and height follow its mass (6.10)
    const b = (model.components || []).find((c) => c.id === id);
    if (b) { b.size[2] = d.height; b.position[2] = d.z; }
  }
  if (rigid) rigid.stale = true;
  if (selected?.type === 'member' && !model.members.some((m) => m.id === selected.id)) selected = null;
  if (selected?.type === 'node' && !(selected.id in model.nodes)) selected = null;
  redraw();
}
// Suspension kinematics for every corner, then the driveline (which needs the corners' halfshaft points).
function analyseSuspension() {      // about 40 ms native, 250 ms in the browser: run only when needed
  const ts = performance.now();
  try { susp = JSON.parse(core.suspension(JSON.stringify(model))); }
  catch (e) { susp = { corners: {}, issues: [{ level: 'ERROR', rule: '7.1', message: pyError(e) }] }; }
  try { drive = model.driveline ? JSON.parse(core.driveline(JSON.stringify(model), JSON.stringify(susp))) : null; }
  catch (e) { drive = { shafts: [], diffs: [], issues: [{ level: 'ERROR', rule: '4.6', message: pyError(e) }] }; }
  timing.suspension = performance.now() - ts;
}
// Rebuild the 3D view and every panel from the current results (no Python calls except panel previews).
function redraw() { drawScene(); drawLists(); drawInspector(); drawBudget(); showIssues(liveIssues()); drawTiming(); }

// last line of a Python traceback, without the exception class
const pyError = (e) => String(e.message || e).trim().split('\n').pop().replace(/^\w+Error: /, '');

// ---------- 3D ----------
const view = $('view');
const renderer = new THREE.WebGLRenderer({ antialias: true, alpha: true });
renderer.setPixelRatio(window.devicePixelRatio);
view.appendChild(renderer.domElement);
const scene = new THREE.Scene();
const camera = new THREE.PerspectiveCamera(40, 1, 0.05, 100);
const controls = new OrbitControls(camera, renderer.domElement);
controls.enableDamping = true;
scene.add(new THREE.HemisphereLight(0xffffff, 0x8a8f99, 1.1));
const sun = new THREE.DirectionalLight(0xffffff, 1.4);
sun.position.set(3, 5, 2);
scene.add(sun);
scene.add(new THREE.GridHelper(6, 30, 0xb8c0c8, 0xdde2e7));
// one group per layer, cleared and rebuilt by its draw function: world (tubes, joints, jbeam) drawScene;
// ghosts (spec tyres, envelope, structure CoG) drawGhosts; overlay (work plane, rubber band) drawOverlay;
// suspG drawSuspension; meshG (imported SVJ glTF) loadSvjMeshes; compG drawComponents; aeroG drawAero;
// bodyG drawBody; driveG drawDriveline
const world = new THREE.Group(), ghosts = new THREE.Group(), overlay = new THREE.Group(), suspG = new THREE.Group();
const meshG = new THREE.Group(), compG = new THREE.Group(), aeroG = new THREE.Group(), bodyG = new THREE.Group(), driveG = new THREE.Group();
const fbeamRoot = new THREE.Group();   // every FBeam group; hidden in the base car workspace
fbeamRoot.add(world, ghosts, overlay, suspG, meshG, compG, aeroG, bodyG, driveG);
scene.add(fbeamRoot);

// BeamNG axes (X left, Y rear, Z up) <-> three.js (Y up)
const v3 = (p) => new THREE.Vector3(p[0], p[2], p[1]);
const bng = (v) => [v.x, v.z, v.y];
const snap = (p) => p.map((c) => Math.round(c / SNAP) * SNAP);

// keep the canvas and camera aspect in step with the view element
function resize() {
  const w = view.clientWidth, h = view.clientHeight;
  renderer.setSize(w, h);
  camera.aspect = w / Math.max(h, 1);
  camera.updateProjectionMatrix();
}
new ResizeObserver(resize).observe(view);

// aim the camera at the frame (or the envelope when empty); after loads and workspace switches
function fitCamera() {
  const box = new THREE.Box3().setFromObject(workspace === 'vehicle' && vehG.children.length ? vehG : world.children.length ? world : ghosts);
  const c = box.isEmpty() ? new THREE.Vector3(0, 0.6, 0) : box.getCenter(new THREE.Vector3());
  controls.target.copy(c);
  camera.position.set(c.x + 3.6, c.y + 2.2, c.z - 3.4);
  controls.update();
}

// tube colour for the Colour menu: failure mode, role, or a blue ramp of allowed load / kg per m
function colourFor(m) {
  const mode = $('colour').value;
  if (mode === 'mode') return COL[m.governs];
  if (mode === 'role') return COL[m.role];
  const vals = result.members.map((x) => (mode === 'load' ? x.beamDeform : x.kg_per_m));
  const lo = Math.min(...vals), hi = Math.max(...vals);
  const t = mode === 'load' ? m.beamDeform : m.kg_per_m;
  return new THREE.Color().setHSL(0.6, 0.65, 0.78 - 0.5 * (hi > lo ? (t - lo) / (hi - lo) : 0.5)).getHex();
}

// cylinder of radius r between two three.js points; shared by tubes, links and shafts
function tubeMesh(a, b, r, color, opacity = 1) {
  const dir = new THREE.Vector3().subVectors(b, a);
  const mesh = new THREE.Mesh(new THREE.CylinderGeometry(r, r, dir.length(), 14),
    new THREE.MeshStandardMaterial({ color, roughness: 0.5, metalness: 0.2, transparent: opacity < 1, opacity }));
  mesh.position.copy(a).addScaledVector(dir, 0.5);
  mesh.quaternion.setFromUnitVectors(new THREE.Vector3(0, 1, 0), dir.clone().normalize());
  return mesh;
}

// rebuild every 3D group; tubes drawn at 1.4 x real radius for visibility, dimmed outside Chassis
function drawScene() {
  world.clear();
  const mech = new Set(rigid && !rigid.stale ? rigid.jbeam.mechanism_joints || [] : []);
  const show = $('show').value, dim = workspace === 'chassis' ? 1 : 0.25;
  for (const m of result.members) {
    const isSel = selected?.type === 'member' && selected.id === m.id && workspace === 'chassis';
    if (show !== 'jbeam') {
      const r = (parseFloat(m.tube) / 2000) * 1.4 * (isSel ? 1.6 : 1);
      const mesh = tubeMesh(v3(model.nodes[m.a]), v3(model.nodes[m.b]), r, isSel ? COL.sel : colourFor(m),
        show === 'both' ? 0.3 * dim : dim);
      mesh.userData = { type: 'member', id: m.id };
      world.add(mesh);
    }
  }
  if (show !== 'tubes') drawJbeam();
  for (const [name, p] of Object.entries(model.nodes)) {
    if (show === 'jbeam' || workspace !== 'chassis') continue;
    const isSel = selected?.type === 'node' && selected.id === name;
    const orphan = !(name in result.node_mass);
    const rad = 0.014 + 0.006 * Math.sqrt(result.node_mass[name] || 0);
    const s = new THREE.Mesh(new THREE.SphereGeometry(rad, 16, 12), new THREE.MeshStandardMaterial({
      color: isSel ? 0xcf222e : orphan ? 0xffb000 : pendingA === name ? 0x1a7f37 : 0xf4f6f8, roughness: 0.4 }));
    s.position.copy(v3(p));
    s.userData = { type: 'node', id: name };
    world.add(s);
    if (mech.has(name)) {
      const halo = new THREE.Mesh(new THREE.SphereGeometry(rad * 2.4, 16, 12),
        new THREE.MeshBasicMaterial({ color: 0xe0782a, transparent: true, opacity: 0.28, depthWrite: false }));
      halo.position.copy(v3(p));
      world.add(halo);
    }
  }
  drawGhosts();
  drawOverlay();
  drawLegend();
  drawSuspension();
  drawComponents();
  drawDriveline();
  drawAero();
  drawBody();
  fbeamRoot.visible = workspace !== 'vehicle';
  drawVehicle();
}

// the jbeam the editor will export: one node per joint (sized by nodeWeight), one beam per straight tube
function drawJbeam() {
  const pos = [], col = [];
  for (const m of result.members) {
    const c = new THREE.Color(m.id === selected?.id && selected?.type === 'member' ? 0xffffff : colourFor(m));
    for (const n of [m.a, m.b]) { const v = v3(model.nodes[n]); pos.push(v.x, v.y, v.z); col.push(c.r, c.g, c.b); }
  }
  const g = new THREE.BufferGeometry();
  g.setAttribute('position', new THREE.Float32BufferAttribute(pos, 3));
  g.setAttribute('color', new THREE.Float32BufferAttribute(col, 3));
  world.add(new THREE.LineSegments(g, new THREE.LineBasicMaterial({ vertexColors: true, transparent: workspace !== 'chassis', opacity: 0.35 })));
  if ((result.bending || []).length) {                   // weightless bending beams (8.10), dashed
    const bp = [];
    for (const b of result.bending) for (const n of [b.a, b.b]) { const v = v3(model.nodes[n]); bp.push(v.x, v.y, v.z); }
    const bg = new THREE.BufferGeometry();
    bg.setAttribute('position', new THREE.Float32BufferAttribute(bp, 3));
    const lines = new THREE.LineSegments(bg, new THREE.LineDashedMaterial({ color: 0xa475f9, dashSize: 0.03, gapSize: 0.025,
      transparent: true, opacity: workspace === 'chassis' ? 0.7 : 0.2 }));
    lines.computeLineDistances();
    world.add(lines);
  }
  for (const [n, mass] of Object.entries(result.node_mass)) {
    const s = new THREE.Mesh(new THREE.SphereGeometry(0.008 + 0.0045 * Math.sqrt(mass), 10, 8),
      new THREE.MeshBasicMaterial({ color: selected?.type === 'node' && selected.id === n ? 0xcf222e : 0xffffff }));
    s.position.copy(v3(model.nodes[n]));
    s.userData = { type: 'node', id: n };
    world.add(s);
  }
}

// reference geometry: spec tyres at the hubs (when no suspension is solved), the length x width
// envelope box (1.25 m tall, 0.10 m up: a visual guide, not a rule) and the structure CoG
function drawGhosts() {
  ghosts.clear();
  if (!$('ghosts').checked || !result) return;
  const hasSusp = susp && Object.keys(susp.corners).length;
  for (const h of hasSusp ? [] : Object.values(model.hubs || {})) {           // spec tyre
    const w = new THREE.Mesh(new THREE.CylinderGeometry(TYRE.r, TYRE.r, TYRE.w, 36),
      new THREE.MeshStandardMaterial({ color: 0x8c959f, transparent: true, opacity: 0.35, depthWrite: false }));
    w.position.copy(v3(h));
    w.rotation.z = Math.PI / 2;
    ghosts.add(w);
  }
  const env = result.envelope, ax = model.axles;
  let ymid = 0;
  const hubs = Object.values(model.hubs || {});
  if (hubs.length) ymid = hubs.reduce((s, h) => s + h[1], 0) / hubs.length;          // wheel centres = axle lines
  else if (ax && ax.front && ax.rear && ax.front.concat(ax.rear).every((n) => model.nodes[n])) {
    const all = ax.front.concat(ax.rear);
    if (all.length) ymid = all.reduce((s, n) => s + model.nodes[n][1], 0) / all.length;
  }
  const box = new THREE.LineSegments(new THREE.EdgesGeometry(new THREE.BoxGeometry(env.width, 1.25, env.length)),
    new THREE.LineDashedMaterial({ color: 0x8c959f, dashSize: 0.06, gapSize: 0.04 }));
  box.position.set(0, 0.10 + 1.25 / 2, ymid);
  box.computeLineDistances();
  ghosts.add(box);
  const cog = result.cog;
  if (cog && workspace === 'chassis' && !(model.components || []).length) {
    const c = new THREE.Mesh(new THREE.SphereGeometry(0.06, 20, 14),
      new THREE.MeshBasicMaterial({ color: 0xffd33d, depthTest: false, transparent: true, opacity: 0.95 }));
    c.renderOrder = 10;
    c.position.copy(v3([cog.x, cog.y, cog.z]));
    ghosts.add(c);
    const line = new THREE.Line(new THREE.BufferGeometry().setFromPoints([v3([cog.x, cog.y, cog.z]), v3([cog.x, cog.y, 0])]),
      new THREE.LineDashedMaterial({ color: 0xbf8700, dashSize: 0.04, gapSize: 0.03 }));
    line.computeLineDistances();
    ghosts.add(line);
  }
}

// work plane, hover marker and the rubber-band tube while drawing
function workPlane() {   // plane through anchor, normal to the chosen axis
  const n = { top: [0, 1, 0], side: [1, 0, 0], front: [0, 0, 1] }[$('plane').value];
  const normal = new THREE.Vector3(...n);
  return new THREE.Plane(normal, -normal.dot(v3(anchor)));
}

function drawOverlay() {   // on every pointer move while the Joint or Tube tool is active
  overlay.clear();
  if (tool !== 'joint' && tool !== 'tube') return;
  const pl = new THREE.Mesh(new THREE.PlaneGeometry(6, 6, 30, 30),
    new THREE.MeshBasicMaterial({ color: 0x2f6fdf, transparent: true, opacity: 0.06, side: THREE.DoubleSide, depthWrite: false }));
  const grid = new THREE.LineSegments(new THREE.WireframeGeometry(new THREE.PlaneGeometry(6, 6, 30, 30)),
    new THREE.LineBasicMaterial({ color: 0x2f6fdf, transparent: true, opacity: 0.12 }));
  for (const obj of [pl, grid]) {
    const mode = $('plane').value;
    if (mode === 'top') obj.rotation.x = -Math.PI / 2;
    if (mode === 'side') obj.rotation.y = Math.PI / 2;
    obj.position.copy(v3(anchor));
    overlay.add(obj);
  }
  if (hoverPoint) {
    const g = new THREE.Mesh(new THREE.SphereGeometry(0.02, 12, 10), new THREE.MeshBasicMaterial({ color: 0x1a7f37 }));
    g.position.copy(v3(hoverPoint));
    overlay.add(g);
    if (tool === 'tube' && pendingA && model.nodes[pendingA]) {
      const role = $('newrole').value;
      overlay.add(tubeMesh(v3(model.nodes[pendingA]), v3(hoverPoint),
        parseFloat(result.default_tube[role]) / 2000 * 1.4, COL[role], 0.55));
    }
  }
}

// legend for the Colour menu, under the view
function drawLegend() {
  const mode = $('colour').value;
  const sw = (c, t) => `<span class="sw" style="background:#${c.toString(16).padStart(6, '0')}"></span>${t}`;
  $('legend').innerHTML = {
    mode: sw(COL.yield, 'yield-governed') + sw(COL.buckling, 'buckling-governed'),
    role: sw(COL.frame, 'frame') + sw(COL.main, 'cage main') + sw(COL.other, 'cage other'),
  }[mode] || `<span class="sw" style="background:hsl(216,65%,78%)"></span>low<span class="sw" style="background:hsl(216,65%,28%)"></span>high ${mode === 'load' ? 'allowed load' : 'kg/m'}`;
  $('legend').innerHTML += `<span class="sw" style="background:#ffd33d"></span>CoG`;
}

// ---------- picking and tools ----------
const ray = new THREE.Raycaster();
function pointerRay(e) {   // set the shared raycaster from a pointer event
  camera.updateMatrixWorld();
  const rect = renderer.domElement.getBoundingClientRect();
  ray.setFromCamera(new THREE.Vector2(((e.clientX - rect.left) / rect.width) * 2 - 1,
    -((e.clientY - rect.top) / rect.height) * 2 + 1), camera);
}
const SNAP_PX = 14;
function nearestJoint(e) {             // joints snap to the pointer in screen space, not by hitting the sphere
  camera.updateMatrixWorld();
  const r = renderer.domElement.getBoundingClientRect();
  let best = null, bestD = SNAP_PX;
  for (const [name, p] of Object.entries(model.nodes)) {
    const q = v3(p).project(camera);
    if (q.z > 1) continue;
    const d = Math.hypot(r.left + (q.x + 1) / 2 * r.width - e.clientX, r.top + (1 - q.y) / 2 * r.height - e.clientY);
    if (d < bestD) { best = name; bestD = d; }
  }
  return best && { name: best, d: bestD };
}
const TUBE_PX = 8;
function nearestTube(e) {              // tubes are picked by screen distance too, so thin tubes are easy to hit
  camera.updateMatrixWorld();
  const r = renderer.domElement.getBoundingClientRect();
  const scr = (p) => { const q = v3(p).project(camera); return [r.left + (q.x + 1) / 2 * r.width, r.top + (1 - q.y) / 2 * r.height, q.z]; };
  let best = null, bestD = TUBE_PX;
  for (const m of model.members) {
    const A = scr(model.nodes[m.a]), B = scr(model.nodes[m.b]);
    if (A[2] > 1 || B[2] > 1) continue;
    const dx = B[0] - A[0], dy = B[1] - A[1], L2 = dx * dx + dy * dy || 1;
    const t = Math.min(1, Math.max(0, ((e.clientX - A[0]) * dx + (e.clientY - A[1]) * dy) / L2));
    const d = Math.hypot(A[0] + t * dx - e.clientX, A[1] + t * dy - e.clientY);
    if (d < bestD) { bestD = d; best = { m, t }; }
  }
  if (!best) return null;
  const pa = model.nodes[best.m.a], pb = model.nodes[best.m.b];
  return { object: { userData: { type: 'member', id: best.m.id } }, point: v3(pa.map((c, i) => c + (pb[i] - c) * best.t)), d: bestD };
}
function pick(e, type) {                // the closer of joint and tube wins; joints get a 6 px head start
  const j = type !== 'member' ? nearestJoint(e) : null;
  const t = type !== 'node' ? nearestTube(e) : null;
  if (j && (!t || j.d <= t.d + 6)) return { object: { userData: { type: 'node', id: j.name } }, point: v3(model.nodes[j.name]) };
  return t;
}
function planePoint(e) {   // snapped model point where the pointer meets the work plane, or null
  pointerRay(e);
  const p = new THREE.Vector3();
  return ray.ray.intersectPlane(workPlane(), p) ? snap(bng(p)) : null;
}

// a click is a pointerup within 4 px of its pointerdown, so orbit drags do not edit
let down = null;
renderer.domElement.addEventListener('pointerdown', (e) => { down = [e.clientX, e.clientY, e.button]; });
renderer.domElement.addEventListener('contextmenu', (e) => { e.preventDefault(); });
renderer.domElement.addEventListener('pointermove', (e) => {
  if (!result || workspace !== 'chassis' || (tool !== 'joint' && tool !== 'tube')) return;
  const hit = pick(e, 'node');
  hoverPoint = hit ? model.nodes[hit.object.userData.id] : planePoint(e);
  drawOverlay();
});
renderer.domElement.addEventListener('pointerup', (e) => {
  if (!down || Math.hypot(e.clientX - down[0], e.clientY - down[1]) > 4 || !result) return;
  if (down[2] === 2) { pendingA = null; drawScene(); hint(); return; }   // right click ends a tube chain
  onClick(e);
});

// click in the view: pick and act according to the workspace and, in Chassis, the active tool
function onClick(e) {
  if (workspace === 'suspension') { selPoint = nearestPickup(e); if (selPoint) suspCorner = selPoint.corner; redraw(); return; }
  if (workspace === 'components') { selComp = pickComponent(e); redraw(); return; }
  if (workspace === 'aero') { pointerRay(e); const h = ray.intersectObjects(aeroG.children).find((x) => x.object.userData.aero); selAero = h ? h.object.userData.aero : null; redraw(); return; }
  const hit = pick(e, tool === 'split' ? 'member' : undefined);
  const ud = hit?.object.userData;
  if (tool === 'select') return select(ud || null);
  if (tool === 'joint') {
    if (ud?.type === 'node') return select(ud);
    const p = planePoint(e);
    if (p) change(() => { const n = addJoint(p); selected = { type: 'node', id: n }; anchor = model.nodes[n]; });
    return;
  }
  if (tool === 'tube') {
    let target = ud?.type === 'node' ? ud.id : null;
    if (!target) {
      const p = planePoint(e);
      if (!p) return;
      undoStack.push(JSON.stringify(model));
      if (undoStack.length > 100) undoStack.shift();
      target = addJoint(p);
    } else if (pendingA) {
      undoStack.push(JSON.stringify(model));
      if (undoStack.length > 100) undoStack.shift();
    }
    if (pendingA && pendingA !== target) addTube(pendingA, target);
    pendingA = target;
    anchor = model.nodes[target];
    evaluate(); hint();
    return;
  }
  if (tool === 'split' && ud?.type === 'member') return change(() => splitTube(ud.id, bng(hit.point)));
  if (tool === 'delete' && ud) return change(() => (ud.type === 'node' ? deleteJoint(ud.id) : deleteTube(ud.id)));
}

function select(s) {   // chassis selection from the view or the lists; moves the work plane to a joint
  selected = s;
  if (s?.type === 'node') anchor = model.nodes[s.id];
  drawScene(); drawLists(); drawInspector();
}

// ---------- model edits (geometry only; no engineering numbers here) ----------
const SIDE = /^([a-z]+\d+)(ll|rr|l|r)$/, SWAP = { ll: 'rr', rr: 'll', l: 'r', r: 'l' };   // also BeamNG names: f10ll / f10rr, fx2l / fx2r
const mirrorOf = (n) => n.endsWith('_L') ? n.slice(0, -2) + '_R' : n.endsWith('_R') ? n.slice(0, -2) + '_L'
  : SIDE.test(n) ? n.replace(SIDE, (_, b, s) => b + SWAP[s]) : n;
const sym = () => $('mirror').checked;
const nextId = () => model.members.reduce((m, x) => Math.max(m, x.id), -1) + 1;
const hasTube = (a, b) => model.members.some((m) => (m.a === a && m.b === b) || (m.a === b && m.b === a));
function nextJoint() {   // next free J<n> name (mirrored pairs get _L / _R)
  let k = 0;
  for (const n of Object.keys(model.nodes)) { const m = /^J(\d+)/.exec(n); if (m) k = Math.max(k, +m[1]); }
  return 'J' + (k + 1);
}
function mirrorMember(m) {   // the tube's mirror twin, or null (centre-plane tube or none)
  const a = mirrorOf(m.a), b = mirrorOf(m.b);
  if ((a === m.a && b === m.b) || (a === m.b && b === m.a)) return null;     // lies on the centre plane
  return model.members.find((x) => x.id !== m.id && ((x.a === a && x.b === b) || (x.a === b && x.b === a))) || null;
}

// new joint at p, mirrored if Symmetric (snapped to the centre plane within 10 mm); returns the clicked side
function addJoint(p) {
  const base = nextJoint();
  if (!sym() || Math.abs(p[0]) < 0.01) {
    const q = sym() ? [0, p[1], p[2]] : p;
    model.nodes[base] = q;
    return base;
  }
  const L = base + '_L', R = base + '_R', ax = Math.abs(p[0]);
  model.nodes[L] = [ax, p[1], p[2]];
  model.nodes[R] = [-ax, p[1], p[2]];
  return p[0] > 0 ? L : R;
}

// new tube a-b with the New tube role and its default section, plus its mirror if Symmetric
function addTube(a, b) {
  const role = $('newrole').value;
  const id = nextId();
  const group = `Tube ${id}`;
  if (!hasTube(a, b)) model.members.push({ id, group, a, b, role, tube: result.default_tube[role] });
  if (sym()) {
    const ma = mirrorOf(a), mb = mirrorOf(b);
    const self = (ma === a && mb === b) || (ma === b && mb === a);
    if (!self && model.nodes[ma] && model.nodes[mb] && !hasTube(ma, mb)) {
      model.members.push({ id: nextId(), group, a: ma, b: mb, role, tube: result.default_tube[role] });
    }
  }
}

function splitOne(m, t, name) {   // joint `name` at fraction t along m; m becomes two tubes
  const pa = model.nodes[m.a], pb = model.nodes[m.b];
  model.nodes[name] = snap(pa.map((c, i) => c + (pb[i] - c) * t));
  model.members.push({ ...m, id: nextId(), a: name, b: m.b });
  m.b = name;
}
// Split tool: joint at the clicked point (kept 5-95 % along), on the mirror twin too if Symmetric
function splitTube(id, point) {
  const m = model.members.find((x) => x.id === id);
  const pa = model.nodes[m.a], pb = model.nodes[m.b];
  const d = pb.map((c, i) => c - pa[i]);
  let t = d.reduce((s, c, i) => s + c * (point[i] - pa[i]), 0) / d.reduce((s, c) => s + c * c, 0);
  t = Math.min(0.95, Math.max(0.05, t));
  const twin = sym() ? mirrorMember(m) : null;
  const base = nextJoint();
  if (!twin) { splitOne(m, t, base); selected = { type: 'node', id: base }; return; }
  const x = pa[0] + d[0] * t;
  const mine = x >= 0 ? base + '_L' : base + '_R';
  const tt = twin.a === mirrorOf(m.a) ? t : 1 - t;
  splitOne(m, t, mine);
  splitOne(twin, tt, mirrorOf(mine));
  selected = { type: 'node', id: mine };
}

function deleteJoint(n) {   // joint and its tubes (and its mirror if Symmetric)
  for (const j of new Set([n, sym() ? mirrorOf(n) : n])) {
    delete model.nodes[j];
    model.members = model.members.filter((m) => m.a !== j && m.b !== j);
  }
  if (pendingA && !model.nodes[pendingA]) pendingA = null;
}
function deleteTube(id) {   // tube and its mirror twin if Symmetric
  const m = model.members.find((x) => x.id === id);
  const twin = sym() ? mirrorMember(m) : null;
  model.members = model.members.filter((x) => x.id !== id && x.id !== twin?.id);
}

function setTool(t) {   // toolbar buttons and shortcuts; also reset to select on workspace switch
  tool = t;
  pendingA = null; hoverPoint = null;
  document.querySelectorAll('#toolbar [data-tool]').forEach((b) => b.classList.toggle('on', b.dataset.tool === t));
  renderer.domElement.style.cursor = t === 'select' ? '' : 'crosshair';
  hint(); if (result) drawScene();
}
function hint() {   // one-line help under the toolbar for the workspace and tool
  if (workspace === 'vehicle') { $('hint').textContent = 'A vanilla vehicle from your install: change parts and tuning on the right, then save the configuration. Drag to orbit.'; return; }
  if (workspace === 'suspension') { $('hint').textContent = 'Click a pickup point to move it. Drag the slider to move both wheels through their travel.'; return; }
  if (workspace === 'components') { $('hint').textContent = 'Click a component box to move or resize it. The yellow ball is the whole-car CG.'; return; }
  if (workspace === 'body') { $('hint').textContent = 'Edit the shell on the right. Its skin area and panel material set the body mass and CG.'; return; }
  if (workspace === 'aero') { $('hint').textContent = 'Click the wing or splitter to edit it. The dashed box is the aero limit (7.4): body width, roof height, bumpers.'; return; }
  $('hint').textContent = {
    select: 'Click a tube or joint to inspect it. Drag to orbit, right-drag to pan, wheel to zoom.',
    joint: 'Click on the work plane to place a joint (mirrored if Symmetric is on). Click a joint to move the plane there.',
    tube: pendingA ? `From ${pendingA}: click a joint or the plane to draw a tube. Right-click or Esc to finish.`
                   : 'Click a joint (or the plane) to start a tube.',
    split: 'Click a tube to insert a welded joint on it. A joint with no other tube does not brace it (8.9.7).',
    delete: 'Click a joint or tube to delete it (and its mirror).',
  }[tool];
}
document.querySelectorAll('#toolbar [data-tool]').forEach((b) => b.onclick = () => setTool(b.dataset.tool));
$('undo').onclick = undo;
function undo() { if (undoStack.length) { model = JSON.parse(undoStack.pop()); pendingA = null; evaluate(); } }

// shortcuts: Ctrl+Z everywhere; tool keys, Esc and Delete in Chassis only
window.addEventListener('keydown', (e) => {
  if (e.target.tagName === 'INPUT' || e.target.tagName === 'SELECT') return;
  if (workspace !== 'chassis' && !((e.ctrlKey || e.metaKey) && e.key.toLowerCase() === 'z')) return;
  if ((e.ctrlKey || e.metaKey) && e.key.toLowerCase() === 'z') { e.preventDefault(); return undo(); }
  const k = e.key.toLowerCase();
  if (k === 'escape') { pendingA = null; hint(); drawScene(); }
  else if (k === 's') setTool('select');
  else if (k === 'j') setTool('joint');
  else if (k === 't') setTool('tube');
  else if (k === 'p') setTool('split');
  else if (k === 'd') setTool('delete');
  else if ((k === 'delete' || k === 'backspace') && selected) {
    change(() => (selected.type === 'node' ? deleteJoint(selected.id) : deleteTube(selected.id)));
  }
});

// render loop (damped orbit controls need an update every frame)
(function loop() { requestAnimationFrame(loop); controls.update(); renderer.render(scene, camera); })();

// ---------- panels ----------
// Panels are HTML strings set with innerHTML; their handlers are bound right after, on every redraw.
const fmt = (x, d = 1) => (x === null || x === undefined || Number.isNaN(x)) ? '–'
  : x.toLocaleString('en', { minimumFractionDigits: d, maximumFractionDigits: d });
const pct = (r) => (r === null || r === undefined) ? '–' : fmt(r * 100, 0) + ' %';   // a ratio as a percentage
const sci = (x) => { const e = Math.floor(Math.log10(x)); return `${(x / 10 ** e).toFixed(2)} × 10<sup>${e}</sup>`; };
const tubeLabel = (t) => t.replace('x', ' × ');
const esc = (s) => String(s).replace(/[&<>"]/g, (c) => ({ '&': '&amp;', '<': '&lt;', '>': '&gt;', '"': '&quot;' }[c]));

// left panel: shows the workspace's container, then fills it (chassis: tube and joint lists)
function drawLists() {
  $('leftveh').hidden = workspace !== 'vehicle';
  if (workspace === 'vehicle') { $('leftchassis').hidden = true; $('leftsusp').hidden = true; $('leftcomp').hidden = true; $('leftaero').hidden = true; return drawVehList(); }
  $('leftsusp').hidden = workspace !== 'suspension';
  $('leftchassis').hidden = workspace !== 'chassis';
  $('leftcomp').hidden = workspace !== 'components';
  $('leftaero').hidden = workspace !== 'aero';
  if (workspace === 'aero') return drawAeroList();
  if (workspace === 'body') { $('leftcomp').hidden = true; $('leftchassis').hidden = true; return; }
  if (workspace === 'suspension') return drawSuspList();
  if (workspace === 'components') return drawCompList();
  $('ntubes').textContent = `(${result.members.length})`;
  $('njoints').textContent = `(${Object.keys(model.nodes).length})`;
  $('members').innerHTML = result.members.map((m) => {
    const side = mirrorMember(m) ? (m.a.endsWith('_L') || m.b.endsWith('_L') ? ' L' : ' R') : '';
    return `<li data-id="${m.id}" class="${selected?.type === 'member' && selected.id === m.id ? 'sel' : ''}">
       <span>${esc(m.group)}${side}</span><span class="q">${tubeLabel(m.tube)}</span></li>`;
  }).join('');
  $('members').querySelectorAll('li').forEach((li) => li.onclick = () => select({ type: 'member', id: +li.dataset.id }));
  $('joints').innerHTML = Object.keys(model.nodes).map((n) =>
    `<li data-id="${n}" class="${selected?.type === 'node' && selected.id === n ? 'sel' : ''}">
       <span>${n}</span><span class="q">${n in result.node_mass ? fmt(result.node_mass[n], 2) + ' kg' : 'no tubes'}</span></li>`).join('');
  $('joints').querySelectorAll('li').forEach((li) => li.onclick = () => select({ type: 'node', id: li.dataset.id }));
}

// chassis inspector with nothing selected: totals, CoG, bending beams, SVJ source and rigidity
function summaryHTML() {
  const t = result.totals, cog = result.cog;
  const deficit = t.maxwell_needed - t.members - (t.bending_beams || 0);
  const auto = (model.bending_beams || 'auto') !== 'off';
  let rig = `<p class="quiet">Torsion and bending stiffness, three ways (regulations 8.10.6).</p>
    <button id="rigbtn">${rigid ? 'Recompute rigidity' : 'Compute rigidity'}</button>`;
  if (rigid) {
    const f = rigid.frame, p = rigid.pinned, j = rigid.jbeam;
    const show = (x, ref, unit) => (x < ref * 1e-4 ? '≈ 0' : fmt(x, 0)) + ' ' + unit;
    rig += `${rigid.stale ? '<p class="WARN">Out of date: the model changed.</p>' : ''}
      <table class="opts"><tr><th>Model</th><th>Torsion</th><th>Bending</th></tr>
      <tr><td>Real welded frame</td><td>${show(f.torsion_Nm_per_deg, 1, 'N·m/°')}</td><td>${show(f.bending_N_per_mm, 1, 'N/mm')}</td></tr>
      <tr><td>Same tubes, pin joints</td><td>${show(p.torsion_Nm_per_deg, f.torsion_Nm_per_deg, 'N·m/°')}</td><td>${show(p.bending_N_per_mm, f.bending_N_per_mm, 'N/mm')}</td></tr>
      ${rigid.pinned_bending ? `<tr><td>+ bending beams (real stiffness)</td><td>${show(rigid.pinned_bending.torsion_Nm_per_deg, f.torsion_Nm_per_deg, 'N·m/°')}</td><td>${show(rigid.pinned_bending.bending_N_per_mm, f.bending_N_per_mm, 'N/mm')}</td></tr>` : ''}
      <tr><td>As BeamNG runs it</td><td>${show(j.torsion_Nm_per_deg, f.torsion_Nm_per_deg, 'N·m/°')}</td><td>${show(j.bending_N_per_mm, f.bending_N_per_mm, 'N/mm')}</td></tr></table>
      <p class="quiet">${rigid.jbeam.mechanisms ? `As BeamNG runs it, it folds: ${rigid.jbeam.mechanisms} mechanism${rigid.jbeam.mechanisms > 1 ? 's' : ''}, at the joints marked orange. Add real diagonal tubes, or turn bending beams on (8.10).`
        : `As BeamNG runs it, it is rigid and keeps ${fmt(rigid.jbeam_vs_frame_torsion * 100, 1)} % of the welded frame's torsional stiffness (springs are stability-capped, 8.9.6).`}
      ${rigid.check_8_10_6 ? ` <span class="${rigid.check_8_10_6.passed ? 'ok' : 'bad'}">8.10.6: with bending beams at real stiffness it reaches ${pct(rigid.check_8_10_6.torsion_ratio)} (torsion), ${pct(rigid.check_8_10_6.bending_ratio)} (bending), ${pct(rigid.check_8_10_6.lateral_ratio)} (sideways at the hoop top) and ${pct(rigid.check_8_10_6.roof_ratio)} (roof) of the welded frame; the limit is 110 %.</span>` : ''}
      Axles: ${rigid.front.join(', ')} / ${rigid.rear.join(', ')}. Bending load at ${rigid.load_nodes.join(', ')}.</p>`;
  }
  return `<h2>Chassis</h2>
    <div class="kv">
      <span>Tubes / joints</span><span>${t.members} / ${t.joints}</span>
      <span>Pin-jointed rigidity needs</span><span>≥ ${t.maxwell_needed} tubes (3N − 6)</span>
      ${cog ? `<span>CoG height</span><span>${fmt(cog.z * 1000, 0)} mm</span>
      <span>CoG from wheelbase centre</span><span>${fmt(cog.y * 1000, 0)} mm ${cog.y < 0 ? 'forward' : 'rearward'}</span>
      <span>CoG lateral</span><span>${fmt(cog.x * 1000, 0)} mm</span>
      ${cog.front_share !== undefined ? `<span>Front / rear (structure)</span><span>${fmt(cog.front_share * 100, 1)} / ${fmt((1 - cog.front_share) * 100, 1)} %</span>` : ''}` : ''}
    </div>
    <div class="kv"><span>Bending beams (8.10)</span><span>${t.bending_beams || 0} · <button id="bendtoggle">${auto ? 'Turn off' : 'Turn on'}</button></span></div>
    <p class="quiet">Weightless braces across each welded corner, sized to give the pin joints back the bending of the two tubes (3EI/L in series, over the lever arm squared), and scaled like the tubes (8.10.5). Shown dashed in the jbeam view.</p>
    ${deficit > 0 ? `<p class="quiet">Maxwell's rule: ${deficit} beam${deficit > 1 ? 's' : ''} short of a rigid pin-jointed frame. Welded joints make up for it in reality; in jbeam they don't (8.10).</p>` : ''}
    <p class="quiet">CoG is the structure alone (tubes and joints), from node masses. Engine, driver and body come with those workspaces.</p>
    ${model.svj_info ? `<h2>SVJ source</h2><div class="kv">
      <span>Vehicle</span><span>${esc(model.svj_info.vehicle || '–')}</span>
      <span>SVJ version</span><span>${esc(model.svj_info.version || '–')}</span>
      <span>mass_total (file)</span><span>${model.svj_info.mass_total ?? '–'} kg</span>
      <span>Wheelbase (file)</span><span>${model.svj_info.wheelbase ?? '–'} m</span></div>
      <p class="quiet">Fields FBeam does not design (tyres, powertrain, masses, brakes) are kept and written back unchanged on SVJ export.</p>
      ${svjBundle?.bindings.length ? `<h2>SVJ meshes (glTF)</h2><div class="kv">${svjBundle.bindings.map((b) => {
        const m = svjBundle.meshes.find((x) => x.id === b.mesh_ref);
        const ok = m && m.file && m.nodes.includes(b.node);
        return `<span>${esc(b.path)}</span><span class="${ok ? 'ok' : 'bad'}">${esc(b.node)}${ok ? '' : m?.file ? ' (node missing)' : ' (file missing)'}</span>`;
      }).join('')}</div><p class="quiet">Meshes are shown as reference geometry and written back into the SVJ bundle on export.</p>` : ''}` : ''}
    <h2>Rigidity</h2>${rig}`;
}

// right panel: dispatches to the workspace inspector; in Chassis, the selected tube or joint
function drawInspector() {
  const el = $('inspector');
  if (workspace === 'vehicle') return drawVehInspector(el);
  if (workspace === 'suspension') return drawSuspInspector(el);
  if (workspace === 'components') return drawCompInspector(el);
  if (workspace === 'aero') return drawAeroInspector(el);
  if (workspace === 'body') return drawBodyInspector(el);
  if (!selected) {
    el.innerHTML = summaryHTML();
    $('rigbtn').onclick = computeRigidity;
    $('bendtoggle').onclick = () => change(() => { model.bending_beams = (model.bending_beams || 'auto') === 'off' ? 'auto' : 'off'; });
    return;
  }
  if (selected.type === 'member') {
    const m = result.members.find((x) => x.id === selected.id);
    const twin = sym() ? mirrorMember(m) : null;
    const opts = result.catalogue[m.role].map((t) => {
      const p = JSON.parse(core.section_preview(JSON.stringify(model), m.id, t));
      const cur = t === m.tube;
      return `<tr class="${cur ? 'cur' : 'pick'}" data-t="${t}"><td>${tubeLabel(t)}${cur ? ' (now)' : ''}</td>
        <td>${cur ? '' : (p.d_mass >= 0 ? '+' : '') + fmt(p.d_mass, 2) + ' kg'}</td><td>${fmt(p.new_deform_kN)} kN</td></tr>`;
    }).join('');
    const roles = Object.entries(result.role_label).map(([k, v]) => `<option value="${k}" ${k === m.role ? 'selected' : ''}>${v}</option>`).join('');
    el.innerHTML = `
      <h2>Tube</h2>
      <div class="inl"><input type="text" id="gname" value="${esc(m.group)}"></div>
      <div class="quiet">${m.a} → ${m.b}</div>
      <div class="inl"><select id="role">${roles}</select></div>
      <div class="kv">
        <span>Section</span><span>tube ${tubeLabel(m.tube)} mm, steel 1018</span>
        <span>Length</span><span>${fmt(m.L, 3)} m</span>
        <span>Buckling length</span><span>${fmt(m.L_buckle, 3)} m${m.L_buckle > m.L + 1e-6 ? ' (runs through a joint)' : ''}</span>
        <span>Mass</span><span>${fmt(m.mass, 2)} kg (${fmt(m.kg_per_m, 2)} kg/m)</span>
        <span>E·A/L</span><span>${sci(m.beamSpring)} N/m</span>
        <span>beamSpring (stability cap)</span><span>${sci(m.k_set)} N/m</span>
        <span>Yield σy·A</span><span>${fmt(m.yield_force / 1e3)} kN</span>
        <span>Buckling π²EI/Lb²</span><span>${fmt(m.buckling_force / 1e3)} kN</span>
        <span>beamDeform</span><span>${fmt(m.beamDeform / 1e3)} kN (${m.governs})</span>
        <span>beamStrength σu·A</span><span>${fmt(m.beamStrength / 1e3)} kN</span>
      </div>
      <p class="quiet">All values are derived from the tube; none can be typed.</p>
      <h2>Exported jbeam</h2>
      <pre class="jb">${esc(m.jbeam)}</pre>
      <div class="kv"><span>nodeWeight ${m.a}</span><span>${fmt(result.node_mass[m.a], 2)} kg</span>
        <span>nodeWeight ${m.b}</span><span>${fmt(result.node_mass[m.b], 2)} kg</span></div>
      <p class="quiet">One straight tube is one jbeam beam. Node weights carry half of every tube joined there, plus 15 % for joints.</p>
      <h2>Change section</h2>
      <table class="opts"><tr><th>Tube (mm)</th><th>Mass</th><th>beamDeform</th></tr>${opts}</table>
      <p class="quiet">${m.role === 'frame' ? 'Chassis frame tubes: 1.5 mm minimum wall.' : 'Only sections at or above the FIA 253-8.3.3 minimum for this role are offered.'}
      ${twin ? 'Changes apply to both sides.' : ''}</p>`;
    const both = (fn) => change(() => { fn(model.members.find((x) => x.id === m.id)); if (twin) fn(model.members.find((x) => x.id === twin.id)); });
    el.querySelectorAll('tr.pick').forEach((tr) => tr.onclick = () => both((x) => { x.tube = tr.dataset.t; }));
    $('gname').onchange = (e) => both((x) => { x.group = e.target.value || x.group; });
    $('role').onchange = (e) => both((x) => {
      x.role = e.target.value;
      if (!result.catalogue[x.role].includes(x.tube)) x.tube = result.default_tube[x.role];
    });
  } else {
    const n = selected.id, p = model.nodes[n], twin = sym() ? mirrorOf(n) : n;
    const ax = model.axles || {};
    const axle = (ax.front || []).includes(n) ? 'front' : (ax.rear || []).includes(n) ? 'rear' : '';
    const nt = result.members.filter((m) => m.a === n || m.b === n).length;
    el.innerHTML = `
      <h2>Joint</h2>
      <div><b>${n}</b> · welded</div>
      <div class="kv">
        <span>Node mass (derived)</span><span>${n in result.node_mass ? fmt(result.node_mass[n], 2) + ' kg' : 'no tubes yet'}</span>
        <span>Tubes joined</span><span>${nt}</span>
      </div>
      <h2>Position (m, BeamNG axes)</h2>
      <div class="kv">
        <span>X left +</span><span><input type="number" step="0.01" data-i="0" value="${p[0]}"></span>
        <span>Y rear +</span><span><input type="number" step="0.01" data-i="1" value="${p[1]}"></span>
        <span>Z up +</span><span><input type="number" step="0.01" data-i="2" value="${p[2]}"></span>
      </div>
      <div class="inl">Axle reference
        <select id="axle"><option value="">none</option><option value="front" ${axle === 'front' ? 'selected' : ''}>front axle</option>
        <option value="rear" ${axle === 'rear' ? 'selected' : ''}>rear axle</option></select></div>
      <p class="quiet">Moving a joint changes tube lengths, so masses, buckling loads and node masses update.
      ${twin !== n && model.nodes[twin] ? `Mirrored to ${twin}.` : ''} Axle joints (left and right) set the wheelbase, weight split and rigidity tests.</p>`;
    el.querySelectorAll('input').forEach((inp) => inp.onchange = () => {
      const i = +inp.dataset.i, v = parseFloat(inp.value);
      if (Number.isNaN(v)) return;
      change(() => {
        model.nodes[n][i] = v;
        if (twin !== n && model.nodes[twin]) model.nodes[twin][i] = i === 0 ? -v : v;
      });
    });
    $('axle').onchange = (e) => change(() => {
      const a = model.axles = model.axles || {};
      for (const k of ['front', 'rear']) a[k] = (a[k] || []).filter((x) => x !== n && x !== twin);
      if (e.target.value) {
        const pair = twin !== n && model.nodes[twin] ? [n, twin] : [n];
        a[e.target.value] = (a[e.target.value] || []).concat(pair).slice(-2);
      }
    });
  }
}

function computeRigidity() {   // Compute rigidity button: core.rigidity (slow, so on demand)
  const t = performance.now();
  try { rigid = JSON.parse(core.rigidity(JSON.stringify(model))); rigid.stale = false; }
  catch (e) { showIssues([{ level: 'WARN', rule: '8.10.6', message: pyError(e) }]); return; }
  timing.rigidity = performance.now() - t;
  drawScene(); drawInspector(); drawBudget(); drawTiming();
}

// footer key numbers for the workspace
function drawBudget() {
  if (workspace === 'vehicle') return drawVehBudget();
  if (workspace === 'suspension') return drawSuspBudget();
  if (workspace === 'components') return drawCompBudget();
  if (workspace === 'aero') return drawAeroBudget();
  if (workspace === 'body') return drawBodyBudget();
  const t = result.totals, cog = result.cog;
  $('budget').innerHTML = `
    <div><span>Tube mass</span><b>${fmt(t.tube_mass)} kg</b> <span>frame ${fmt(t.frame_mass)} · cage ${fmt(t.cage_mass)}</span></div>
    <div><span>Node mass incl. 15 % joints</span><b>${fmt(t.node_mass)} kg</b></div>
    <div><span>CoG height (structure)</span><b>${cog ? fmt(cog.z * 1000, 0) + ' mm' : '–'}</b></div>
    <div><span>Front / rear (structure)</span><b>${cog?.front_share !== undefined ? fmt(cog.front_share * 100, 0) + ' / ' + fmt((1 - cog.front_share) * 100, 0) + ' %' : '–'}</b></div>
    <div><span>Torsion, welded / BeamNG</span><b>${rigid && !rigid.stale ? fmt(rigid.frame.torsion_Nm_per_deg, 0) + ' / ' + (rigid.jbeam.mechanisms ? '≈ 0' : fmt(rigid.jbeam.torsion_Nm_per_deg, 0)) + ' N·m/°' : '–'}</b></div>
    <div><span>Bending beams</span><b>${t.bending_beams || 0}</b></div>
    <div><span>Weakest tube</span><b>${t.members ? fmt(t.weakest_kN) + ' kN' : '–'}</b> <span>${esc(t.weakest_group)}</span></div>`;
}

// findings shown after every redraw: the workspace's own Python issues, or in Chassis all of them
function liveIssues() {
  if (workspace === 'vehicle') return vehIssues();
  const t = result.totals, out = [];
  for (const n of result.orphans) out.push({ level: 'WARN', rule: 'editor', message: `Joint ${n} has no tubes; it will be left out of the jbeam.` });
  const deficit = t.maxwell_needed - t.members - (t.bending_beams || 0);
  if (t.members && deficit > 0) out.push({ level: 'WARN', rule: '8.10', message: `Pin-jointed, this frame is ${deficit} beam${deficit > 1 ? 's' : ''} short of rigid (3N − 6 = ${t.maxwell_needed}). In jbeam it will fold unless braced by real tubes or bending beams. Compute rigidity to see where.` });
  if (t.members) out.push({ level: 'INFO', rule: '8.9.3', message: `${t.buckling_members} of ${t.members} tubes buckle before they yield; the weakest is ${t.weakest_group} at ${fmt(t.weakest_kN)} kN.` });
  if (!t.members) out.push({ level: 'INFO', rule: 'editor', message: 'Empty model: pick the Joint or Tube tool and start drawing.' });
  if (workspace === 'body') {
    if (!bodyRes) return [{ level: bodyError ? 'ERROR' : 'INFO', rule: '6', message: bodyError || 'No body shell. Load the full-chassis example.' }];
    return bodyRes.issues.length ? bodyRes.issues : [{ level: 'PASS', rule: '6', message: 'Body inside the envelope: length, width, roof height, overhangs, closed roof and ground clearance.' }];
  }
  if (workspace === 'aero') {
    if (!aeroRes) return [{ level: aeroError ? 'ERROR' : 'INFO', rule: '7.4', message: aeroError || 'No aero devices. Add a rear wing or a splitter.' }];
    return aeroRes.issues.length ? aeroRes.issues : [{ level: 'PASS', rule: '7.4', message: 'Wing and splitter are inside the aero box (7.4.2, 7.4.3) and fixed (7.4.4).' }];
  }
  if (workspace === 'components') {
    if (!mass) return [{ level: 'ERROR', rule: '6', message: massError || 'No masses: the model has no joints with tubes.' }];
    const di = drive?.issues || [];
    return mass.issues.length || di.length ? [...mass.issues, ...di] : [{ level: 'PASS', rule: '6', message: 'Mass rules met: driver ballast, engine position, ballast placement and minimum Race Mass. Driveline joint angles and plunge within the usual limits.' }];
  }
  if (workspace === 'suspension') {
    return susp?.issues.length ? susp.issues
      : [{ level: 'PASS', rule: '7.1', message: 'Every corner assembles, has at least 150 mm of travel, and has every inner pickup on a braced frame joint (7.1.6).' }];
  }
  if (susp) out.unshift(...susp.issues);
  if (mass) out.push(...mass.issues);
  if (aeroRes) out.push(...aeroRes.issues);
  if (bodyRes) out.push(...bodyRes.issues);
  return out;
}

function showIssues(list) {   // footer findings: [{ level, rule, message }]
  $('issues').innerHTML = list.map((f) =>
    `<div><span class="lvl ${f.level}">${f.level}</span><span class="quiet">[${f.rule}]</span> ${esc(f.message)}</div>`).join('');
}

function drawTiming() {   // footer timings; boot is done when it shows 'Python engine'
  const parts = [];
  if (timing.pyodide) parts.push(`Python engine ${fmt(timing.pyodide / 1000, 1)} s`);
  if (timing.files) parts.push(`FBeam code ${fmt(timing.files / 1000, 1)} s`);
  if (timing.evaluate) parts.push(`last evaluation ${fmt(timing.evaluate, 0)} ms`);
  if (timing.suspension) parts.push(`suspension ${fmt(timing.suspension, 0)} ms`);
  if (timing.rigidity) parts.push(`rigidity ${fmt(timing.rigidity, 0)} ms`);
  if (timing.build) parts.push(`build + check ${fmt(timing.build, 0)} ms`);
  $('timing').textContent = parts.join(' · ');
}

// ---------- header actions ----------
// Build: core.build writes the mod zip and runs the offline checker; offers the zip as a download link
$('build').onclick = () => {
  const t = performance.now();
  let res;
  try { res = JSON.parse(core.build(JSON.stringify(model), '/tmp/fbeam_car')); }
  catch (e) { showIssues([{ level: 'ERROR', rule: 'editor', message: pyError(e) }]); return; }
  timing.build = performance.now() - t;
  const dims = res.dimensions || {};
  const head = { level: res.passed ? 'PASS' : 'ERROR', rule: 'checker',
    message: `${res.passed ? 'Passes' : 'Fails'} the offline checker` +
      (dims.length_mm ? ` · length ${dims.length_mm} mm, width ${dims.width_mm} mm${dims.wheelbase_mm ? `, wheelbase ${dims.wheelbase_mm} mm` : ''}` : '') +
      ` · file hash ${res.file_hash.slice(0, 19)}…` };
  showIssues([head, ...res.findings]);
  const a = document.createElement('a');
  a.href = URL.createObjectURL(new Blob([py.FS.readFile(res.zip_path)], { type: 'application/zip' }));
  a.download = 'fbeam_car.zip';
  a.textContent = 'Download the car (mod zip)';
  const d = document.createElement('div');
  d.appendChild(a);
  $('issues').prepend(d);
  drawTiming();
};
// Save: the model as .fbeam.json (SVJ meshes are not included)
$('save').onclick = () => {
  const a = document.createElement('a');
  a.href = URL.createObjectURL(new Blob([JSON.stringify(model, null, 2)], { type: 'application/json' }));
  a.download = 'car.fbeam.json';
  a.click();
};
// ---------- SVJ (Standard Vehicle JSON) ----------
// Export: SVJ zip bundle (svj + glTF meshes) via core.export_svj_bundle; computes rigidity if out of date
$('svjout').onclick = () => {
  let rig = rigid && !rigid.stale ? rigid : null;
  if (!rig) { try { rig = JSON.parse(core.rigidity(JSON.stringify(model))); } catch (e) { rig = null; } }
  let r;
  try { r = JSON.parse(core.export_svj_bundle(JSON.stringify(model), rig ? JSON.stringify(rig) : null, '/tmp/fbeam_car_svj.zip', svjBundle?.base_dir || null)); }
  catch (e) { showIssues([{ level: 'ERROR', rule: 'SVJ', message: pyError(e) }]); return; }
  const a = document.createElement('a');
  a.href = URL.createObjectURL(new Blob([py.FS.readFile(r.zip_path)], { type: 'application/zip' }));
  a.download = 'fbeam_car_svj.zip';
  a.click();
  const d = JSON.parse(core.export_svj(JSON.stringify(model), rig ? JSON.stringify(rig) : null));
  showIssues([{ level: 'PASS', rule: 'SVJ', message: `Exported SVJ ${d._metadata.version} (SAE J670): wheelbase ${d.chassis.wheelbase} m, ` +
    `${Object.keys(d.suspension || {}).length} corners, torsion ${d.chassis.chassis_stiffness ? d.chassis.chassis_stiffness.torsional_stiffness + ' N·m/°' : 'not computed'}. ` +
    'The tube frame travels in x_fbeam, so re-importing restores the full design.' },
    { level: 'INFO', rule: 'SVJ §22', message: `Bundle: ${r.files.join(', ')}` + (r.frame_node ? ` · tube frame mesh bound as ${r.frame_node}` : '') },
    ...r.missing_meshes.map((u) => ({ level: 'WARN', rule: 'SVJ §22', message: `mesh ${u} is referenced but was not available, so it is not in the zip` })),
    ...(d.chassis._est ? [{ level: 'INFO', rule: 'SVJ', message: `mass_total ${d.chassis.mass_total} kg is an estimate (FIA class minimum); CG is the structure only.` }] : [])]);
};
// Import SVJ: the options dialog first. "Apply to the current frame" (default on when there is a
// frame) keeps the tubes and applies the file's suspension (core.apply_svj); off replaces the model.
$('svjin').onclick = () => {
  const hasFrame = (model.members || []).length > 0;
  $('svjapply').checked = hasFrame;
  $('svjapply').disabled = !hasFrame;
  $('svjsub').hidden = !hasFrame;
  $('svjdlg').showModal();
};
$('svjapply').onchange = () => { $('svjsub').hidden = !$('svjapply').checked; };
$('svjcancel').onclick = () => $('svjdlg').close();
$('svjgo').onclick = () => { $('svjdlg').close(); $('svjfile').click(); };
// Import: an .svj.json with loose mesh files, or a .zip bundle; replaces the model (undoable)
$('svjfile').onchange = async (e) => {
  const picked = [...e.target.files];
  if (!picked.length) return;
  const f = picked.find((x) => /\.zip$/i.test(x.name)) || picked.find((x) => /\.json$/i.test(x.name));
  if (!f) { showIssues([{ level: 'ERROR', rule: 'SVJ', message: 'Pick an .svj.json (with its mesh files) or a .zip bundle.' }]); return; }
  try {
    const raw = '/tmp/svj_raw';
    try { py.runPython(`import shutil; shutil.rmtree('${raw}', ignore_errors=True)`); } catch (x) { /* none */ }
    py.FS.mkdirTree(raw + '/meshes');
    for (const x of picked) {                        // loose mesh files go beside the svj and under meshes/
      const data = new Uint8Array(await x.arrayBuffer());
      py.FS.writeFile(`${raw}/${x.name}`, data);
      if (!/\.(json|zip)$/i.test(x.name)) py.FS.writeFile(`${raw}/meshes/${x.name}`, data);
    }
    const apply = $('svjapply').checked && (model.members || []).length > 0;
    const opts = { move_joints: $('svjmove').checked, layout: $('svjlayout').checked, rates: $('svjrates').checked, meshes: $('svjmesh').checked };
    const r = JSON.parse(apply
      ? core.import_svj_bundle(`${raw}/${f.name}`, '/tmp/svj_in', JSON.stringify(model), JSON.stringify(opts))
      : core.import_svj_bundle(`${raw}/${f.name}`, '/tmp/svj_in'));
    if (!apply || opts.meshes) svjBundle = r;           // applying without meshes keeps the meshes already shown
    change(() => { model = r.model; selected = null; selPoint = null; rigid = null; });
    await loadSvjMeshes();
    analyseSuspension();
    redraw();
    if (!apply) fitCamera();
    showIssues([{ level: 'PASS', rule: 'SVJ', message: apply ? `Applied ${f.name} to the current frame.`
      : `Imported ${f.name}${model.svj_info?.vehicle ? ' (' + model.svj_info.vehicle + ')' : ''}.` },
      ...r.notes.map((n) => ({ level: /not supported|not the mirror|over 5|assumed|placed|not provided|not found|not in assets/.test(n) ? 'WARN' : 'INFO', rule: 'SVJ', message: n })),
      ...r.meshes.filter((m) => m.file).map((m) => ({ level: 'INFO', rule: 'SVJ §22', message: `mesh ${m.uri}: nodes ${m.nodes.join(', ') || '(none)'}` }))]);
  } catch (err) { showIssues([{ level: 'ERROR', rule: 'SVJ', message: 'Could not import: ' + pyError(err) }]); }
  e.target.value = '';
};

// glTF assets of an imported SVJ, placed with the §22.4 axis convention
function gltfToThree(axes, yf) {
  const vec = (t) => { const s = t.startsWith('-') ? -1 : 1, a = t.replace('-', ''); return new THREE.Vector3(a === 'X' ? s : 0, a === 'Y' ? s : 0, a === 'Z' ? s : 0); };
  const u = vec(axes.up), f = vec(axes.forward), r = new THREE.Vector3().crossVectors(f, u);
  // glTF -> SAE (X = f, Y = r, Z = -u) -> FBeam (x = -Y, y = yf - X, z = -Z) -> view (x, z, y)
  return new THREE.Matrix4().set(-r.x, -r.y, -r.z, 0, u.x, u.y, u.z, 0, -f.x, -f.y, -f.z, yf, 0, 0, 0, 1);
}
// load the imported glTF meshes into meshG as translucent reference geometry; after an SVJ import
async function loadSvjMeshes() {
  meshG.clear();
  $('meshtoggle').hidden = !(svjBundle && svjBundle.meshes.some((m) => m.file));
  if (!svjBundle) return;
  const M = gltfToThree(svjBundle.axes, svjBundle.y_front);
  const loader = new GLTFLoader();
  for (const m of svjBundle.meshes) {
    if (!m.file) continue;
    const bytes = py.FS.readFile(m.file);
    const dir = m.file.slice(0, m.file.lastIndexOf('/') + 1);
    loader.manager.setURLModifier((url) => {            // external .bin / textures of a .gltf, from the same folder
      if (/^(blob|data):/.test(url)) return url;
      try { return URL.createObjectURL(new Blob([py.FS.readFile(dir + url.split('/').pop())])); } catch (e) { return url; }
    });
    try {
      const gl = await new Promise((ok, bad) => loader.parse(bytes.buffer.slice(bytes.byteOffset, bytes.byteOffset + bytes.byteLength), '', ok, bad));
      gl.scene.traverse((o) => {
        if (o.isMesh) {
          o.material = o.material.clone();
          Object.assign(o.material, { transparent: true, opacity: 0.55, side: THREE.DoubleSide, depthWrite: false });
        }
      });
      const holder = new THREE.Group();
      holder.matrixAutoUpdate = false;
      holder.matrix.copy(M);
      holder.add(gl.scene);
      holder.userData.uri = m.uri;
      meshG.add(holder);
    } catch (err) {
      showIssues([{ level: 'WARN', rule: 'SVJ §22', message: `could not load mesh ${m.uri}: ${err.message || err}` }]);
    }
  }
  meshG.visible = $('meshes').checked;
}
$('meshes').onchange = () => { meshG.visible = $('meshes').checked; };

// Open: a saved .fbeam.json (undoable)
$('open').onclick = () => $('openfile').click();
$('openfile').onchange = async (e) => {
  const f = e.target.files[0];
  if (!f) return;
  try {
    const m = JSON.parse(await f.text());
    if (!m.nodes || !m.members) throw new Error('not an FBeam model (needs nodes and members)');
    change(() => { model = m; selected = null; });
    fitCamera();
  } catch (err) { showIssues([{ level: 'ERROR', rule: 'editor', message: 'Could not open: ' + err.message }]); }
  e.target.value = '';
};
$('example').onchange = (e) => { loadExample(e.target.value); rigid = null; fitCamera(); };
// view options and workspace switching
$('colour').onchange = () => result && drawScene();
$('show').onchange = () => result && drawScene();
document.querySelectorAll('[data-space]').forEach((b) => b.onclick = () => {
  workspace = b.dataset.space;
  document.querySelectorAll('[data-space]').forEach((x) => x.classList.toggle('on', x === b));
  $('drawtools').hidden = workspace !== 'chassis';
  $('susptools').hidden = workspace !== 'suspension';
  setTool('select');
  if (result && (workspace === 'suspension' || workspace === 'components') && susp?.stale) analyseSuspension();
  if (result) redraw();
  if (result) (workspace === 'suspension' ? focusCorner(suspCorner) : fitCamera());
});
// wheel travel slider: redraws suspension and shafts at that travel without calling Python
$('travel').oninput = () => {
  $('travelv').textContent = `${$('travel').value} mm`;
  if (result) { drawSuspension(); drawDriveline(); drawSuspInspector($('inspector')); }
};
$('mirror').onchange = () => drawInspector();
$('ghosts').onchange = () => drawGhosts();
$('plane').onchange = () => drawOverlay();
hint();

// ---------- components workspace: masses, full-car CG and inertia ----------
// colour and label per component kind (kinds come from fbeam_mass)
const KCOL = { powertrain: 0xcf222e, driver: 0x2f6fdf, fuel: 0xbf8700, cooling: 0x1f883d, exhaust: 0x6e7781, battery: 0x8250df,
  seat: 0x57606a, steering: 0x57606a, body: 0x8c959f, ballast: 0x111111, other: 0xa475f9 };
const KLABEL = { powertrain: 'Engine + gearbox', driver: 'Driver ballast', fuel: 'Fuel + tank', cooling: 'Radiator + coolant', exhaust: 'Exhaust',
  battery: 'Battery', seat: 'Seat + harness', steering: 'Steering rack', body: 'Body panels', ballast: 'Ballast', other: 'Other' };

// component boxes (solid in Components, faint ghosts elsewhere) and the full-car CG
function drawComponents() {
  compG.clear();
  const comps = model.components || [];
  const full = workspace === 'components';
  if (!comps.length || (!full && !$('ghosts').checked)) return;
  for (const c of comps) {
    if (c.mass <= 0 && c.kind !== 'ballast') continue;
    const isSel = full && selComp === c.id;
    const [a, b, h] = c.size;
    const box = new THREE.Mesh(new THREE.BoxGeometry(a, h, b), new THREE.MeshStandardMaterial({
      color: KCOL[c.kind] || KCOL.other, transparent: true, opacity: full ? (isSel ? 0.75 : 0.4) : 0.12, depthWrite: false }));
    box.position.copy(v3(c.position));
    box.userData = { type: 'component', id: c.id };
    compG.add(box);
    if (isSel) compG.add(new THREE.LineSegments(new THREE.EdgesGeometry(box.geometry), new THREE.LineBasicMaterial({ color: 0xffffff })).translateX(box.position.x).translateY(box.position.y).translateZ(box.position.z));
  }
  if (mass && full) {                                     // full-car CG, with a plumb line to the ground
    const cg = mass.cg;
    const s = new THREE.Mesh(new THREE.SphereGeometry(0.07, 20, 14), new THREE.MeshBasicMaterial({ color: 0xffd33d, depthTest: false }));
    s.renderOrder = 10;
    s.position.copy(v3(cg));
    compG.add(s);
    const line = new THREE.Line(new THREE.BufferGeometry().setFromPoints([v3(cg), v3([cg[0], cg[1], 0])]),
      new THREE.LineDashedMaterial({ color: 0xbf8700, dashSize: 0.04, gapSize: 0.03 }));
    line.computeLineDistances();
    compG.add(line);
  }
}

// driveline (4.6): differentials, halfshafts and propshaft; outer ends follow the wheels at the current travel
function drawDriveline() {
  driveG.clear();
  if (!drive || !model.driveline) return;
  const full = workspace === 'components' || workspace === 'suspension';
  if (!full && !$('ghosts').checked) return;
  const t = workspace === 'suspension' ? travelNow() : 0, op = full ? 0.95 : 0.4;
  const at = (ref, fallback) => {
    const res = ref && susp?.corners?.[ref.corner];
    if (!res) return fallback;
    const wc = wcAt(res, t);
    if (ref.centre) return [0, wc[1] + ref.offset[1], wc[2] + ref.offset[2]];
    return [wc[0] * ref.side, wc[1], wc[2]];
  };
  for (const s of drive.shafts) {
    const b = at(s.b_ref, s.b);
    driveG.add(tubeMesh(v3(s.a), v3(b), s.kind === 'prop' ? 0.022 : 0.015, s.kind === 'prop' ? 0xfb8f44 : 0xffa657, op));
    for (const p of [s.a, b]) {                          // CV / universal joints
      const j = new THREE.Mesh(new THREE.SphereGeometry(0.026, 12, 8), new THREE.MeshStandardMaterial({ color: 0xbc4c00, transparent: true, opacity: op }));
      j.position.copy(v3(p));
      driveG.add(j);
    }
  }
  for (const d of drive.diffs) {
    const p = d.moving ? at(d.moving, d.pos) : d.pos;
    const box = new THREE.Mesh(new THREE.BoxGeometry(0.22, 0.2, 0.2), new THREE.MeshStandardMaterial({ color: 0xbc4c00, transparent: true, opacity: op * 0.85 }));
    box.position.copy(v3(p));
    driveG.add(box);
  }
  if (drive.gearbox_output && drive.shafts.some((s) => s.kind === 'prop')) {
    const g = new THREE.Mesh(new THREE.SphereGeometry(0.04, 14, 10), new THREE.MeshStandardMaterial({ color: 0xbc4c00 }));
    g.position.copy(v3(drive.gearbox_output));
    driveG.add(g);
  }
  if (full) driveG.traverse((o) => { if (o.isMesh) { o.material.depthTest = false; o.renderOrder = 8; } });   // seen through the boxes
}

// driveline panel: layout picker (FWD / RWD / AWD from Python) and joint angles and plunge per shaft
function driveSummary() {
  if (!model.driveline) return '';
  const d = drive || { shafts: [], layouts: ['FWD', 'RWD', 'AWD'] }, layout = model.driveline.layout;
  const rows = d.shafts.map((s) => `<span>${esc(s.label)}</span><span>${fmt(s.static_angle_deg, 1)}° static · max ${fmt(s.max_angle_deg, 1)}°${s.kind === 'half' || s.b_ref ? ` · plunge ${fmt(s.plunge_range_mm[0], 0)} to ${fmt(s.plunge_range_mm[1], 0)} mm` : ''}</span>`).join('');
  return `<h2>Driveline (4.6)</h2>
    <div class="kv"><span>Driven axles</span><span><select id="drivelayout">${(d.layouts || ['FWD', 'RWD', 'AWD']).map((k) => `<option ${k === layout ? 'selected' : ''}>${k}</option>`).join('')}</select></span></div>
    <p class="quiet">${layout === 'FWD' ? 'Spec: the Covet drives its front wheels through unequal halfshafts from the gearbox differential.' : 'Not legal in v1 (4.6: the donor\'s driven axle). Shown for testing.'} Angles and plunge are over the usable suspension travel; the shafts are drawn orange, on top of the component boxes.</p>
    <div class="kv">${rows || '<span class="quiet">No driven corner analysed.</span><span></span>'}</div>`;
}

function pickComponent(e) {   // id of the component box under the pointer, or null
  pointerRay(e);
  const hit = ray.intersectObjects(compG.children).find((h) => h.object.userData.type === 'component');
  return hit ? hit.object.userData.id : null;
}

// left panel in Components: list, add ballast / component, fill ballast, unsprung mass
function drawCompList() {
  const el = $('leftcomp'), comps = model.components || [];
  el.innerHTML = `<h2>Components <span class="q">(${comps.length})</span></h2>
    <ul>${comps.map((c) => `<li data-id="${esc(c.id)}" class="${selComp === c.id ? 'sel' : ''}">
      <span>${esc(KLABEL[c.kind] || c.kind)}${c.estimate ? ' <span class="q">est.</span>' : ''}</span><span class="q">${fmt(c.mass, 1)} kg</span></li>`).join('')}</ul>
    <div class="inl"><button id="addballast">Add ballast block</button><button id="addcomp">Add component</button></div>
    <div class="inl"><button id="fillballast">Fill ballast to ${mass ? fmt(mass.min_mass, 0) : 'minimum'} kg</button></div>
    <h2>Unsprung, per corner</h2>
    <div class="inl"><input type="number" id="unsprung" step="1" value="${model.unsprung_mass_per_corner ?? 0}"> kg at each wheel centre</div>`;
  el.querySelectorAll('li').forEach((li) => li.onclick = () => { selComp = li.dataset.id; redraw(); });
  const add = (kind, extra) => change(() => {
    model.components = model.components || [];
    let k = 1; while (model.components.some((c) => c.id === `${kind}_${k}`)) k++;
    const id = `${kind}_${k}`;
    model.components.push({ id, kind, mass: kind === 'ballast' ? 20 : 10, position: [0, 0, kind === 'ballast' ? 0.22 : 0.4],
      size: kind === 'ballast' ? [0.3, 0.3, 0] : [0.3, 0.3, 0.3], estimate: true, note: '', ...extra });
    selComp = id;
  });
  $('addballast').onclick = () => add('ballast', { note: 'Solid block on the cockpit floor (6.10)' });
  $('addcomp').onclick = () => add('other', {});
  $('fillballast').onclick = () => {
    const r = JSON.parse(core.fill_ballast(JSON.stringify(model)));
    if (r.added_kg <= 0) { showIssues([{ level: 'PASS', rule: '6.11', message: 'Already at or above the minimum Race Mass.' }]); return; }
    change(() => { model = r.model; });
    showIssues([{ level: 'PASS', rule: '6.11', message: `Added ${fmt(r.added_kg, 1)} kg of steel ballast on the cockpit floor.` }, ...liveIssues()]);
  };
  $('unsprung').onchange = (e) => { const v = parseFloat(e.target.value); if (v >= 0) change(() => { model.unsprung_mass_per_corner = v; }); };
}

function massSummary() {   // whole-car mass, CG and inertia (core.masses)
  if (!mass) return '<p class="quiet">No masses yet.</p>';
  const m = mass, I = m.inertia;
  return `<h2>Whole car (estimate)</h2><div class="kv">
      <span>Race Mass</span><span>${fmt(m.mass_total, 1)} kg</span>
      <span>Minimum (6.11)</span><span>${fmt(m.min_mass, 0)} kg</span>
      <span>Ballast needed</span><span class="${m.ballast_needed > 0 ? 'bad' : 'ok'}">${fmt(m.ballast_needed, 1)} kg</span>
      <span>Structure / components / unsprung</span><span>${fmt(m.structure_mass, 0)} / ${fmt(m.components_mass, 0)} / ${fmt(m.unsprung_total, 0)} kg</span>
      <span>CG height</span><span>${fmt(m.cg[2] * 1000, 0)} mm</span>
      ${m.front_share !== undefined ? `<span>Front / rear</span><span>${fmt(m.front_share * 100, 1)} / ${fmt((1 - m.front_share) * 100, 1)} %</span>
      <span>Left / right</span><span>${fmt(m.left_share * 100, 1)} / ${fmt((1 - m.left_share) * 100, 1)} %</span>` : ''}
      <span>Roll inertia Ixx</span><span>${fmt(I.Ixx_roll, 0)} kg·m²</span>
      <span>Pitch inertia Iyy</span><span>${fmt(I.Iyy_pitch, 0)} kg·m²</span>
      <span>Yaw inertia Izz</span><span>${fmt(I.Izz_yaw, 0)} kg·m²</span></div>
    <p class="quiet">Structure is every jbeam node as a point mass; components are boxes; unsprung masses sit at the wheel centres. Inertia is about the car's CG. Exported to SVJ as mass_bodies, CG and inertia.</p>`;
}

function bindDrive() {   // bind the driveline layout picker after driveSummary is rendered
  if ($('drivelayout')) $('drivelayout').onchange = (e) => change(() => { model = JSON.parse(core.set_driveline(JSON.stringify(model), e.target.value)); });
}

// right panel in Components: the selected box's mass, CG and size, or the whole car
function drawCompInspector(el) {
  const c = (model.components || []).find((x) => x.id === selComp);
  if (!c) { el.innerHTML = massSummary() + driveSummary(); bindDrive(); return; }
  const kinds = Object.entries(KLABEL).map(([k, v]) => `<option value="${k}" ${k === c.kind ? 'selected' : ''}>${v}</option>`).join('');
  const num = (key, i, step) => `<input type="number" step="${step}" data-k="${key}" data-i="${i}" value="${i === undefined ? c[key] : c[key][i]}">`;
  el.innerHTML = `<h2>Component</h2>
    <div class="inl"><b>${esc(c.id)}</b></div>
    <div class="inl"><select id="ckind">${kinds}</select></div>
    <div class="kv">
      <span>Mass (kg)</span><span>${num('mass', undefined, 0.5)}</span>
      <span>CG X left + (m)</span><span>${num('position', 0, 0.01)}</span>
      <span>CG Y rear + (m)</span><span>${num('position', 1, 0.01)}</span>
      <span>CG Z up + (m)</span><span>${c.kind === 'ballast' ? fmt(c.position[2], 3) + ' (on the floor)' : num('position', 2, 0.01)}</span>
      <span>Size X (m)</span><span>${num('size', 0, 0.01)}</span>
      <span>Size Y (m)</span><span>${num('size', 1, 0.01)}</span>
      <span>Size Z (m)</span><span>${c.kind === 'ballast' ? fmt(c.size[2], 3) + ' (steel, from mass)' : num('size', 2, 0.01)}</span>
      <span>Estimate</span><span><input type="checkbox" id="cest" ${c.estimate ? 'checked' : ''}></span>
    </div>
    ${c.note ? `<p class="quiet">${esc(c.note)}</p>` : ''}
    <div class="inl"><button id="cdel">Delete component</button><button id="cback">Whole car</button></div>
    ${massSummary()}${driveSummary()}`;
  bindDrive();
  el.querySelectorAll('input[data-k]').forEach((inp) => inp.onchange = () => {
    const v = parseFloat(inp.value);
    if (Number.isNaN(v)) return;
    change(() => {
      const x = model.components.find((y) => y.id === c.id);
      if (inp.dataset.i === 'undefined') x[inp.dataset.k] = Math.max(0, v); else x[inp.dataset.k][+inp.dataset.i] = v;
    });
  });
  $('ckind').onchange = (e) => change(() => { model.components.find((y) => y.id === c.id).kind = e.target.value; });
  $('cest').onchange = (e) => change(() => { model.components.find((y) => y.id === c.id).estimate = e.target.checked; });
  $('cdel').onclick = () => change(() => { model.components = model.components.filter((y) => y.id !== c.id); selComp = null; });
  $('cback').onclick = () => { selComp = null; redraw(); };
}

function drawCompBudget() {   // footer in Components
  if (!mass) { $('budget').innerHTML = ''; return; }
  const m = mass;
  $('budget').innerHTML = `
    <div><span>Race Mass (estimate)</span><b>${fmt(m.mass_total, 0)} kg</b> <span>minimum ${fmt(m.min_mass, 0)} kg</span></div>
    <div><span>Ballast needed</span><b class="${m.ballast_needed > 0 ? 'bad' : 'ok'}">${fmt(m.ballast_needed, 0)} kg</b></div>
    <div><span>CG height</span><b>${fmt(m.cg[2] * 1000, 0)} mm</b></div>
    <div><span>Front / rear</span><b>${m.front_share !== undefined ? fmt(m.front_share * 100, 0) + ' / ' + fmt((1 - m.front_share) * 100, 0) + ' %' : '–'}</b></div>
    <div><span>Inertia roll / pitch / yaw</span><b>${fmt(m.inertia.Ixx_roll, 0)} / ${fmt(m.inertia.Iyy_pitch, 0)} / ${fmt(m.inertia.Izz_yaw, 0)}</b> <span>kg·m²</span></div>`;
}

// ---------- body workspace ----------
// body shell triangles from core.body (solid in Body, faint ghost elsewhere)
function drawBody() {
  bodyG.clear();
  if (!bodyRes) return;
  const full = workspace === 'body';
  if (!full && !$('ghosts').checked) return;
  const pos = [];
  for (const t of bodyRes.triangles) for (const p of t) { const v = v3(p); pos.push(v.x, v.y, v.z); }
  const g = new THREE.BufferGeometry();
  g.setAttribute('position', new THREE.Float32BufferAttribute(pos, 3));
  g.computeVertexNormals();
  bodyG.add(new THREE.Mesh(g, new THREE.MeshStandardMaterial({ color: 0xc9d1d9, transparent: true, opacity: full ? 0.35 : 0.08,
    side: THREE.DoubleSide, depthWrite: false })));
  if (full) bodyG.add(new THREE.LineSegments(new THREE.EdgesGeometry(g, 30), new THREE.LineBasicMaterial({ color: 0x8c959f })));
}

// right panel in Body: shell dimensions, panel material, closed roof, and the result
function drawBodyInspector(el) {
  const b = model.body;
  if (!b) { el.innerHTML = '<p class="quiet">No body shell in this model.</p>'; return; }
  const num = (k, step, label, unit) => `<span>${label}</span><span><input type="number" step="${step}" data-b="${k}" value="${b[k]}"> ${unit}</span>`;
  const mats = ['steel_0.8mm', 'aluminium_1.0mm', 'composite_2.0mm'].map((m) => `<option ${m === b.material ? 'selected' : ''}>${m}</option>`).join('');
  const r = bodyRes;
  el.innerHTML = `<h2>Body shell</h2><div class="kv">
      ${num('length', 0.01, 'Length', 'm')}${num('width', 0.01, 'Width', 'm')}${num('height', 0.01, 'Roof height', 'm')}
      ${num('front_overhang', 0.01, 'Front overhang', 'm')}${num('ground_clearance', 0.005, 'Ground clearance', 'm')}
      ${num('bonnet_height', 0.01, 'Bonnet height', 'm')}${num('boot_height', 0.01, 'Boot height', 'm')}
      ${num('windscreen_deg', 1, 'Windscreen angle', '°')}${num('rear_window_deg', 1, 'Rear window angle', '°')}${num('roof_length', 0.01, 'Roof length', 'm')}
      <span>Panels</span><span><select id="bmat">${mats}</select></span>
      <span>Closed roof (6.7)</span><span><input type="checkbox" id="broof" ${b.closed_roof ? 'checked' : ''}></span>
    </div>
    ${r ? `<h2>Result</h2><div class="kv">
      <span>Skin area</span><span>${fmt(r.skin_area_m2, 1)} m²</span>
      <span>Panel mass</span><span>${fmt(r.mass, 1)} kg (${fmt(r.kg_per_m2, 2)} kg/m²)</span>
      <span>Body CG height</span><span>${fmt(r.cg[2] * 1000, 0)} mm</span>
      <span>Bumpers (Y)</span><span>${fmt(r.front_bumper, 2)} / ${fmt(r.rear_bumper, 2)} m</span></div>
      <p class="quiet">The body panels component takes this mass and CG, so lighter panels mean more ballast to reach 1,100 kg (6.11), placed where the team chooses. Exported to SVJ as the body_panels mass body, with a glTF shell mesh bound to it.</p>` : ''}`;
  el.querySelectorAll('input[data-b]').forEach((inp) => inp.onchange = () => {
    const v = parseFloat(inp.value);
    if (!Number.isNaN(v)) change(() => { model.body[inp.dataset.b] = v; });
  });
  $('bmat').onchange = (e) => change(() => { model.body.material = e.target.value; });
  $('broof').onchange = (e) => change(() => { model.body.closed_roof = e.target.checked; });
}

function drawBodyBudget() {   // footer in Body
  if (!bodyRes) { $('budget').innerHTML = ''; return; }
  const b = model.body;
  $('budget').innerHTML = `
    <div><span>Body length / width</span><b>${fmt(b.length * 1000, 0)} / ${fmt(b.width * 1000, 0)} mm</b></div>
    <div><span>Roof height</span><b>${fmt(b.height * 1000, 0)} mm</b></div>
    <div><span>Panels</span><b>${fmt(bodyRes.mass, 1)} kg</b> <span>${esc(b.material)}</span></div>
    <div><span>Race Mass</span><b>${mass ? fmt(mass.mass_total, 0) + ' kg' : '–'}</b> <span>${mass && mass.ballast_needed > 0 ? fmt(mass.ballast_needed, 0) + ' kg ballast needed' : 'at the minimum'}</span></div>`;
}

// ---------- aero workspace ----------
// wing and splitter plates (wing pitched by its angle of attack) and, in Aero, the aero box (7.4)
function drawAero() {
  aeroG.clear();
  const devs = model.aero || [];
  const full = workspace === 'aero';
  if (!devs.length || (!full && !$('ghosts').checked)) return;
  for (const d of devs) {
    const isSel = full && selAero === d.id;
    const [w, dep] = d.kind === 'wing' ? [d.span, d.chord] : [d.width, d.depth];
    const plate = new THREE.Mesh(new THREE.BoxGeometry(w, 0.012, dep), new THREE.MeshStandardMaterial({
      color: isSel ? 0xcf222e : d.kind === 'wing' ? 0x2f6fdf : 0x1f883d, transparent: true, opacity: full ? 0.85 : 0.35 }));
    plate.position.copy(v3(d.position));
    if (d.kind === 'wing') plate.rotation.x = -THREE.MathUtils.degToRad(d.aoa_deg);   // leading edge down = downforce
    plate.userData = { aero: d.id };
    aeroG.add(plate);
    if (d.kind === 'wing' && d.endplates) for (const s of [1, -1]) {
      const ep = new THREE.Mesh(new THREE.BoxGeometry(0.008, 0.16, dep * 1.2), plate.material);
      ep.position.copy(v3([d.position[0] + s * w / 2, d.position[1], d.position[2]]));
      aeroG.add(ep);
    }
  }
  if (full && aeroRes) {                                  // the aero box: body width, roof, bumpers
    const L = aeroRes.limits, wbx = L.body_width, len = L.rear_bumper - L.front_bumper + 0.05;
    const box = new THREE.LineSegments(new THREE.EdgesGeometry(new THREE.BoxGeometry(wbx, L.roof, len)),
      new THREE.LineDashedMaterial({ color: 0x2f6fdf, dashSize: 0.05, gapSize: 0.04 }));
    box.position.set(0, L.roof / 2, (L.front_bumper - 0.05 + L.rear_bumper) / 2);
    box.computeLineDistances();
    aeroG.add(box);
  }
}

// left panel in Aero: devices, add wing / splitter (one of each)
function drawAeroList() {
  const el = $('leftaero'), devs = model.aero || [];
  el.innerHTML = `<h2>Aero devices <span class="q">(${devs.length})</span></h2>
    <ul>${devs.map((d) => `<li data-id="${esc(d.id)}" class="${selAero === d.id ? 'sel' : ''}"><span>${d.kind === 'wing' ? 'Rear wing' : 'Front splitter'} · ${esc(d.id)}</span>
      <span class="q">${d.kind === 'wing' ? fmt(d.aoa_deg, 1) + '°' : fmt(d.depth * 1000, 0) + ' mm'}</span></li>`).join('')}</ul>
    <div class="inl"><button id="addwing" ${devs.some((d) => d.kind === 'wing') ? 'disabled' : ''}>Add rear wing</button>
      <button id="addsplit" ${devs.some((d) => d.kind === 'splitter') ? 'disabled' : ''}>Add splitter</button></div>
    <p class="quiet">7.4.2 allows one rear wing and 7.4.3 one splitter.</p>`;
  el.querySelectorAll('li').forEach((li) => li.onclick = () => { selAero = li.dataset.id; redraw(); });
  $('addwing').onclick = () => change(() => { (model.aero = model.aero || []).push({ id: 'rear_wing', kind: 'wing', span: 1.2, chord: 0.2, aoa_deg: 5, endplates: true, position: [0, 1.5, 1.1] }); selAero = 'rear_wing'; });
  $('addsplit').onclick = () => change(() => { (model.aero = model.aero || []).push({ id: 'splitter', kind: 'splitter', width: 1.3, depth: 0.1, position: [0, -1.75, 0.14] }); selAero = 'splitter'; });
}

function aeroTable() {   // forces per speed from core.aero
  if (!aeroRes) return '';
  const rows = aeroRes.speeds.map((s) => `<tr><td>${s.kmh} km/h</td><td>${fmt(s.downforce_front_N, 0)}</td><td>${fmt(s.downforce_rear_N, 0)}</td><td>${fmt(s.drag_N, 0)}</td><td>${s.balance_front === null ? '–' : fmt(s.balance_front * 100, 0) + ' %'}</td></tr>`).join('');
  return `<h2>Forces (N, + = down)</h2><table class="opts"><tr><th>Speed</th><th>Front</th><th>Rear</th><th>Drag</th><th>Front share</th></tr>${rows}</table>
    <p class="quiet">Axle loads include the nose-up pitch of drag acting at its height. Frontal area ${fmt(aeroRes.frontal_area_m2, 2)} m² (estimate); body Cd ${aeroRes.estimates.body_cd}, Cl ${aeroRes.estimates.body_cl}; splitter Cl ${aeroRes.estimates.splitter_cl}. First-order estimates, to calibrate in-game.</p>`;
}

// right panel in Aero: the selected device's dimensions and coefficients, or the force table
function drawAeroInspector(el) {
  const d = (model.aero || []).find((x) => x.id === selAero);
  if (!d) { el.innerHTML = `<h2>Aero</h2>${aeroTable()}`; return; }
  const r = aeroRes?.devices.find((x) => x.id === d.id);
  const num = (key, i, step, label) => `<span>${label}</span><span><input type="number" step="${step}" data-k="${key}" data-i="${i}" value="${i === undefined ? d[key] : d[key][i]}"></span>`;
  el.innerHTML = `<h2>${d.kind === 'wing' ? 'Rear wing' : 'Front splitter'}</h2>
    <div class="kv">
      ${d.kind === 'wing' ? num('span', undefined, 0.01, 'Span (m)') + num('chord', undefined, 0.01, 'Chord (m)') + num('aoa_deg', undefined, 0.5, 'Angle of attack (°)')
        + `<span>Endplates</span><span><input type="checkbox" id="endp" ${d.endplates ? 'checked' : ''}></span>`
        : num('width', undefined, 0.01, 'Width (m)') + num('depth', undefined, 0.01, 'Depth (m)')}
      ${num('position', 0, 0.01, 'X left + (m)')}${num('position', 1, 0.01, 'Y rear + (m)')}${num('position', 2, 0.01, 'Z up + (m)')}
      ${r ? `<span>Plan area</span><span>${fmt(r.area, 3)} m²</span><span>Downforce coefficient</span><span>${fmt(r.cl_down, 2)}${r.stalled ? ' (stalled)' : ''}${r.capped ? ' (capped, 7.4.5)' : ''}</span><span>Drag coefficient</span><span>${fmt(r.cd, 3)}</span>` : ''}
    </div>
    <div class="inl"><button id="adel">Delete</button><button id="aback">All aero</button></div>
    ${aeroTable()}`;
  el.querySelectorAll('input[data-k]').forEach((inp) => inp.onchange = () => {
    const v = parseFloat(inp.value);
    if (Number.isNaN(v)) return;
    change(() => { const x = model.aero.find((y) => y.id === d.id); if (inp.dataset.i === 'undefined') x[inp.dataset.k] = v; else x[inp.dataset.k][+inp.dataset.i] = v; });
  });
  if ($('endp')) $('endp').onchange = (e) => change(() => { model.aero.find((y) => y.id === d.id).endplates = e.target.checked; });
  $('adel').onclick = () => change(() => { model.aero = model.aero.filter((y) => y.id !== d.id); selAero = null; });
  $('aback').onclick = () => { selAero = null; redraw(); };
}

function drawAeroBudget() {   // footer in Aero, at 120 km/h
  if (!aeroRes) { $('budget').innerHTML = ''; return; }
  const s = aeroRes.speeds.find((x) => x.kmh === 120) || aeroRes.speeds[0];
  $('budget').innerHTML = `
    <div><span>Downforce at ${s.kmh} km/h</span><b>${fmt(s.downforce_N, 0)} N</b></div>
    <div><span>Front / rear axle</span><b>${fmt(s.downforce_front_N, 0)} / ${fmt(s.downforce_rear_N, 0)} N</b></div>
    <div><span>Drag at ${s.kmh} km/h</span><b>${fmt(s.drag_N, 0)} N</b> <span>${fmt(s.drag_power_kW, 1)} kW</span></div>
    <div><span>Frontal area</span><b>${fmt(aeroRes.frontal_area_m2, 2)} m²</b></div>`;
}

// ---------- suspension workspace ----------
// suspension colours by line kind (the third item of a layout line)
const SCOL = { arm: 0x1a7f37, tie: 0x8250df, strut: 0xbf8700, upright: 0x8c959f, shaft: 0x57606a };
// layouts come from Python (fbeam_suspension.TOPOLOGY): names, pickups, moving points and lines to draw.
// A line is [a, b, kind, once]: once = draw a single copy at its own position (Panhard rod, Watt's linkage,
// axle tube); a '~' before a key means that point's mirror, so a solid axle can join the left and right ends.
const LAY = (t) => susp?.layouts?.[t] || { name: t, about: '', moving: [], inner: [], lines: [], single: [] };
const travelNow = () => +$('travel').value;

function frameAt(res, t) {   // solver frame (points, spin axis) nearest to travel t (mm)
  let best = res.frames[0];
  for (const f of res.frames) if (Math.abs(f.travel_mm - t) < Math.abs(best.travel_mm - t)) best = f;
  return best;
}
function wcAt(res, t) {                  // wheel centre at travel t, interpolated on the solver's 2.5 mm steps
  const tr = res.curves.travel_mm, wc = res.curves.wc;
  if (!wc) return frameAt(res, t).pts.WC;
  if (t <= tr[0]) return wc[0];
  for (let i = 1; i < tr.length; i++) {
    if (t <= tr[i]) { const f = (t - tr[i - 1]) / (tr[i] - tr[i - 1]); return wc[i - 1].map((v, k) => v + (wc[i][k] - v) * f); }
  }
  return wc[wc.length - 1];
}
// all left-side points of a corner at travel t
function cornerPoints(res, t) { return { ...res.points, ...frameAt(res, t).pts }; }

// links, wheels and (in Suspension) pickup spheres of every corner at the slider travel, both sides
function drawSuspension() {
  suspG.clear();
  if (!susp || !model.suspension) return;
  const full = workspace === 'suspension';
  if (!full && !$('ghosts').checked) return;
  const op = full ? 1 : 0.45;
  for (const [name, res] of Object.entries(susp.corners)) {
    const f = frameAt(res, full ? travelNow() : 0), P = { ...res.points, ...f.pts };
    const links = LAY(res.type).lines.map((l) => [...l]);
    if (P.DIFF && !model.driveline) links.push(['DIFF', 'WC', 'shaft']);   // with a driveline, drawDriveline draws the shafts
    const rad = { arm: 0.012, tie: 0.009, strut: 0.022, upright: 0.016, shaft: 0.013 };
    for (const side of [1, -1]) {
      const m = (p) => v3([p[0] * side, p[1], p[2]]);
      const at = (k) => k.startsWith('~') ? (P[k.slice(1)] && v3([-P[k.slice(1)][0] * side, P[k.slice(1)][1], P[k.slice(1)][2]])) : (P[k] && m(P[k]));
      for (const [a, b, kind, once] of links) {
        if (once && side < 0) continue;                  // Panhard rod, Watt's linkage, axle tube: drawn once, as they are
        const pa = once && !a.startsWith('~') ? (P[a] && v3(P[a])) : at(a), pb = once && !b.startsWith('~') ? (P[b] && v3(P[b])) : at(b);
        if (pa && pb) suspG.add(tubeMesh(pa, pb, rad[kind], SCOL[kind], op));
      }
      const spin = [f.spin[0] * side, f.spin[1], f.spin[2]];
      const w = new THREE.Mesh(new THREE.CylinderGeometry(TYRE.r, TYRE.r, TYRE.w, 36),
        new THREE.MeshStandardMaterial({ color: 0x57606a, transparent: true, opacity: full ? 0.45 : 0.3, depthWrite: false }));
      w.position.copy(m(P.WC));
      w.quaternion.setFromUnitVectors(new THREE.Vector3(0, 1, 0), v3(spin).normalize());
      suspG.add(w);
      if (!full) continue;
      for (const k of Object.keys(res.points)) {
        if (side < 0 && LAY(res.type).single.includes(k)) continue;
        const mount = res.mounts.find((x) => x.point === k);
        const isSel = selPoint && selPoint.corner === name && selPoint.key === k;
        const col = isSel ? 0xcf222e : mount ? (mount.on_frame && mount.braced ? SCOL.arm : 0xff8c00) : 0xffffff;
        const s = new THREE.Mesh(new THREE.SphereGeometry(isSel ? 0.026 : 0.018, 14, 10), new THREE.MeshStandardMaterial({ color: col }));
        s.position.copy(m(P[k]));
        suspG.add(s);
      }
    }
  }
}

function focusCorner(name) {           // frame the left corner, from outboard and ahead (front) or behind (rear)
  const res = susp?.corners[name];
  if (!res) return;
  const wc = v3(res.points.WC), front = name === 'front' ? -1 : 1;
  controls.target.set(wc.x * 0.55, wc.y + 0.1, wc.z);
  camera.position.set(wc.x + 1.5, wc.y + 0.75, wc.z + front * 1.3);
  controls.update();
}

function nearestPickup(e) {   // pickup within SNAP_PX of the pointer, either side: { corner, key } or null
  camera.updateMatrixWorld();
  const r = renderer.domElement.getBoundingClientRect();
  let best = null, bestD = SNAP_PX;
  for (const [name, res] of Object.entries(susp?.corners || {})) {
    const P = cornerPoints(res, travelNow());
    for (const k of Object.keys(res.points)) {
      for (const side of [1, -1]) {
        if (side < 0 && LAY(res.type).single.includes(k)) continue;   // drawn once, so only pickable once
        const q = v3([P[k][0] * side, P[k][1], P[k][2]]).project(camera);
        const d = Math.hypot(r.left + (q.x + 1) / 2 * r.width - e.clientX, r.top + (1 - q.y) / 2 * r.height - e.clientY);
        if (d < bestD) { bestD = d; best = { corner: name, key: k }; }
      }
    }
  }
  return best;
}

// left panel in Suspension: every pickup per corner with its frame-mount status (7.1.6)
function drawSuspList() {
  const el = $('leftsusp');
  if (!model.suspension) { el.innerHTML = '<p class="quiet">This model has no suspension. Load the full-chassis example.</p>'; return; }
  el.innerHTML = Object.entries(susp.corners).map(([name, res]) => `
    <h2>${name} · ${esc(LAY(res.type).name)}</h2>
    <ul>${Object.keys(res.points).map((k) => {
      const mount = res.mounts.find((x) => x.point === k);
      const spec = model.suspension[name].points[k];
      const st = mount ? (!mount.on_frame ? `<span class="bad">not on a joint</span>` : mount.braced ? `<span class="ok">on ${esc(mount.joint)}</span>` : `<span class="bad">${esc(mount.joint)} not braced</span>`)
        : LAY(res.type).moving.includes(k) ? 'moves' : '';
      const sel = selPoint && selPoint.corner === name && selPoint.key === k;
      return `<li data-c="${name}" data-k="${k}" class="${sel ? 'sel' : ''}"><span>${k} · ${res.labels[k]}${spec?.node ? ` (${spec.node})` : ''}</span><span class="q">${st}</span></li>`;
    }).join('')}</ul>`).join('');
  el.querySelectorAll('li').forEach((li) => li.onclick = () => { selPoint = { corner: li.dataset.c, key: li.dataset.k }; suspCorner = li.dataset.c; redraw(); });
}

// small SVG line chart of one curve over wheel travel, with the usable band and the slider position
function chart(title, xs, ys, unit, cur, usable, digits = 1) {
  const pts = xs.map((x, i) => [x, ys[i]]).filter((p) => p[1] !== null && p[1] !== undefined && Number.isFinite(p[1]));
  if (pts.length < 2) return '';
  const W = 270, H = 104, L = 40, B = 18, T = 6;
  const x0 = Math.min(...pts.map((p) => p[0])), x1 = Math.max(...pts.map((p) => p[0]));
  let y0 = Math.min(...pts.map((p) => p[1])), y1 = Math.max(...pts.map((p) => p[1]));
  if (y1 - y0 < 1e-6) { y0 -= 1; y1 += 1; }
  const pad = (y1 - y0) * 0.08; y0 -= pad; y1 += pad;
  const X = (x) => L + (x - x0) / (x1 - x0) * (W - L - 6), Y = (y) => T + (y1 - y) / (y1 - y0) * (H - T - B);
  const path = pts.map((p, i) => `${i ? 'L' : 'M'}${X(p[0]).toFixed(1)},${Y(p[1]).toFixed(1)}`).join('');
  const here = pts.reduce((b, p) => (Math.abs(p[0] - cur) < Math.abs(b[0] - cur) ? p : b), pts[0]);
  const zeroY = y0 < 0 && y1 > 0 ? `<line x1="${L}" x2="${W - 6}" y1="${Y(0)}" y2="${Y(0)}" stroke="currentColor" stroke-opacity=".25"/>` : '';
  const band = usable ? `<rect x="${X(Math.max(usable[0], x0))}" y="${T}" width="${X(Math.min(usable[1], x1)) - X(Math.max(usable[0], x0))}" height="${H - T - B}" fill="#2f6fdf" fill-opacity=".07"/>` : '';
  return `<div class="chart"><div class="t"><span>${title}</span><span>${fmt(here[1], digits)} ${unit} at ${fmt(here[0], 0)} mm</span></div>
    <svg viewBox="0 0 ${W} ${H}" style="color:var(--ink)">${band}${zeroY}
      <line x1="${X(0)}" x2="${X(0)}" y1="${T}" y2="${H - B}" stroke="currentColor" stroke-opacity=".25"/>
      <path d="${path}" fill="none" stroke="#2f6fdf" stroke-width="1.8"/>
      <circle cx="${X(here[0])}" cy="${Y(here[1])}" r="3.5" fill="#cf222e"/>
      <text x="${L - 4}" y="${Y(y1 - pad) + 4}" text-anchor="end" font-size="10" fill="currentColor" fill-opacity=".6">${fmt(y1 - pad, digits)}</text>
      <text x="${L - 4}" y="${Y(y0 + pad) + 4}" text-anchor="end" font-size="10" fill="currentColor" fill-opacity=".6">${fmt(y0 + pad, digits)}</text>
      <text x="${L}" y="${H - 4}" font-size="10" fill="currentColor" fill-opacity=".6">${fmt(x0, 0)}</text>
      <text x="${W - 6}" y="${H - 4}" text-anchor="end" font-size="10" fill="currentColor" fill-opacity=".6">+${fmt(x1, 0)} mm</text>
    </svg></div>`;
}

// SVG chart of several series over one x axis (steering angles over rack travel)
function multiChart(title, xs, series, unit, xunit) {
  const W = 270, H = 120, L = 34, B = 18, T = 6;
  const all = series.flatMap((s) => s.ys).filter(Number.isFinite);
  if (xs.length < 2 || !all.length) return '';
  const x0 = Math.min(...xs), x1 = Math.max(...xs), y0 = Math.min(0, ...all), y1 = Math.max(...all);
  const X = (x) => L + (x - x0) / (x1 - x0) * (W - L - 6), Y = (y) => T + (y1 - y) / (y1 - y0 || 1) * (H - T - B);
  const paths = series.map((s) => `<path d="${xs.map((x, i) => `${i ? 'L' : 'M'}${X(x).toFixed(1)},${Y(s.ys[i]).toFixed(1)}`).join('')}" fill="none" stroke="${s.c}" stroke-width="1.8" ${s.dash ? 'stroke-dasharray="4 3"' : ''}/>`).join('');
  const key = series.map((s) => `<span style="color:${s.c}">■</span> ${s.n}`).join(' &nbsp; ');
  return `<div class="chart"><div class="t"><span>${title}</span><span>${key}</span></div>
    <svg viewBox="0 0 ${W} ${H}" style="color:var(--ink)">${paths}
      <text x="${L - 4}" y="${Y(y1) + 8}" text-anchor="end" font-size="10" fill="currentColor" fill-opacity=".6">${fmt(y1, 0)}${unit}</text>
      <text x="${L - 4}" y="${Y(y0)}" text-anchor="end" font-size="10" fill="currentColor" fill-opacity=".6">${fmt(y0, 0)}</text>
      <text x="${W - 6}" y="${H - 4}" text-anchor="end" font-size="10" fill="currentColor" fill-opacity=".6">${fmt(x1, 0)} ${xunit}</text></svg></div>`;
}

// the 'car' tab in Suspension: steering (rack, Ackermann) and roll / load transfer from core.dynamics
function drawCarDynamics(el) {
  let d;
  try { d = JSON.parse(core.dynamics(JSON.stringify(model), JSON.stringify(susp))); }
  catch (e) { el.innerHTML = `<p class="bad">${esc(pyError(e))}</p>`; return; }
  const tabs = Object.keys(susp.corners).concat(['car']).map((n) => `<button data-cn="${n}" class="${n === 'car' ? 'on' : ''}">${n}</button>`).join(' ');
  const f = model.suspension.front || {}, r = model.suspension.rear || {};
  const st = d.steering, ro = d.roll;
  const row = (k, v) => `<span>${k}</span><span>${v}</span>`;
  const verdict = ro ? (ro.lltd_front - ro.weight_front > 0.08 ? 'understeer-leaning' : ro.lltd_front - ro.weight_front < 0.02 ? 'oversteer-leaning' : 'close to neutral') : '';
  el.innerHTML = `<div class="inl">${tabs}</div>
    <h2>Steering (front rack)</h2>
    <div class="kv">
      <span>Rack travel, each way</span><span><input type="number" id="rtravel" step="5" value="${f.rack_travel_mm ?? 70}"> mm</span>
      <span>Rack gain</span><span><input type="number" id="rgain" step="5" value="${f.rack_mm_per_rev ?? 65}"> mm/rev</span>
      ${st ? row('Overall ratio', fmt(st.overall_ratio, 1) + ' : 1') + row('Lock to lock', fmt(st.lock_to_lock_turns, 2) + ' turns')
        + row('Lock, inner / outer', `${fmt(st.lock_inner_deg, 1)}° / ${fmt(st.lock_outer_deg, 1)}°`)
        + row('Ackermann at lock', fmt(st.ackermann_at_lock_pct, 0) + ' %') + row('Turning circle', fmt(st.turning_circle_m, 1) + ' m') : ''}
    </div>
    ${st ? multiChart('Steer angle over rack travel', st.rack_mm, [{ n: 'inner', ys: st.inner_deg, c: '#2f6fdf' }, { n: 'outer', ys: st.outer_deg, c: '#e0782a' },
      { n: '100 % Ackermann', ys: st.ideal_inner_deg, c: '#8c959f', dash: true }], '°', 'mm') : ''}
    <h2>Roll and load transfer (per g)</h2>
    <div class="kv">
      <span>Front anti-roll bar</span><span><input type="number" id="arbf" step="50" value="${f.arb_Nm_per_deg ?? 0}"> N·m/°</span>
      <span>Rear anti-roll bar</span><span><input type="number" id="arbr" step="50" value="${r.arb_Nm_per_deg ?? 0}"> N·m/°</span>
      ${ro ? row('Roll gradient', fmt(ro.roll_gradient_deg_per_g, 2) + ' °/g')
        + row('Roll stiffness front / rear', `${fmt(ro.roll_stiffness_front_Nm_per_deg, 0)} / ${fmt(ro.roll_stiffness_rear_Nm_per_deg, 0)} N·m/°`)
        + row('Load transfer front (LLTD)', fmt(ro.lltd_front * 100, 1) + ' %') + row('Weight on the front axle', fmt(ro.weight_front * 100, 1) + ' %')
        + row('Balance tendency', verdict)
        + row('Roll-axis height at the CG', fmt(ro.roll_axis_height_m * 1000, 0) + ' mm') + row('Sprung CG height', fmt(ro.sprung_cg_height_m * 1000, 0) + ' mm')
        + row('Ride frequency front / rear', `${fmt(ro.ride_freq_front_hz, 2)} / ${fmt(ro.ride_freq_rear_hz, 2)} Hz`) : ''}
    </div>
    <p class="quiet">Steady state, roll-axis method: roll stiffness from the wheel rates in series with the tyre (${ro ? fmt(ro.tyre_rate_N_per_mm, 0) : 200} N/mm, estimate) plus the bars. Rule of thumb: front load transfer a few percent above the front weight is near neutral. ${d.notes.map(esc).join(' ')}</p>`;
  el.querySelectorAll('[data-cn]').forEach((b) => b.onclick = () => { suspCorner = b.dataset.cn; selPoint = null; redraw(); if (b.dataset.cn !== 'car') focusCorner(suspCorner); });
  const set = (id, corner, key) => { $(id).onchange = (e) => { const v = parseFloat(e.target.value); if (v >= 0) change(() => { model.suspension[corner][key] = v; }); }; };
  set('rtravel', 'front', 'rack_travel_mm'); set('rgain', 'front', 'rack_mm_per_rev'); set('arbf', 'front', 'arb_Nm_per_deg'); set('arbr', 'rear', 'arb_Nm_per_deg');
}

// right panel in Suspension: corner tabs, layout picker, selected pickup, rates, static geometry, curves
function drawSuspInspector(el) {
  if (workspace !== 'suspension') return;
  if (model.suspension && suspCorner === 'car' && susp.corners.front && susp.corners.rear) return drawCarDynamics(el);
  if (!model.suspension || !model.suspension[suspCorner]) {
    if (model.suspension && Object.keys(model.suspension).length && suspCorner !== Object.keys(model.suspension)[0]) { suspCorner = Object.keys(model.suspension)[0]; return drawSuspInspector(el); }
    el.innerHTML = '<p class="quiet">No suspension in this model.</p>'; return;
  }
  const name = suspCorner, res = susp.corners[name], corner = model.suspension[name];
  const tabs = Object.keys(model.suspension).concat(susp.corners.front && susp.corners.rear ? ['car'] : [])
    .map((n) => `<button data-cn="${n}" class="${n === name ? 'on' : ''}">${n}</button>`).join(' ');
  const layoutPicker = `<div class="kv"><span>Layout</span><span><select id="layout">${Object.entries(susp.layouts || {})
      .map(([k, v]) => `<option value="${k}" ${k === corner.type ? 'selected' : ''}>${esc(v.name)}</option>`).join('')}</select></span></div>
    <p class="quiet">${esc(LAY(corner.type).about)} Switching lays the new layout out around the same wheel centre; then put its pickups on frame joints (7.1.6).</p>
    ${res && res.mounts.some((m) => !m.on_frame) ? '<button id="mkjoints">Make frame joints at the free pickups</button>' : ''}`;
  const bindLayout = () => {
    el.querySelectorAll('[data-cn]').forEach((b) => b.onclick = () => { suspCorner = b.dataset.cn; selPoint = null; redraw(); if (b.dataset.cn !== 'car') focusCorner(suspCorner); });
    $('layout').onchange = (e) => change(() => { model = JSON.parse(core.suspension_layout(JSON.stringify(model), name, e.target.value)); selPoint = null; });
    if ($('mkjoints')) $('mkjoints').onclick = () => change(() => { model = JSON.parse(core.pickups_to_joints(JSON.stringify(model), name)); });
  };
  if (!res) {
    const why = (susp.issues || []).filter((i) => i.message.startsWith(name)).map((i) => `<p class="bad">${esc(i.message)}</p>`).join('');
    el.innerHTML = `<div class="inl">${tabs}</div><h2>${name} · ${esc(LAY(corner.type).name)}</h2>${layoutPicker}${why}`;
    bindLayout();
    return;
  }
  const st = res.static, c = res.curves;
  const t = travelNow(), usable = [-st.rebound_mm, st.bump_mm];
  let point = '';
  if (selPoint && selPoint.corner === name) {
    const k = selPoint.key, spec = corner.points[k], P = cornerPoints(res, 0)[k];
    const mount = res.mounts.find((x) => x.point === k);
    let near = null;
    for (const [n, q] of Object.entries(model.nodes)) { const d = Math.hypot(q[0] - P[0], q[1] - P[1], q[2] - P[2]); if (d < 0.06 && (!near || d < near.d)) near = { n, d }; }
    point = `<h2>Pickup ${k}</h2><div><b>${res.labels[k]}</b> · left side (right is mirrored)</div>
      ${spec.node ? `<p>Attached to chassis joint <b>${spec.node}</b>: it moves with the frame.</p><button id="detach">Detach from joint</button>`
        : `<div class="kv">
          <span>X left +</span><span><input type="number" step="0.005" data-i="0" value="${spec[0]}"></span>
          <span>Y rear +</span><span><input type="number" step="0.005" data-i="1" value="${spec[1]}"></span>
          <span>Z up +</span><span><input type="number" step="0.005" data-i="2" value="${spec[2]}"></span></div>
          ${mount && near ? `<button id="attach">Attach to joint ${near.n} (${fmt(near.d * 1000, 0)} mm away)</button>` : ''}`}
      ${mount ? (mount.on_frame ? `<p class="${mount.braced ? 'ok' : 'bad'}">On joint ${esc(mount.joint)}: ${mount.tubes} tubes, 3D span ${fmt(mount.span, 2)} (needs 0.10, rule 7.1.6)${mount.braced ? '' : '. Add a tube out of the plane of the others.'}</p>` : `<p class="bad">Not on a frame joint (rule 7.1.6): ${fmt(mount.distance_mm, 0)} mm from ${esc(mount.nearest)}. Attach it to a joint, or split that tube here.</p>`) : `<p class="quiet">${LAY(res.type).moving.includes(k) ? 'Moves with the wheel.' : ''}</p>`}`;
  }
  const row = (k, v) => `<span>${k}</span><span>${v}</span>`;
  el.innerHTML = `<div class="inl">${tabs}</div>
    <h2>${name} · ${esc(LAY(res.type).name)}</h2>
    ${layoutPicker}
    ${point}
    <h2>Springs and dampers</h2>
    <div class="kv">
      <span>Spring rate</span><span><input type="number" id="k" step="5" value="${corner.spring_N_per_mm}"> N/mm</span>
      <span>Damper stroke</span><span><input type="number" id="stroke" step="10" value="${corner.damper_stroke_mm}"> mm</span>
      ${row('Motion ratio (static)', fmt(st.motion_ratio, 2))}
      ${row('Wheel rate', fmt(st.wheel_rate_N_per_mm, 1) + ' N/mm')}
    </div>
    <h2>Static geometry</h2>
    <div class="kv">
      ${row('Kingpin inclination', st.kpi_deg === null ? '–' : fmt(st.kpi_deg, 1) + '°')}
      ${row('Caster', st.caster_deg === null ? '–' : fmt(st.caster_deg, 1) + '°')}
      ${row('Scrub radius', st.scrub_radius_mm === null ? '–' : fmt(st.scrub_radius_mm, 0) + ' mm')}
      ${row('Mechanical trail', st.trail_mm === null ? '–' : fmt(st.trail_mm, 0) + ' mm')}
      ${row('Roll-centre height', st.roll_centre_mm === null ? '–' : fmt(st.roll_centre_mm, 0) + ' mm')}
      ${row('Track', fmt(st.track_mm, 0) + ' mm')}
      ${row('Travel: bump / rebound', `${fmt(st.bump_mm, 0)} / ${fmt(st.rebound_mm, 0)} mm`)}
      <span>Usable travel (7.1.1: ≥ 150)</span><span class="${st.usable_travel_mm >= 150 ? 'ok' : 'bad'}">${fmt(st.usable_travel_mm, 0)} mm</span>
    </div>
    <h2>Over wheel travel (blue band = usable)</h2>
    ${chart('Camber', c.travel_mm, c.camber_deg, '°', t, usable, 2)}
    ${chart('Toe, + = in (bump steer)', c.travel_mm, c.toe_deg, '°', t, usable, 2)}
    ${chart('Roll-centre height (heave)', c.travel_mm, c.roll_centre_mm, 'mm', t, usable, 0)}
    ${chart('Motion ratio', c.travel_mm, c.motion_ratio, '', t, usable, 3)}
    ${chart('Wheel rate', c.travel_mm, c.wheel_rate_N_per_mm, 'N/mm', t, usable, 1)}
    ${c.axle_shift_mm ? chart('Axle sideways shift', c.travel_mm, c.axle_shift_mm, 'mm', t, usable, 1) : chart('Track change (both sides)', c.travel_mm, c.track_change_mm, 'mm', t, usable, 1)}
    ${c.halfshaft_deg ? chart('Halfshaft angle', c.travel_mm, c.halfshaft_deg, '°', t, usable, 1) + chart('Halfshaft plunge', c.travel_mm, c.plunge_mm, 'mm', t, usable, 1) : ''}
    <p class="quiet">Kinematics with the steering straight ahead. Right side is the mirror image. Pickups on braced frame joints are green, others orange. ${LAY(res.type).steered ? 'Kingpin, caster, scrub and trail are taken about the axis the upright turns about when the rack moves (a virtual axis for multi-link).' : 'This layout has no steering link, so it has no kingpin axis.'}</p>`;
  bindLayout();
  el.querySelectorAll('input[data-i]').forEach((inp) => inp.onchange = () => {
    const v = parseFloat(inp.value);
    if (!Number.isNaN(v)) change(() => { model.suspension[name].points[selPoint.key][+inp.dataset.i] = v; });
  });
  if ($('detach')) $('detach').onclick = () => change(() => { const k = selPoint.key; model.suspension[name].points[k] = [...model.nodes[corner.points[k].node]]; });
  if ($('attach')) $('attach').onclick = () => change(() => {
    const k = selPoint.key, P = corner.points[k];
    let best = null;
    for (const [n, q] of Object.entries(model.nodes)) { const d = Math.hypot(q[0] - P[0], q[1] - P[1], q[2] - P[2]); if (!best || d < best.d) best = { n, d }; }
    model.suspension[name].points[k] = { node: best.n };
  });
  $('k').onchange = (e) => { const v = parseFloat(e.target.value); if (v > 0) change(() => { model.suspension[name].spring_N_per_mm = v; }); };
  $('stroke').onchange = (e) => { const v = parseFloat(e.target.value); if (v > 0) change(() => { model.suspension[name].damper_stroke_mm = v; }); };
}

function drawSuspBudget() {   // footer in Suspension: travel, wheel rate, roll centre, bump steer per corner
  if (!susp || !Object.keys(susp.corners).length) { $('budget').innerHTML = '<div><span>Suspension</span><b>none in this model</b></div>'; return; }
  $('budget').innerHTML = Object.entries(susp.corners).map(([name, res]) => {
    const st = res.static, c = res.curves;
    const near = c.travel_mm.map((x, i) => [x, i]).filter(([x]) => Math.abs(x) <= 60).map(([, i]) => i);
    const steer = Math.max(...near.map((i) => Math.abs(c.toe_deg[i])));
    const gain = c.camber_deg[near[near.length - 1]];
    return `<div><span>${name} travel</span><b class="${st.usable_travel_mm >= 150 ? 'ok' : 'bad'}">${fmt(st.usable_travel_mm, 0)} mm</b></div>
      <div><span>${name} wheel rate</span><b>${fmt(st.wheel_rate_N_per_mm, 1)} N/mm</b></div>
      <div><span>${name} roll centre</span><b>${st.roll_centre_mm === null ? '–' : fmt(st.roll_centre_mm, 0) + ' mm'}</b></div>
      <div><span>${name} bump steer ±60 mm</span><b>${fmt(steer, 2)}°</b> <span>camber at +60: ${fmt(gain, 1)}°</span></div>`;
  }).join('');
}

// ---------- base car workspace: a vanilla BeamNG vehicle from the user's own install ----------
// Nothing from the game ships with FBeam. The user picks their BeamNG.drive/content/vehicles folder
// (or zips, or an unpacked mod folder); only the text files inside (.jbeam, .pc, .json) are read,
// entry by entry with zip.js, and handed to Python (fbeam_beamng). It works like the game's Vehicle
// Config menu: a parts tree with the alternatives for each slot, tuning sliders by category, and a
// .pc configuration to save. State: lib (the picked files), cat (core catalog), veh (the configured
// vehicle from fbeam_beamng.configure), vehEdit (the user's slot and variable changes).
let lib = null, cat = null, veh = null, vehError = null, vehBusy = '';
let vehEdit = { model: null, config: null, parts: {}, vars: {} };
let vehFilter = '', commonLoaded = false;
const vehG = new THREE.Group();
scene.add(vehG);
const TEXT_FILE = /\.(jbeam|pc|json)$/i;

// the picked files: zips by name, and loose files under vehicles/<model>/ (unpacked mods)
function libraryFrom(fileList) {
  const zips = {}, loose = {};
  for (const f of fileList) {
    const rel = (f.webkitRelativePath || f.name).replace(/\\/g, '/');
    if (/\.zip$/i.test(f.name)) zips[f.name.replace(/\.zip$/i, '')] = f;
    else if (TEXT_FILE.test(f.name)) {
      const i = rel.indexOf('vehicles/');
      if (i >= 0) loose[rel.slice(i)] = f;
    }
  }
  return { zips, loose };
}

// text entries of one zip that pass keep(path); reads only those entries, not the whole archive
async function zipTexts(file, keep) {
  const reader = new ZipReader(new BlobReader(file));
  const out = {};
  try {
    for (const e of await reader.getEntries()) {
      const name = e.filename.replace(/\\/g, '/');
      if (!e.directory && keep(name)) out[name] = await e.getData(new TextWriter());
    }
  } finally { await reader.close(); }
  return out;
}

// the vehicle list: info.json and the configs of every vehicle zip / loose folder
async function scanLibrary() {
  const entries = {};
  const zipNames = Object.keys(lib.zips).filter((n) => n.toLowerCase() !== 'common');
  let done = 0;
  for (const z of zipNames) {
    vehBusy = `Reading the vehicle list: ${++done} of ${zipNames.length} (${z})`;
    drawVehList();
    try {
      const texts = await zipTexts(lib.zips[z], (p) => /^vehicles\/[^/]+\/(info[^/]*\.json|[^/]+\.pc)$/i.test(p));
      for (const [p, t] of Object.entries(texts)) {
        const m = p.split('/')[1], base = p.split('/').pop();
        const e = entries[m] || (entries[m] = { info: null, configs: {}, zip: z });
        if (base.toLowerCase() === 'info.json') e.info = t;
        else if (/\.pc$/i.test(base)) { const c = base.replace(/\.pc$/i, ''); if (!(c in e.configs)) e.configs[c] = null; }
        else { const c = base.replace(/^info_/i, '').replace(/\.json$/i, ''); e.configs[c] = t; }
      }
    } catch (err) { /* not a vehicle zip */ }
  }
  for (const [p, f] of Object.entries(lib.loose)) {             // unpacked mods
    const m = p.split('/')[1], base = p.split('/').pop();
    if (!/^(info[^/]*\.json|[^/]+\.pc)$/i.test(base) || p.split('/').length !== 3) continue;
    const e = entries[m] || (entries[m] = { info: null, configs: {}, zip: null });
    const t = await f.text();
    if (base.toLowerCase() === 'info.json') e.info = t;
    else if (/\.pc$/i.test(base)) { const c = base.replace(/\.pc$/i, ''); if (!(c in e.configs)) e.configs[c] = null; }
    else e.configs[base.replace(/^info_/i, '').replace(/\.json$/i, '')] = t;
  }
  const keep = {};                                   // what Python needs: {model: {info, configs: {name: info text}}}
  for (const [m, e] of Object.entries(entries)) keep[m] = { info: e.info, configs: e.configs };
  cat = JSON.parse(vehpy.catalog(JSON.stringify(keep)));
  for (const v of cat) v.zip = entries[v.model].zip;
  vehBusy = '';
}

// load one vehicle's text files (and vehicles/common the first time) into Python, then configure it
async function openVehicle(model, config) {
  const v = cat.find((x) => x.model === model);
  try {
    vehError = null;
    if (!commonLoaded) {
      vehBusy = 'Reading the shared parts (vehicles/common): once per session…'; drawVehList();
      let texts = {};
      if (lib.zips.common) texts = await zipTexts(lib.zips.common, (p) => /^vehicles\/common\/.*\.jbeam$/i.test(p));
      for (const [p, f] of Object.entries(lib.loose)) if (p.startsWith('vehicles/common/')) texts[p] = await f.text();
      vehBusy = `Parsing ${Object.keys(texts).length} shared part files…`; drawVehList();
      await new Promise((r) => setTimeout(r, 30));
      vehpy.add_files(JSON.stringify(texts));
      commonLoaded = true;
    }
    if (!JSON.parse(vehpy.held_models()).includes(model)) {
      vehBusy = `Reading ${v ? v.name : model}…`; drawVehList();
      let texts = {};
      if (v && v.zip) texts = await zipTexts(lib.zips[v.zip], (p) => p.startsWith(`vehicles/${model}/`) && TEXT_FILE.test(p));
      for (const [p, f] of Object.entries(lib.loose)) if (p.startsWith(`vehicles/${model}/`)) texts[p] = await f.text();
      await new Promise((r) => setTimeout(r, 30));
      vehpy.add_files(JSON.stringify(texts));
    }
    vehEdit = { model, config: config || null, parts: {}, vars: {} };
    vehBusy = '';
    configureVehicle();
    fitCamera();
  } catch (err) { vehBusy = ''; vehError = pyError(err); redraw(); }
}

// rebuild the configured vehicle from the held files and the user's edits (fast: no parsing)
function configureVehicle() {
  try {
    veh = JSON.parse(vehpy.configure(vehEdit.model, vehEdit.config, JSON.stringify(vehEdit.parts), JSON.stringify(vehEdit.vars)));
    vehEdit.config = veh.config;
    vehError = null;
  } catch (err) { vehError = pyError(err); }
  redraw();
}

// the vehicle's node-and-beam structure: beams as one line set, nodes as points, coloured by part
function drawVehicle() {
  vehG.clear();
  vehG.visible = workspace === 'vehicle';
  if (!veh || workspace !== 'vehicle') return;
  const g = veh.geometry, pos = [];
  for (const [a, b] of g.beams) pos.push(...v3(g.nodes[a]).toArray(), ...v3(g.nodes[b]).toArray());
  const lines = new THREE.BufferGeometry();
  lines.setAttribute('position', new THREE.Float32BufferAttribute(pos, 3));
  vehG.add(new THREE.LineSegments(lines, new THREE.LineBasicMaterial({ color: 0x8c959f, transparent: true, opacity: 0.55 })));
  const pts = new THREE.BufferGeometry();
  pts.setAttribute('position', new THREE.Float32BufferAttribute(Object.values(g.nodes).flatMap((p) => v3(p).toArray()), 3));
  vehG.add(new THREE.Points(pts, new THREE.PointsMaterial({ color: 0x2f6fdf, size: 0.025 })));
}

// left panel: pick the install folder, then the vehicle list (cars first)
function drawVehList() {
  const el = $('leftveh');
  if (workspace !== 'vehicle') return;
  if (!lib) {
    el.innerHTML = `<h2>Base car</h2>
      <p class="quiet">Start from a vanilla BeamNG vehicle. Pick the <b>content/vehicles</b> folder of your BeamNG.drive install
      (or a mod's unpacked folder, or single vehicle zips). Only its jbeam, config and info files are read, in your browser; nothing is uploaded or bundled with FBeam.</p>
      <div class="inl"><button id="bngpick" class="primary">Choose vehicles folder…</button><button id="bngpickzips">Pick zips…</button></div>`;
    $('bngpick').onclick = () => $('bngdir').click();
    $('bngpickzips').onclick = () => $('bngzips').click();
    return;
  }
  if (vehBusy || !cat) { el.innerHTML = `<h2>Base car</h2><p class="quiet">${esc(vehBusy || 'Reading…')}</p>`; return; }
  const f = vehFilter.toLowerCase();
  const list = cat.filter((v) => !f || `${v.name} ${v.brand} ${v.model} ${v.type}`.toLowerCase().includes(f));
  el.innerHTML = `<h2>Vehicles <span class="q">(${cat.length})</span></h2>
    <div class="inl"><input id="vehfilter" placeholder="Filter" value="${esc(vehFilter)}"></div>
    <ul>${list.map((v) => `<li data-m="${esc(v.model)}" class="${veh && veh.model === v.model ? 'sel' : ''}">
      <span>${esc(v.brand ? v.brand + ' ' : '')}${esc(v.name)}</span><span class="q">${esc(v.type || '')} · ${v.configs.length}</span></li>`).join('')}</ul>
    <div class="inl"><button id="bngagain">Choose another folder…</button></div>`;
  $('vehfilter').oninput = (e) => { vehFilter = e.target.value; drawVehList(); $('vehfilter').focus(); $('vehfilter').setSelectionRange(vehFilter.length, vehFilter.length); };
  el.querySelectorAll('li[data-m]').forEach((li) => li.onclick = () => openVehicle(li.dataset.m));
  $('bngagain').onclick = () => $('bngdir').click();
}

// right panel: configuration, parts tree (a select per slot) and tuning sliders by category
function drawVehInspector(el) {
  if (!veh) { el.innerHTML = `<p class="quiet">${vehError ? esc(vehError) : 'Pick a vehicle on the left.'}</p>`; return; }
  const v = cat && cat.find((x) => x.model === veh.model);
  const cfgTitle = (c) => { const ci = v && v.configs.find((x) => x.name === c); return ci ? ci.title : c; };
  const slotRow = (n, depth) => {
    const opts = n.options.length ? n.options : [[n.part, n.title]];
    const sel = `<select data-slot="${esc(n.slot)}">${n.core ? '' : `<option value="" ${n.part ? '' : 'selected'}>(empty)</option>`}
      ${opts.map(([p, t]) => `<option value="${esc(p)}" ${p === n.part ? 'selected' : ''}>${esc(t)}</option>`).join('')}</select>`;
    const kids = n.children.filter((c) => c.options.length || c.children.length || c.part);
    return `<li style="padding-left:${depth * 10}px"><span class="q">${esc(n.description || n.slot)}</span>${sel}</li>` +
      kids.map((c) => slotRow(c, depth + 1)).join('');
  };
  const cats = {};
  for (const x of veh.variables) if (!x.hidden) (cats[x.category + (x.subCategory ? ' · ' + x.subCategory.trim() : '')] ||= []).push(x);
  const tune = Object.entries(cats).sort().map(([c, xs]) => `<h3>${esc(c)}</h3><div class="kv">${xs.map((x) => {
    const step = x.step || (typeof x.max === 'number' && typeof x.min === 'number' ? (x.max - x.min) / 100 : 0.01);
    return `<span title="${esc(x.description)}">${esc(x.title)}</span><span><input type="range" data-var="${esc(x.name)}" min="${x.min}" max="${x.max}" step="${step}" value="${x.value}">
      <b>${fmt(Number(x.value), step < 0.01 ? 3 : step < 1 ? 2 : 0)}</b> ${esc(x.unit)}</span>`;
  }).join('')}</div>`).join('');
  el.innerHTML = `<h2>${esc(v ? (v.brand ? v.brand + ' ' : '') + v.name : veh.model)}</h2>
    <div class="kv"><span>Configuration</span><span><select id="vehcfg">${veh.configs.map((c) => `<option value="${esc(c)}" ${c === veh.config ? 'selected' : ''}>${esc(cfgTitle(c))}</option>`).join('')}</select></span></div>
    ${veh.missing.length ? `<p class="bad">Not found in the picked files: ${veh.missing.map((m) => esc(m[1])).join(', ')}</p>` : ''}
    <div class="inl"><button id="vehsave" class="primary">Save configuration (.pc)…</button><button id="vehreset">Reset to the configuration</button></div>
    <details open><summary><b>Parts</b> <span class="q">(as the game's Parts menu)</span></summary><ul class="vehparts">${slotRow(veh.tree, 0)}</ul></details>
    <details><summary><b>Tuning</b> <span class="q">(${veh.variables.length} variables)</span></summary>${tune}</details>`;
  $('vehcfg').onchange = (e) => { vehEdit = { model: veh.model, config: e.target.value, parts: {}, vars: {} }; configureVehicle(); };
  el.querySelectorAll('select[data-slot]').forEach((s) => s.onchange = () => { vehEdit.parts[s.dataset.slot] = s.value; configureVehicle(); });
  el.querySelectorAll('input[data-var]').forEach((r) => {
    r.oninput = () => { r.nextElementSibling.textContent = fmt(Number(r.value), Number(r.step) < 0.01 ? 3 : Number(r.step) < 1 ? 2 : 0); };
    r.onchange = () => { vehEdit.vars[r.dataset.var] = Number(r.value); configureVehicle(); };
  });
  $('vehreset').onclick = () => { vehEdit = { model: veh.model, config: veh.config, parts: {}, vars: {} }; configureVehicle(); };
  $('vehsave').onclick = () => {
    const name = prompt('Name of the configuration (saved as <name>.pc; put it in your BeamNG user folder, vehicles/' + veh.model + '/)', 'fbeam_' + veh.config);
    if (!name) return;
    const blob = new Blob([vehpy.write_pc(JSON.stringify(veh.pc))], { type: 'application/json' });
    const a = document.createElement('a');
    a.href = URL.createObjectURL(blob);
    a.download = name.replace(/[^\w.-]+/g, '_').replace(/\.pc$/i, '') + '.pc';
    a.click();
  };
}

function drawVehBudget() {
  if (!veh) { $('budget').innerHTML = '<div><span>Base car</span><b>none open</b></div>'; return; }
  const count = (n) => (n.part ? 1 : 0) + n.children.reduce((s, c) => s + count(c), 0);
  $('budget').innerHTML = `<div><span>Vehicle</span><b>${esc(veh.model)}</b> <span>${esc(veh.config || '')}</span></div>
    <div><span>Parts</span><b>${count(veh.tree)}</b></div>
    <div><span>Nodes / beams</span><b>${Object.keys(veh.geometry.nodes).length} / ${veh.geometry.beams.length}</b></div>
    <div><span>Tuning variables</span><b>${veh.variables.length}</b> <span>${Object.keys(vehEdit.vars).length} changed</span></div>`;
}

function vehIssues() {
  if (vehError) return [{ level: 'ERROR', rule: 'vehicle', message: vehError }];
  if (!veh) return [{ level: 'INFO', rule: 'vehicle', message: 'Pick your BeamNG vehicles folder, then a vehicle, to use it as a base.' }];
  const out = veh.missing.map((m) => ({ level: 'WARN', rule: 'vehicle', message: `slot ${m[0]}: part ${m[1]} is not in the picked files (pick the whole content/vehicles folder, common.zip included)` }));
  const changed = Object.keys(vehEdit.parts).length + Object.keys(vehEdit.vars).length;
  out.push({ level: 'PASS', rule: 'vehicle', message: `${veh.model} · ${veh.config}: ${changed ? changed + ' change(s) from the configuration' : 'as configured'}. Save it as a .pc for BeamNG.` });
  return out;
}

$('bngdir').onchange = async (e) => {
  if (!e.target.files.length) return;
  lib = libraryFrom(e.target.files); cat = null; veh = null; commonLoaded = false;
  e.target.value = '';
  await scanLibrary(); redraw();
};
$('bngzips').onchange = async (e) => {
  if (!e.target.files.length) return;
  const more = libraryFrom(e.target.files);
  lib = lib ? { zips: { ...lib.zips, ...more.zips }, loose: { ...lib.loose, ...more.loose } } : more;
  e.target.value = '';
  await scanLibrary(); redraw();
};

// debug hook for automated UI tests: current model and results, SVJ export, picking, and screen
// positions of joints and pickups (client pixels). Not used by the editor itself.
window.fbeam = {
  get model() { return model; },
  get veh() { return veh; },
  get susp() { return susp; },
  svjText() { return core.export_svj(JSON.stringify(model), null); },
  get meshCount() { let n = 0; meshG.traverse((o) => { if (o.isMesh) n++; }); return n; },
  svjBundleZip() { const r = JSON.parse(core.export_svj_bundle(JSON.stringify(model), null, '/tmp/t.zip', svjBundle?.base_dir || null)); return { files: r.files, bytes: py.FS.readFile(r.zip_path) }; },
  pickupScreen(corner, key, side = 1) {
    camera.updateMatrixWorld();
    const P = cornerPoints(susp.corners[corner], travelNow())[key];
    const q = v3([P[0] * side, P[1], P[2]]).project(camera), r = renderer.domElement.getBoundingClientRect();
    return [r.left + (q.x + 1) / 2 * r.width, r.top + (1 - q.y) / 2 * r.height];
  },
  pickAt(x, y, type) { camera.updateMatrixWorld(); const h = pick({ clientX: x, clientY: y }, type); return h && h.object.userData; },
  screenOf(name) {
    camera.updateMatrixWorld();
    const p = v3(model.nodes[name]).project(camera), r = renderer.domElement.getBoundingClientRect();
    return [r.left + (p.x + 1) / 2 * r.width, r.top + (1 - p.y) / 2 * r.height];
  },
};

// start; a failure replaces the loading message
boot().catch((e) => { $('loading').textContent = 'Could not start: ' + e.message; console.error(e); });
