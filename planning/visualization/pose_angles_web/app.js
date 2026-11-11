import * as THREE from 'three';
import { OrbitControls } from './vendor/OrbitControls.js';

const $ = id => document.getElementById(id);
const names = {outer_left: '外侧 · 左后牙', outer_front: '外侧 · 前牙', outer_right: '外侧 · 右后牙', inner_right: '内侧 · 右后牙', inner_front: '内侧 · 前牙', inner_left: '内侧 · 左后牙'};
const angleIds = ['angle', 'azimuth', 'flip'];
['固定轴角度 α', '固定轴方位 β', '工具轴翻转 γ'].forEach((name, i) => {
  const id = angleIds[i];
  $('angles').insertAdjacentHTML('beforeend', `<div class="slider-field"><label for="${id}">${name}<span>°</span></label><input id="${id}-number" type="number" min="-180" max="180" step="0.01" aria-label="${name}数值"><input id="${id}" type="range" min="-180" max="180" step="0.01" aria-label="${name}"></div>`);
});
lucide.createIcons();

let data, region, pose, mode, picked = null, poseSequence = 0, regionSequence = 0;
let renderer, controls, tool, fullTool, workingTool, tooth, jaw, opposing, path;
let toolAxes, referenceAxes, contact;
const scene = new THREE.Scene();
scene.background = new THREE.Color('#edf1f2');
const camera = new THREE.PerspectiveCamera(38, 1, .01, 10000);
camera.up.set(0, 0, 1);
const viewport = $('viewport');
const draw = () => renderer?.render(scene, camera);
const setError = message => { $('error').textContent = message; $('error').hidden = !message; };
const matrix = rows => new THREE.Matrix4().set(...rows[0], 0, ...rows[1], 0, ...rows[2], 0, 0, 0, 0, 1);

async function json(url) {
  const response = await fetch(url);
  const result = await response.json();
  if (!response.ok) throw new Error(result.error || `HTTP ${response.status}`);
  return result;
}

function mesh(record, color = '#d4c6b9', metallic = false) {
  const geometry = new THREE.BufferGeometry();
  geometry.setAttribute('position', new THREE.Float32BufferAttribute(record.positions, 3));
  geometry.setIndex(record.indices);
  geometry.setAttribute('normal', new THREE.Float32BufferAttribute(record.normals, 3));
  if (record.colors) {
    const linear = record.colors.map(v => v < .04045 ? v/12.92 : ((v+.055)/1.055)**2.4);
    geometry.setAttribute('color', new THREE.Float32BufferAttribute(linear, 3));
  }
  const material = new THREE.MeshStandardMaterial({color: record.colors ? '#ffffff' : color,
    vertexColors: Boolean(record.colors), side: THREE.DoubleSide, roughness: metallic ? .36 : .78,
    metalness: metallic ? .52 : 0});
  return new THREE.Mesh(geometry, material);
}

function workingGeometry(geometry) {
  const positions = geometry.getAttribute('position');
  const indices = [];
  const index = geometry.index.array;
  for (let i = 0; i < index.length; i += 3) {
    const ids = [index[i], index[i + 1], index[i + 2]];
    if (ids.every(j => Math.hypot(positions.getX(j), positions.getY(j), positions.getZ(j)) <= 65)) indices.push(...ids);
  }
  const result = geometry.clone();
  result.setIndex(indices);
  // Excluded vertices must not expand the camera's bounding box.
  return result.toNonIndexed();
}

function axes(size, opacity = 1) {
  const group = new THREE.Group();
  [0xda4545, 0x32844b, 0x397acf].forEach((color, i) => {
    const direction = new THREE.Vector3(); direction.setComponent(i, 1);
    const arrow = new THREE.ArrowHelper(direction, new THREE.Vector3(), size, color, size*.17, size*.07);
    arrow.line.material.transparent = arrow.cone.material.transparent = true;
    arrow.line.material.opacity = arrow.cone.material.opacity = opacity;
    group.add(arrow);
  });
  return group;
}

function fit() {
  if (!tool) return;
  scene.updateMatrixWorld(true);
  const scope = $('scope').value;
  const box = new THREE.Box3().setFromObject(scope === 'tooth' ? tooth : jaw);
  if (scope === 'both') box.expandByObject(opposing);
  box.expandByObject($('tool-full').checked ? fullTool : workingTool);
  const center = box.getCenter(new THREE.Vector3());
  const radius = box.getSize(new THREE.Vector3()).length()/2;
  const halfFov = Math.min(camera.fov*Math.PI/360, Math.atan(Math.tan(camera.fov*Math.PI/360)*camera.aspect));
  const distance = radius / Math.sin(halfFov) * 1.12;
  const direction = camera.position.clone().sub(controls.target).normalize();
  if (!direction.lengthSq()) direction.set(1, -1, .8).normalize();
  camera.position.copy(center).addScaledVector(direction, distance);
  controls.target.copy(center);
  controls.minDistance = Math.max(.8, radius*.015);
  controls.maxDistance = Math.max(600, distance*8);
  camera.near = .01;
  camera.far = Math.max(10000, distance*20);
  camera.updateProjectionMatrix(); controls.update(); draw();
}

function showScope() {
  tooth.visible = $('scope').value === 'tooth';
  jaw.visible = !tooth.visible;
  opposing.visible = $('scope').value === 'both';
  fit();
}

function setPair(id, value) {
  $(id).value = value;
  $(`${id}-number`).value = Number(value).toFixed(2);
}

function params() {
  const query = new URLSearchParams({region: region.region, mode, offset: $('offset').value,
    station: $('station').value, angle: $('angle').value, azimuth: $('azimuth').value, flip: $('flip').value});
  if (picked) query.set('picked', picked.join(','));
  return query;
}

async function updatePose() {
  if (!region) return;
  const sequence = ++poseSequence;
  $('copy').disabled = $('save').disabled = true;
  try {
    const result = await json(`/api/pose?${params()}`);
    if (sequence !== poseSequence) return;
    pose = result;
    tool.matrixAutoUpdate = false;
    tool.matrix.copy(matrix(pose.rotation_world)).setPosition(...pose.position_display_mm);
    tool.matrixWorldNeedsUpdate = true;
    referenceAxes.matrixAutoUpdate = false;
    referenceAxes.matrix.copy(matrix(pose.reference_frame_world)).setPosition(...pose.position_display_mm);
    referenceAxes.matrixWorldNeedsUpdate = true;
    contact.position.set(...pose.position_display_mm);
    $('angle-values').textContent = pose.pose_angles_deg.map(a => a.toFixed(2)).join(' / ');
    $('point-values').textContent = pose.point_scan_mm.map(v => v.toFixed(2)).join(' / ');
    $('reference-value').textContent = picked ? '所选牙面' : `牙龈路径 · ${pose.planning_mode === 'simple' ? '区域固定' : '局部牙面'}`;
    $('scene-status').textContent = `${names[region.region]} · ${Number(pose.station_mm).toFixed(2)} / ${region.length_mm.toFixed(2)} mm`;
    $('station').disabled = $('station-number').disabled = Boolean(picked);
    setError(''); draw();
    $('copy').disabled = $('save').disabled = false;
    window.dispatchEvent(new CustomEvent('pose-updated'));
  } catch (error) { if (sequence === poseSequence) setError(error.message); }
}

function installRegion(record) {
  region = record; picked = null;
  if (tooth) { scene.remove(tooth); tooth.geometry.dispose(); tooth.material.dispose(); }
  tooth = mesh(record.tooth_mesh); scene.add(tooth);
  if (path) { scene.remove(path); path.geometry.dispose(); path.material.dispose(); }
  const geometry = new THREE.BufferGeometry().setFromPoints(record.path.map(p => new THREE.Vector3(...p)));
  path = new THREE.Line(geometry, new THREE.LineBasicMaterial({color: 0x009eaa}));
  scene.add(path);
  for (const id of ['station', 'station-number']) $(id).max = record.length_mm;
  setPair('station', record.station_mm);
  $('region').value = record.region;
}

function bindPair(id) {
  $(id).addEventListener('input', () => { setPair(id, $(id).value); updatePose(); });
  $(`${id}-number`).addEventListener('change', () => {
    const input = $(`${id}-number`);
    if (!Number.isFinite(input.valueAsNumber) || input.valueAsNumber < Number(input.min) || input.valueAsNumber > Number(input.max)) {
      setError('数值超出允许范围'); return;
    }
    setPair(id, input.valueAsNumber); updatePose();
  });
}

async function start() {
  data = await json('/api/scene');
  renderer = new THREE.WebGLRenderer({antialias: true, preserveDrawingBuffer: true});
  renderer.setPixelRatio(Math.min(window.devicePixelRatio, 2));
  renderer.setClearColor('#edf1f2');
  renderer.toneMapping = THREE.ACESFilmicToneMapping;
  renderer.toneMappingExposure = 1.;
  viewport.prepend(renderer.domElement);
  controls = new OrbitControls(camera, renderer.domElement);
  controls.addEventListener('change', draw);
  camera.position.set(90, -120, 85);
  scene.add(new THREE.HemisphereLight(0xffffff, 0x798186, 2.));
  const key = new THREE.DirectionalLight(0xffffff, 2.); key.position.set(80, -50, 150); scene.add(key);
  const fill = new THREE.DirectionalLight(0xffffff, 1.); fill.position.set(-100, 100, 50); scene.add(fill);
  jaw = mesh(data.jaw_mesh); opposing = mesh(data.opposing_mesh); scene.add(jaw, opposing);
  tool = new THREE.Group();
  fullTool = mesh(data.tool_mesh, '#c3cbd0', true);
  workingTool = new THREE.Mesh(workingGeometry(fullTool.geometry), fullTool.material);
  fullTool.visible = false;
  toolAxes = axes(12); tool.add(fullTool, workingTool, toolAxes); scene.add(tool);
  referenceAxes = axes(19, .4); referenceAxes.visible = false; scene.add(referenceAxes);
  contact = new THREE.Mesh(new THREE.SphereGeometry(.5, 16, 12), new THREE.MeshBasicMaterial({color: 0xc02e46})); scene.add(contact);
  for (const name of data.regions) $('region').add(new Option(names[name], name));
  installRegion(data.initial);
  angleIds.forEach((id, i) => setPair(id, data.angles_deg[i]));
  setPair('offset', data.standoff_mm);
  mode = data.planning_mode;
  document.querySelectorAll('[data-mode]').forEach(button => {
    button.setAttribute('aria-pressed', String(mode === button.dataset.mode));
    button.addEventListener('click', () => {
      mode = button.dataset.mode; picked = null;
      document.querySelectorAll('[data-mode]').forEach(b => b.setAttribute('aria-pressed', String(b === button)));
      updatePose();
    });
  });
  const scope = new URLSearchParams(location.search).get('scope');
  $('scope').value = ['tooth', 'mandibular', 'both'].includes(scope) ? scope : 'mandibular';
  $('scope').addEventListener('change', () => {
    if (picked) { picked = null; updatePose(); }
    showScope();
  });
  $('region').addEventListener('change', async () => {
    const sequence = ++regionSequence; ++poseSequence;
    $('region').disabled = true;
    try {
      const record = await json(`/api/region?region=${encodeURIComponent($('region').value)}`);
      if (sequence !== regionSequence) return;
      installRegion(record); await updatePose(); showScope();
    } catch (error) { $('region').value = region.region; setError(error.message); }
    finally { $('region').disabled = false; }
  });
  [...angleIds, 'offset', 'station'].forEach(bindPair);
  $('fit').addEventListener('click', fit);
  $('reset').addEventListener('click', () => { angleIds.forEach(id => setPair(id, 0)); updatePose(); });
  $('restore').addEventListener('click', () => { picked = null; setPair('station', region.station_mm); updatePose(); });
  $('tool-full').addEventListener('change', () => { fullTool.visible = $('tool-full').checked; workingTool.visible = !fullTool.visible; fit(); });
  $('axes').addEventListener('change', () => { toolAxes.visible = $('axes').checked; draw(); });
  $('reference').addEventListener('change', () => { referenceAxes.visible = $('reference').checked; draw(); });
  $('copy').addEventListener('click', async () => {
    if (!pose) return;
    try { await navigator.clipboard.writeText(pose.pose_angles_deg.join(' ')); $('scene-status').textContent = '已复制三个角度'; }
    catch { setError('无法访问剪贴板'); }
  });
  $('save').addEventListener('click', () => {
    if (!pose) return;
    const record = {...pose, perception_result: data.perception_result};
    const url = URL.createObjectURL(new Blob([JSON.stringify(record, null, 2)], {type: 'application/json'}));
    const link = document.createElement('a'); link.href = url; link.download = `pose_${region.region}.json`; link.click();
    setTimeout(() => URL.revokeObjectURL(url), 1000);
  });
  const raycaster = new THREE.Raycaster();
  let pointerStart;
  renderer.domElement.addEventListener('pointerdown', event => { pointerStart = [event.clientX, event.clientY]; });
  renderer.domElement.addEventListener('pointerup', event => {
    if (!$('pick').checked || !pointerStart || Math.hypot(event.clientX-pointerStart[0], event.clientY-pointerStart[1]) > 5) return;
    const bounds = renderer.domElement.getBoundingClientRect();
    raycaster.setFromCamera(new THREE.Vector2((event.clientX-bounds.left)/bounds.width*2-1, -(event.clientY-bounds.top)/bounds.height*2+1), camera);
    const surfaces = [tooth, jaw, opposing].filter(object => object.visible);
    const hit = raycaster.intersectObjects(surfaces)[0];
    if (!hit) { $('scene-status').textContent = '未选中牙面'; return; }
    const normal = hit.normal.clone().transformDirection(hit.object.matrixWorld);
    if (normal.dot(raycaster.ray.direction) > 0) normal.negate();
    picked = [...hit.point.toArray(), ...normal.toArray()]; updatePose();
  });
  new ResizeObserver(() => {
    const {width, height} = viewport.getBoundingClientRect();
    camera.aspect = width/height; camera.updateProjectionMatrix(); renderer.setSize(width, height); fit();
  }).observe(viewport);
  await updatePose();
  const frame = pose.reference_frame_world;
  const outward = new THREE.Vector3(-frame[0][0], -frame[1][0], -frame[2][0]);
  outward.addScaledVector(new THREE.Vector3(frame[0][2], frame[1][2], frame[2][2]), 1.3);
  outward.add(new THREE.Vector3(0, 0, .8)).normalize();
  controls.target.set(0, 0, 0); camera.position.copy(outward).multiplyScalar(150);
  // Initial sizing precedes camera fitting, including narrow mobile viewports.
  const {width, height} = viewport.getBoundingClientRect();
  camera.aspect = width/height; renderer.setSize(width, height); showScope();
  $('loading').hidden = true;
  window.posePreview = {get pose() { return pose; }, get region() { return region; }, camera, controls, renderer, scene, tool, fit};
  window.dispatchEvent(new CustomEvent('preview-ready'));
}

start().catch(error => { $('loading').hidden = true; setError(error.message); });
