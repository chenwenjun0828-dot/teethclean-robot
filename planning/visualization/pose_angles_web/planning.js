const $ = id => document.getElementById(id);
const labels = {running: '规划运行中', succeeded: '检查通过', failed: '规划失败', cancelled: '已取消'};

function download(record, filename) {
  const url = URL.createObjectURL(new Blob([JSON.stringify(record, null, 2)], {type: 'application/json'}));
  const link = document.createElement('a'); link.href = url; link.download = filename; link.click();
  setTimeout(() => URL.revokeObjectURL(url), 1000);
}

export function installPlanning(data, current, showPath) {
  let task = null, busy = false, timer = null, displayedTask = null;
  const defaults = data.planning_defaults;
  $('trace-speed').value = defaults.speed_mm_s;
  $('transfer-speed').value = defaults.transfer_speed_mm_s;
  $('pose-spacing').value = defaults.pose_spacing_mm;
  $('local-search').checked = defaults.local_angle_search;
  $('source-status').textContent = data.session
    ? `${data.session.id} · ${data.jaw === 'mandibular' ? '下颌' : '上颌'} · ${data.session.camera_frame} · ${data.session.status === 'optical_frame_hypothesis_live_tf_unverified' ? '标定待核实' : '归档标定'} · 仅仿真`
    : `${data.jaw === 'mandibular' ? '下颌' : '上颌'} · ${data.perception_result || '项目默认数据'} · 仅仿真`;
  if (data.jaw === 'maxillary') $('scope').options[1].textContent = '完整上颌';

  function parameters(checkMode = 'full') {
    const state = current();
    const inputs = ['angle-number', 'azimuth-number', 'flip-number', 'offset-number', 'trace-speed', 'transfer-speed', 'pose-spacing'];
    if (!state.pose || state.picked || !state.ready || inputs.some(id => !$(id).checkValidity())) return null;
    const angles = inputs.slice(0, 3).map(id => $(id).valueAsNumber);
    if (angles.some((angle, i) => angle !== state.pose.pose_angles_deg[i]) ||
        $('offset-number').valueAsNumber !== state.pose.standoff_mm ||
        state.mode !== state.pose.planning_mode || $('region').value !== state.pose.region ||
        state.pose.standoff_mm <= defaults.clearance_mm) return null;
    return {region: state.pose.region, planning_mode: state.mode, pose_angles_deg: angles,
      standoff_mm: state.pose.standoff_mm, speed_mm_s: $('trace-speed').valueAsNumber,
      transfer_speed_mm_s: $('transfer-speed').valueAsNumber,
      pose_spacing_mm: $('pose-spacing').valueAsNumber,
      local_angle_search: $('local-search').checked, check_mode: checkMode};
  }

  async function request(path, body) {
    const response = await fetch(path, body === undefined ? {} : {method: 'POST',
      headers: {'Content-Type': 'application/json', 'X-Planning-Token': data.planning_token},
      body: JSON.stringify(body)});
    const result = await response.json();
    if (!response.ok) throw new Error(result.error || `HTTP ${response.status}`);
    return result;
  }

  function refresh() {
    const values = parameters(task?.parameters.check_mode);
    const valid = Boolean(values);
    $('check-reach').disabled = $('plan-full').disabled = busy || task?.state === 'running' || !valid;
    $('export-plan').disabled = busy || !valid;
    $('cancel-plan').disabled = task?.state !== 'running' || task.cancel_requested;
    const stale = Boolean(task && (!values || Object.keys(values).some(key =>
      JSON.stringify(values[key]) !== JSON.stringify(task.parameters[key]))));
    $('plan-stale').hidden = !stale;
    if (task) showPath(null, {stale, visible: $('trajectory-visible').checked && current().pose?.region === task.parameters.region});
  }

  function render(value) {
    task = value;
    if (!task) { refresh(); return; }
    const result = task.result;
    $('plan-result').hidden = false;
    $('plan-status').textContent = task.cancel_requested && task.state === 'running' ? '正在取消' :
      (result?.validation === 'simulation' ? '完整仿真验收通过 · 仅仿真' :
        result?.validation === 'reachability' ? '可达性通过 · 碰撞与动力学未验证' :
          `${labels[task.state]}${task.state === 'running' && task.report_status ? ` · ${task.report_status}` : ''}`);
    $('plan-status').dataset.state = task.state;
    $('plan-parameters').textContent = JSON.stringify(task.parameters, null, 2);
    $('plan-log').textContent = task.log || '';
    $('plan-directory').textContent = `${task.directory}/plan`;
    $('replay-command').textContent = task.replay_command;
    $('copy-replay').disabled = !result?.path;
    const errors = [task.error, result?.error, result?.display_error,
      result && !result.passed && result.attempts?.length ? JSON.stringify(result.attempts, null, 2) : null].filter(Boolean);
    $('plan-error').textContent = errors.join('\n');
    $('plan-error').hidden = !errors.length;
    $('plan-metrics').replaceChildren();
    if (result) {
      const rows = [['轨迹时长', result.duration_s == null ? '—' : `${result.duration_s.toFixed(2)} s`],
        ['最小间隙', result.minimum_clearance_mm == null ? '未验证' : `${result.minimum_clearance_mm.toFixed(3)} mm`],
        ['曲线 / 运动审计', `${result.trace_passed ? '通过' : '未通过'} / ${result.motion_passed ? '通过' : '未通过'}`],
        ['动力学', result.dynamics_passed ? '通过' : '未验证或未通过'],
        ['覆盖', result.coverage_passed ? '通过' : '未验证或未通过'],
        ['姿态修正', result.orientation ? '见本次姿态报告' : '见规划目录']];
      for (const [label, text] of rows) {
        const row = document.createElement('div'), title = document.createElement('dt'), content = document.createElement('dd');
        title.textContent = label; content.textContent = text; row.append(title, content); $('plan-metrics').append(row);
      }
      if (result.orientation) $('plan-parameters').textContent += `\n\n实际姿态参考与修正\n${JSON.stringify(result.orientation, null, 2)}`;
    }
    const key = `${task.id}:${task.state}`;
    if (displayedTask !== key) { showPath(result?.path || false, {visible: true, stale: false}); displayedTask = key; }
    refresh();
    window.dispatchEvent(new CustomEvent('planning-updated'));
    clearTimeout(timer);
    if (task.state === 'running') timer = setTimeout(poll, 1200);
  }

  async function poll() {
    try { render(await request(`/api/task?id=${encodeURIComponent(task.id)}`)); }
    catch (error) { $('plan-error').hidden = false; $('plan-error').textContent = error.message; timer = setTimeout(poll, 2500); }
  }

  async function submit(checkMode) {
    const values = parameters(checkMode);
    if (!values || busy || task?.state === 'running') return;
    busy = true; refresh();
    try { render(await request('/api/plan', values)); }
    catch (error) { $('plan-error').hidden = false; $('plan-error').textContent = error.message; }
    finally { busy = false; refresh(); }
  }

  $('check-reach').addEventListener('click', () => submit('geometry_only'));
  $('plan-full').addEventListener('click', () => submit('full'));
  $('cancel-plan').addEventListener('click', async () => {
    try { render(await request('/api/cancel', {id: task.id})); }
    catch (error) { $('plan-error').hidden = false; $('plan-error').textContent = error.message; }
  });
  $('export-plan').addEventListener('click', async () => {
    const values = parameters();
    if (!values) return;
    try { download(await request('/api/configuration', values), `planning_${values.region}.json`); }
    catch (error) { $('plan-error').hidden = false; $('plan-error').textContent = error.message; }
  });
  $('copy-replay').addEventListener('click', async () => {
    try { await navigator.clipboard.writeText(task.replay_command); }
    catch { $('plan-error').hidden = false; $('plan-error').textContent = '无法访问剪贴板'; }
  });
  $('trajectory-visible').addEventListener('change', refresh);
  document.querySelector('aside').addEventListener('input', refresh);
  window.addEventListener('pose-updated', refresh);
  request('/api/task').then(render).catch(error => {
    $('plan-error').hidden = false; $('plan-error').textContent = error.message;
  });
  refresh();
  return {get task() { return task; }, parameters, refresh};
}
