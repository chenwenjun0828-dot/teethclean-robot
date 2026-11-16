// Reusable browser integration check; requires the session preview server and Playwright.
const {chromium} = require('playwright');
const assert = require('node:assert/strict');
const fs = require('node:fs/promises');
const path = require('node:path');

const url = process.argv[2] || 'http://127.0.0.1:8765/';
const output = process.argv[3] || 'outputs/interactive_planning/browser_checks';
const completeFull = process.argv[4] === '--full';

(async () => {
  await fs.mkdir(output, {recursive: true});
  const browser = await chromium.launch({channel: 'chrome', headless: true,
    args: ['--use-angle=swiftshader', '--enable-unsafe-swiftshader', '--ignore-gpu-blocklist']});
  const page = await browser.newPage({viewport: {width: 1440, height: 1000}, acceptDownloads: true});
  const errors = [];
  page.on('pageerror', error => errors.push(error.message));
  try {
    await page.goto(url);
    await page.waitForFunction(() => window.posePreview?.planning);
    const setPair = async (id, value) => {
      await page.locator(`#${id}-number`).fill(String(value));
      await page.locator(`#${id}-number`).dispatchEvent('change');
      await page.waitForFunction(({id, value}) => {
        const pose = window.posePreview.pose;
        const i = {angle: 0, azimuth: 1, flip: 2}[id];
        return i === undefined ? pose.standoff_mm === value : pose.pose_angles_deg[i] === value;
      }, {id, value});
    };
    await page.locator('[data-mode="continuous"]').click();
    await page.waitForFunction(() => window.posePreview.pose.planning_mode === 'continuous');
    await setPair('angle', 20); await setPair('azimuth', -20); await setPair('flip', 10); await setPair('offset', 3);
    await page.locator('details').first().locator('summary').click();
    await page.locator('#pose-spacing').fill('2');
    await page.locator('#trace-speed').fill('1'); await page.locator('#transfer-speed').fill('10');
    await page.locator('#pick').check();
    assert(await page.locator('#plan-full').isDisabled());
    await page.locator('#pick').uncheck();
    await page.waitForFunction(() => !document.querySelector('#export-plan').disabled);
    const [download] = await Promise.all([page.waitForEvent('download'), page.locator('#export-plan').click()]);
    await download.saveAs(path.join(output, 'planning_configuration.json'));
    const configuration = JSON.parse(await fs.readFile(path.join(output, 'planning_configuration.json'), 'utf8'));
    assert.equal(configuration.execute, false); assert.equal(configuration.geometry_only, false);
    assert.equal(configuration.session_camera_frame, 'color_optical');
    assert.deepEqual(configuration.pose_angles_deg, [20, -20, 10]);
    assert.equal(configuration.standoff_mm, 3);
    await page.locator('#check-reach').click();
    await page.waitForFunction(() => window.posePreview.planning.task?.state === 'running');
    const taskId = await page.evaluate(() => window.posePreview.planning.task.id);
    assert(await page.locator('#plan-full').isDisabled());
    await setPair('angle', 21);
    assert(await page.locator('#plan-stale').isVisible());
    assert.deepEqual(await page.evaluate(() => window.posePreview.planning.task.parameters.pose_angles_deg), [20, -20, 10]);
    await setPair('angle', 20);
    await page.waitForFunction(() => window.posePreview.planning.task?.state !== 'running', null, {timeout: 240000});
    const task = await page.evaluate(() => window.posePreview.planning.task);
    assert.equal(task.id, taskId); assert.equal(task.state, 'succeeded', task.log);
    assert.equal(task.result.validation, 'reachability'); assert(task.result.path.points_world_m.length > 100);
    assert.equal(await page.locator('#plan-stale').isHidden(), true);
    assert.match(await page.locator('#plan-status').textContent(), /碰撞与动力学未验证/);
    let full = null;
    if (completeFull) {
      await page.locator('#pose-spacing').fill('0.25');
      await page.locator('#plan-full').click();
      await page.waitForFunction(() => window.posePreview.planning.task?.state === 'running');
      await page.waitForFunction(() => window.posePreview.planning.task?.state !== 'running', null, {timeout: 360000});
      full = await page.evaluate(() => window.posePreview.planning.task);
      assert.equal(full.state, 'succeeded', full.error || full.log);
      assert.equal(full.result.validation, 'simulation');
      assert.equal(full.result.passed, true);
      assert.equal(full.result.robot_execution_allowed, false);
      assert(full.result.path.points_world_m.length > 100);
    }
    await page.locator('#trajectory-visible').uncheck();
    await page.locator('#fit').click();
    const cameraBefore = await page.evaluate(() => window.posePreview.camera.position.toArray());
    await page.locator('#trajectory-visible').check(); await page.locator('#fit').click();
    assert.notDeepEqual(await page.evaluate(() => window.posePreview.camera.position.toArray()), cameraBefore);
    const layouts = [];
    for (const viewport of [{width: 1440, height: 1000}, {width: 390, height: 844}, {width: 320, height: 720}]) {
      await page.setViewportSize(viewport); await page.locator('#fit').click();
      const pixels = await page.evaluate(() => {
        const {renderer, scene, camera} = window.posePreview;
        renderer.render(scene, camera);
        const gl = renderer.getContext(), w = gl.drawingBufferWidth, h = gl.drawingBufferHeight;
        const values = new Uint8Array(w*h*4); gl.readPixels(0, 0, w, h, gl.RGBA, gl.UNSIGNED_BYTE, values);
        let changed = 0;
        for (let i = 0; i < values.length; i += 16) {
          if (Math.abs(values[i]-values[0])+Math.abs(values[i+1]-values[1])+Math.abs(values[i+2]-values[2]) > 40) changed++;
        }
        return {changed, w, h};
      });
      assert(pixels.changed > 300, 'Session and actual path must render');
      const overflow = await page.evaluate(() => document.documentElement.scrollWidth > innerWidth);
      assert.equal(overflow, false);
      for (const id of ['check-reach', 'plan-full', 'cancel-plan', 'export-plan']) {
        const fits = await page.locator(`#${id}`).evaluate(e => e.scrollWidth <= e.clientWidth);
        assert(fits, `${id} text must fit at ${viewport.width}`);
      }
      await page.screenshot({path: path.join(output, `planning_${viewport.width}.png`), fullPage: true});
      layouts.push({viewport, pixels, overflow});
    }
    await page.setViewportSize({width: 1440, height: 1000});
    let cancelled = null;
    if (!completeFull) {
      await page.locator('#plan-full').click();
      await page.waitForFunction(() => window.posePreview.planning.task?.state === 'running');
      await page.locator('#cancel-plan').click();
      await page.waitForFunction(() => window.posePreview.planning.task?.state === 'cancelled', null, {timeout: 30000});
      cancelled = await page.evaluate(() => window.posePreview.planning.task);
      assert.equal(cancelled.result, null);
      assert.match(await page.locator('#plan-status').textContent(), /已取消/);
    }
    assert.deepEqual(errors, []);
    await fs.writeFile(path.join(output, 'interactive_checks.json'), JSON.stringify({task, full, cancelled, configuration, layouts, errors}, null, 2));
    console.log(`Passed session config export, immutable parameters, reachability, actual FK display, ${completeFull ? 'full planning' : 'cancellation'}, and ${layouts.length} viewport checks. Task: ${(full || task).id}`);
  } finally { await browser.close(); }
})().catch(error => { console.error(error); process.exitCode = 1; });
