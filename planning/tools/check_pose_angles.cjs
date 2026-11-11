// Optional browser regression: NODE_PATH must include a Playwright installation.
const {chromium} = require('playwright');
const assert = require('node:assert/strict');
const fs = require('node:fs/promises');
const path = require('node:path');
const crypto = require('node:crypto');

const url = process.argv[2] || 'http://127.0.0.1:8765/';
const output = process.argv[3] || 'outputs/pose_angles_restore';
const hash = value => crypto.createHash('sha256').update(value).digest('hex');

(async () => {
  await fs.mkdir(output, {recursive: true});
  const browser = await chromium.launch({channel: 'chrome', headless: true,
    args: ['--use-angle=swiftshader', '--enable-unsafe-swiftshader', '--ignore-gpu-blocklist']});
  const errors = [];
  const page = await browser.newPage({viewport: {width: 1440, height: 1000}, acceptDownloads: true});
  page.on('pageerror', error => errors.push(error.message));
  page.on('response', response => { if (response.status() >= 400) errors.push(`${response.status()} ${response.url()}`); });
  const report = [];
  try {
    await page.goto(url);
    await page.waitForFunction(() => window.posePreview?.pose, null, {timeout: 60000});
    const canvasHash = async () => hash(await page.locator('canvas').screenshot());
    const camera = () => page.evaluate(() => ({position: window.posePreview.camera.position.toArray(), target: window.posePreview.controls.target.toArray()}));
    const setValue = async (id, value) => {
      await page.locator(`#${id}`).evaluate((element, value) => {
        element.value = value; element.dispatchEvent(new Event('input', {bubbles: true}));
      }, value);
      const field = {angle: 0, azimuth: 1, flip: 2}[id];
      await page.waitForFunction(({id, value, field}) => {
        const pose = window.posePreview.pose;
        return field !== undefined ? Math.abs(pose.pose_angles_deg[field]-value) < 1e-8
          : Math.abs(pose[id === 'station' ? 'station_mm' : 'standoff_mm']-value) < 1e-8;
      }, {id, value, field});
    };
    const original = await canvasHash();
    const originalCamera = await camera();
    for (const [id, value] of [['angle', 35], ['azimuth', -20], ['flip', -141]]) {
      const before = await canvasHash();
      await setValue(id, value);
      assert.notEqual(await canvasHash(), before, `${id} must change actual rendering`);
      assert.deepEqual(await camera(), originalCamera, `${id} must preserve camera`);
    }
    await page.locator('#angle-number').fill('36.25');
    await page.locator('#angle-number').dispatchEvent('change');
    await page.waitForFunction(() => window.posePreview.pose.pose_angles_deg[0] === 36.25);
    assert.equal(await page.locator('#angle').inputValue(), '36.25');
    await setValue('angle', 35);
    await setValue('offset', 3.5);
    await setValue('station', 3);
    const initialRotation = await page.evaluate(() => window.posePreview.pose.rotation_world);
    await setValue('station', 12);
    assert.deepEqual(await page.evaluate(() => window.posePreview.pose.rotation_world), initialRotation);
    const [download] = await Promise.all([page.waitForEvent('download'), page.locator('#save').click()]);
    await download.saveAs(path.join(output, 'selected_pose.json'));
    const saved = JSON.parse(await fs.readFile(path.join(output, 'selected_pose.json'), 'utf8'));
    assert.equal(saved.execution_ready, false);
    assert.deepEqual(saved.pose_angles_deg, [35, -20, -141]);
    assert.equal(saved.standoff_mm, 3.5);
    await page.locator('#region').selectOption('inner_front');
    await page.waitForFunction(() => window.posePreview.pose.region === 'inner_front');
    assert.deepEqual(await page.evaluate(() => window.posePreview.pose.pose_angles_deg), [35, -20, -141]);
    await page.locator('#region').selectOption('outer_front');
    await page.waitForFunction(() => window.posePreview.pose.region === 'outer_front');
    await page.locator('[data-mode="continuous"]').click();
    await page.waitForFunction(() => window.posePreview.pose.planning_mode === 'continuous');
    await page.locator('[data-mode="simple"]').click();
    await page.waitForFunction(() => window.posePreview.pose.planning_mode === 'simple');
    // Return to the README parameter tuple before checking framing.
    await setValue('angle', 6.37); await setValue('azimuth', 13.26); await setValue('flip', -.4);
    await setValue('offset', 1.8);
    await page.locator('#restore').click();
    for (const viewport of [{width: 1440, height: 1000}, {width: 390, height: 844}, {width: 320, height: 720}]) {
      await page.setViewportSize(viewport);
      for (const scope of ['tooth', 'mandibular', 'both']) {
        await page.locator('#scope').selectOption(scope);
        await page.locator('#fit').click();
        await page.waitForTimeout(100);
        const pixels = await page.evaluate(() => {
          const {renderer, scene, camera} = window.posePreview;
          renderer.render(scene, camera);
          const gl = renderer.getContext();
          const w = gl.drawingBufferWidth, h = gl.drawingBufferHeight;
          const values = new Uint8Array(w*h*4); gl.readPixels(0, 0, w, h, gl.RGBA, gl.UNSIGNED_BYTE, values);
          let changed = 0, lowX = w, lowY = h, highX = 0, highY = 0;
          const background = values.slice(0, 3);
          for (let y = 0; y < h; y += 2) for (let x = 0; x < w; x += 2) {
            const i = (y*w+x)*4;
            if (Math.abs(values[i]-background[0])+Math.abs(values[i+1]-background[1])+Math.abs(values[i+2]-background[2]) > 40) {
              changed++; lowX = Math.min(lowX, x); lowY = Math.min(lowY, y); highX = Math.max(highX, x); highY = Math.max(highY, y);
            }
          }
          return {w, h, changed, lowX, lowY, highX, highY};
        });
        assert(pixels.changed > 300, `blank ${viewport.width} ${scope}`);
        assert(pixels.lowX > 0 && pixels.lowY > 0 && pixels.highX < pixels.w-2 && pixels.highY < pixels.h-2, 'geometry must fit canvas');
        const layout = await page.evaluate(() => {
          const elements = [...document.querySelectorAll('button,input,select,h1,.legend')];
          return {overflow: document.documentElement.scrollWidth > innerWidth,
            outside: elements.filter(e => {const r = e.getBoundingClientRect(); return r.left < 0 || r.right > innerWidth+1;}).map(e => e.id || e.tagName)};
        });
        assert.equal(layout.overflow, false); assert.deepEqual(layout.outside, []);
        await page.screenshot({path: path.join(output, `${viewport.width}_${scope}.png`), fullPage: true});
        report.push({viewport, scope, pixels, layout});
      }
    }
    // Raycast a visible surface in the actual scene, then click its screen point.
    await page.setViewportSize({width: 1440, height: 1000});
    await page.locator('#scope').selectOption('mandibular'); await page.locator('#fit').click();
    await page.locator('#pick').check();
    const point = await page.evaluate(async () => {
      const THREE = await import('/vendor/three.module.js');
      const {scene, camera} = window.posePreview;
      const ray = new THREE.Raycaster();
      const meshes = scene.children.filter(o => o.isMesh && o.visible && o.geometry.attributes.position.count > 500);
      for (let x = -.4; x <= .4; x += .05) for (let y = -.4; y <= .4; y += .05) {
        ray.setFromCamera(new THREE.Vector2(x, y), camera);
        if (ray.intersectObjects(meshes).length) return [x, y];
      }
    });
    assert(point, 'visible dental surface must be pickable');
    const bounds = await page.locator('canvas').boundingBox();
    await page.mouse.click(bounds.x+(point[0]+1)*bounds.width/2, bounds.y+(1-point[1])*bounds.height/2);
    await page.waitForFunction(() => window.posePreview.pose.reference === 'picked_surface');
    assert.equal(await page.locator('#station').isDisabled(), true);
    await page.locator('#restore').click();
    await page.waitForFunction(() => window.posePreview.pose.reference === 'perception_path');
    await page.locator('#pick').uncheck();
    const beforeOrbit = await camera();
    await page.mouse.move(bounds.x+bounds.width*.5, bounds.y+bounds.height*.5);
    await page.mouse.down(); await page.mouse.move(bounds.x+bounds.width*.6, bounds.y+bounds.height*.55, {steps: 5}); await page.mouse.up();
    assert.notDeepEqual(await camera(), beforeOrbit);
    await page.locator('#tool-full').check(); await page.locator('#tool-full').uncheck();
    await page.locator('#reset').click();
    await page.waitForFunction(() => window.posePreview.pose.pose_angles_deg.every(v => v === 0));
    assert.notEqual(await canvasHash(), original);
    assert.deepEqual(errors, []);
    await fs.writeFile(path.join(output, 'browser_checks.json'), JSON.stringify({report, errors, saved}, null, 2));
    console.log(`Passed slider, numeric state, path, region, modes, export, picking, orbit, and ${report.length} viewport/scope checks.`);
  } finally { await browser.close(); }
})().catch(error => {console.error(error); process.exitCode = 1;});
