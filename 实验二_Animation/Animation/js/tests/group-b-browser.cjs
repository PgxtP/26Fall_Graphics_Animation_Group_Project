const fs = require('node:fs');
const path = require('node:path');
const http = require('node:http');
const assert = require('node:assert/strict');
const { chromium } = require(process.argv[2] || 'playwright');
const root = path.resolve(__dirname, '../..');

// Run against the same local ES modules and GLB assets used by VS Code Live Server.
// Usage: node js/tests/group-b-browser.cjs [absolute path to playwright package]
(async () => {
  const errors = [];
  const server = http.createServer((req, res) => {
    const file = path.resolve(root, '.' + decodeURIComponent(req.url.split('?')[0]));
    if (!file.startsWith(root + path.sep)) return res.writeHead(403).end();
    fs.readFile(file, (error, data) => {
      if (error) return res.writeHead(404).end();
      res.setHeader('Content-Type', ({ '.js': 'text/javascript', '.html': 'text/html', '.css': 'text/css' })[path.extname(file)] || 'application/octet-stream');
      res.end(data);
    });
  });
  await new Promise((resolve) => server.listen(0, '127.0.0.1', resolve));
  let browser;
  try {
    browser = await chromium.launch({
      executablePath: process.env.CHROME_PATH || 'C:/Program Files/Google/Chrome/Application/chrome.exe',
      headless: true, args: ['--use-angle=swiftshader', '--enable-unsafe-swiftshader'],
    });
    const page = await browser.newPage({ viewport: { width: 1440, height: 1000 }, acceptDownloads: true });
    page.on('pageerror', (error) => errors.push(error.message));
    page.on('console', (message) => { if (message.type() === 'error') errors.push(message.text()); });
    page.on('response', (response) => { if (response.status() >= 400) errors.push(`${response.status()} ${response.url()}`); });
    const ready = () => page.waitForFunction(() => window.robotSimulation?.calibration?.captureTCPSample && window.robotSimulation?.grip && window.robotSimulation?.workstation);
    await page.goto(`http://127.0.0.1:${server.address().port}/index.html`);
    await ready();
    assert.equal(await page.isChecked('#motion-enable'), false);

    // Exercise the real initial workpiece and nominal TCP through the normal
    // IK -> guarded MOVJ -> GRIP path before loading synthetic calibration data.
    // In particular, the target must not inflate the table's obstacle AABB.
    const pickup = await page.evaluate(async () => {
      const THREE = await import('three');
      const api = window.robotSimulation, station = api.workstation;
      const start = api.joints.map((joint) => joint.angle);
      const matrixError = (a, b) => Math.max(...a.elements.map((value, i) => Math.abs(value - b.elements[i])));
      const world = () => { station.workpiece.updateWorldMatrix(true, false); return station.workpiece.matrixWorld.clone(); };
      const run = (angles) => {
        api.motion.moveJ(angles);
        for (let i = 0; i < 10000 && api.motion.busy(); i++) api.motion.update(.05);
        if (api.motion.busy() || api.joints.some((joint, i) => Math.abs(joint.angle - angles[i]) > 1e-7)) {
          throw new Error('Normal guarded MOVJ stopped before its pickup/lift target');
        }
      };
      const solve = (target) => {
        const result = api.solveIK(target, null, api.joints.map((joint) => joint.angle), { maxIterations: 200 });
        if (!result.converged) throw new Error('Initial workpiece pickup/lift IK did not converge');
        return result;
      };
      api.motion.state.enabled = true; api.motion.state.speed = 100;
      try {
        const initial = world();
        const target = station.workpiece.localToWorld(new THREE.Vector3(0, station.blockSize / 2, 0));
        const approach = solve(target);
        run(approach.angles);
        const distance = station.gripStatus.distance;
        const close = api.grip('CLOSE');
        const attachError = matrixError(world(), initial);
        const heldLocal = station.workpiece.matrix.clone();
        const lift = solve(target.clone().add(new THREE.Vector3(0, station.blockSize * 1.5, 0)));
        run(lift.angles);
        const transported = world();
        const followingError = matrixError(station.workpiece.matrix, heldLocal);
        const transportedDistance = new THREE.Vector3().setFromMatrixPosition(transported)
          .distanceTo(new THREE.Vector3().setFromMatrixPosition(initial));
        const open = api.grip('OPEN');
        const releaseError = matrixError(world(), transported);
        run(start);
        const releasedStill = matrixError(world(), transported);
        station.resetWorkpiece();
        return { source: 'actual-initial-workpiece-and-nominal-TCP', distance, tolerance: close.tolerance,
          closeAttached: close.attached, attachError, followingError, transportedDistance,
          openAttached: open.attached, releaseError, releasedStill, resetError: matrixError(world(), initial),
          tableContainsWorkpiece: !!station.table.getObjectById(station.workpiece.id) };
      } finally { api.motion.stop(); api.motion.state.enabled = false; api.motion.state.speed = 25; }
    });
    assert(pickup.distance <= pickup.tolerance);
    assert.equal(pickup.closeAttached, true); assert(pickup.attachError < 1e-8);
    assert(pickup.followingError < 1e-8); assert(pickup.transportedDistance > .5);
    assert.equal(pickup.openAttached, false); assert(pickup.releaseError < 1e-8);
    assert(pickup.releasedStill < 1e-8); assert(pickup.resetError < 1e-8);
    assert.equal(pickup.tableContainsWorkpiece, false);

    await page.click('#tab-tcp');
    await page.getByText('算法演示数据', { exact: true }).click();
    await page.click('#tcp-demo');
    assert.equal(await page.locator('#tcp-sample-list tr').count(), 4);
    await page.click('#tcp-fit');
    const sampleRecord = await page.evaluate(() => window.robotSimulation.calibration.record);
    assert.equal(sampleRecord.version, 3);
    assert.equal(sampleRecord.tcpCalibration.samples.length, 4);
    assert(sampleRecord.tcpCalibration.fit.rms < 1e-8);
    assert(sampleRecord.tcpCalibration.fit.sphere.rms < 1e-8);
    assert.equal(sampleRecord.activeTool.source, 'asset-nominal', 'Fitting must not apply the TCP implicitly');

    // A fitted translation invalidates a previously defined user frame and retains explicit tool RPY.
    await page.click('#tab-frame');
    await page.fill('#frame-o', '1,2,3');
    await page.fill('#frame-x', '2,2,3');
    await page.fill('#frame-xy', '1,3,3');
    await page.click('#solve-frame');
    await page.click('#verify-frame');
    assert.equal(await page.evaluate(() => window.robotSimulation.calibration.record.validations.length), 1);
    await page.fill('#frame-x', '3,2,3');
    assert.equal(await page.evaluate(() => window.robotSimulation.calibration.record.validations.length), 0);
    assert.equal(await page.textContent('#frame-matrix'), '尚无结果');
    await page.click('#solve-frame');
    assert(await page.evaluate(() => !!window.robotSimulation.calibration.userMatrix));
    await page.click('#tab-tcp');
    await page.fill('#tcp-rotation', '10,20,30');
    await page.click('#tcp-apply-fit');
    const applied = await page.evaluate(() => window.robotSimulation.calibration.record);
    assert.equal(applied.activeTool.source, 'four-point-sphere-demo');
    assert.deepEqual(applied.activeTool.rpyDeg, [10, 20, 30]);
    assert.equal(await page.evaluate(() => window.robotSimulation.calibration.userMatrix), null);

    const [download] = await Promise.all([page.waitForEvent('download'), page.click('#export-calibration')]);
    const exported = JSON.parse(fs.readFileSync(await download.path(), 'utf8'));
    assert.equal(exported.tcpCalibration.samples.length, 4);
    assert.equal(exported.activeTool.source, 'four-point-sphere-demo');
    await page.reload();
    await ready();
    assert.equal(await page.evaluate(() => window.robotSimulation.calibration.record.activeTool.source), 'asset-nominal');
    await page.click('#tab-tcp');
    await page.locator('#import-calibration').setInputFiles({ name: 'tcp-record.json', mimeType: 'application/json', buffer: Buffer.from(JSON.stringify(exported)) });
    await page.waitForFunction(() => window.robotSimulation.calibration.record.tcpCalibration.samples.length === 4);
    assert.equal(await page.evaluate(() => window.robotSimulation.calibration.record.activeTool.source), 'asset-nominal');
    await page.click('#tcp-fit');
    await page.click('#tcp-apply-fit');
    assert.equal(await page.evaluate(() => window.robotSimulation.calibration.record.activeTool.source), 'four-point-sphere-demo');

    const guards = await page.evaluate(async () => {
      const THREE = await import('three');
      const api = window.robotSimulation;
      const samples = structuredClone(api.calibration.record.tcpCalibration.samples);
      api.calibration.setTCPSamples([]);
      let blockedBusy = false;
      api.motion.state.jog = { name: 'J1', direction: 1 };
      try { api.calibration.captureTCPSample(); } catch (error) { blockedBusy = /停止.*运动|运动.*停止/.test(error.message); }
      finally { api.motion.stop(); }
      const unchangedCount = api.calibration.record.tcpCalibration.samples.length === 0;
      api.calibration.captureTCPSample();
      const realSample = api.calibration.record.tcpCalibration.samples[0];
      let blockedDuplicate = false;
      try { api.calibration.captureTCPSample(); } catch { blockedDuplicate = true; }
      const sampledFlange = realSample.source === 'flange-contact' && realSample.matrix.length === 16 && realSample.angles.length === 5;

      // The same four origins still define an exact sphere even when one orientation is wrong.
      // The reconstructed tip consistency check must reject applying this contradictory data.
      const bad = structuredClone(samples);
      const matrix = new THREE.Matrix4().fromArray(bad[0].matrix);
      const position = new THREE.Vector3(), rotation = new THREE.Quaternion(), scale = new THREE.Vector3();
      matrix.decompose(position, rotation, scale);
      rotation.multiply(new THREE.Quaternion().setFromAxisAngle(new THREE.Vector3(1, 0, 0), Math.PI * .73));
      bad[0].matrix = new THREE.Matrix4().compose(position, rotation, scale).toArray();
      let blockedBadContact = false;
      try {
        api.calibration.setTCPSamples(bad);
        api.calibration.fitTCP();
        api.calibration.applyFittedTCP();
      } catch { blockedBadContact = true; }
      api.calibration.setTCPSamples(samples);
      api.calibration.fitTCP();
      return { blockedBusy, unchangedCount, blockedDuplicate, sampledFlange, blockedBadContact };
    });
    assert.deepEqual(guards, { blockedBusy: true, unchangedCount: true, blockedDuplicate: true, sampledFlange: true, blockedBadContact: true });

    // Restore the nominal tool for scene tests; synthetic demonstration offsets are not robot measurements.
    await page.fill('#tcp-translation', '0,0,0');
    await page.fill('#tcp-rotation', '0,0,0');
    await page.click('#apply-known-tcp');
    await page.click('#tab-workstation');
    const grip = await page.evaluate(async () => {
      const THREE = await import('three');
      const api = window.robotSimulation, station = api.workstation;
      const matrixError = (a, b) => Math.max(...a.elements.map((value, i) => Math.abs(value - b.elements[i])));
      const worldMatrix = () => { station.workpiece.updateWorldMatrix(true, false); return station.workpiece.matrixWorld.clone(); };
      station.resetWorkpiece();
      const restingWorld = worldMatrix();
      const far = api.grip('CLOSE');
      const farError = matrixError(restingWorld, worldMatrix());
      api.grip('OPEN');

      // Test fixture: place a workpiece at the TCP while preserving its arbitrary orientation.
      // Production close itself must never move a remote workpiece into position.
      station.scene.attach(station.workpiece);
      station.workpiece.quaternion.setFromEuler(new THREE.Euler(.12, -.18, .09));
      station.workpiece.updateMatrix();
      const tcp = api.tcp.getWorldPosition(new THREE.Vector3());
      const contact = station.workpiece.localToWorld(new THREE.Vector3(0, station.blockSize / 2, 0));
      station.workpiece.position.add(tcp.sub(contact));
      station.scene.updateMatrixWorld(true);
      const beforeAttach = worldMatrix();
      const close = api.grip('CLOSE');
      const attachError = matrixError(beforeAttach, worldMatrix());
      const heldLocal = station.workpiece.matrix.clone();
      const initialFlangePosition = station.flange.position.clone();
      station.flange.position.x += .15;
      station.scene.updateMatrixWorld(true);
      const afterFollow = worldMatrix();
      const followDistance = new THREE.Vector3().setFromMatrixPosition(afterFollow).distanceTo(new THREE.Vector3().setFromMatrixPosition(beforeAttach));
      const localError = matrixError(heldLocal, station.workpiece.matrix);
      const open = api.grip('OPEN');
      const releaseError = matrixError(afterFollow, worldMatrix());
      station.flange.position.copy(initialFlangePosition);
      station.scene.updateMatrixWorld(true);
      const releasedStill = matrixError(afterFollow, worldMatrix());
      station.resetWorkpiece();
      const resetError = matrixError(restingWorld, worldMatrix());
      api.motion.state.estop = true;
      let blockedEstop = false;
      try { api.grip('CLOSE'); } catch { blockedEstop = true; }
      finally { api.motion.state.estop = false; }
      return { farAttached: far.attached, farError, closeAttached: close.attached, attachError, followDistance, localError,
        openAttached: open.attached, releaseError, releasedStill, resetError, blockedEstop, gripperState: station.gripperState };
    });
    assert.equal(grip.farAttached, false);
    assert(grip.farError < 1e-8);
    assert.equal(grip.closeAttached, true);
    assert(grip.attachError < 1e-8);
    assert(grip.followDistance > .01);
    assert(grip.localError < 1e-8);
    assert.equal(grip.openAttached, false);
    assert(grip.releaseError < 1e-8);
    assert(grip.releasedStill < 1e-8);
    assert(grip.resetError < 1e-8);
    assert.equal(grip.blockedEstop, true);
    assert.equal(grip.gripperState, 'OPEN');

    await page.click('#tab-tcp');
    await page.evaluate(() => { document.querySelector('.task-workbench').scrollTop = 0; });
    const screenshotDir = path.join(root, 'tmp', 'qa');
    fs.mkdirSync(screenshotDir, { recursive: true });
    await page.screenshot({ path: path.join(screenshotDir, 'group-b-tcp.png'), fullPage: true });
    assert.deepEqual(errors, []);
    console.log(JSON.stringify({ result: 'PASS', checks: ['scene bootstrap', 'four-pose fit and explicit application', 'RPY preservation',
      'user-frame invalidation', 'JSON roundtrip unapplied', 'busy sampling guard', 'contradictory contact guard',
      'actual guarded MOVJ pickup/lift/release', 'far close without teleport', 'world-preserving attach/follow/release/reset', 'estop grip guard'], guards, pickup, grip }, null, 2));
  } finally {
    await browser?.close();
    await new Promise((resolve) => server.close(resolve));
  }
})().catch((error) => { console.error(error); process.exitCode = 1; });
