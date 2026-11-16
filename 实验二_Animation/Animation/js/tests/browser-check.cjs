const fs = require('fs');
const path = require('path');
const http = require('http');
const assert = require('assert/strict');
const { chromium } = require(process.argv[2] || 'playwright');
const root = path.resolve(__dirname, '../..');

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
    browser = await chromium.launch({ executablePath: 'C:/Program Files/Google/Chrome/Application/chrome.exe', headless: true,
      args: ['--use-angle=swiftshader', '--enable-unsafe-swiftshader'] });
    const page = await browser.newPage({ viewport: { width: 1440, height: 1000 }, acceptDownloads: true });
    page.on('pageerror', (error) => errors.push(error.message));
    page.on('response', (response) => { if (response.status() >= 400) errors.push(`${response.status()} ${response.url()}`); });
    await page.goto(`http://127.0.0.1:${server.address().port}/index.html`);
    await page.waitForFunction(() => window.robotSimulation?.motion && document.querySelector('#tcp-pose').textContent.includes('世界'));
    const placement = await page.evaluate(async () => {
      const THREE = await import('three'), api = window.robotSimulation, station = api.workstation;
      api.robot.updateMatrixWorld(true); station.scene.updateMatrixWorld(true);
      const robotBox = new THREE.Box3().setFromObject(api.robot), tableBox = new THREE.Box3().setFromObject(station.table);
      if (robotBox.intersectsBox(tableBox)) throw Error('Initial robot/table overlap');
      return { robotBox: { min: robotBox.min.toArray(), max: robotBox.max.toArray() }, tableBox: { min: tableBox.min.toArray(), max: tableBox.max.toArray() },
        initialBoxesIntersect: false, clearanceZ: tableBox.min.z - robotBox.max.z, tablePosition: station.table.position.toArray() };
    });
    assert.equal(await page.locator('#task-data').count(), 0);
    assert.equal(await page.locator('#task-records').count(), 0);
    assert(await page.locator('.task-workbench').isHidden());
    assert(await page.locator('#angle-J1').isDisabled());
    await page.check('#motion-enable');
    await page.waitForFunction(() => !document.querySelector('#angle-J1').disabled);
    const jog = page.getByRole('button', { name: 'J1 正向点动', exact: true });
    await jog.scrollIntoViewIfNeeded(); const box = await jog.boundingBox();
    await page.mouse.move(box.x + box.width / 2, box.y + box.height / 2); await page.mouse.down();
    await page.waitForFunction(() => window.robotSimulation.joints[0].angle > .02);
    await page.mouse.up();
    const released = await page.evaluate(() => window.robotSimulation.joints[0].angle);
    await page.waitForTimeout(200);
    assert.equal(await page.evaluate(() => window.robotSimulation.joints[0].angle), released);
    await page.fill('#number-J1', '30'); await page.locator('.joint-control').first().getByRole('button', { name: '到位', exact: true }).click();
    await page.waitForFunction((previous) => window.robotSimulation.joints[0].angle > previous, released);
    await page.click('#estop');
    const stopped = await page.evaluate(() => window.robotSimulation.joints[0].angle);
    await page.waitForTimeout(200); assert.equal(await page.evaluate(() => window.robotSimulation.joints[0].angle), stopped);
    assert(await page.locator('#angle-J1').isDisabled());
    assert(await page.evaluate(() => { try { window.robotSimulation.setJoint('J1', 0); return false; } catch { return true; } }));
    await page.click('#release-estop'); assert.equal(await page.isChecked('#motion-enable'), false);
    await page.check('#motion-enable');
    await page.fill('#speed-override', '100'); await page.locator('#speed-override').dispatchEvent('input');
    await page.selectOption('#jog-mode', 'inch'); await page.selectOption('#inch-step', '1');
    const beforeInch = await page.evaluate(() => window.robotSimulation.joints[0].angle);
    await page.getByRole('button', { name: 'J1 正向点动', exact: true }).click();
    await page.waitForFunction((previous) => Math.abs(window.robotSimulation.joints[0].angle - previous - Math.PI / 180) < 1e-8, beforeInch);
    await page.waitForFunction(() => !window.robotSimulation.motion.busy());
    await page.uncheck('#motion-enable');
    await page.click('#tab-tcp');
    await page.fill('#tcp-translation', '0.2,0.1,0.4');
    await page.fill('#tcp-rotation', '10,20,30'); await page.click('#apply-known-tcp');
    assert.equal(await page.locator('#legacy-pivot').count(), 0);
    await page.click('#tab-frame');
    await page.fill('#frame-o', '1,2,3'); await page.fill('#frame-x', '3,2,3'); await page.fill('#frame-xy', '1,3,4');
    await page.click('#solve-frame'); await page.click('#verify-frame');
    const record = await page.evaluate(() => window.robotSimulation.calibration.record);
    assert.equal(record.version, 2); assert.equal(record.validations.length, 1);
    assert.equal(record.activeTool.source, 'known-tool-offset');
    await page.click('#tab-records');
    const [download] = await Promise.all([page.waitForEvent('download'), page.click('#export-calibration')]);
    const exported = JSON.parse(fs.readFileSync(await download.path(), 'utf8'));
    assert.equal(exported.activeTool.source, 'known-tool-offset'); assert(!('pivotResult' in exported));
    await page.reload(); await page.waitForFunction(() => window.robotSimulation?.motion);
    assert.equal(await page.inputValue('#tcp-translation'), '0.2,0.1,0.4');
    await page.click('#tab-frame');
    await page.fill('#frame-o', '0,0,0'); await page.fill('#frame-x', '1,0,0'); await page.fill('#frame-xy', '2,0,0');
    await page.click('#solve-frame'); assert((await page.textContent('#frame-result')).includes('共线'));
    await page.click('#tab-records');
    await page.locator('#import-calibration').setInputFiles({ name: 'record.json', mimeType: 'application/json', buffer: Buffer.from(JSON.stringify(exported)) });
    await page.waitForFunction(() => document.querySelector('#frame-o').value === '1,2,3');
    await page.click('#tab-frame');
    await page.click('#solve-frame'); assert((await page.textContent('#frame-result')).includes('已建立'));
    await page.click('#tab-program');
    await page.click('#scan-program');
    assert.equal(await page.locator('#pose-list tr').count(), 5);
    assert.equal(await page.inputValue('#movj-code'), 'MOVJ P1\nMOVJ P2\nMOVJ P3\nMOVJ P4\nMOVJ P5');
    await page.check('#motion-enable');
    await page.click('#run-program');
    await page.waitForFunction(() => window.robotSimulation.program.runner.state.running);
    await page.waitForFunction(() => document.querySelector('#angle-J1').disabled);
    await page.evaluate(() => {
      const api = window.robotSimulation;
      for (let i = 0; i < 10000 && api.program.runner.state.running; i++) { api.motion.update(.05); api.program.runner.update(); }
      const goal = api.program.runner.state.instructions.at(-1).angles;
      if (api.program.runner.state.running || api.joints.some((joint, i) => Math.abs(joint.angle - goal[i]) > 1e-8)) throw Error('Preset program did not finish');
    });
    await page.waitForFunction(() => document.querySelector('#program-status').textContent.includes('执行完成'));
    await page.click('#clear-poses');
    for (let index = 0; index < 4; index++) {
      await page.evaluate((i) => {
        const api = window.robotSimulation;
        api.motion.moveJ([i * .01, 0, 0, 0, i * .02]);
        for (let step = 0; step < 10000 && api.motion.busy(); step++) api.motion.update(.05);
      }, index);
      await page.click('#record-pose');
    }
    assert.equal(await page.locator('#pose-list tr').count(), 4);
    await page.click('#generate-movj');
    await page.fill('#movj-code', 'MOVJ P4\nMOVJ P3\nMOVJ P2\nMOVJ P1');
    await page.click('#run-program');
    await page.evaluate(() => {
      const api = window.robotSimulation;
      for (let i = 0; i < 10000 && api.program.runner.state.running; i++) { api.motion.update(.05); api.program.runner.update(); }
      if (api.joints.some((joint) => Math.abs(joint.angle) > 1e-8)) throw Error('Recorded program endpoint mismatch');
    });
    await page.waitForFunction(() => document.querySelector('#program-status').textContent.includes('执行完成'));
    await page.click('#run-program'); await page.click('#stop-program');
    assert.equal(await page.evaluate(() => window.robotSimulation.program.runner.state.running), false);
    await page.click('#run-program'); await page.click('#estop');
    await page.waitForFunction(() => !window.robotSimulation.program.runner.state.running);
    assert(await page.locator('#run-program').isDisabled());
    await page.screenshot({ path: path.join(process.env.TEMP, 'robot-program-qa.png') });
    // Group B: known TCP configuration, actual scene marker coordinates, attachment and release.
    await page.click('#tab-tcp');
    await page.fill('#tcp-translation', '0.2,0.1,0.4');
    await page.fill('#tcp-rotation', '10,20,30');
    await page.click('#apply-known-tcp');
    assert((await page.textContent('#known-tcp-result')).includes('已应用已知 TCP'));
    assert.equal(await page.evaluate(() => window.robotSimulation.calibration.record.activeTool.source), 'known-tool-offset');
    await page.click('#tab-frame');
    await page.click('#use-workpiece-points'); await page.click('#solve-frame');
    assert((await page.textContent('#frame-result')).includes('工件几何标记'));
    const groupB = await page.evaluate(async () => {
      const THREE = await import('./js/vendor/three/build/three.module.js');
      const { worldToUserFrame } = await import('./js/Calibration.js');
      const api = window.robotSimulation, station = api.workstation;
      const points = station.getCalibrationPoints();
      const matrix = api.calibration.userMatrix;
      const values = Object.fromEntries(Object.entries(points).map(([name, point]) => [name, { world: point.toArray(), user: worldToUserFrame(point, matrix).toArray() }]));
      const probe = new THREE.Vector3(points.O.x + .25, points.O.y + .4, points.O.z + .1);
      const actual = worldToUserFrame(probe, matrix), expected = new THREE.Vector3(.25, -.1, .4);
      const error = actual.distanceTo(expected);
      if (error > 1e-8 || !matrix.isMatrix4) throw Error('Group B coordinate mismatch');
      station.workpiece.updateMatrixWorld(true);
      return { source: 'threejs-scene-simulation-not-physical-measurement', points: values,
        independentPoint: { world: probe.toArray(), expectedUser: expected.toArray(), actualUser: actual.toArray(), error },
        userMatrixColumnMajor: matrix.toArray(), tcpOffsetMatrixColumnMajor: api.calibration.tcpOffsetMatrix.toArray(),
        restingWorldMatrix: station.workpiece.matrixWorld.toArray() };
    });
    await page.click('#tab-workstation'); await page.click('#attach-workpiece');
    const attachment = await page.evaluate(async () => {
      const THREE = await import('./js/vendor/three/build/three.module.js');
      const api = window.robotSimulation, station = api.workstation;
      const contact = station.workpiece.localToWorld(new THREE.Vector3(0, station.blockSize / 2, 0));
      return { attached: station.attached, parent: station.workpiece.parent.name, contactError: contact.distanceTo(api.tcp.getWorldPosition(new THREE.Vector3())), beforeMotion: station.workpiece.getWorldPosition(new THREE.Vector3()).toArray() };
    });
    assert.equal(attachment.attached, true); assert.equal(attachment.parent, 'link_flange'); assert(attachment.contactError < 1e-8);
    await page.click('#release-estop'); await page.check('#motion-enable');
    const following = await page.evaluate(async () => {
      const THREE = await import('./js/vendor/three/build/three.module.js');
      const api = window.robotSimulation, station = api.workstation;
      const local = station.workpiece.matrix.clone(), before = station.workpiece.getWorldPosition(new THREE.Vector3());
      api.motion.target('J1', .2);
      for (let i = 0; i < 10000 && api.motion.busy(); i++) api.motion.update(.05);
      station.workpiece.updateMatrixWorld(true);
      const localError = Math.max(...local.elements.map((v, i) => Math.abs(v - station.workpiece.matrix.elements[i])));
      const worldMotion = station.workpiece.getWorldPosition(new THREE.Vector3()).distanceTo(before);
      if (localError > 1e-10 || worldMotion < .01) throw Error('Attached workpiece failed to follow');
      return { localMatrixError: localError, worldMotion };
    });
    await page.click('#detach-workpiece');
    const release = await page.evaluate((rest) => {
      const station = window.robotSimulation.workstation;
      station.scene.updateMatrixWorld(true);
      const worldError = Math.max(...station.workpiece.matrixWorld.elements.map((v, i) => Math.abs(v - rest[i])));
      for (let i = 0; i < 3; i++) { station.attachWorkpiece(); station.detachWorkpiece(); }
      station.scene.updateMatrixWorld(true);
      const repeatError = Math.max(...station.workpiece.matrixWorld.elements.map((v, i) => Math.abs(v - rest[i])));
      return { attached: station.attached, parent: station.workpiece.parent.name, worldError, repeatError };
    }, groupB.restingWorldMatrix);
    assert.equal(release.parent, 'table'); assert.equal(release.attached, false); assert(release.worldError < 1e-8); assert(release.repeatError < 1e-8);
    fs.writeFileSync(path.join(root, '组员B_工位验证数据.json'), JSON.stringify({ ...groupB, placement, workstationSourceSHA256: require('crypto').createHash('sha256').update(fs.readFileSync(path.join(root, 'js/Workstation.js'))).digest('hex'), attachment, following, release }, null, 2) + '\n');
    await page.screenshot({ path: path.join(process.env.TEMP, 'robot-workstation-qa.png') });
    // Inspect main controls and calibration layout separately in screenshots.
    await page.evaluate(() => { document.querySelector('.panel').scrollTop = 0; });
    await page.click('#tab-operation');
    await page.waitForTimeout(250);
    await page.screenshot({ path: path.join(process.env.TEMP, 'robot-pendant-qa.png') });
    await page.click('#tab-tcp');
    await page.locator('#apply-known-tcp').scrollIntoViewIfNeeded();
    await page.screenshot({ path: path.join(process.env.TEMP, 'robot-calibration-qa.png') });
    assert.deepEqual(errors, []);
    console.log('PASS: model rendering, jog/release, emergency stop, calibration, JSON records, MOVJ scan, four recorded poses, edited code execution and program interruption.');
    console.log('QA screenshots:', path.join(process.env.TEMP, 'robot-pendant-qa.png'), path.join(process.env.TEMP, 'robot-calibration-qa.png'));
  } finally { if (browser) await browser.close(); server.close(); }
})().catch((error) => { console.error(error); process.exitCode = 1; });

