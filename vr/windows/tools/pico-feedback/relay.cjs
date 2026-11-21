const fs = require('node:fs');
const path = require('node:path');
const net = require('node:net');
const dgram = require('node:dgram');
const { execFileSync, spawn } = require('node:child_process');

const root = path.resolve(__dirname, '../..');
const adb = path.join(root, 'tools/platform-tools/adb.exe');
const serial = process.argv[2];
if (!serial) throw new Error('Pass the authorized USB headset serial.');
const statePath = path.join(root, 'pico-feedback-state.json');
const robot = '6.6.7.100';
let localAddress;
const sockets = [];
const pending = new Map();
const state = {
  pid: process.pid, nodePath: process.execPath, worker: __filename, serial,
  startedAt: new Date().toISOString(), phase: 'starting', localAddress, robot,
  received: { 9002: 0, 9003: 0 }, forwarded: { 9002: 0, 9003: 0 },
  coalesced: 0, rejected: 0, headset: null,
};
let tunnel;
let helper;
let timer;
let blocked = false;
let stopping = false;
let ownsForward = false;

function adbRun(args) {
  return execFileSync(adb, ['-s', serial, ...args], {
    encoding: 'utf8', timeout: 15000, windowsHide: true,
  }).trim();
}

function save() {
  state.updatedAt = new Date().toISOString();
  const temporary = statePath + '.tmp';
  fs.writeFileSync(temporary, JSON.stringify(state, null, 2));
  fs.renameSync(temporary, statePath);
}

function stop(error) {
  if (stopping) return;
  stopping = true;
  clearInterval(timer);
  for (const socket of sockets) {
    try { socket.close(); } catch {}
  }
  tunnel?.destroy();
  helper?.kill();
  if (ownsForward) {
    try {
      const mapping = adbRun(['forward', '--list']);
      if (mapping.split(/\r?\n/).some(line => line.trim() === `${serial} tcp:39002 tcp:39002`)) {
        adbRun(['forward', '--remove', 'tcp:39002']);
      }
    } catch {}
  }
  state.phase = error ? 'error' : 'stopped';
  if (error) state.error = String(error.message || error);
  save();
  if (error) console.error(state.error);
  process.exit(error ? 1 : 0);
}

function send(port, payload) {
  const frame = Buffer.allocUnsafe(4 + payload.length);
  frame.writeUInt16BE(port, 0);
  frame.writeUInt16BE(payload.length, 2);
  payload.copy(frame, 4);
  blocked = !tunnel.write(frame);
  state.forwarded[port]++;
}

function flushLatest() {
  for (const [port, payload] of pending) {
    if (blocked) break;
    pending.delete(port);
    send(port, payload);
  }
}

async function start() {
  if (adbRun(['get-state']) !== 'device') throw new Error('PICO USB is not authorized.');
  // Select the source address used by the current route to the robot.
  const routeProbe = dgram.createSocket('udp4');
  try {
    await new Promise((resolve, reject) => {
      routeProbe.once('error', reject);
      routeProbe.connect(9002, robot, resolve);
    });
    localAddress = routeProbe.address().address;
    state.localAddress = localAddress;
  } finally {
    routeProbe.close();
  }
  save();
  for (const port of [9002, 9003]) {
    const socket = dgram.createSocket('udp4');
    sockets.push(socket);
    socket.on('error', stop);
    socket.on('message', (payload, remote) => {
      if (remote.address !== robot || payload.length < 2 || payload.length > 8192) {
        state.rejected++;
        return;
      }
      state.received[port]++;
      // Keep only the freshest pose during congestion instead of queuing stale feedback.
      if (state.phase !== 'connected' || blocked) {
        if (pending.has(port)) state.coalesced++;
        pending.set(port, payload);
      } else {
        send(port, payload);
      }
    });
    await new Promise((resolve, reject) => {
      socket.once('error', reject);
      socket.bind(port, localAddress, resolve);
    });
  }
  adbRun(['push', path.join(__dirname, 'classes.dex'), '/data/local/tmp/luna-pico-feedback.dex']);
  const mappings = adbRun(['forward', '--list']).split(/\r?\n/);
  const existing = mappings.find(line => line.trim().split(/\s+/)[1] === 'tcp:39002');
  if (existing && existing.trim() !== `${serial} tcp:39002 tcp:39002`) {
    throw new Error('ADB port 39002 belongs to another connection.');
  }
  if (!existing) adbRun(['forward', '--no-rebind', 'tcp:39002', 'tcp:39002']);
  ownsForward = true;
  helper = spawn(adb, [
    '-s', serial, 'shell', '-T',
    'CLASSPATH=/data/local/tmp/luna-pico-feedback.dex app_process /system/bin PicoFeedbackRelay',
  ], { windowsHide: true, stdio: ['ignore', 'pipe', 'pipe'] });
  helper.stderr.on('data', data => console.error(data.toString().trim()));
  helper.on('error', stop);
  helper.on('exit', (code, signal) => {
    if (!stopping) stop(new Error(`PICO feedback helper exited: ${code ?? signal}`));
  });
  await new Promise((resolve, reject) => {
    const timeout = setTimeout(() => reject(new Error('PICO feedback helper startup timed out.')), 10000);
    let output = '';
    helper.stdout.on('data', data => {
      output = (output + data.toString()).slice(-2048);
      if (output.includes('LISTENING 127.0.0.1:39002')) {
        clearTimeout(timeout);
        resolve();
      }
    });
  });
  tunnel = net.createConnection({ host: '127.0.0.1', port: 39002 });
  tunnel.setNoDelay(true);
  tunnel.on('error', stop);
  tunnel.on('close', () => { if (!stopping) stop(new Error('PICO feedback tunnel closed.')); });
  tunnel.on('drain', () => { blocked = false; flushLatest(); });
  let acknowledgements = '';
  tunnel.on('data', data => {
    acknowledgements += data.toString('utf8');
    if (acknowledgements.length > 8192) return stop(new Error('Invalid feedback acknowledgement.'));
    let newline;
    while ((newline = acknowledgements.indexOf('\n')) >= 0) {
      const line = acknowledgements.slice(0, newline);
      acknowledgements = acknowledgements.slice(newline + 1);
      try {
        state.headset = { ...JSON.parse(line), acknowledgedAt: new Date().toISOString() };
        save();
      } catch (error) { stop(error); }
    }
  });
  await new Promise(resolve => tunnel.once('connect', resolve));
  state.phase = 'connected';
  flushLatest();
  save();
  console.log('Feedback relay connected: robot UDP 9002/9003 -> USB -> PICO loopback UDP 9002/9003.');
  timer = setInterval(() => {
    if (!blocked) blocked = !tunnel.write(Buffer.alloc(4));
    save();
  }, 5000);
}

process.on('SIGINT', () => stop());
process.on('SIGTERM', () => stop());
start().catch(stop);
