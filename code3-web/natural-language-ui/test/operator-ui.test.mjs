// Offline integration: execute the actual operator app with real policy/parser,
// a small DOM, and a fake transport. Never imports or dials the Viam SDK.
import test from 'node:test';
import assert from 'node:assert/strict';
import { readFile } from 'node:fs/promises';
import { Script } from 'node:vm';
import { parseIntent, MAX_TEXT_CHARS, MAX_TEXT_WORDS } from '../src/parser.js';
import { CODE2_REFERENCE, previewCode2Command } from '../src/code2-contract.js';
import { ReviewController } from '../src/review-controller.js';

const source = await readFile(new URL('../src/app.js', import.meta.url), 'utf8');
const html = await readFile(new URL('../index.html', import.meta.url), 'utf8');
const flush = async () => { for (let i = 0; i < 8; i++) await Promise.resolve(); };
const deferred = () => { let resolve, reject; const promise = new Promise((yes, no) => { resolve = yes; reject = no; }); return { promise, resolve, reject }; };
const initialStatus = () => ({ implementation: CODE2_REFERENCE.implementation, calibrated: true,
  busy: false, state: 'idle', stage: 'idle', request_id: '', error: '', available_goals: ['ring', 'box'] });

function harness({ start, storage = new Map(), storageWriteFails = false } = {}) {
  const elements = new Map();
  for (const [, id] of html.matchAll(/\bid="([^"]+)"/g)) elements.set(id, {
    value: '', textContent: '', disabled: false, checked: false, hidden: false,
    handlers: {}, classList: { toggle() {}, add() {}, remove() {} },
    addEventListener(event, handler) { this.handlers[event] = handler; },
  });
  const el = id => { assert.ok(elements.has(id), `app references absent DOM id ${id}`); return elements.get(id); };
  el('auth-mode').value = 'manual';
  const calls = [], connects = [], timers = [], copied = [];
  let current = initialStatus(), disconnects = 0;
  const sessionStorage = {
    getItem: key => storage.has(key) ? storage.get(key) : null,
    setItem: (key, value) => { if (storageWriteFails) throw new Error('fake storage unavailable'); storage.set(key, String(value)); },
    removeItem: key => storage.delete(key),
  };
  const context = {
    parseIntent, MAX_TEXT_CHARS, MAX_TEXT_WORDS, CODE2_REFERENCE, previewCode2Command, ReviewController,
    setupSpeech() {},
    async createViamConnection(options) {
      connects.push(options);
      return { host: options.host, async disconnect() { disconnects++; }, async command(payload) {
        calls.push(structuredClone(payload));
        if (payload.action === 'status') return structuredClone(current);
        assert.equal(payload.action, 'start');
        if (start) return start(payload);
        current = { ...current, request_id: payload.request_id, state: 'start', busy: true };
        return { ...current, accepted: true };
      } };
    },
    getHostedConnection() { throw new Error('Hosted connection not used by offline test'); },
    document: { getElementById: el, querySelectorAll: () => [], cookie: '' },
    window: { addEventListener() {}, sessionStorage, localStorage: sessionStorage },
    sessionStorage, localStorage: sessionStorage,
    location: { pathname: '/', hostname: 'localhost', origin: 'http://localhost', protocol: 'http:' },
    navigator: { clipboard: { async writeText(text) { copied.push(text); } } },
    setInterval(fn) { timers.push(fn); return timers.length; }, clearInterval() {},
    setTimeout(fn) { timers.push(fn); return timers.length; }, clearTimeout() {},
  };
  const code = source.replace(/^import [^\n]+;\n/gm, '');
  assert.doesNotMatch(code, /^\s*import\b/m);
  new Script(code, { filename: 'actual-operator-app.js' }).runInNewContext(context);
  const event = async (id, type) => {
    const handler = el(id).handlers[type]; assert.equal(typeof handler, 'function', `${id}:${type}`);
    await handler({ preventDefault() {} }); await flush();
  };
  return { el, calls, connects, timers, copied, storage, event,
    get disconnects() { return disconnects; },
    setStatus(value) { current = value; },
    async connect() {
      el('host').value = 'offline-fixture.viam.cloud'; el('key-id').value = 'fake-id'; el('key').value = 'fake-key';
      await event('connect-form', 'submit');
    },
    async translate(text = 'Move the cube to the ring') {
      el('instruction').value = text; await event('instruction', 'input'); await event('translate', 'click');
    },
    async acknowledge() { el('reviewed').checked = true; await event('reviewed', 'change'); },
  };
}

test('operator page is inert on load; explicit connect only reads status and clears credential fields', async () => {
  const ui = harness();
  assert.equal(ui.connects.length, 0); assert.equal(ui.calls.length, 0);
  for (const tick of ui.timers) tick(); await flush();
  assert.equal(ui.calls.length, 0); assert.equal(ui.connects.length, 0);
  await ui.translate(); assert.equal(ui.calls.length, 0);
  assert.equal(ui.el('send-reviewed').disabled, true);
  await ui.connect();
  assert.equal(ui.connects.length, 1);
  assert.deepEqual(ui.calls, [{ action: 'status' }]);
  assert.equal(ui.el('key').value, ''); assert.equal(ui.el('key-id').value, '');
  assert.equal(ui.el('send-reviewed').disabled, true);
  assert.equal(JSON.stringify([...ui.storage]).includes('fake-key'), false);
});

test('Tab fills the suggested instruction only when it would complete it, and never sends', async () => {
  const ui = harness(), box = ui.el('instruction');
  box.placeholder = 'Pick up the cube and place it on the ring.';
  const press = (extra = {}) => { let prevented = false;
    box.handlers.keydown({ key: 'Tab', preventDefault() { prevented = true; }, ...extra }); return prevented; };
  assert.equal(press(), true); assert.equal(box.value, box.placeholder);
  assert.equal(press(), false);
  box.value = 'pick up'; assert.equal(press(), true); assert.equal(box.value, box.placeholder);
  box.value = 'Move the cube'; assert.equal(press(), false); assert.equal(box.value, 'Move the cube');
  box.value = ''; assert.equal(press({ shiftKey: true }), false); assert.equal(box.value, '');
  assert.deepEqual(ui.calls, []);
});

test('supported review plus acknowledgment sends exactly once; acceptance is not completion', async () => {
  const ui = harness(); await ui.connect(); await ui.translate();
  assert.equal(ui.el('send-reviewed').disabled, true);
  await ui.acknowledge(); assert.equal(ui.el('send-reviewed').disabled, false);
  await Promise.all([ui.event('send-reviewed', 'click'), ui.event('send-reviewed', 'click')]);
  const starts = ui.calls.filter(x => x.action === 'start'); assert.equal(starts.length, 1);
  assert.deepEqual(Object.keys(starts[0]).sort(), ['action', 'goal', 'request_id', 'workcell_ready']);
  assert.equal(starts[0].goal, 'ring');
  assert.equal(starts[0].workcell_ready, true);
  assert.match(starts[0].request_id, /^[0-9a-f-]{36}$/i);
  assert.equal(ui.el('request-state').textContent.toLowerCase(), 'accepted');
  assert.match(ui.el('status-message').textContent, /acceptance is not completion/i);
  assert.equal(ui.el('send-reviewed').disabled, true);
  ui.setStatus({ ...initialStatus(), request_id: starts[0].request_id, state: 'complete', stage: 'complete' });
  await ui.event('refresh-status', 'click');
  assert.equal(ui.el('request-state').textContent.toLowerCase(), 'completed');
  assert.equal(ui.el('send-reviewed').disabled, true);
  assert.equal(ui.calls.filter(x => x.action === 'start').length, 1);
});

test('qualifiers and edits invalidate send authorization, even if a disabled handler is invoked', async () => {
  const ui = harness(); await ui.connect(); await ui.translate(); await ui.acknowledge();
  ui.el('instruction').value = 'Move the white cube to the ring'; await ui.event('instruction', 'input');
  assert.equal(ui.el('reviewed').checked, false); assert.equal(ui.el('send-reviewed').disabled, true);
  await ui.event('send-reviewed', 'click'); assert.equal(ui.calls.some(x => x.action === 'start'), false);
  await ui.event('translate', 'click'); await ui.acknowledge(); await ui.event('send-reviewed', 'click');
  assert.equal(JSON.parse(ui.el('command').textContent).source_selector.color, 'white');
  assert.equal(ui.calls.some(x => x.action === 'start'), false);
  assert.equal(ui.el('copy-template').disabled, true);
});

test('lost connection disarms review and cannot send or retry', async () => {
  const ui = harness(); await ui.connect(); await ui.translate(); await ui.acknowledge();
  ui.connects[0].onDisconnect(); await flush();
  assert.equal(ui.el('send-reviewed').disabled, true); assert.equal(ui.el('reviewed').checked, false);
  await ui.event('send-reviewed', 'click');
  for (const tick of ui.timers) tick(); await flush();
  assert.equal(ui.calls.some(x => x.action === 'start'), false);
});

test('unknown RPC result blocks repeat after new text, status mismatch, and reconnect', async () => {
  const ui = harness({ start: async () => { throw new Error('fake transport timeout'); } });
  await ui.connect(); await ui.translate(); await ui.acknowledge(); await ui.event('send-reviewed', 'click');
  assert.equal(ui.el('request-state').textContent.toLowerCase(), 'unknown');
  await ui.translate(); await ui.acknowledge(); await ui.event('send-reviewed', 'click');
  await ui.event('refresh-status', 'click');
  await ui.event('disconnect', 'click'); await ui.connect(); await ui.translate(); await ui.acknowledge();
  await ui.event('send-reviewed', 'click');
  assert.equal(ui.calls.filter(x => x.action === 'start').length, 1);
  assert.equal(ui.el('send-reviewed').disabled, true);
});

test('disconnect during pending send ignores late acceptance and keeps outcome unresolved', async () => {
  const pending = deferred(), ui = harness({ start: () => pending.promise });
  await ui.connect(); await ui.translate(); await ui.acknowledge();
  await ui.event('send-reviewed', 'click');
  const sent = ui.calls.find(x => x.action === 'start'); assert.ok(sent);
  ui.connects[0].onDisconnect();
  pending.resolve({ accepted: true, request_id: sent.request_id }); await flush();
  assert.equal(ui.el('request-state').textContent.toLowerCase(), 'unknown');
  assert.equal(ui.el('send-reviewed').disabled, true);
  assert.equal(ui.calls.filter(x => x.action === 'start').length, 1);
});

test('pending send survives a page reload without credentials and clears only on correlated terminal status', async () => {
  const storage = new Map();
  const before = harness({ storage, start: async () => { throw new Error('fake lost reply'); } });
  await before.connect(); await before.translate(); await before.acknowledge(); await before.event('send-reviewed', 'click');
  const sent = before.calls.find(x => x.action === 'start'); assert.ok(sent);
  assert.equal(storage.size, 1);
  const saved = JSON.parse([...storage.values()][0]);
  assert.deepEqual(saved, { request_id: sent.request_id, phase: 'unknown' });
  assert.equal(JSON.stringify([...storage]).includes('fake-key'), false);
  const after = harness({ storage });
  assert.equal(after.calls.length, 0);
  await after.connect(); await after.translate(); await after.acknowledge(); await after.event('send-reviewed', 'click');
  assert.equal(after.calls.some(x => x.action === 'start'), false);
  assert.equal(after.el('send-reviewed').disabled, true);
  after.setStatus({ ...initialStatus(), request_id: sent.request_id, state: 'complete', stage: 'complete' });
  await after.event('refresh-status', 'click');
  assert.equal(storage.size, 0);
  assert.equal(after.el('request-state').textContent, 'Completed');
  assert.equal(after.calls.some(x => x.action === 'start'), false);
});

test('failure to persist pending request prevents dispatch', async () => {
  const ui = harness({ storageWriteFails: true });
  await ui.connect(); await ui.translate(); await ui.acknowledge(); await ui.event('send-reviewed', 'click');
  assert.equal(ui.calls.some(x => x.action === 'start'), false);
  assert.match(ui.el('status-message').textContent, /journal could not be saved/i);
  assert.equal(ui.el('send-reviewed').disabled, true);
});
