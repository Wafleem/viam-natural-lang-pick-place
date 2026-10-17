import test from 'node:test';
import assert from 'node:assert/strict';
import { ReviewController, STATUS_MAX_AGE_MS } from '../src/review-controller.js';
import { parseIntent } from '../src/parser.js';
import { CODE2_REFERENCE } from '../src/code2-contract.js';

const TEXT = 'Move the cube to the ring';
const intent = () => parseIntent(TEXT).intent;
const status = (extra = {}) => ({ implementation: CODE2_REFERENCE.implementation,
  calibrated: true, busy: false, state: 'idle', request_id: '', available_goals: ['ring', 'box'], ...extra });
const deferred = () => {
  let resolve, reject; const promise = new Promise((yes, no) => { resolve = yes; reject = no; });
  return { promise, resolve, reject };
};
function fixture(options = {}) {
  let time = 10000, seq = 0, reply = status(), nextSend = command => ({ accepted: true, request_id: command.request_id });
  const calls = [], journal = [];
  const controller = new ReviewController({ now: () => time, uuid: () => `request-${++seq}`,
    savePending: record => journal.push(record), ...options });
  const transport = async command => {
    calls.push({ ...command });
    return command.action === 'status' ? (typeof reply === 'function' ? reply(command) : reply) : nextSend(command);
  };
  return { controller, calls, journal, transport, time: value => { time = value; },
    reply: value => { reply = value; }, sendReply: value => { nextSend = value; },
    ready: async () => { controller.attach(transport); await controller.refresh(); controller.review(intent(), TEXT); controller.acknowledge(true); } };
}

test('explicit attach, review and acknowledgment never call transport; only refresh reads status', async () => {
  const f = fixture(), c = f.controller;
  c.attach(f.transport); c.review(intent(), TEXT); c.acknowledge(true);
  assert.equal(c.canSend, false); assert.deepEqual(f.calls, []);
  await c.refresh(); assert.deepEqual(f.calls, [{ action: 'status' }]);
  assert.equal(c.canSend, false); assert.equal(c.acknowledge(true), true); assert.equal(c.canSend, true);
});

test('only exact supported intent matching its text can be reviewed; caller mutation has no effect', async () => {
  const f = fixture(); await f.ready(); const c = f.controller;
  const original = intent(); c.review(original, TEXT); original.source_selector.kind = 'box';
  assert.equal(c.state.review.intent.source_selector.kind, 'cube');
  assert.throws(() => { c.state.review.intent.source_selector.color = 'white'; }, TypeError);
  for (const text of ['Move white cube to ring', 'Move cube on right to ring', 'Put cube on the box', 'Move cube to ring with pink paper']) {
    assert.equal(c.review(parseIntent(text).intent, text), false); assert.equal(c.acknowledge(true), false);
    assert.equal((await c.send()).sent, false);
  }
  assert.equal(c.review(intent(), 'Move white cube to ring'), false);
  assert.equal(c.review({ ...intent(), repeat: 2 }, TEXT), false);
});

test('status must match implementation, booleans and known safe state; freshness is bounded', async () => {
  for (const bad of [{ implementation: 'other-version' }, { available_goals: ['box'] }, { available_goals: undefined }, { calibrated: 'true' }, { busy: 0 },
    { state: 'mystery' }, { request_id: undefined }, { calibrated: false }, { busy: true },
    { state: 'running', busy: false }]) {
    const f = fixture(); f.reply(status(bad)); await f.ready();
    assert.equal(f.controller.canSend, false, JSON.stringify(bad));
    assert.equal((await f.controller.send()).sent, false); assert.equal(f.calls.filter(c => c.action === 'start').length, 0);
  }
  const f = fixture(); await f.ready();
  f.time(10000 + STATUS_MAX_AGE_MS); assert.equal(f.controller.canSend, true);
  f.time(10001 + STATUS_MAX_AGE_MS); assert.equal(f.controller.canSend, false);
  f.time(9999); assert.equal(f.controller.canSend, false);
});

test('slow status replies expire from request start rather than receiving a new freshness window', async () => {
  const f = fixture(), pending = deferred(); f.reply(() => pending.promise); f.controller.attach(f.transport);
  const reading = f.controller.refresh(); f.time(16000); pending.resolve(status()); await reading;
  f.controller.review(intent(), TEXT); assert.equal(f.controller.acknowledge(true), false);
});

test('edits, retranslation, reconnect and meaningful status changes clear acknowledgment', async () => {
  const f = fixture(); await f.ready(); const c = f.controller;
  c.setInstruction(TEXT); assert.equal(c.canSend, false); assert.equal(c.state.review, null);
  c.review(intent(), TEXT); c.acknowledge(true); c.review(intent(), TEXT); assert.equal(c.state.acknowledged, false);
  c.acknowledge(true); f.reply(status({ request_id: 'another', state: 'complete' })); await c.refresh();
  assert.equal(c.state.acknowledged, false);
  c.acknowledge(true); c.attach(f.transport); assert.equal(c.state.review, null); assert.equal(c.canSend, false);
});

test('explicit send consumes review before await, creates only exact start payload, and cannot duplicate', async () => {
  const f = fixture(); await f.ready(); const pending = deferred(); f.sendReply(() => pending.promise);
  const c = f.controller, sending = c.send();
  assert.equal(c.state.review.consumed, true); assert.equal(c.state.request.phase, 'sending');
  assert.equal((await c.send()).sent, false);
  assert.deepEqual(f.calls.at(-1), { action: 'start', goal: 'ring', request_id: 'request-1', workcell_ready: true });
  assert.deepEqual(f.journal[0], { request_id: 'request-1', phase: 'unknown' });
  pending.resolve({ accepted: true, request_id: 'request-1', state: 'complete', busy: false }); await sending;
  assert.equal(c.state.request.phase, 'accepted'); assert.equal(c.state.request.resolved, false);
  assert.equal(f.calls.filter(x => x.action === 'start').length, 1);
});

test('cube-into-box sends box_once, but only while code-2 reports its box workflow enabled', async () => {
  const BOX = 'Put the cube into the box';
  const boxFixture = async extra => {
    const f = fixture(); f.reply(status(extra)); f.controller.attach(f.transport); await f.controller.refresh();
    assert.equal(f.controller.review(parseIntent(BOX).intent, BOX), true); f.controller.acknowledge(true); return f;
  };
  let f = await boxFixture({});
  assert.equal(f.controller.state.blockedReason, 'box_disabled');
  assert.equal((await f.controller.send()).sent, false); assert.equal(f.calls.filter(x => x.action === 'start').length, 0);
  f = await boxFixture({ surveyed_box_enabled: true, available_goals: ['ring'] });
  assert.equal(f.controller.state.blockedReason, 'goal_unavailable');
  f = await boxFixture({ surveyed_box_enabled: true });
  assert.equal(f.controller.canSend, true); await f.controller.send();
  assert.deepEqual(f.calls.at(-1), { action: 'box_once', request_id: 'request-1', workcell_ready: true });
});

test('an old status read cannot overwrite state after dispatch; a new status read can run', async () => {
  const f = fixture(); await f.ready(); const old = deferred(); f.reply(() => old.promise);
  const reading = f.controller.refresh(); await f.controller.send();
  f.reply(status({ state: 'running', busy: true, request_id: 'request-1' })); await f.controller.refresh();
  old.resolve(status()); await reading;
  assert.equal(f.controller.state.status.request_id, 'request-1'); assert.equal(f.controller.state.request.phase, 'running');
});

test('stale status cannot revive a disconnected session or replace a newer connection read', async () => {
  const f = fixture(), old = deferred(); f.reply(() => old.promise); f.controller.attach(f.transport);
  const reading = f.controller.refresh(); f.controller.lostConnection();
  old.resolve(status()); await reading; assert.equal(f.controller.state.connected, false); assert.equal(f.controller.state.status, null);
  const next = deferred(); f.reply(() => next.promise); f.controller.attach(f.transport); const oldRead = f.controller.refresh();
  f.controller.attach(async () => status({ request_id: 'new-session' })); await f.controller.refresh();
  next.resolve(status({ request_id: 'old-session' })); await oldRead;
  assert.equal(f.controller.state.status.request_id, 'new-session');
});

test('ambiguous send stays unresolved across edits, disconnect and reconnect; only matching nonbusy terminal resolves', async () => {
  const f = fixture(); await f.ready(); f.sendReply(() => { throw new Error('timeout'); });
  await f.controller.send(); assert.equal(f.controller.state.request.phase, 'unknown');
  f.controller.detach(); f.controller.attach(f.transport);
  for (const s of [status(), status({ request_id: 'another', state: 'complete' }),
    status({ request_id: 'request-1', state: 'complete', busy: true }),
    status({ request_id: 'request-1', state: 'running', busy: false })]) {
    f.reply(s); await f.controller.refresh(); f.controller.review(intent(), TEXT);
    assert.equal(f.controller.acknowledge(true), false); assert.equal(f.controller.state.request.resolved, false);
  }
  f.reply(status({ request_id: 'request-1', state: 'complete' })); await f.controller.refresh();
  assert.equal(f.controller.state.request.resolved, true); assert.equal(f.controller.state.request.phase, 'complete');
  assert.equal(f.journal.at(-1), null); assert.equal(f.controller.canSend, false);
});

test('accepted request still requires correlated terminal status and consumed review never rearms', async () => {
  const f = fixture(); await f.ready(); await f.controller.send();
  f.reply(status({ request_id: 'request-1', state: 'failed' })); await f.controller.refresh();
  assert.equal(f.controller.state.request.resolved, true); assert.equal(f.controller.acknowledge(true), false);
  assert.equal((await f.controller.send()).sent, false);
  f.controller.review(intent(), TEXT); assert.equal(f.controller.acknowledge(true), true);
});

test('terminal status observed before late send reply cannot regress to accepted or unknown', async () => {
  for (const failure of [false, true]) {
    const f = fixture(); await f.ready(); const pending = deferred(); f.sendReply(() => pending.promise);
    const sending = f.controller.send(); f.reply(status({ request_id: 'request-1', state: 'stopped' })); await f.controller.refresh();
    if (failure) pending.reject(new Error('late timeout')); else pending.resolve({ accepted: true, request_id: 'request-1' });
    await sending; assert.equal(f.controller.state.request.phase, 'stopped'); assert.equal(f.controller.state.request.resolved, true);
  }
});

test('a late reply after reconnect cannot become current session status or rearm review', async () => {
  const f = fixture(); await f.ready(); const pending = deferred(); f.sendReply(() => pending.promise);
  const sending = f.controller.send(); f.controller.detach(); f.controller.attach(f.transport);
  pending.resolve({ accepted: true, request_id: 'request-1' }); await sending;
  assert.equal(f.controller.state.request.phase, 'unknown'); assert.equal(f.controller.state.status, null);
  assert.equal(f.controller.state.review, null);
});

test('busy rejection is not acceptance of another task; malformed/mismatched replies remain unknown', async () => {
  const f = fixture(); await f.ready(); f.sendReply(() => ({ ...status({ busy: true, state: 'running', request_id: 'other' }), accepted: false, already_running: true }));
  await f.controller.send(); assert.equal(f.controller.state.request.phase, 'rejected_busy'); assert.equal(f.controller.state.request.resolved, true);
  assert.equal(f.controller.canSend, false);
  for (const reply of [null, {}, { accepted: true, request_id: 'wrong' }, { duplicate_request: true },
    { accepted: false, already_running: true },
    { ...status(), accepted: false, already_running: true }]) {
    const other = fixture(); await other.ready(); other.sendReply(() => reply); await other.controller.send();
    assert.equal(other.controller.state.request.phase, 'unknown'); assert.equal(other.controller.state.request.resolved, false);
  }
});

test('journal is written before transport; journal failure or asynchronous writer prevents dispatch', async () => {
  for (const savePending of [() => { throw new Error('storage denied'); }, () => false, async () => {}]) {
    const f = fixture({ savePending }); await f.ready();
    assert.equal((await f.controller.send()).reason, 'journal_unavailable');
    assert.equal(f.calls.filter(x => x.action === 'start').length, 0);
  }
  const order = [], f = fixture({ savePending: record => order.push(record ? 'journal' : 'clear') });
  await f.ready(); f.sendReply(command => { order.push('dispatch'); return { accepted: true, request_id: command.request_id }; });
  await f.controller.send(); assert.deepEqual(order, ['journal', 'dispatch']);
});

test('reload restores unknown request with no credentials and cannot send until correlated final status', async () => {
  const f = fixture({ pendingRequest: { request_id: 'old-request', phase: 'unknown' } }); await f.ready();
  assert.equal(f.controller.canSend, false); assert.equal(f.controller.state.request.request_id, 'old-request');
  f.reply(status({ request_id: 'old-request', state: 'complete' })); await f.controller.refresh();
  assert.equal(f.controller.state.request.resolved, true);
  for (const pendingRequest of [{}, { request_id: '' }, { request_id: 'x'.repeat(81) }]) {
    const bad = fixture({ pendingRequest }); await bad.ready(); assert.equal(bad.controller.canSend, false);
  }
});

test('journal clearing failures keep sends blocked even after known completion', async () => {
  const f = fixture({ savePending: record => { if (record === null) throw new Error('write denied'); } }); await f.ready();
  await f.controller.send(); f.reply(status({ request_id: 'request-1', state: 'complete' })); await f.controller.refresh();
  assert.equal(f.controller.state.request.phase, 'complete'); assert.equal(f.controller.state.request.resolved, false);
  assert.equal(f.controller.canSend, false);
});

test('synchronous callbacks cannot change the reviewed text/connection during UUID or dispatch notification', async () => {
  let c;
  const f = fixture({ uuid: () => { c.setInstruction('Move white cube to ring'); return 'id'; } }); c = f.controller; await f.ready();
  assert.equal((await c.send()).sent, false); assert.equal(f.calls.filter(x => x.action === 'start').length, 0);
  let second;
  const g = fixture({ changed: state => { if (state.request?.phase === 'sending') second.detach(); } }); second = g.controller; await g.ready();
  assert.equal((await second.send()).sent, false); assert.equal(g.calls.filter(x => x.action === 'start').length, 0);
  assert.equal(second.state.request.phase, 'not_sent'); assert.equal(second.state.request.resolved, true);
});

test('request IDs cannot be invalid or reused locally, including a recovered request ID', async () => {
  for (const uuid of [() => '', () => null, () => 'x'.repeat(81), () => { throw new Error('unavailable'); }]) {
    const f = fixture({ uuid }); await f.ready(); assert.equal((await f.controller.send()).sent, false);
  }
  const f = fixture({ uuid: () => 'fixed' }); await f.ready(); await f.controller.send();
  f.reply(status({ request_id: 'fixed', state: 'complete' })); await f.controller.refresh();
  f.controller.review(intent(), TEXT); f.controller.acknowledge(true);
  assert.equal((await f.controller.send()).reason, 'invalid_request_id');
});
