import test from 'node:test';
import assert from 'node:assert/strict';
import { createViamConnection, getHostedConnection, validateHost } from '../src/viam-connection.js';
import { GenericServiceClient } from '@viamrobotics/sdk';
import { Struct } from '@bufbuild/protobuf';

const options = { host: 'machine-main.example.viam.cloud', keyId: 'fixture-id', apiKey: 'fixture-secret' };
function fakeSdk() {
  const calls = [], listeners = new Map(); let config, disconnected = 0, resource;
  const machine = { on: (event, cb) => listeners.set(event, cb), off: event => listeners.delete(event),
    async disconnect() { disconnected++; } };
  const sdk = { async createRobotClient(value) { config = value; return machine; },
    GenericServiceClient: class { constructor(client, name) { assert.equal(client, machine); resource = name; }
      async doCommand(payload, callOptions) { calls.push({ payload, callOptions }); return { accepted: true }; } } };
  return { sdk, calls, listeners, get config() { return config; }, get disconnected() { return disconnected; }, get resource() { return resource; } };
}

test('connect authenticates once, targets code-2 service and makes no implicit commands', async () => {
  const fake = fakeSdk(); const connection = await createViamConnection(options, { loadSdk: async () => fake.sdk });
  assert.equal(fake.calls.length, 0); assert.equal(fake.resource, 'code-2');
  assert.deepEqual(fake.config.credentials, { type: 'api-key', authEntity: 'fixture-id', payload: 'fixture-secret' });
  assert.equal(fake.config.noReconnect, true); assert.equal(fake.config.disableSessions, undefined);
  await connection.command({ action: 'status' });
  const payload = { action: 'pick_place', pick: { object: 'cube', select: 'only' }, place: { type: 'ring' }, request_id: '91cb63f3-066d-41f7-9a5a-95772f20e089', workcell_ready: true };
  await connection.command(payload);
  assert.deepEqual(fake.calls, [{ payload: { action: 'status' }, callOptions: { timeoutMs: 15000 } },
    { payload, callOptions: { timeoutMs: 15000 } }]);
  await connection.disconnect(); assert.equal(fake.disconnected, 1);
  assert.equal(fake.calls.length, 2);
});

test('adapter rejects unsupported payloads and disarms immediately on connection loss', async () => {
  const fake = fakeSdk(); let lost = 0;
  const connection = await createViamConnection({ ...options, onDisconnect: () => lost++ }, { loadSdk: async () => fake.sdk });
  for (const payload of [{ action: 'search' }, { action: 'stop' }, { action: 'run_once' },
    { action: 'status', extra: true }, { action: 'start', workcell_ready: true },
    { action: 'start', workcell_ready: false, request_id: '91cb63f3-066d-41f7-9a5a-95772f20e089' },
    { action: 'start', goal: 'ring', workcell_ready: true, request_id: '91cb63f3-066d-41f7-9a5a-95772f20e089', source_selector: {} },
    { action: 'start', workcell_ready: true, request_id: '91cb63f3-066d-41f7-9a5a-95772f20e089' },
    { action: 'start', goal: 'hoop', workcell_ready: true, request_id: '91cb63f3-066d-41f7-9a5a-95772f20e089' }]) {
    await assert.rejects(connection.command(payload));
  }
  fake.listeners.get('connectionstatechange')({ eventType: 'reconnecting' });
  assert.equal(lost, 1);
  await assert.rejects(connection.command({ action: 'status' })); assert.equal(fake.calls.length, 0);
  await connection.disconnect();
});

test('RPC error is never retried; connection errors never expose credentials', async () => {
  const fake = fakeSdk(); let calls = 0;
  fake.sdk.GenericServiceClient = class { async doCommand() { calls++; throw new Error('network failure'); } };
  const connection = await createViamConnection(options, { loadSdk: async () => fake.sdk });
  await assert.rejects(connection.command({ action: 'status' })); assert.equal(calls, 1);
  await assert.rejects(createViamConnection(options, { loadSdk: async () => ({
    async createRobotClient() { throw new Error('fixture-secret'); },
  }) }), error => !error.message.includes('fixture-secret'));
});

test('manual credentials go only to a valid public Viam machine hostname', async () => {
  assert.equal(validateHost(' Machine-Main.Example.Viam.Cloud '), options.host);
  for (const host of ['https://example.viam.cloud', 'example.viam.cloud/path', 'example.viam.cloud.evil.com',
    'evil.com', 'localhost:8080', 'example.viam.cloud@evil.com', '.viam.cloud', 'bad..viam.cloud']) {
    let loaded = false;
    await assert.rejects(createViamConnection({ ...options, host }, { loadSdk: async () => { loaded = true; } }));
    assert.equal(loaded, false);
  }
});

test('hosted mode reads only the matching machine cookie and verifies its path identity', () => {
  const session = { machineId: 'machine-id', hostname: options.host,
    credentials: { type: 'api-key', authEntity: options.keyId, payload: options.apiKey } };
  const cookie = value => `unrelated=invalid; machine-id=${encodeURIComponent(JSON.stringify(value))}; userToken=ignored`;
  assert.deepEqual(getHostedConnection('/machine/machine-id/', cookie(session)), options);
  for (const path of ['/', '/machine/another-id/']) assert.throws(() => getHostedConnection(path, cookie(session)));
  for (const changed of [{ ...session, machineId: 'another-id' }, { ...session, hostname: 'evil.com' },
    { ...session, credentials: { type: 'access-token' } }]) assert.throws(() => getHostedConnection('/machine/machine-id/', cookie(changed)));
});

test('real pinned SDK serializes the reviewed command through the service API with a fake RPC endpoint', async () => {
  const calls = [];
  const machine = { on() {}, off() {}, async disconnect() {}, createServiceClient(descriptor) {
    assert.equal(descriptor.typeName, 'viam.service.generic.v1.GenericService');
    return { async doCommand(request, callOptions) {
      calls.push({ name: request.name, command: request.command.toJson(), callOptions });
      return { result: Struct.fromJson({ accepted: true, request_id: request.command.toJson().request_id }) };
    } };
  } };
  const connection = await createViamConnection(options, { loadSdk: async () => ({
    async createRobotClient() { return machine; }, GenericServiceClient,
  }) });
  const payload = { action: 'pick_place', pick: { object: 'cube', select: 'only' }, place: { type: 'ring' }, request_id: '91cb63f3-066d-41f7-9a5a-95772f20e089', workcell_ready: true };
  assert.deepEqual(await connection.command(payload), { accepted: true, request_id: payload.request_id });
  assert.deepEqual(calls, [{ name: 'code-2', command: payload, callOptions: { timeoutMs: 15000 } }]);
});
