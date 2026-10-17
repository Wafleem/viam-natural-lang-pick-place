import test from 'node:test';
import assert from 'node:assert/strict';
import { createHash } from 'node:crypto';
import { readFile } from 'node:fs/promises';
import { parseIntent } from '../src/parser.js';
import { CODE2_REFERENCE, previewCode2Command } from '../src/code2-contract.js';

test('unqualified cube-to-ring paraphrases match only a deliberately blocked start template', () => {
  for (const text of ['Move the cube to the ring', 'Grab cube and put it on ring',
    'Transfer the block onto the hoop', 'Please place cube on top of ring']) {
    const result = parseIntent(text);
    assert.equal(result.status, 'parsed', text);
    const mapping = previewCode2Command(result.intent);
    assert.equal(mapping.status, 'template_only', text);
    assert.deepEqual(mapping.command_template, { action: 'start', goal: 'ring', workcell_ready: false });
    assert.equal('plan_id' in mapping.command_template, false);
    assert.equal('source_selector' in mapping.command_template, false);
  }
});

test('cube into the box maps to goal box; cube on the box has no command', () => {
  assert.deepEqual(previewCode2Command(parseIntent('Put the cube into the box').intent).command_template,
    { action: 'box_once', workcell_ready: false });
  assert.equal(previewCode2Command(parseIntent('Put the cube on the box').intent).command_template, null);
});

test('qualifiers and unsupported destinations remain intact and cannot turn into generic start', () => {
  for (const text of ['Move the white cube to the ring', 'Move the rightmost cube to the hoop',
    'Move cube to red ring', 'Move cube to ring on the left',
    'Move cube with pink paper to ring', 'Move cube to ring with pink paper',
    'Grab white cube put inside box', 'Put cube in box with pink paper',
    'Put cube inside ring', 'Move cube to cube']) {
    const result = parseIntent(text);
    assert.equal(result.status, 'parsed', text);
    const original = structuredClone(result.intent);
    const mapping = previewCode2Command(result.intent);
    assert.equal(mapping.status, 'unsupported', text);
    assert.equal(mapping.command_template, null, text);
    assert.deepEqual(result.intent, original, text);
  }
});

test('unknown or incomplete intent fields cannot be silently discarded', () => {
  const intent = parseIntent('Move cube to ring').intent;
  for (const changed of [null, {}, { ...intent, schema_version: 2 }, { ...intent, action: 'start' },
    { ...intent, repeat: 2 }, { ...intent, source_selector: { ...intent.source_selector, size: 'small' } },
    { ...intent, source_selector: { kind: 'cube' } }]) {
    assert.equal(previewCode2Command(changed).command_template, null);
  }
});

test('mapping identifies the exact saved code-2 source snapshot rather than implying live readiness', async () => {
  const reference = JSON.parse(await readFile(new URL('../../code2-reference-0.0.23.json', import.meta.url), 'utf8'));
  assert.equal(reference.reference_only, true);
  assert.equal(CODE2_REFERENCE.version, reference.module_version);
  assert.equal(CODE2_REFERENCE.implementation, reference.implementation);
  assert.ok(reference.start_contract.goal_values.includes(CODE2_REFERENCE.goal));
  assert.equal(CODE2_REFERENCE.goal, 'ring');
});
