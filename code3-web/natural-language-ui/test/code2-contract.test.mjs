import test from 'node:test';
import assert from 'node:assert/strict';
import { parseIntent } from '../src/parser.js';
import { CODE2_REFERENCE, isSupportedImplementation, previewCode2Command } from '../src/code2-contract.js';

const T = (select, place) => ({ action: 'pick_place', pick: { object: 'cube', select }, place, workcell_ready: false });

test('unqualified cube-to-ring paraphrases match only a deliberately blocked pick_place template', () => {
  for (const text of ['Move the cube to the ring', 'Grab cube and put it on ring',
    'Transfer the block onto the hoop', 'Please place cube on top of ring']) {
    const result = parseIntent(text);
    assert.equal(result.status, 'parsed', text);
    const mapping = previewCode2Command(result.intent);
    assert.equal(mapping.status, 'template_only', text);
    assert.deepEqual(mapping.command_template, T('only', { type: 'ring' }));
    assert.equal('source_selector' in mapping.command_template, false);
  }
});

test('cube into a box maps to place type/colour; cube on the box has no command', () => {
  assert.deepEqual(previewCode2Command(parseIntent('Put the cube into the box').intent).command_template, T('only', { type: 'box' }));
  assert.deepEqual(previewCode2Command(parseIntent('Put the cube into the orange box').intent).command_template, T('only', { color: 'orange' }));
  assert.equal(previewCode2Command(parseIntent('Put the cube on the box').intent).command_template, null);
});

test('left/right cube and the white cube colour map onto the pick selector', () => {
  assert.deepEqual(previewCode2Command(parseIntent('Move the rightmost cube to the hoop').intent).command_template, T('right', { type: 'ring' }));
  assert.deepEqual(previewCode2Command(parseIntent('Grab the left cube and put it inside the blue box').intent).command_template, T('left', { color: 'blue' }));
  assert.deepEqual(previewCode2Command(parseIntent('Move the white cube to the ring').intent).command_template, T('only', { type: 'ring' }));
});

test('unsupported qualifiers remain intact and cannot turn into a generic command', () => {
  for (const text of ['Move the red cube to the ring', 'Move cube to red ring', 'Move cube to ring on the left',
    'Move cube with pink paper to ring', 'Move cube to ring with pink paper',
    'Put cube in box with pink paper', 'Put cube in the left box', 'Put cube inside ring', 'Move cube to cube']) {
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

test('implementation check accepts the pick_place module family only', () => {
  assert.equal(isSupportedImplementation(CODE2_REFERENCE.implementation), true);
  assert.equal(isSupportedImplementation('pickplace-v1-open-loop-suction-press+dynamic-scene'), true);
  for (const old of ['vision-cube-ring-box-v21-surveyed-one-cycle', 'vision-cube-ring-box-v17-wrist-camera-survey', '', null]) {
    assert.equal(isSupportedImplementation(old), false);
  }
});
