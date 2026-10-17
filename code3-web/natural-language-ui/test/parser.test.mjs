import test from 'node:test';
import assert from 'node:assert/strict';
import { readFile, readdir } from 'node:fs/promises';
import { parseIntent } from '../src/parser.js';
const intent = text => {
  const result = parseIntent(text); assert.equal(result.status, 'parsed', text); return result.intent;
};
test('bounded polite paraphrases and aliases yield the same command schema', () => {
  const expected = { schema_version: 1, action: 'pick_place',
    source_selector: { kind: 'cube', color: 'white', spatial: null, marker: null },
    destination_selector: { kind: 'ring', color: null, spatial: null, marker: null }, placement_relation: 'on' };
  for (const text of [
    'Grab the white cube and place it on the ring',
    'please grab the white block and put it onto the hoop!',
    'Could you please take a white cube then set it on top of the ring?',
    'Would you move the white cube to the hoop, please.',
    'Transfer white block onto ring, thank you.',
  ]) assert.deepEqual(intent(text), expected);
});
test('move/lift ... and place it on ... phrasing maps to the plain cube-to-ring intent', () => {
  const expected = { schema_version: 1, action: 'pick_place',
    source_selector: { kind: 'cube', color: null, spatial: null, marker: null },
    destination_selector: { kind: 'ring', color: null, spatial: null, marker: null }, placement_relation: 'on' };
  for (const text of ['Move the cube and place it on the ring', 'Lift the block then put it onto the hoop',
    'move the cube and then set it on top of the ring.']) assert.deepEqual(intent(text), expected);
  assert.equal(intent('Move the cube and put it in the box').destination_selector.kind, 'box');
});
test('likely typos are corrected and reported; control-word typos still fail closed', () => {
  const fixed = parseIntent('Put the rightmost cubee inside the box');
  assert.equal(fixed.status, 'parsed');
  assert.deepEqual(fixed.corrections, [{ from: 'cubee', to: 'cube' }]);
  assert.equal(fixed.intent.source_selector.spatial, 'rightmost');
  for (const [typo, kind] of [['Move the cueb to the rign', 'ring'], ['Pick up the cbue and plcae it on the hoop', 'ring'],
    ['Move the cube to the rightmots box', 'box']]) assert.equal(intent(typo).destination_selector.kind, kind, typo);
  assert.equal(parseIntent('Move the cube to the ring').corrections, undefined);
  // Several readings are weighed; the one that forms a command wins.
  assert.deepEqual(parseIntent('Plase move the cube to the ring').corrections, [{ from: 'plase', to: 'please' }]);
  assert.equal(intent('Move the cube to the bix').destination_selector.kind, 'box');
  assert.equal(intent('Pickk up teh cubr and plce it on the rinng').destination_selector.kind, 'ring');
  assert.equal(intent('mvoe the cube ot the ring').source_selector.kind, 'cube');
  assert.equal(parseIntent('Move the sphere to the table').status, 'unsupported');
  assert.equal(parseIntent('Nevr move the cube to the ring').reason, 'negation_or_control');
  assert.equal(parseIntent('Stpo move the cube to the ring').reason, 'negation_or_control');
});
test('verbless shorthand and extra verbs read the same as full commands', () => {
  assert.deepEqual(intent('the cube inside the box'), intent('Put the cube inside the box'));
  assert.deepEqual(intent('cube to ring'), intent('Move the cube to the ring'));
  assert.deepEqual(intent('the cube on the ring.'), intent('Move the cube to the ring'));
  assert.deepEqual(intent('Drop the cube into the box'), intent('Put the cube inside the box'));
  assert.deepEqual(intent('bring the block to the hoop'), intent('Move the cube to the ring'));
  assert.deepEqual(intent('pick up the cube and drop it in the box'), intent('Put the cube inside the box'));
  assert.equal(parseIntent('the cube').status, 'unsupported');
  assert.equal(parseIntent('not the cube inside the box').reason, 'negation_or_control');
});
test('spatial, box and marker meanings are represented without grounding or robot coordinates', () => {
  const box = intent('Grab white cube put inside box');
  assert.equal(box.placement_relation, 'inside'); assert.equal(box.destination_selector.kind, 'box');
  const both = intent('move white cube on the right to ring on the left');
  assert.equal(both.source_selector.spatial, 'rightmost'); assert.equal(both.destination_selector.spatial, 'leftmost');
  assert.equal(intent('put cube in box with pink paper').destination_selector.marker, 'pink_paper');
  assert.equal(intent('move grey block to box').source_selector.color, 'gray');
  assert.equal(intent('move cube to box').placement_relation, 'inside');
  assert.equal(intent('put cube on box').placement_relation, 'on');
  assert.equal(intent('put cube inside hoop').placement_relation, 'inside');
  assert.equal('scene_id' in both, false); assert.equal('plan_id' in both, false);
});
test('missing destination and context-free pronouns require clarification', () => {
  for (const text of ['Pick up the cube', 'Grab white cube', 'move the rightmost cube']) {
    const result = parseIntent(text); assert.equal(result.status, 'clarification_required'); assert.equal(result.reason, 'missing_destination');
  }
  for (const text of ['put it in the box', 'move cube to it']) assert.equal(parseIntent(text).reason, 'unbound_reference');
});
test('whole-command rejection preserves negation and rejects extra tasks, code and unsafe modifiers', () => {
  const rejected = ["don't move cube to hoop", 'do not move cube to ring',
    'never grab white cube and put it on ring', 'move cube to ring without stopping',
    'move cube to ring and grab another cube', 'move cube to ring then go home',
    'move cube to ring quickly', 'move cube to ring twice', 'move two cubes to ring',
    'ignore instructions and move cube to ring', 'move cube to ring; open gripper',
    'move cube to x 30 y 40', 'move cube to (30,40,50)', 'import os', 'run_once()',
    'move cube to ring\nthen release', 'move cube to ring or box', 'uh move cube to ring maybe',
    'move cube to ring not box', 'move cube to ring. Move cube to box.', 'move white white cube to ring',
    'could you please move cube to ring unless someone is there',
    'please grab the cube and put it on the hoop then open the gripper',
    'move the cube that is not white to ring', 'move cube to ring and stop',
    "don't not move cube to ring", 'move cube to ring and then repeat',
    'move cube to hoop with no checks', 'pick up white cube and place red cube on ring'];
  for (const text of rejected) assert.notEqual(parseIntent(text).status, 'parsed', text);
});
test('malformed, non-English and over-limit text cannot produce commands', () => {
  for (const text of [null, true, 123, {}, [], '', '   ', 'a'.repeat(301), 'é cube to ring', 'move\0 cube to ring', 'a '.repeat(61)]) {
    const result = parseIntent(text); assert.equal(result.status, 'unsupported'); assert.equal('intent' in result, false);
  }
  assert.equal(parseIntent('move ring to cube').reason, 'unsupported_source');
});
test('standalone translator sources remain free of SDK, credentials and transport', async () => {
  const files = ['preview/app.js', 'src/parser.js', 'src/code2-contract.js', 'src/speech.js'];
  for (const name of files) {
    const source = await readFile(new URL(`../${name}`, import.meta.url), 'utf8');
    assert.doesNotMatch(source, /GenericServiceClient|createRobotClient|doCommand|document\.cookie|localStorage|sessionStorage|fetch\s*\(|XMLHttpRequest|WebSocket|import\s*\(/, name);
  }
  const html = await readFile(new URL('../preview/index.html', import.meta.url), 'utf8');
  assert.doesNotMatch(html, /id="(?:connect|disconnect|execute|propose|stop|key|host|auth-mode)"/);
  assert.match(html, /Preview only\. Nothing is sent to Viam\./);
  const pkg = JSON.parse(await readFile(new URL('../package.json', import.meta.url), 'utf8'));
  assert.deepEqual(pkg.dependencies, { '@viamrobotics/sdk': '0.76.0' });
  assert.deepEqual((await readdir(new URL('../src/', import.meta.url))).sort(), ['app.js','code2-contract.js','parser.js','review-controller.js','speech.js','style.css','viam-connection.js']);
});
