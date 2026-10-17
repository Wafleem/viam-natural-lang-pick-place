import test from 'node:test';
import assert from 'node:assert/strict';
import { setupSpeech } from '../src/speech.js';

function element() {
  return { handlers: {}, textContent: '', disabled: false, value: '',
    classList: { add() {}, remove() {} }, setPointerCapture() {},
    addEventListener(name, callback) { this.handlers[name] = callback; } };
}
test('speech ignores interim results, deduplicates finals and changes only text after recognition ends', () => {
  let recognizer, changes = 0, stops = 0;
  class Recognition {
    constructor() { recognizer = this; }
    start() {}
    stop() { stops++; }
  }
  globalThis.window = { SpeechRecognition: Recognition, addEventListener() {} };
  globalThis.document = { addEventListener() {} };
  const button = element(), input = element(), message = element();
  input.value = 'previous instruction';
  setupSpeech(button, input, message, () => { changes++; });
  button.handlers.pointerdown({ button: 0, pointerId: 1 });
  const interim = Object.assign([{ transcript: 'wrong' }], { isFinal: false });
  recognizer.onresult({ resultIndex: 0, results: [interim] });
  assert.equal(input.value, 'previous instruction'); assert.equal(changes, 0);
  const final = Object.assign([{ transcript: 'Put the white cube on the ring.' }], { isFinal: true });
  recognizer.onresult({ resultIndex: 0, results: [final] });
  recognizer.onresult({ resultIndex: 0, results: [final] });
  button.handlers.pointerup(); button.handlers.lostpointercapture();
  assert.equal(stops, 1); assert.equal(changes, 0);
  recognizer.onend();
  assert.equal(input.value, 'Put the white cube on the ring.'); assert.equal(changes, 1);
  button.handlers.pointerdown({ button: 0, pointerId: 2 });
  const longText = 'one '.repeat(80) + 'do not move';
  const oversized = Object.assign([{ transcript: longText }], { isFinal: true });
  recognizer.onresult({ resultIndex: 0, results: [oversized] }); recognizer.onend();
  assert.equal(input.value, longText); assert.match(message.textContent, /exceeds 300/);
  delete globalThis.window; delete globalThis.document;
});
