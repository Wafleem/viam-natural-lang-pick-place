import test from 'node:test';
import assert from 'node:assert/strict';
import { readFile } from 'node:fs/promises';
import { Script } from 'node:vm';

test('standalone meaning and blocked code-2 previews copy separately and clear safely without a server', async () => {
  const html = await readFile(new URL('../dist/translator.html', import.meta.url), 'utf8');
  assert.doesNotMatch(html, /<(?:script|img|link|iframe)\b[^>]*(?:src|href)=/i);
  assert.doesNotMatch(html, /(?:https?:)?\/\/[^\s]+/);
  const scripts = [...html.matchAll(/<script>([\s\S]*?)<\/script>/g)];
  assert.equal(scripts.length, 1);
  assert.doesNotMatch(scripts[0][1], /^\s*(?:import|export)\b/m);
  const elements = new Map();
  function el(id) {
    if (!elements.has(id)) elements.set(id, { value: '', textContent: '', disabled: false, handlers: {},
      addEventListener(event, handler) { this.handlers[event] = handler; } });
    return elements.get(id);
  }
  const document = { getElementById: el, querySelectorAll: () => [] };
  let clipboard = '';
  const sandbox = { document, window: {}, navigator: { clipboard: { async writeText(text) { clipboard = text; } } } };
  new Script(scripts[0][1], { filename: 'translator.html' }).runInNewContext(sandbox);
  assert.equal(el('talk').disabled, true);
  el('instruction').value = 'Move the cube to the ring';
  el('instruction').handlers.input(); el('translate').handlers.click();
  assert.equal(JSON.parse(el('command').textContent).action, 'pick_place');
  assert.equal(el('copy').disabled, false);
  assert.deepEqual(JSON.parse(el('command-template').textContent), { action: 'start', goal: 'ring', workcell_ready: false });
  assert.equal(el('copy-template').disabled, false);
  assert.match(el('template-state').textContent, /BLOCKED/);
  await el('copy').handlers.click(); assert.equal(clipboard, el('command').textContent);
  await el('copy-template').handlers.click(); assert.equal(clipboard, el('command-template').textContent);
  el('instruction').value = 'Move the white cube to the ring';
  el('instruction').handlers.input();
  assert.equal(el('copy').disabled, true); assert.equal(el('copy-template').disabled, true);
  assert.equal(el('copy-status').textContent, ''); assert.equal(el('template-copy-status').textContent, '');
  el('translate').handlers.click();
  assert.equal(JSON.parse(el('command').textContent).source_selector.color, 'white');
  assert.equal(el('copy').disabled, false); assert.equal(el('copy-template').disabled, true);
  assert.match(el('template-state').textContent, /UNSUPPORTED/);
  el('instruction').value = 'Move the cube to the ring and then repeat';
  el('instruction').handlers.input(); assert.equal(el('copy').disabled, true);
  el('translate').handlers.click();
  assert.equal(JSON.parse(el('command').textContent).status, 'unsupported');
  assert.equal(el('copy').disabled, true); assert.equal(el('copy-template').disabled, true);
  assert.equal(el('reference-version').textContent, 'Source reference 0.0.23');
});
