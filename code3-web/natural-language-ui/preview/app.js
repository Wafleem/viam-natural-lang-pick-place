import { parseIntent, MAX_TEXT_CHARS, MAX_TEXT_WORDS } from './parser.js';
import { setupSpeech } from './speech.js';
import { CODE2_REFERENCE, previewCode2Command } from './code2-contract.js';
const el = id => document.getElementById(id);
let copyText = '', templateText = '';
function resetTemplate() {
  templateText = ''; el('copy-template').disabled = true; el('template-copy-status').textContent = '';
  el('template-state').textContent = 'AWAITING TRANSLATION';
  el('template-message').textContent = 'Translate an instruction to check this source reference.';
  el('command-template').textContent = 'No command template for the current text.';
}
function edited() {
  const text = el('instruction').value, chars = [...text].length;
  const words = text.trim() ? text.trim().split(/\s+/).length : 0;
  el('instruction-limit').textContent = `${chars} / ${MAX_TEXT_CHARS} characters · ${words} / ${MAX_TEXT_WORDS} words`;
  el('translate').disabled = !text.trim();
  copyText = ''; el('copy').disabled = true; el('copy-status').textContent = '';
  el('result-state').textContent = 'AWAITING TRANSLATION';
  el('summary').textContent = 'Translate to refresh the meaning.';
  el('result-message').textContent = 'Preview only. Nothing is sent to Viam.';
  el('command').textContent = 'No translation for the current text.';
  resetTemplate();
}
function describe(selector) {
  return [selector.spatial, selector.color, selector.kind].filter(Boolean).join(' ')
    + (selector.marker === 'pink_paper' ? ' with pink paper' : '');
}
const correctionNote = result => result.corrections
  ? `Read ${result.corrections.map(c => `“${c.from}” as “${c.to}”`).join(', ')}. ` : '';
function translate() {
  const result = parseIntent(el('instruction').value);
  el('result-state').textContent = result.status.replaceAll('_', ' ').toUpperCase();
  el('copy-status').textContent = ''; resetTemplate();
  if (result.status === 'parsed') {
    const intent = result.intent;
    copyText = JSON.stringify(intent, null, 2);
    el('summary').textContent = `${describe(intent.source_selector)} → ${intent.placement_relation} ${describe(intent.destination_selector)}`;
    el('result-message').textContent = correctionNote(result) + 'Semantic meaning only. Qualifiers are preserved; no objects have been observed.';
    el('command').textContent = copyText; el('copy').disabled = false;
    const compatibility = previewCode2Command(intent);
    el('template-state').textContent = compatibility.status === 'template_only' ? 'BLOCKED TEMPLATE · PREVIEW ONLY' : 'UNSUPPORTED BY THIS CODE-2 REFERENCE';
    el('template-message').textContent = compatibility.message;
    if (compatibility.status === 'template_only' && compatibility.command_template) {
      templateText = JSON.stringify(compatibility.command_template, null, 2);
      el('command-template').textContent = templateText; el('copy-template').disabled = false;
    } else el('command-template').textContent = 'No compatible command template. The requested qualifiers have not been discarded.';
  } else {
    copyText = ''; el('copy').disabled = true;
    el('summary').textContent = result.status === 'clarification_required' ? 'A detail is missing.' : 'This instruction is unsupported.';
    el('result-message').textContent = correctionNote(result) + result.message;
    el('command').textContent = JSON.stringify(result, null, 2);
    el('template-state').textContent = 'NO COMMAND TEMPLATE';
    el('template-message').textContent = 'Resolve the instruction before checking code-2 compatibility.';
  }
}
async function copyPreview(getText, statusId) {
  const captured = getText();
  if (!captured) return;
  try {
    await navigator.clipboard.writeText(captured);
    if (getText() === captured) el(statusId).textContent = 'JSON copied. Nothing was sent to Viam.';
  } catch {
    if (getText() === captured) el(statusId).textContent = 'Clipboard access is unavailable. Select the JSON and copy it manually.';
  }
}
el('reference-version').textContent = `Source reference ${CODE2_REFERENCE.version}`;
el('instruction').addEventListener('input', edited);
el('translate').addEventListener('click', translate);
el('copy').addEventListener('click', () => copyPreview(() => copyText, 'copy-status'));
el('copy-template').addEventListener('click', () => copyPreview(() => templateText, 'template-copy-status'));
for (const button of document.querySelectorAll('.example')) button.addEventListener('click', () => {
  el('instruction').value = button.textContent; edited(); translate();
});
setupSpeech(el('talk'), el('instruction'), el('speech-status'), edited);
