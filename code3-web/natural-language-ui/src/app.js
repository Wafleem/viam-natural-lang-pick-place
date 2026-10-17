import { parseIntent, MAX_TEXT_CHARS, MAX_TEXT_WORDS } from './parser.js';
import { setupSpeech } from './speech.js';
import { CODE2_REFERENCE, previewCode2Command } from './code2-contract.js';
import { ReviewController } from './review-controller.js';
import { createViamConnection, getHostedConnection } from './viam-connection.js';

const el = id => document.getElementById(id);
let copyText = '', templateText = '', connection = null, connecting = false, connectionGeneration = 0;
let connectionNotice = 'Connect only when you want to inspect code-2 and send a reviewed task.';
let policy;
const pendingKey = 'armfarm.reviewed-code2.pending-request.v1';
let pendingRequest = null;
try {
  const saved = sessionStorage.getItem(pendingKey);
  if (saved !== null) pendingRequest = JSON.parse(saved) ?? { invalid: true };
} catch { pendingRequest = { invalid: true }; }
const controller = new ReviewController({
  pendingRequest,
  savePending(record) {
    if (record === null) sessionStorage.removeItem(pendingKey);
    else sessionStorage.setItem(pendingKey, JSON.stringify({ request_id: record.request_id, phase: 'unknown' }));
  },
  changed(snapshot) { policy = snapshot; renderConnection(); },
});
policy = controller.state;
const readable = value => typeof value === 'string' ? value.replaceAll('_', ' ') : '';
const reasons = {
  disconnected: 'Connect and read code-2 status before sending.',
  unresolved_send: 'A previous send is unresolved. Inspect its request status before sending another task.',
  review_required: 'Translate the current instruction, then review the supported task.',
  review_consumed: 'This review has already been used. Translate and review again for another task.',
  status_unverified: 'Read a valid status from the referenced code-2 implementation.',
  status_stale: 'Status is out of date. Refresh it before sending.',
  calibration_required: 'Code-2 reports that calibration is required.',
  service_busy: 'Code-2 is busy. Wait for its current task to finish.',
  acknowledgment_required: 'Confirm your task review and that the workcell is clear.',
  goal_unavailable: 'Code-2 has no configured destination matching this request (check the box colour).',
};
const phases = { sending: 'Sending', accepted: 'Accepted', running: 'Running', complete: 'Completed',
  failed: 'Failed', stopped: 'Stopped', unknown: 'Unknown', rejected_busy: 'Rejected · busy', not_sent: 'Not sent' };
function renderConnection() {
  if (!policy) return;
  el('connection-pill').textContent = connecting ? 'Connecting' : policy.connected ? 'Connected to code-2' : 'Not connected';
  el('connection-pill').classList.toggle('connected', !!policy.connected);
  el('connect').disabled = connecting || !!policy.connected;
  el('disconnect').disabled = !connecting && !connection;
  for (const id of ['auth-mode','host','key-id','key']) el(id).disabled = connecting || !!policy.connected;
  el('connection-notice').textContent = connectionNotice;
  el('refresh-status').disabled = !policy.connected || policy.refreshing;
  el('reviewed').disabled = !policy.canAcknowledge;
  el('reviewed').checked = !!policy.acknowledged;
  el('send-reviewed').disabled = !policy.canSend;
  el('send-reason').textContent = reasons[policy.blockedReason] || readable(policy.blockedReason) || (policy.canSend ? 'Ready to send this reviewed task once.' : 'Review a supported task before sending.');
  el('machine-state').textContent = readable(policy.status?.state) || 'No current status';
  el('machine-stage').textContent = readable(policy.status?.stage) || '—';
  el('request-state').textContent = phases[policy.request?.phase] || readable(policy.request?.phase) || 'Nothing sent';
  el('request-id').textContent = policy.request?.request_id || '—';
  el('observed-request-id').textContent = policy.status?.request_id || '—';
  el('status-message').textContent = policy.message || 'No command is sent by translation, connection or status refresh.';
  el('status-error').textContent = policy.status?.error || policy.status?.calibration_error || '';
  el('status-error').hidden = !el('status-error').textContent;
  el('last-command').textContent = policy.lastCommand ? JSON.stringify(policy.lastCommand, null, 2) : 'No command sent from this page.';
}
function resetTemplate() {
  templateText = ''; el('copy-template').disabled = true; el('template-copy-status').textContent = '';
  el('template-state').textContent = 'AWAITING TRANSLATION';
  el('template-message').textContent = 'Translate an instruction to check this source reference.';
  el('command-template').textContent = 'No command template for the current text.';
}
function edited() {
  const text = el('instruction').value, chars = [...text].length;
  const words = text.trim() ? text.trim().split(/\s+/).length : 0;
  controller.setInstruction(text);
  el('instruction-limit').textContent = `${chars} / ${MAX_TEXT_CHARS} characters · ${words} / ${MAX_TEXT_WORDS} words`;
  el('translate').disabled = !text.trim();
  copyText = ''; el('copy').disabled = true; el('copy-status').textContent = '';
  el('result-state').textContent = 'AWAITING TRANSLATION';
  el('summary').textContent = 'Translate to refresh the meaning.';
  el('result-message').textContent = 'Translation stays local. Sending requires a separate review.';
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
  const text = el('instruction').value;
  const result = parseIntent(text);
  controller.setInstruction(text);
  el('result-state').textContent = result.status.replaceAll('_', ' ').toUpperCase();
  el('copy-status').textContent = ''; resetTemplate();
  if (result.status === 'parsed') {
    const intent = result.intent;
    copyText = JSON.stringify(intent, null, 2);
    el('summary').textContent = `${describe(intent.source_selector)} → ${intent.placement_relation} ${describe(intent.destination_selector)}`;
    el('result-message').textContent = correctionNote(result) + 'Semantic meaning only. Qualifiers are preserved; no objects have been observed by this page.';
    el('command').textContent = copyText; el('copy').disabled = false;
    const compatibility = previewCode2Command(intent);
    el('template-state').textContent = compatibility.status === 'template_only' ? 'SUPPORTED TASK · REVIEW REQUIRED' : 'UNSUPPORTED BY THIS CODE-2 REFERENCE';
    el('template-message').textContent = compatibility.message;
    if (compatibility.status === 'template_only' && compatibility.command_template) {
      templateText = JSON.stringify(compatibility.command_template, null, 2);
      el('command-template').textContent = templateText; el('copy-template').disabled = false;
      controller.review(intent, text);
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
    if (getText() === captured) el(statusId).textContent = 'JSON copied. Copying does not send a command.';
  } catch {
    if (getText() === captured) el(statusId).textContent = 'Clipboard access is unavailable. Select the JSON and copy it manually.';
  }
}
function authModeChanged() {
  el('manual-fields').hidden = el('auth-mode').value === 'hosted';
  el('key').value = ''; el('key-id').value = '';
}
el('auth-mode').addEventListener('change', authModeChanged);
el('connect-form').addEventListener('submit', async event => {
  event.preventDefault();
  if (connecting || policy.connected) return;
  // Clear secret fields immediately, even when credentials fail validation.
  let credentials = { host: el('host').value.trim(), keyId: el('key-id').value.trim(), apiKey: el('key').value.trim() };
  el('key-id').value = ''; el('key').value = '';
  const mode = el('auth-mode').value;
  const generation = ++connectionGeneration;
  connecting = true; connectionNotice = 'Connecting, then reading code-2 status…'; renderConnection();
  try {
    const previous = connection; connection = null; controller.detach();
    if (previous) { try { await previous.disconnect(); } catch { /* no raw auth/transport logging */ } }
    if (mode === 'hosted') credentials = getHostedConnection(location.pathname, document.cookie);
    const next = await createViamConnection({ ...credentials, onDisconnect() {
      if (generation !== connectionGeneration) return;
      controller.lostConnection(); connectionNotice = 'Connection lost. Reconnect manually and inspect the tracked request; no command is retried.'; renderConnection();
    } });
    if (generation !== connectionGeneration) { await next.disconnect(); return; }
    connection = next;
    controller.attach(payload => next.command(payload));
    await controller.refresh();
    connectionNotice = policy.connected ? 'Connected. Status was read; no movement command was sent.' : 'Status unavailable. Reconnect and inspect before sending.';
  } catch {
    if (generation === connectionGeneration) {
      controller.lostConnection(); connectionNotice = mode === 'hosted'
        ? 'Could not connect. Open through a valid Viam single-machine session and try again.'
        : 'Could not connect. Check the machine hostname, API key ID, key and network, then try again.';
    }
  } finally {
    credentials = undefined;
    if (generation === connectionGeneration) { connecting = false; renderConnection(); }
  }
});
el('disconnect').addEventListener('click', async () => {
  connectionGeneration++; connecting = false;
  const previous = connection; connection = null; controller.detach();
  connectionNotice = 'Disconnected. This does not confirm that a running robot task stopped.'; renderConnection();
  if (previous) { try { await previous.disconnect(); } catch { /* do not log transport details */ } }
});
el('refresh-status').addEventListener('click', () => void controller.refresh());
el('reviewed').addEventListener('change', () => controller.acknowledge(el('reviewed').checked));
el('send-reviewed').addEventListener('click', () => void controller.send());
el('reference-version').textContent = `Source reference ${CODE2_REFERENCE.version}`;
el('instruction').addEventListener('input', edited);
// Tab accepts the placeholder suggestion when the box is empty or holds its start; otherwise Tab moves focus.
el('instruction').addEventListener('keydown', event => {
  if (event.key !== 'Tab' || event.shiftKey || event.altKey || event.ctrlKey || event.metaKey) return;
  const box = el('instruction'), suggestion = box.placeholder || '';
  if (!suggestion || box.value === suggestion || !suggestion.toLowerCase().startsWith(box.value.toLowerCase())) return;
  event.preventDefault(); box.value = suggestion; edited();
});
el('translate').addEventListener('click', translate);
el('copy').addEventListener('click', () => copyPreview(() => copyText, 'copy-status'));
el('copy-template').addEventListener('click', () => copyPreview(() => templateText, 'template-copy-status'));
for (const button of document.querySelectorAll('.example')) button.addEventListener('click', () => {
  el('instruction').value = button.textContent; edited(); translate();
});
setupSpeech(el('talk'), el('instruction'), el('speech-status'), edited);
setInterval(() => { policy = controller.state; renderConnection(); if (policy.connected) void controller.refresh(); }, 1000);
authModeChanged(); renderConnection();
