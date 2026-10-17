// Transport-independent policy. No SDK, DOM, timers, persistence, or automatic RPCs.
import { parseIntent } from './parser.js';
import { describeTarget, isSupportedImplementation, placeTarget, previewCode2Command } from './code2-contract.js';

export const STATUS_MAX_AGE_MS = 5000;
const READY_STATES = new Set(['idle', 'complete', 'failed', 'stopped']);
const ACTIVE_STATES = new Set(['running']);
const TERMINAL_STATES = new Set(['complete', 'failed', 'stopped']);
const copy = value => value == null ? value : JSON.parse(JSON.stringify(value));
const freeze = value => {
  if (value && typeof value === 'object') {
    for (const child of Object.values(value)) freeze(child);
    Object.freeze(value);
  }
  return value;
};
// Is the template's place target configured on the robot right now (status.goal_details)?
const goalAvailable = (template, status) => {
  const target = placeTarget(template), goals = status.goal_details;
  if (target.type === 'ring') return status.available_goals.includes('ring');
  return goals.some(g => g.type === 'box' && (!target.color || g.color === target.color));
};
const intentKey = intent => JSON.stringify([intent.schema_version, intent.action,
  ...['source_selector', 'destination_selector'].map(key => {
    const s = intent[key]; return [s.kind, s.color, s.spatial, s.marker];
  }), intent.placement_relation]);

export class ReviewController {
  constructor({ changed = () => {}, now = () => Date.now(), uuid = () => globalThis.crypto.randomUUID(),
    pendingRequest = null, savePending = () => {} } = {}) {
    this._changed = changed; this._now = now; this._uuid = uuid;
    this._transport = null; this._generation = 0; this._activity = 0; this._revision = 0;
    this._text = ''; this._review = null; this._acknowledged = false;
    this._status = null; this._statusAt = null; this._refreshToken = null;
    this._request = null; this._usedIds = new Set(); this._lastCommand = null; this._savePending = savePending;
    this._message = 'Connect and read status before reviewing a send.';
    if (pendingRequest !== null) {
      const id = pendingRequest && pendingRequest.request_id;
      this._request = { request_id: typeof id === 'string' && id.length > 0 && id.length <= 80 ? id : null,
        phase: 'unknown', resolved: false };
      if (this._request.request_id) this._usedIds.add(this._request.request_id);
      this._message = 'A previous send is unresolved. Read matching code-2 status before sending again.';
    }
  }

  get canSend() { return this._blockedReason(true) === null; }
  get state() {
    return freeze({ connected: this._transport !== null, refreshing: this._refreshToken !== null,
      status: copy(this._status), statusAt: this._statusAt, review: copy(this._review),
      acknowledged: this._acknowledged, canAcknowledge: this._blockedReason(false) === null,
      canSend: this.canSend, blockedReason: this._blockedReason(true), message: this._message,
      lastCommand: copy(this._lastCommand), request: copy(this._request) });
  }
  _emit() {
    // A rendering exception must not unwind command consumption or enable a retry.
    try { this._changed(this.state); } catch { /* The policy remains authoritative. */ }
  }
  _clearReview() { this._review = null; this._acknowledged = false; this._revision++; }
  _journal(record) {
    try {
      const result = this._savePending(record);
      if (result === false) return false;
      if (result && typeof result.then === 'function') {
        Promise.resolve(result).catch(() => {}); return false;
      }
      return true;
    } catch { return false; }
  }
  _resolveRequest(phase) {
    this._request.phase = phase;
    this._request.resolved = this._journal(null);
    if (!this._request.resolved) this._message = 'The request outcome is known, but its journal could not be cleared. Sending remains blocked.';
    return this._request.resolved;
  }
  _changeConnection(transport) {
    this._generation++; this._activity++; this._transport = transport;
    this._status = null; this._statusAt = null; this._refreshToken = null; this._clearReview();
    if (this._request && !this._request.resolved) this._request.phase = 'unknown';
  }
  attach(transport) {
    if (typeof transport !== 'function') throw new TypeError('A command transport function is required');
    this._changeConnection(transport);
    this._message = this._request && !this._request.resolved
      ? 'Previous send outcome is unresolved. Read status; do not resend.'
      : 'Connected. Read code-2 status, then review the instruction.';
    this._emit();
  }
  detach() {
    this._changeConnection(null);
    this._message = this._request && !this._request.resolved
      ? 'Disconnected with an unresolved send. Its outcome remains unknown.' : 'Disconnected.';
    this._emit();
  }
  lostConnection() { this.detach(); }

  setInstruction(text) {
    this._text = typeof text === 'string' ? text : '';
    this._clearReview(); this._message = 'Translate and review the current instruction.'; this._emit();
  }
  review(intent, text) {
    this._text = typeof text === 'string' ? text : ''; this._clearReview();
    const parsed = parseIntent(text), mapping = previewCode2Command(intent);
    const compatible = mapping.status === 'template_only' && parsed.status === 'parsed'
      && intentKey(parsed.intent) === intentKey(intent);
    if (!compatible) {
      this._message = mapping.status !== 'template_only' ? mapping.message : 'The intent does not match the current instruction.';
      this._emit(); return false;
    }
    this._review = freeze({ intent: copy(parsed.intent), text: this._text, revision: this._revision,
      generation: this._generation, consumed: false, command_template: copy(mapping.command_template) });
    this._message = `Review this ${describeTarget(mapping.command_template)} request and confirm the workcell is ready.`;
    this._emit(); return true;
  }
  acknowledge(value) {
    this._acknowledged = value === true && this._blockedReason(false) === null;
    this._emit(); return this._acknowledged;
  }

  _validStatus(status) {
    return status && typeof status === 'object' && !Array.isArray(status)
      && isSupportedImplementation(status.implementation)
      && typeof status.calibrated === 'boolean' && typeof status.busy === 'boolean'
      && (READY_STATES.has(status.state) || ACTIVE_STATES.has(status.state))
      && typeof status.request_id === 'string' && status.request_id.length <= 80
      && Array.isArray(status.available_goals) && Array.isArray(status.goal_details)
      && status.goal_details.every(g => g && typeof g.name === 'string' && ['ring', 'box'].includes(g.type));
  }
  _blockedReason(requireAcknowledgment) {
    if (!this._transport) return 'disconnected';
    if (this._request && !this._request.resolved) return 'unresolved_send';
    if (!this._review || this._review.generation !== this._generation || this._review.revision !== this._revision
      || this._review.text !== this._text) return 'review_required';
    if (this._review.consumed) return 'review_consumed';
    if (!this._validStatus(this._status)) return 'status_unverified';
    const age = this._now() - this._statusAt;
    if (!Number.isFinite(age) || age < 0 || age > STATUS_MAX_AGE_MS) return 'status_stale';
    if (!this._status.calibrated) return 'calibration_required';
    if (this._status.busy || !READY_STATES.has(this._status.state)) return 'service_busy';
    if (!goalAvailable(this._review.command_template, this._status)) return 'goal_unavailable';
    if (requireAcknowledgment && !this._acknowledged) return 'acknowledgment_required';
    return null;
  }

  async refresh() {
    if (!this._transport || this._refreshToken) return false;
    const transport = this._transport, generation = this._generation, activity = this._activity;
    const token = {}; this._refreshToken = token;
    const startedAt = this._now(); this._emit();
    // changed() may synchronously detach the session.
    if (generation !== this._generation || transport !== this._transport) return false;
    try {
      const reply = await transport({ action: 'status' });
      if (generation !== this._generation || activity !== this._activity || transport !== this._transport) return false;
      if (!this._validStatus(reply)) {
        this._status = null; this._statusAt = null; this._acknowledged = false;
        this._message = 'Status does not match the reviewed code-2 implementation and schema.'; return false;
      }
      const previous = this._status;
      const status = { implementation: reply.implementation, calibrated: reply.calibrated,
        busy: reply.busy, state: reply.state, request_id: reply.request_id,
        available_goals: [...reply.available_goals],
        goal_details: reply.goal_details.map(g => ({ name: g.name, type: g.type, color: typeof g.color === 'string' ? g.color : null })),
        stage: typeof reply.stage === 'string' ? reply.stage : '',
        error: typeof reply.error === 'string' ? reply.error : '',
        calibration_error: typeof reply.calibration_error === 'string' ? reply.calibration_error : '' };
      this._status = status; this._statusAt = startedAt;
      if (previous && ['implementation', 'calibrated', 'busy', 'state', 'request_id'].some(key => previous[key] !== status[key])) {
        this._acknowledged = false;
      }
      if (!status.calibrated || status.busy || !READY_STATES.has(status.state)) this._acknowledged = false;
      if (this._request && !this._request.resolved && status.request_id === this._request.request_id) {
        if (!status.busy && TERMINAL_STATES.has(status.state)) {
          if (this._resolveRequest(status.state)) this._message = `Code-2 reports ${status.state} for this request. Review a new instruction before another send.`;
        } else {
          this._request.phase = status.busy || ACTIVE_STATES.has(status.state) ? 'running' : 'accepted';
          this._message = 'Code-2 identifies this request. Its final outcome has not been confirmed.';
        }
      } else if (this._request && !this._request.resolved) {
        this._message = 'Status does not identify the unresolved request. Sending remains blocked.';
      } else this._message = 'Code-2 status read. Calibration status does not establish physical workcell readiness.';
      return true;
    } catch {
      if (generation === this._generation && activity === this._activity) {
        this._status = null; this._statusAt = null; this._acknowledged = false;
        this._message = 'Status could not be read. Sending is blocked.';
      }
      return false;
    } finally {
      if (this._refreshToken === token) { this._refreshToken = null; this._emit(); }
    }
  }

  async send() {
    let reason = this._blockedReason(true);
    if (reason) return { sent: false, reason };
    const generation = this._generation, revision = this._revision, transport = this._transport;
    let requestId;
    try { requestId = this._uuid(); } catch { return { sent: false, reason: 'request_id_unavailable' }; }
    if (typeof requestId !== 'string' || requestId.length < 1 || requestId.length > 80 || this._usedIds.has(requestId)) {
      return { sent: false, reason: 'invalid_request_id' };
    }
    reason = this._blockedReason(true);
    if (reason || generation !== this._generation || revision !== this._revision || transport !== this._transport) {
      return { sent: false, reason: reason || 'review_changed' };
    }
    const command = Object.freeze({ ...this._review.command_template, request_id: requestId, workcell_ready: true });
    if (!this._journal({ request_id: requestId, phase: 'unknown' })) {
      this._acknowledged = false; this._message = 'The pending-request journal could not be saved. Nothing was sent.';
      this._emit(); return { sent: false, reason: 'journal_unavailable' };
    }
    reason = this._blockedReason(true);
    if (reason || generation !== this._generation || revision !== this._revision || transport !== this._transport) {
      if (!this._journal(null)) this._request = { request_id: requestId, phase: 'not_sent', resolved: false };
      this._emit(); return { sent: false, reason: reason || 'review_changed' };
    }
    this._usedIds.add(requestId); this._activity++;
    this._refreshToken = null;
    this._review = freeze({ ...this._review, consumed: true }); this._acknowledged = false;
    this._request = { request_id: requestId, phase: 'sending', resolved: false };
    this._lastCommand = freeze({ ...command }); this._status = null; this._statusAt = null;
    this._message = 'Sending one reviewed pick_place request. Do not repeat it.'; this._emit();
    if (generation !== this._generation || revision !== this._revision || transport !== this._transport) {
      if (this._resolveRequest('not_sent')) this._message = 'The review or connection changed before dispatch. Nothing was sent.';
      this._emit();
      return { sent: false, reason: 'review_changed' };
    }
    try {
      const reply = await transport(command);
      if (generation !== this._generation || transport !== this._transport) return { sent: true, reason: 'connection_changed' };
      if (this._request.request_id !== requestId || this._request.resolved || TERMINAL_STATES.has(this._request.phase)) return { sent: true };
      if (reply && reply.accepted === true && reply.request_id === requestId) {
        this._request.phase = 'accepted';
        this._message = 'Code-2 accepted this request. Read status to confirm its outcome; acceptance is not completion.';
      } else if (this._validStatus(reply) && reply.busy === true && reply.accepted === false && reply.already_running === true) {
        if (this._resolveRequest('rejected_busy')) this._message = 'Code-2 rejected this request because another task was already running. Nothing is retried.';
      } else {
        this._request.phase = 'unknown';
        this._message = 'Send outcome is uncertain. Read status; this request will not be retried.';
      }
      this._emit(); return { sent: true };
    } catch {
      if (generation === this._generation && transport === this._transport && this._request.request_id === requestId
        && !this._request.resolved && !TERMINAL_STATES.has(this._request.phase)) {
        this._request.phase = 'unknown'; this._message = 'Send outcome is uncertain. Read status; do not resend.'; this._emit();
      }
      return { sent: true, reason: 'outcome_unknown' };
    }
  }
}
