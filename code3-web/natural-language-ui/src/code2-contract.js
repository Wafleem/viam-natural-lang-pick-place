// Static source reference only. This module has no Viam connection or transport.
export const CODE2_REFERENCE = Object.freeze({
  resource: 'code-2',
  version: '0.0.23',
  implementation: 'vision-cube-ring-box-v21-surveyed-one-cycle',
  // v17 falls back to the configured default_goal (live: "box") when goal is omitted,
  // so every start names the ring explicitly.
  goal: 'ring',
  // Destination kind and relation -> code-2 goal. Code-2 has no fields for colors, sides or markers.
  goals: Object.freeze({ ring: 'on', box: 'inside' }),
  checkedAt: '2026-10-16T22:46Z',
});

const unsupported = (reason, message) => ({
  status: 'unsupported', reason, message, command_template: null,
});
const exactKeys = (value, keys) => value && typeof value === 'object'
  && !Array.isArray(value) && Object.keys(value).length === keys.length
  && keys.every(key => Object.hasOwn(value, key));

export function previewCode2Command(intent) {
  // Fail closed if a future parser adds meaning that this mapping cannot honor.
  if (!exactKeys(intent, ['schema_version', 'action', 'source_selector', 'destination_selector', 'placement_relation'])
    || intent.schema_version !== 1 || intent.action !== 'pick_place'
    || ![intent.source_selector, intent.destination_selector].every(selector =>
      exactKeys(selector, ['kind', 'color', 'spatial', 'marker']))) {
    return unsupported('invalid_intent', 'This meaning does not match the reviewed code-2 mapping.');
  }
  const goal = intent.destination_selector.kind;
  if (intent.source_selector.kind !== 'cube' || !Object.hasOwn(CODE2_REFERENCE.goals, goal)) {
    return unsupported('unsupported_objects', 'Code-2 supports moving a cube onto the ring or into the box. This object combination has no code-2 command.');
  }
  if (intent.placement_relation !== CODE2_REFERENCE.goals[goal]) {
    return unsupported('unsupported_relation', goal === 'ring'
      ? 'Code-2 places the cube on the ring. Placement inside the ring has no matching command.'
      : 'Code-2 places the cube inside the box. Placement on top of the box has no matching command.');
  }
  if ([intent.source_selector, intent.destination_selector].some(selector =>
    ['color', 'spatial', 'marker'].some(key => selector[key] !== null))) {
    return unsupported('unsupported_selection', 'Code-2 has no color, left/right or marker selection fields. Your qualifiers are preserved in the meaning; no command is generated.');
  }
  return {
    status: 'template_only',
    reason: 'readiness_not_confirmed',
    message: 'Reference template only. Readiness is deliberately false, so code-2 would reject this command. Nothing is sent.'
      + (goal === 'box' ? ' Box tasks use code-2\'s box_once workflow and can be sent only while code-2 reports it enabled.' : ''),
    // start generates its own request_id when omitted; goal must be explicit (see CODE2_REFERENCE.goal).
    // Code-2 v21 runs box tasks through its dedicated box_once workflow (always the box).
    command_template: goal === 'box' ? { action: 'box_once', workcell_ready: false } : { action: 'start', goal, workcell_ready: false },
  };
}
