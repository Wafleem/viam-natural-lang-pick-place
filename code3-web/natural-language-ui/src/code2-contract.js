// Static contract for code-2's pick_place API (pickplace-module, >= 0.0.30). No Viam connection
// or transport here. See ../../../nl-integration/command_schema.json and {"action":"describe"}.
export const CODE2_REFERENCE = Object.freeze({
  resource: 'code-2',
  module: 'hackathons:armfarm19-vision-pickplace',
  version: '0.0.30',
  implementation: 'pickplace-v1-open-loop-suction-press',
  // Any implementation id with this prefix speaks the pick_place API checked here.
  implementationPrefix: 'pickplace-v1',
  apiVersion: 1,
  // Destination kind -> the placement relation code-2 performs.
  goals: Object.freeze({ ring: 'on', box: 'inside' }),
  // The only pickable object is the white cube; the ring is black.
  cubeColor: 'white',
  ringColor: 'black',
  checkedAt: '2026-10-17T05:30Z',
});

export const isSupportedImplementation = id => typeof id === 'string'
  && id.startsWith(CODE2_REFERENCE.implementationPrefix);

const unsupported = (reason, message) => ({
  status: 'unsupported', reason, message, command_template: null,
});
const exactKeys = (value, keys) => value && typeof value === 'object'
  && !Array.isArray(value) && Object.keys(value).length === keys.length
  && keys.every(key => Object.hasOwn(value, key));
const SELECT = Object.freeze({ leftmost: 'left', rightmost: 'right' });

// Which configured place target a template needs, for availability checks against status.
export function placeTarget(template) {
  const place = template?.place ?? {};
  if (place.type === 'ring') return { type: 'ring' };
  if (typeof place.color === 'string') return { type: 'box', color: place.color };
  return { type: 'box' };
}

export function describeTarget(template) {
  const target = placeTarget(template);
  const cube = template?.pick?.select && template.pick.select !== 'only' ? `${template.pick.select} cube` : 'cube';
  return target.type === 'ring' ? `${cube}-to-ring` : `${cube}-to-${target.color ? target.color + ' ' : ''}box`;
}

export function previewCode2Command(intent) {
  // Fail closed if a future parser adds meaning that this mapping cannot honor.
  if (!exactKeys(intent, ['schema_version', 'action', 'source_selector', 'destination_selector', 'placement_relation'])
    || intent.schema_version !== 1 || intent.action !== 'pick_place'
    || ![intent.source_selector, intent.destination_selector].every(selector =>
      exactKeys(selector, ['kind', 'color', 'spatial', 'marker']))) {
    return unsupported('invalid_intent', 'This meaning does not match the reviewed code-2 mapping.');
  }
  const source = intent.source_selector, destination = intent.destination_selector, goal = destination.kind;
  if (source.kind !== 'cube' || !Object.hasOwn(CODE2_REFERENCE.goals, goal)) {
    return unsupported('unsupported_objects', 'Code-2 moves a cube onto the ring or into a box. This object combination has no code-2 command.');
  }
  if (intent.placement_relation !== CODE2_REFERENCE.goals[goal]) {
    return unsupported('unsupported_relation', goal === 'ring'
      ? 'Code-2 places the cube on the ring. Placement inside the ring has no matching command.'
      : 'Code-2 places the cube inside the box. Placement on top of the box has no matching command.');
  }
  if (source.marker !== null || destination.marker !== null) {
    return unsupported('unsupported_selection', 'Code-2 cannot select objects by paper marker yet. Your qualifiers are preserved in the meaning; no command is generated.');
  }
  if (source.color !== null && source.color !== CODE2_REFERENCE.cubeColor) {
    return unsupported('unsupported_selection', `Code-2 only picks ${CODE2_REFERENCE.cubeColor} cubes.`);
  }
  if (goal === 'ring' && (destination.spatial !== null || (destination.color !== null && destination.color !== CODE2_REFERENCE.ringColor))) {
    return unsupported('unsupported_selection', 'There is one black ring; it has no color or side to choose.');
  }
  if (goal === 'box' && destination.spatial !== null) {
    return unsupported('unsupported_selection', 'Code-2 picks boxes by name or color, not left/right yet.');
  }
  const select = source.spatial === null ? 'only' : SELECT[source.spatial];
  if (!select) return unsupported('unsupported_selection', 'Unknown cube side.');
  const place = goal === 'ring' ? { type: 'ring' } : destination.color !== null ? { color: destination.color } : { type: 'box' };
  return {
    status: 'template_only',
    reason: 'readiness_not_confirmed',
    message: 'Reference template only. Readiness is deliberately false, so code-2 would reject this command. Nothing is sent.',
    command_template: { action: 'pick_place', pick: { object: 'cube', select }, place, workcell_ready: false },
  };
}
