// Pure local translation. No SDK, network, observations, planning, or execution.
// Mirrors natural_language.py parse_intent's bounded whole-command grammar.
export const MAX_TEXT_CHARS = 300;
export const MAX_TEXT_WORDS = 60;
const colors = new Set('white black red green blue yellow pink orange purple gray brown'.split(' '));
const kinds = { cube: 'cube', block: 'cube', ring: 'ring', hoop: 'ring', box: 'box' };
const spatialRules = { right: 'rightmost', rightmost: 'rightmost', left: 'leftmost', leftmost: 'leftmost' };
const outcome = (status, reason, message) => ({ schema_version: 1, status, reason, message });

// Every word the grammar understands. Negation/control words are included so a misspelled
// "stop" or "never" is recognized and rejected rather than guessed into something else.
const VOCABULARY = [...'pick up grab take get move lift place put set drop bring carry transfer and then it on top of onto inside into in to the a an with paper at could would can you please kindly thanks thank that this one cube block ring hoop box right rightmost left leftmost grey no not never dont cannot cant without except instead unless stop cancel'.split(' '), ...colors];
const VOCABULARY_SET = new Set(VOCABULARY);
const CONTROL_WORDS = new Set('no not never dont cannot cant without except instead unless stop cancel'.split(' '));
const KEY_POSITION = new Map(['qwertyuiop', 'asdfghjkl', 'zxcvbnm']
  .flatMap((row, r) => [...row].map((key, c) => [key, [r, c + r * 0.5]])));
const nearbyKeys = (a, b) => {
  const p = KEY_POSITION.get(a), q = KEY_POSITION.get(b);
  return Boolean(p && q) && Math.abs(p[0] - q[0]) <= 1 && Math.abs(p[1] - q[1]) <= 1;
};

// Weighted edit distance tuned for typing slips: neighbouring keys, swapped letters and
// doubled/dropped repeat letters cost half as much as an unrelated edit.
function typoCost(typed, known) {
  const d = Array.from({ length: typed.length + 1 }, () => Array(known.length + 1).fill(0));
  const extra = i => (i > 1 && typed[i - 1] === typed[i - 2] ? 0.5 : 1);
  const missing = j => (j > 1 && known[j - 1] === known[j - 2] ? 0.5 : 1);
  for (let i = 1; i <= typed.length; i++) d[i][0] = d[i - 1][0] + extra(i);
  for (let j = 1; j <= known.length; j++) d[0][j] = d[0][j - 1] + missing(j);
  for (let i = 1; i <= typed.length; i++) for (let j = 1; j <= known.length; j++) {
    const a = typed[i - 1], b = known[j - 1];
    d[i][j] = Math.min(d[i - 1][j] + extra(i), d[i][j - 1] + missing(j),
      d[i - 1][j - 1] + (a === b ? 0 : nearbyKeys(a, b) ? 0.5 : 1));
    if (i > 1 && j > 1 && a === known[j - 2] && typed[i - 2] === b) d[i][j] = Math.min(d[i][j], d[i - 2][j - 2] + 0.5);
  }
  return d[typed.length][known.length];
}

// Up to four plausible known words for an unknown word, most likely first.
function typoCandidates(word) {
  const limit = word.length < 2 ? 0 : word.length === 2 ? 0.5 : word.length === 3 ? 1 : word.length <= 5 ? 1.5 : 2;
  return VOCABULARY.map(known => ({ word: known, cost: typoCost(word, known) }))
    .filter(candidate => candidate.cost <= limit)
    .sort((a, b) => a.cost - b.cost).slice(0, 4);
}

function selector(phrase) {
  phrase = phrase.trim().replace(/^(?:the|a|an) /, '');
  if (['it', 'that', 'this', 'one'].includes(phrase)) return { pronoun: true };
  let marker = null, spatial = null, color = null;
  if (/ with (?:the )?pink paper$/.test(phrase)) {
    phrase = phrase.replace(/ with (?:the )?pink paper$/, ''); marker = 'pink_paper';
  }
  const suffix = / (?:on|at|to) (?:the )?(right|left)$/.exec(phrase);
  if (suffix) { spatial = spatialRules[suffix[1]]; phrase = phrase.slice(0, suffix.index); }
  const words = phrase.split(/\s+/);
  if (!words.length || !Object.hasOwn(kinds, words.at(-1)) || words.length > 3) return null;
  const kind = kinds[words.pop()];
  for (let word of words) {
    if (word === 'grey') word = 'gray';
    if (colors.has(word) && color === null) color = word;
    else if (Object.hasOwn(spatialRules, word) && spatial === null) spatial = spatialRules[word];
    else return null;
  }
  return { kind, color, spatial, marker };
}

export function parseIntent(input) {
  if (typeof input !== 'string' || [...input].length < 1 || [...input].length > MAX_TEXT_CHARS) {
    return outcome('unsupported', 'invalid_text', 'Enter one short text command (1–300 characters).');
  }
  if (!input.trim() || input.trim().split(/\s+/).length > MAX_TEXT_WORDS) {
    return outcome('unsupported', 'invalid_text', 'Enter one short pick-and-place command (at most 60 words).');
  }
  const lowered = input.toLowerCase().replaceAll('’', "'").trim();
  const options = [];
  for (const word of new Set(lowered.match(/[a-z]+/g) ?? [])) {
    if (VOCABULARY_SET.has(word)) continue;
    const found = typoCandidates(word);
    if (!found.length) continue;
    // Fail closed: if a stop/never-style word is a plausible reading, read it that way.
    const control = found.find(c => CONTROL_WORDS.has(c.word) && c.cost <= found[0].cost + 0.5);
    options.push([word, control ? [control] : found]);
  }
  // Like a search box's "did you mean": rank whole-sentence readings by total typo cost and
  // keep the most likely one that forms a command (or the most likely one overall).
  let readings = [{ cost: 0, map: new Map() }];
  for (const [word, found] of options) {
    readings = readings.flatMap(r => found.map(c => ({ cost: r.cost + c.cost, map: new Map([...r.map, [word, c.word]]) })))
      .sort((a, b) => a.cost - b.cost).slice(0, 64);
  }
  let chosen = null;
  for (const reading of readings) {
    const result = parseText(lowered.replace(/[a-z]+/g, word => reading.map.get(word) ?? word));
    if (!chosen || result.status === 'parsed') chosen = { reading, result };
    if (result.status === 'parsed') break;
  }
  const corrections = [...chosen.reading.map].map(([from, to]) => ({ from, to }));
  return corrections.length ? { ...chosen.result, corrections } : chosen.result;
}

function parseText(text) {
  if (/\b(?:no|not|never|don't|dont|cannot|can't|cant|without|except|instead|unless|stop|cancel)\b/.test(text)) {
    return outcome('unsupported', 'negation_or_control', 'Negated instructions and control commands are not translated. Nothing is sent to Viam.');
  }
  text = text.replace(/[.!?]$/, '').trim();
  text = text.replace(/^(?:(?:could|would|can) you (?:please )?|please[, ]+|kindly )/, '');
  text = text.replace(/(?:,? please|,? thanks|,? thank you)$/, '').trim();
  if (!/^[a-z ]+$/.test(text)) {
    return outcome('unsupported', 'unsupported_syntax', 'Use words for one cube-to-destination transfer, without code or coordinates.');
  }
  text = text.split(/\s+/).join(' ');
  const pick = '(?:pick up|pick|grab|take|get|move|lift)', place = '(?:place|put|move|set|drop)';
  const relation = '(?:on top of|onto|on|inside|into|in|to)';
  let parsed = null;
  const match = new RegExp(`^${pick} (.+?) (?:(?:and then|and|then) )?${place} (?:it )?(${relation}) (.+)$`).exec(text);
  if (match) {
    const source = selector(match[1]), destination = selector(match[3]);
    if (source && destination) parsed = [source, destination, match[2]];
  }
  // "<source> <relation> <destination>", accepted only when exactly one split names two objects.
  const transfer = body => {
    const alternatives = [];
    for (const boundary of body.matchAll(new RegExp(` (${relation}) `, 'g'))) {
      const source = selector(body.slice(0, boundary.index));
      const destination = selector(body.slice(boundary.index + boundary[0].length));
      if (source && destination) alternatives.push([source, destination, boundary[1]]);
    }
    return alternatives.length === 1 ? alternatives[0] : null;
  };
  const single = /^(?:move|put|place|transfer|take|set|drop|bring|carry) (.+)$/.exec(text);
  if (!parsed && single) parsed = transfer(single[1]);
  // Verbless shorthand such as "the cube inside the box" or "cube to ring".
  if (!parsed) parsed = transfer(text);
  if (!parsed) {
    const missing = new RegExp(`^(?:${pick}|move|put|place|transfer) (.+)$`).exec(text);
    if (missing && selector(missing[1])) return outcome('clarification_required', 'missing_destination', 'Where should the cube be placed?');
    return outcome('unsupported', 'unsupported_syntax', 'Use one command such as: Move the cube to the ring, or Pick up the cube and place it on the ring.');
  }
  const [source, destination, rawRelation] = parsed;
  if (source.pronoun || destination.pronoun) return outcome('clarification_required', 'unbound_reference', 'Name both the source object and destination in this command.');
  if (source.kind !== 'cube') return outcome('unsupported', 'unsupported_source', 'This preview grammar supports cube sources.');
  const placementRelation = rawRelation === 'to'
    ? destination.kind === 'box' ? 'inside' : 'on'
    : ['inside', 'into', 'in'].includes(rawRelation) ? 'inside' : 'on';
  return { schema_version: 1, status: 'parsed', intent: {
    schema_version: 1, action: 'pick_place', source_selector: source,
    destination_selector: destination, placement_relation: placementRelation,
  } };
}
