"""Plain language -> code-2 DoCommand JSON.

Rule-based first pass so the whole chain (text -> command -> robot) can be tested today.
Swap `parse` for an LLM later: give it `describe` output plus command_schema.json and have it
return the same dict; keep `validate` and the confirmation step either way.

    >>> parse("put the left cube in the orange box", DESCRIBE)['command']
    {'action': 'pick_place', 'pick': {'object': 'cube', 'select': 'left'}, 'place': {'color': 'orange'}}
"""
import re

# Offline stand-in for {"action":"describe"}; the live one comes from the robot.
DESCRIBE = {
    'pick': {'objects': ['cube'], 'select': ['only', 'left', 'right', 'nearest', 'farthest']},
    'place': {'goals': [{'name': 'ring', 'type': 'ring'}, {'name': 'box', 'type': 'box', 'color': 'orange', 'label': 'box on the table'},
                        {'name': 'pink', 'type': 'box', 'color': 'pink', 'label': 'box on the wall', 'entry': 'side'}]},
}

SELECT_WORDS = {
    'left': 'left', 'leftmost': 'left', 'right': 'right', 'rightmost': 'right',
    'nearest': 'nearest', 'closest': 'nearest', 'near': 'nearest',
    'farthest': 'farthest', 'furthest': 'farthest', 'far': 'farthest',
}
ORDINALS = {'first': 0, '1st': 0, 'second': 1, '2nd': 1, 'third': 2, '3rd': 2}
RING_WORDS = ('ring', 'hoop', 'donut', 'circle')
CHECK_WORDS = ('can you', 'could you', 'is it possible', 'check', 'dry run', 'simulate', 'plan', 'without moving')
MOVES = {'pick_place', 'reset', 'reset_cube'}


def _words(text):
    return re.findall(r"[a-z0-9_'-]+", text.lower())


# Speech recognisers mishear short nouns, especially with an accent. Map common sound-alikes,
# then fuzzy-match leftover unknown words against the object/place vocabulary. Motion-only words
# (reset, home) are never guessed; a guessed command still goes through review before sending.
SOUND_ALIKES = {
    'cute': 'cube', 'cubed': 'cube', 'cubes': 'cube', 'cub': 'cube', 'tube': 'cube', 'queue': 'cube',
    'q': 'cube', 'cuba': 'cube', 'kube': 'cube', 'coop': 'cube',
    'rink': 'ring', 'wring': 'ring', 'rings': 'ring', 'rang': 'ring', 'rung': 'ring',
    'books': 'box', 'bocks': 'box', 'bucks': 'box', 'fox': 'box', 'pox': 'box', 'boxes': 'box', 'bach': 'box',
    'hope': 'hoop', 'hoops': 'hoop', 'write': 'right', 'rite': 'right', 'wright': 'right',
    'lift': 'left', 'orage': 'orange', 'oranges': 'orange', 'ping': 'pink', 'pinks': 'pink', 'wal': 'wall', 'walls': 'wall',
}
FUZZY_TARGETS = ['cube', 'block', 'ring', 'hoop', 'box', 'bin', 'basket', 'orange', 'pink', 'wall', 'shelf', 'blue', 'left', 'right',
                 'nearest', 'closest', 'farthest', 'furthest', 'leftmost', 'rightmost']
KNOWN = set(FUZZY_TARGETS) | set(SELECT_WORDS) | set(ORDINALS) | set(RING_WORDS) | {
    'put', 'place', 'move', 'pick', 'up', 'drop', 'grab', 'take', 'set', 'bring', 'carry', 'the', 'a', 'an', 'it',
    'in', 'into', 'on', 'onto', 'to', 'top', 'of', 'inside', 'and', 'then', 'can', 'you', 'could', 'please',
    'stop', 'halt', 'reset', 'home', 'go', 'back', 'table', 'status', 'check', 'plan', 'release', 'let'}


def normalize(text):
    """-> text with likely mishearings/typos replaced; `parse` uses it and reports any change."""
    import difflib
    out = []
    for word in _words(text):
        fixed = SOUND_ALIKES.get(word, word)
        if fixed == word and word not in KNOWN and len(word) >= 3 and not word.isdigit():
            close = difflib.get_close_matches(word, FUZZY_TARGETS, n=1, cutoff=0.75)
            fixed = close[0] if close else word
        out.append(fixed)
    return ' '.join(out)


def parse(text, describe=DESCRIBE):
    """-> {'command': dict, 'moves': bool, 'explanation': str} or raises ValueError."""
    heard = ' '.join(_words(text))
    fixed = normalize(text)
    result = _parse(fixed, describe)
    if fixed != heard:
        result['heard_as'] = fixed
        result['explanation'] += f' (read as "{fixed}")'
    return result


def _parse(text, describe):
    low, words = text.lower(), _words(text)
    if any(w in words for w in ('stop', 'halt', 'freeze', 'abort')):
        return {'command': {'action': 'stop'}, 'moves': False, 'explanation': 'Stop the arm now'}
    if 'cube' in words and any(p in low for p in ('reset', 'put it back', 'back home', 'put the cube back', 'cube back', 'return the cube')):
        return {'command': {'action': 'reset_cube'}, 'moves': True, 'explanation': 'Put the cube back on its home spot'}
    if any(p in low for p in ('go home', 'reset', 'home position', 'overview')):
        return {'command': {'action': 'reset'}, 'moves': True, 'explanation': 'Return the arm to the overview pose'}
    if any(p in low for p in ('status', "what are you doing", 'what is the arm doing')):
        return {'command': {'action': 'status'}, 'moves': False, 'explanation': 'Report status'}
    if any(w in words for w in ('release', 'drop', 'let')) and 'go' in words or 'suction off' in low:
        return {'command': {'action': 'release'}, 'moves': False, 'explanation': 'Switch suction off'}

    goals = describe['place']['goals']
    generic = {'box', 'bin', 'container', 'basket'}
    # There is one cube, so naming only where it goes ("box", "the ring please") is enough.
    place_words = set(RING_WORDS) | generic | {g['name'].lower() for g in goals} | {g.get('color') for g in goals if g.get('color')}
    if 'cube' not in words and not any(w in words for w in ('block', 'it', 'object')) and not place_words & set(words):
        raise ValueError('I only know how to move the cube to the ring or the box; try "put the cube in the box"')
    select = next((SELECT_WORDS[w] for w in words if w in SELECT_WORDS), None)
    index = next((ORDINALS[w] for w in words if w in ORDINALS), None)
    pick = {'object': 'cube', 'select': {'index': index} if index is not None else (select or 'only')}

    place = None
    for g in goals:  # specific box names first ("box2", "blue_box"); plain "box" is not a name
        if g['type'] == 'box' and g['name'].lower() not in generic and g['name'].lower() in words:
            place = {'goal': g['name']}
    # Boxes can be named by where they sit: "the box on the wall" / "the shelf" vs "the box on the table".
    boxish = bool(set(words) & (generic | {'shelf', 'cubby'}))
    if place is None and (boxish or 'shelf' in words or 'cubby' in words):
        for where in ('wall', 'shelf', 'cubby', 'table'):
            if where in words:
                hit = next((g for g in goals if g['type'] == 'box' and where in str(g.get('label', '')).lower()
                            or (where in ('shelf', 'cubby') and g.get('entry') == 'side')), None)
                if hit:
                    place = {'goal': hit['name']}
                    break
    if place is None and any(w in words for w in ('table', 'home')):
        place = {'type': 'home'}
    if place is None and any(w in words for w in RING_WORDS):
        place = {'type': 'ring'}
    if place is None:
        colours = {g.get('color') for g in goals if g.get('color')}
        colour = next((w for w in words if w in colours), None)
        if colour:
            place = {'color': colour}
        elif any(w in words for w in generic):
            place = {'type': 'box'}
    if place is None:
        raise ValueError('Where should it go? Say the ring or a box ('+', '.join(g['name'] for g in goals)+')')

    check = any(p in low for p in CHECK_WORDS)
    action = 'dry_run' if check else 'pick_place'
    target = place.get('goal') or place.get('color', '') + (' box' if 'color' in place else place.get('type', ''))
    sel = pick['select']
    which = '' if sel == 'only' else f'{sel} ' if isinstance(sel, str) else f'#{index + 1} '
    return {'command': {'action': action, 'pick': pick, 'place': place}, 'moves': action in MOVES,
            'explanation': f"{'Plan (no motion)' if check else 'Pick'} the {which}cube -> {target}"}


def validate(command, describe=DESCRIBE):
    """Reject anything outside the advertised API before it reaches the robot."""
    action = command.get('action')
    if action not in ('pick_place', 'dry_run', 'reset', 'reset_cube', 'status', 'stop', 'release', 'describe', 'self_test'):
        raise ValueError('Unsupported action '+str(action))
    if action in ('pick_place', 'dry_run'):
        select = command['pick']['select']
        if not (isinstance(select, dict) and isinstance(select.get('index'), int)) and select not in describe['pick']['select']:
            raise ValueError('Unknown cube selector '+str(select))
        place = command['place']
        names = {g['name'] for g in describe['place']['goals']}
        colours = {g.get('color') for g in describe['place']['goals']}
        if not (place.get('goal') in names or place.get('type') in ('ring', 'box', 'home') or place.get('color') in colours):
            raise ValueError('Unknown place target '+str(place))
    return command


def with_confirmation(command, confirmed):
    """Motion needs a human 'yes': only then is workcell_ready added."""
    if command['action'] in MOVES:
        if not confirmed:
            raise PermissionError('This moves the arm. Confirm the workcell is clear first.')
        return {**command, 'workcell_ready': True}
    return command
