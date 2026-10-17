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
    'place': {'goals': [{'name': 'ring', 'type': 'ring'}, {'name': 'box', 'type': 'box', 'color': 'orange'}]},
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


def parse(text, describe=DESCRIBE):
    """-> {'command': dict, 'moves': bool, 'explanation': str} or raises ValueError."""
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

    if 'cube' not in words and not any(w in words for w in ('block', 'it', 'object')):
        raise ValueError('I only know how to move the cube; try "put the cube in the box"')
    select = next((SELECT_WORDS[w] for w in words if w in SELECT_WORDS), None)
    index = next((ORDINALS[w] for w in words if w in ORDINALS), None)
    pick = {'object': 'cube', 'select': {'index': index} if index is not None else (select or 'only')}

    goals = describe['place']['goals']
    place = None
    generic = {'box', 'bin', 'container', 'basket'}
    for g in goals:  # specific box names first ("box2", "blue_box"); plain "box" is not a name
        if g['type'] == 'box' and g['name'].lower() not in generic and g['name'].lower() in words:
            place = {'goal': g['name']}
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
    return {'command': {'action': action, 'pick': pick, 'place': place}, 'moves': action in MOVES,
            'explanation': f"{'Plan (no motion)' if check else 'Pick'} the {pick['select'] if isinstance(pick['select'], str) else 'cube #'+str(index+1)} cube -> {target}"}


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
