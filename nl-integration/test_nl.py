import pytest

from nl_to_command import DESCRIBE, parse, validate, with_confirmation

TWO_BOXES = {**DESCRIBE, 'place': {'goals': DESCRIBE['place']['goals']+[{'name': 'blue_box', 'type': 'box', 'color': 'blue'}]}}


@pytest.mark.parametrize('text,action,pick,place', [
    ('put the left cube in the orange box', 'pick_place', 'left', {'color': 'orange'}),
    ('Move the cube onto the ring', 'pick_place', 'only', {'type': 'ring'}),
    ('place the closest cube in the box', 'pick_place', 'nearest', {'type': 'box'}),
    ('can you put the right cube on the hoop?', 'dry_run', 'right', {'type': 'ring'}),
    ('put the second cube in blue_box', 'pick_place', {'index': 1}, {'goal': 'blue_box'}),
    ('drop the cube in the blue bin', 'pick_place', 'only', {'color': 'blue'}),
])
def test_parse(text, action, pick, place):
    command = parse(text, TWO_BOXES)['command']
    assert command == {'action': action, 'pick': {'object': 'cube', 'select': pick}, 'place': place}
    validate(command, TWO_BOXES)


@pytest.mark.parametrize('text,action', [('stop!', 'stop'), ('go home', 'reset'), ("what's the status", 'status')])
def test_simple_intents(text, action):
    assert parse(text)['command']['action'] == action


def test_unknown_requests_are_refused():
    with pytest.raises(ValueError, match='only know how to move the cube'):
        parse('pick up the banana')
    with pytest.raises(ValueError, match='Where should it go'):
        parse('pick up the cube')
    with pytest.raises(ValueError):
        validate({'action': 'pick_place', 'pick': {'select': 'left'}, 'place': {'color': 'green'}})


def test_motion_needs_confirmation():
    command = parse('put the cube in the box')['command']
    with pytest.raises(PermissionError):
        with_confirmation(command, False)
    assert with_confirmation(command, True)['workcell_ready'] is True
    assert 'workcell_ready' not in with_confirmation({'action': 'status'}, False)
