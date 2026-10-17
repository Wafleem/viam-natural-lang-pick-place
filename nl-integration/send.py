"""Type a sentence, see the command, optionally send it to code-2.

    pickplace-module/.venv/bin/python nl-integration/send.py "can you put the left cube on the ring"
    pickplace-module/.venv/bin/python nl-integration/send.py "put the cube in the box" --send --confirm

Without --send nothing reaches the robot. --confirm means a person checked the workcell is clear.
"""
import argparse
import asyncio
import json
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(ROOT/'pickplace-module'/'tools'), str(Path(__file__).parent)]
from nl_to_command import DESCRIBE, parse, validate, with_confirmation  # noqa: E402


async def main(args):
    describe = DESCRIBE
    if args.send:
        from connect import connect
        from viam.services.generic import Generic
        async with connect() as robot:
            code2 = Generic.from_robot(robot, 'code-2')
            describe = await code2.do_command({'action': 'describe'})
            result = parse(args.text, describe)
            command = with_confirmation(validate(result['command'], describe), args.confirm)
            print(result['explanation'])
            print(json.dumps(command))
            print(json.dumps(await code2.do_command(command), indent=1, default=str)[:3000])
        return
    result = parse(args.text, describe)
    print(result['explanation'])
    print(json.dumps(validate(result['command'], describe)))
    print('(not sent; add --send' + (' --confirm' if result['moves'] else '') + ')')


if __name__ == '__main__':
    p = argparse.ArgumentParser()
    p.add_argument('text')
    p.add_argument('--send', action='store_true')
    p.add_argument('--confirm', action='store_true', help='a person confirmed the workcell is clear')
    asyncio.run(main(p.parse_args()))
