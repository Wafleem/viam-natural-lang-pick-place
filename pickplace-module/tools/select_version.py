"""Point armfarm19's code-2 at a module version (and optionally new attributes).

Preview by default; --apply writes. Touches ONLY the code-2 module entry and the code-2
service attributes; every other part of the machine config (teammate frames, arm,
camera, fragments) is left exactly as it is. Refuses while the arm moves or code-1/
code-2 are busy. Never moves hardware; viam-server reloads code-2 afterwards.

    .venv/bin/python tools/select_version.py --version 0.0.25
    .venv/bin/python tools/select_version.py --version 0.0.25 --attributes config/code2-attributes.json --apply
    .venv/bin/python tools/select_version.py --version 0.0.24 --apply        # roll back
"""
import argparse
import asyncio
import copy
import json
import sys
from pathlib import Path

from viam.components.arm import Arm
from viam.services.generic import Generic

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT/'tools'))
from connect import app_client, connect  # noqa: E402

PART_ID = 'f5c74a65-a023-4983-a317-1e9a88c88372'
MODULE_ID = json.loads((ROOT/'meta.json').read_text())['module_id']
SERVICE = 'code-2'


def updated(config, version, attributes):
    want = copy.deepcopy(config)
    module = next(m for m in want['modules'] if m.get('module_id') == MODULE_ID)
    module['version'] = version
    if attributes is not None:
        service = next(s for s in want['services'] if s.get('name') == SERVICE)
        service['attributes'] = attributes
    return want


async def main(args):
    attributes = json.loads(Path(args.attributes).read_text()) if args.attributes else None
    if attributes is not None:
        sys.path.insert(0, str(ROOT/'src'))
        from pickplace.config import Settings
        Settings.from_attributes(attributes)  # same validation the module runs
    async with app_client() as app:
        part = await app.get_robot_part(PART_ID)
        current = part.robot_config
        module = next(m for m in current['modules'] if m.get('module_id') == MODULE_ID)
        want = updated(current, args.version, attributes)
        changed = [k for k in want if want[k] != current.get(k)]
        print(json.dumps({'mode': 'apply' if args.apply else 'preview', 'current_version': module.get('version'),
                          'new_version': args.version, 'attributes_replaced': attributes is not None,
                          'changed_sections': changed}, indent=2))
        if not args.apply or not changed:
            return
        backup = ROOT/'tools'/f'config-before-{args.version}.json'
        backup.write_text(json.dumps({'module': module, 'service': next(s for s in current['services'] if s.get('name') == SERVICE)}, indent=2))
        async with connect() as robot:
            if await Arm.from_robot(robot, 'arm').is_moving(timeout=3):
                raise SystemExit('Arm is moving; not changing config')
            for name in ('code-1', 'code-2'):
                try:
                    status = await Generic.from_robot(robot, name).do_command({'action': 'status'}, timeout=5)
                except Exception:
                    continue
                if status.get('busy') or status.get('reserved_by'):
                    raise SystemExit(name+' is busy or reserved; not changing config')
        await app.update_robot_part(part.id, part.name, want, last_known_update=part.last_updated)
        saved = await app.get_robot_part(part.id)
        if saved.robot_config != want:
            raise SystemExit('Config readback differs from what was written; inspect in the Viam app')
        print(f'code-2 now on {args.version}. Previous module/service saved to {backup.relative_to(ROOT)}.')
        print('Wait for the module to reload, then check {"action":"status"} on code-2.')


if __name__ == '__main__':
    p = argparse.ArgumentParser()
    p.add_argument('--version', required=True)
    p.add_argument('--attributes', help='JSON file to use as the code-2 service attributes')
    p.add_argument('--apply', action='store_true')
    asyncio.run(main(p.parse_args()))
