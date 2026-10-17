"""Connect to armfarm19 with your Viam CLI login; the machine API key stays in memory.

Requires `viam login` first. Override with VIAM_CLI (CLI binary) and VIAM_CLI_CONFIG
(CLI config file, default ~/.viam/cached_cli_config.json). Nothing is written to disk.
"""
import asyncio
import os
import shutil
from contextlib import asynccontextmanager
from pathlib import Path

from grpclib.client import Channel
from viam.app.app_client import AppClient
from viam.robot.client import RobotClient

MACHINE_ID = os.environ.get('VIAM_MACHINE_ID', 'f0a0dba5-2923-41cf-894f-fe1a576822b6')
HOST = os.environ.get('VIAM_MACHINE_HOST', 'armfarm19-main.310sld03v2.viam.cloud')


async def access_token():
    cli = os.environ.get('VIAM_CLI') or shutil.which('viam') or '/private/tmp/viam-armfarm19'
    config = os.environ.get('VIAM_CLI_CONFIG', str(Path.home()/'.viam'/'cached_cli_config.json'))
    proc = await asyncio.create_subprocess_exec(cli, '--config', config, 'login', 'print-access-token',
                                                stdout=asyncio.subprocess.PIPE, stderr=asyncio.subprocess.PIPE)
    out, _ = await asyncio.wait_for(proc.communicate(), 20)
    if proc.returncode:
        raise RuntimeError('Viam CLI login unavailable; run `viam login` first')
    return out.decode().strip()


@asynccontextmanager
async def app_client():
    channel = Channel('app.viam.com', 443, ssl=True)
    try:
        yield AppClient(channel, {'authorization': 'Bearer '+await access_token()})
    finally:
        channel.close()


@asynccontextmanager
async def connect():
    robot = None
    try:
        async with app_client() as app:
            keys = await asyncio.wait_for(app.get_robot_api_keys(MACHINE_ID), 20)
        if not keys:
            raise RuntimeError('No existing machine API key; none was created')
        key = next((k.api_key for k in keys if 'default' in k.api_key.name.lower()), keys[0].api_key)
        robot = await RobotClient.at_address(HOST, RobotClient.Options.with_api_key(api_key=key.key, api_key_id=key.id))
        yield robot
    finally:
        if robot is not None:
            await robot.close()
