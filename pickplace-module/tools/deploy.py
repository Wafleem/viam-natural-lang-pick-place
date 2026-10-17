"""Upload this module's source to the Viam registry and start a cloud build. Never moves hardware.

    .venv/bin/python tools/deploy.py --version 0.0.25

Uses the official PackageService and BuildService (inline-source packages, the same
path `viam module upload --cloud-build` uses for hosted builds). The build result is
recorded in tools/last-build.json. The machine keeps running its pinned version until
tools/select_version.py --apply moves it.
"""
import argparse
import asyncio
import io
import json
import sys
import tarfile
from pathlib import Path

from google.protobuf import descriptor_pb2, descriptor_pool, message_factory
from grpclib.client import Channel, UnaryUnaryMethod
from viam.gen.app.packages.v1.packages_grpc import PackageServiceStub
from viam.gen.app.packages.v1.packages_pb2 import CreatePackageRequest, FileInfo, ListPackagesRequest, PackageInfo

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT/'tools'))
from connect import access_token  # noqa: E402

META = json.loads((ROOT/'meta.json').read_text())
ORG = '6d6c7293-bc67-43e4-9c8b-31c76aac27d3'
MODULE_ID = META['module_id']
NAME = MODULE_ID.split(':')[1]
FILES = ['build.sh', 'setup.sh', 'requirements.txt', 'meta.json', 'src/main.py']+sorted(
    str(p.relative_to(ROOT)) for p in (ROOT/'src'/'pickplace').glob('*.py'))


def build_messages():
    # Field numbers/types from viam/app/build/v1/build.proto (not shipped in the Python SDK).
    fd = descriptor_pb2.FileDescriptorProto(name='pickplace_build_subset.proto', package='viam.app.build.v1', syntax='proto3')
    specs = [('StartPackageBuildRequest', [('module_id', 1, False), ('package_version', 2, False), ('module_version', 3, False), ('platforms', 4, True)]),
             ('StartPackageBuildResponse', [('build_id', 1, False)])]
    for name, fields in specs:
        msg = fd.message_type.add(name=name)
        for field, number, repeated in fields:
            msg.field.add(name=field, number=number, type=9, label=3 if repeated else 1)
    pool = descriptor_pool.DescriptorPool()
    pool.Add(fd)
    return {n: message_factory.GetMessageClass(pool.FindMessageTypeByName('viam.app.build.v1.'+n)) for n, _ in specs}


async def upload(args):
    md = {'authorization': 'Bearer '+await access_token()}
    channel = Channel('app.viam.com', 443, ssl=True)
    try:
        packages = PackageServiceStub(channel)
        source_version = 'inline-source-'+args.version.replace('.', '-')
        existing = await packages.ListPackages(ListPackagesRequest(organization_id=ORG, name=NAME, include_url=False), metadata=md, timeout=30)
        if any(p.info.version in (source_version, args.version) or p.info.version.startswith(args.version+'-') for p in existing.packages):
            raise SystemExit(f'Version {args.version} already exists; pick the next version')
        payload = io.BytesIO()
        with tarfile.open(fileobj=payload, mode='w:gz') as tar:
            for name in FILES:
                tar.add(ROOT/name, arcname=name, recursive=False)
        info = PackageInfo(organization_id=ORG, name=NAME, version=source_version, type=3,
                           files=[FileInfo(name=n, size=(ROOT/n).stat().st_size, is_directory=False) for n in FILES])
        blob = payload.getvalue()
        async with packages.CreatePackage.open(metadata=md, timeout=60) as stream:
            await stream.send_message(CreatePackageRequest(info=info))
            for i in range(0, len(blob), 16384):  # PackageService accepts chunks <= 32 KiB
                await stream.send_message(CreatePackageRequest(contents=blob[i:i+16384]))
            await stream.end()
            saved = await stream.recv_message()
        print('Source uploaded:', saved.id, saved.version, flush=True)
        msgs = build_messages()
        start = UnaryUnaryMethod(channel, '/viam.app.build.v1.BuildService/StartPackageBuild', msgs['StartPackageBuildRequest'], msgs['StartPackageBuildResponse'])
        result = await start(msgs['StartPackageBuildRequest'](module_id=MODULE_ID, package_version=source_version, module_version=args.version, platforms=META['build']['arch']), metadata=md, timeout=60)
        record = {'build_id': result.build_id, 'source_version': source_version, 'module_version': args.version}
        (ROOT/'tools'/'last-build.json').write_text(json.dumps(record, indent=2)+'\n')
        print(json.dumps(record), flush=True)
        print('Check the build at https://app.viam.com/module/hackathons/'+NAME+' (Builds tab) before selecting it.')
    finally:
        channel.close()


if __name__ == '__main__':
    p = argparse.ArgumentParser()
    p.add_argument('--version', required=True, help='semantic module version, e.g. 0.0.25')
    asyncio.run(upload(p.parse_args()))
