import asyncio

from viam.module.module import Module

from pickplace.service import PickPlaceService  # noqa: F401  (registers the model)

if __name__ == '__main__':
    asyncio.run(Module.run_from_registry())
