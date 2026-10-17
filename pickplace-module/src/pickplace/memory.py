"""Where the cube is: remembered across cycles and module restarts.

The camera can find the cube on the table, but not reliably inside a box or sitting on the
ring. So code-2 remembers the pose it placed the cube at, and the grasp it held it with,
and uses that to pick it back up (reset_cube, or any cycle starting from a box or the ring).

Stored as JSON in $VIAM_MODULE_DATA (the per-module data directory viam-server provides);
falls back to memory only when that is not set (tests, local runs).
"""
import json
import os
import time
from pathlib import Path

PLACES_FROM_VISION = ('table', 'home')  # the overview camera can see and re-detect these


class CubeMemory:
    def __init__(self, directory=None):
        directory = directory or os.environ.get('VIAM_MODULE_DATA')
        self.path = Path(directory)/'cube_location.json' if directory else None
        self.location = {'place': 'unknown'}
        if self.path and self.path.exists():
            try:
                self.location = json.loads(self.path.read_text())
            except (OSError, ValueError):
                self.location = {'place': 'unknown', 'note': 'stored location was unreadable'}

    def set(self, place, top_world_mm=None, yaw_deg=None, tcp_in_cube=None, how=''):
        self.location = {'place': place, 'top_world_mm': top_world_mm, 'yaw_deg': yaw_deg,
                         'tcp_in_cube': tcp_in_cube, 'how': how, 'at': time.strftime('%Y-%m-%dT%H:%M:%S%z')}
        if self.path:
            try:
                self.path.parent.mkdir(parents=True, exist_ok=True)
                self.path.write_text(json.dumps(self.location, indent=1))
            except OSError:
                pass
        return self.location

    def pickable_source(self):
        """The remembered location as a compute_targets source, or None when vision should find it."""
        loc = self.location
        if loc.get('place') in PLACES_FROM_VISION or loc.get('place') == 'unknown':
            return None
        if loc.get('top_world_mm') is None or loc.get('tcp_in_cube') is None:
            return None
        return {'place': loc['place'], 'top_world_mm': loc['top_world_mm'], 'yaw_deg': loc['yaw_deg'],
                'tcp_in_cube': loc['tcp_in_cube']}
