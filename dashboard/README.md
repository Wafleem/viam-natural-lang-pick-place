# Test bench dashboard

A local page for manually testing each command, with basic arm, gripper and config controls.

```bash
pickplace-module/.venv/bin/pip install -r dashboard/requirements.txt
pickplace-module/.venv/bin/python dashboard/server.py
```

Then open <http://localhost:8765>. It uses your own `viam login`; no keys reach the browser.

- **Pick & place**: choose a cube (only/left/right/nearest/farthest) and a target, then *Dry run* (no motion) or *Pick & place*.
- **Raw DoCommand**: send any JSON to code-2, exactly what the web app will send.
- **Arm**: joint readout and small jogs (≤ 10°, ≤ 5°/s, no collision checking).
- **Gripper**: suction on/off.
- **Config**: load, validate and apply code-2's attributes.

Anything that moves needs the *workcell clear* box ticked. **STOP** is a software stop; keep the e-stop in reach.
