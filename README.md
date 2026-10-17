# armfarm-pickplace

Source for the armfarm19 Viam hackathon robot: an xArm6 with a vacuum gripper and a wrist
RealSense picks the white cube and drops it on the black ring or into the orange Post-it box.

| Folder | Contents |
|---|---|
| [pickplace-module/](pickplace-module/) | Viam module for arm motion and vision (code-2), tests, tools, [deployment guide](pickplace-module/DEPLOYMENT.md) |
| [nl-integration/](nl-integration/) | Placeholder for the upcoming natural-language layer |

History before this repo, including calibration evidence, earlier versions and rollback
snapshots, lives in `~/Documents/ChatGPT/Viam Robotics` (not under version control).

Never commit credentials. The tools use your own `viam login` and keep the machine key in memory.
