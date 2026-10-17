# nl-integration (placeholder)

Reserved for the natural-language layer that will sit on top of the arm module. Nothing is
implemented here yet.

The layer should talk to the robot only through code-2's DoCommand interface (see
[../pickplace-module/DEPLOYMENT.md](../pickplace-module/DEPLOYMENT.md#4-operating)), so it never
needs arm, gripper or planner details:

| Intent | DoCommand | Moves |
|---|---|---|
| "what's the arm doing?" | `{"action":"status"}` | no |
| "can you put the cube in the box?" (check only) | `{"action":"dry_run","goal":"box"}` | no |
| "put the cube in the box" | `{"action":"box_once","workcell_ready":true}` | yes |
| "put the cube on the ring" | `{"action":"ring_once","workcell_ready":true}` | yes |
| "go home" | `{"action":"reset","workcell_ready":true}` | yes |
| "stop" | `{"action":"stop"}` | stops |

Ground rules for whoever builds it:
- `workcell_ready: true` means a person confirmed the workcell is clear. The language layer must
  get that confirmation from the operator, never infer it.
- Poll `status` until `busy` is false; report `error` verbatim.
- Keep credentials out of this folder; connect with the operator's own Viam login.
