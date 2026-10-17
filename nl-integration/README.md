# nl-integration

Turns plain language into robot commands. It works today with simple rules; the plan is to
swap in a language model and merge with the team's web app.

```bash
pickplace-module/.venv/bin/python nl-integration/send.py "can you put the left cube on the ring"
```

That prints `{"action": "dry_run", "pick": {"object": "cube", "select": "left"}, "place": {"type": "ring"}}`.
Add `--send` to send it to the robot, and also `--confirm` for commands that move the arm.

| File | What it is |
|---|---|
| `command_schema.json` | JSON Schema of every code-2 command; give it to the web app and to an LLM |
| `nl_to_command.py` | `parse(text, describe)` → command, `validate`, `with_confirmation` |
| `send.py` | command-line demo; dry run unless told otherwise |
| `test_nl.py` | parser tests: `pickplace-module/.venv/bin/python -m pytest nl-integration` |

The dashboard (`dashboard/`) has a **Say it** box that does the same, with a Send button.

## Talking to the robot from a web app

Everything goes through one Viam call: the generic service **code-2**, `DoCommand`, with a JSON
object. With the Viam TypeScript SDK that is
`new VIAM.GenericServiceClient(machine, 'code-2').doCommand(Struct.fromJson(cmd))`.

1. On load, send `{"action":"describe"}` to get the place targets (ring, boxes with colours) and
   cube selectors for the UI.
2. To act, send:
   `{"action":"pick_place","pick":{"object":"cube","select":"left"},"place":{"color":"orange"},"workcell_ready":true,"request_id":"<unique>"}`.
   Use `"action":"dry_run"` to check without moving.
3. Poll `{"action":"status"}` until `busy` is false. Show `state`, `stage` and `error`, and
   `last_result` on success.
4. `{"action":"stop"}` must always be one click away.

Rules for any UI or LLM front end:
- Only a person sets `workcell_ready: true`, after checking the workcell.
- Validate against `describe` before sending.
- Report the robot's `error` text verbatim.

## Swapping in an LLM

Give the model the `describe` output and `command_schema.json`, and have it return exactly one
command object. Keep `validate` and `with_confirmation` between the model and the robot.
