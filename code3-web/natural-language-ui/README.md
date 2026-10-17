# Armfarm review-and-send web app

Translate locally → inspect the meaning → acknowledge the reviewed task and clear workcell → **Send reviewed command** once. The browser connects directly to the existing Viam `code-2` generic service using the official TypeScript SDK. No robot source/config change or code-3 service deployment is required.

**Verified offline (51 tests, Node 22).** This frontend ("code-3") targets code-2's `pick_place` API. The implementation id is checked by prefix on every status read; it does not prove physical readiness.

## Run

Requires Node 22+ and npm:

```sh
npm ci
npm run build
npm test
npm start
```

Open [the console](http://127.0.0.1:5173/). The loopback server serves only generated public assets, including the pinned SDK 0.76.0; there is no CDN or backend proxy. The SDK loads with the page but connects only on explicit **Connect & read status**. The already-loaded page remains usable after the development server stops; reloading requires restarting the server.

`dist/translator.html` is a separate self-contained, translation-only page that can be opened from disk. It contains no SDK, connection controls or send path. Its retained sources live in `preview/`. The connected console needs HTTP/HTTPS to load its modules.

## Connection and operator flow

1. Enter the public machine hostname, machine-scoped API key ID and API key from Viam's machine **CONNECT → TypeScript → Include API key** sample. Use a hostname ending in `.viam.cloud`, without scheme/path. The fields clear immediately on submission. Credentials are held in the page/SDK session only; the app never logs, stores or embeds them in build files.
2. Choose **Connect & read status**. This authenticates and reads only `code-2` status. It does not search, plan or move the arm. Reconnecting is explicit and never retries a movement command.
3. Type a task and choose **Translate locally**. Optional push-to-talk supplies final text only; speech may use the browser's external recognition service. Review the semantic JSON and the separate code-2 command preview.
4. For **“Move the cube to the ring”**, confirm **“I reviewed this task and confirm the workcell is clear.”** Then press **Send reviewed command**. This sends one `start` with `goal: "ring"`, a UUID and `workcell_ready: true`. The goal is always explicit because v17 otherwise uses the configured `default_goal` (live: `box`). With `require_all_targets` enabled, v17 first runs a multi-view arm sweep before any pickup. It initiates the complete existing cycle, including motion to overview/search, planning, pickup and placement. This is a review of the translated task, not a review of a computed motion trajectory.
5. Follow the tracked request ID, service state and stage. Accepted means submitted, not completed. Completion requires matching request ID, terminal state and `busy:false`. If a timeout/disconnect leaves uncertainty, the app blocks further sends and reads status for reconciliation. It never assumes a blank/different request ID means nothing happened.

Edits, retranslation, connection changes, stale/invalid status and meaningful machine-status changes invalidate review acknowledgment. Each review can be sent once. Unsupported qualifiers, boxes and inside-ring requests never generate a generic start by dropping their meaning. The Send button requires a fresh matching implementation/status, configured calibration and an idle/terminal service, in addition to explicit review.

Only the unresolved request ID is journaled in `sessionStorage` before dispatch, so a reload does not silently permit another send. It contains no credentials or natural-language input. If the journal cannot be written/cleared, sending stays blocked. Clearing browser/tab storage or opening another tab can bypass this client-side history; code-2's own deduplication is in memory and its status exposes only its latest request. A restart or another client's request may therefore leave an outcome unresolved and require operator inspection. This prototype does not provide fleet-wide or persistent server-side exactly-once execution.

**Disconnect is not Stop.** An accepted asynchronous code-2 task can continue after a timeout, disconnect or tab closure. Use the existing workcell/Viam controls for physical operation; this app makes no promise that closing it stops motion.

## Payload and supported task

Integrated with code-2's `pick_place` API (pickplace-module ≥ 0.0.30; implementation ids starting
`pickplace-v1`). See `../../nl-integration/command_schema.json` and `{"action":"describe"}`.

| Said | Sent after review |
|---|---|
| Move the cube to the ring | `{"action":"pick_place","pick":{"object":"cube","select":"only"},"place":{"type":"ring"}}` |
| Put the left cube into the orange box | `{"action":"pick_place","pick":{"object":"cube","select":"left"},"place":{"color":"orange"}}` |
| Put the rightmost cube in the box | `{"action":"pick_place","pick":{"object":"cube","select":"right"},"place":{"type":"box"}}` |

Each send also carries a fresh UUID `request_id` and `workcell_ready: true`, and only after the
explicit review checkbox. The unapproved preview always has `workcell_ready: false`.

- Still unsupported, shown but never sent: paper markers, non-white cubes, ring colour or side,
  boxes chosen by side, and "inside the ring" or "on the box".
- Send stays locked unless status is fresh and from a `pickplace-v1` implementation, code-2 is
  idle and calibrated, and the place target exists in `status.goal_details`. For a coloured box,
  a box of that colour must be configured.
- The transport accepts only `status` and this exact `pick_place` shape.

The previous `start`/`box_once` contract (code-2 0.0.23) is described in `../code2-reference-0.0.23.json`
for history. code-2 still accepts `start` as a legacy alias.

## Viam-hosted users

The same `dist/` supports a Viam **single-machine** app. Select **Viam-hosted machine session**, then explicitly Connect. The adapter reads only the cookie named for `/machine/{id}/`, validates its matching machine ID/hostname and passes its API-key credential to the SDK. The platform supplies login and machine selection; credentials are never hardcoded into public files. Relative asset URLs work under Viam's machine prefix.

No hosting upload, registration, publication or live login was performed in this task. Local use is ready; hosting remains a separate deployment step. The old connected controller draft in `archive/control-ui/` remains inactive and is excluded from the build/server.

## Verification

Offline tests use fake machine/RPC endpoints, including the real pinned SDK's protobuf serialization. They cover exact single dispatch, review invalidation, duplicate clicks, stale/late status, unknown outcomes, reconnect/reload recovery, credential handling, hosted-cookie isolation and standalone translation. Browser QA stays disconnected and exercises local translation only. These checks establish implementation behavior, not live reachability, robot acceptance, calibration or physical task success.

Official references checked October 16, 2026:

- [SDK connection API](https://ts.viam.dev/functions/createRobotClient.html)
- [Connection events and reconnect behavior](https://docs.viam.com/build-apps/tasks/handle-connection-state/)
- [Viam hosting and single-machine credentials](https://docs.viam.com/build-apps/hosting/hosting-reference/)
- [Getting machine connection credentials](https://docs.viam.com/build-apps/setup/overview/)
- [Official SDK source](https://github.com/viamrobotics/viam-typescript-sdk)
