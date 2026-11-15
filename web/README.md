# Web dashboard

Svelte page served by `viz_relay`. Build with `scripts/build_web.sh`. Check with `npm --prefix web run check`.

## Dev server

The WebSocket URL defaults to `ws://<page host>:8765` at runtime (`src/lib/connection.ts`). For `npm run dev`, set `VITE_WS_URL` to point the page at a relay on another host or port, and `VITE_HTTP_URL` to where `/healthz` is proxied (the relay's Crow server, default port 8080).

## Generated protocol types

`scripts/gen-protocol.mjs` generates `src/generated/protocol.ts` from the remote protocol table. Run it as `node scripts/gen-protocol.mjs [protocol.json]`. With no argument it runs `../build/remote_protocol_dump`, then `../build-test/remote_protocol_dump`.

Type names come from the schema title `auto_battlebot.<kind>.<Name>`:

- top-level status schema: `<Name>Status` (`auto_battlebot.status.System` becomes `SystemStatus`)
- top-level command schema: `<Name>Command` (`auto_battlebot.command.SetAutonomy` becomes `SetAutonomyCommand`)
- nested object schema: `<Name>` (`auto_battlebot.status.TrackedRobot` becomes `TrackedRobot`)

Nested objects with a title move to `definitions` and are referenced by `$ref`. A struct used in several places (`ImagePoint`) is then declared once instead of as `ImagePoint`, `ImagePoint1`, `ImagePoint2`.

## Behaviour notes

- Aiming (`src/lib/aim.ts`): tap or drag on a view places the held target while auto tracking is off. The camera view sends image fractions and the top-down view sends field meters. The app projects an image point onto the floor itself. A drag sends at most every 100 ms and the lift always sends, so the target ends under the finger.
- Connection (`src/lib/connection.ts`): the WebSocket `error` handler does no cleanup, because a `close` event always follows and does it.
- Robot colors (`src/lib/robots.ts`): one color per robot, shared by the camera view and the top-down view. Colors come from `[ui.label_colors]` in `config/_common.toml` (the class colors training uses), delivered on `/status/app`. A label with no configured color gets one from `FALLBACK`, picked by a hash of the label so every device agrees, skipping colors the config already uses. Detections carry a label and no track id, so the camera view colors by label. In the top-down view, a second or third track with the same label (two generic "opponent"s) gets a lighter shade of that color so the two can be told apart.
- Video (`src/lib/video.ts`): each `/camera/preview_video` message is one Annex-B access unit. Keyframes carry SPS and PPS in band, so the decoder needs no `description`. It is configured from the SPS of the first keyframe, and reconfigured if a later keyframe carries a different SPS. A NAL ends at the next start code, and trailing zero bytes belong to that start code, which is why the end index subtracts 3 and strips zeros.
