# 2026-11-15 rl nav metrics

## Notes moved from code comments (2026-11-15)

### `playground/rl/evaluate_nav_policy.py`

- A flip or a hole ends the episode early, so danger seconds per episode reward dying fast. The share of time alive spent in danger compares policies that survive for different times. (at `alive = sum(float(r.get("seconds", episode_s)) for r in rows)`)

### `playground/rl/score_field_session.py`

Reads both robots' field poses from `/status/tracks` and scores only the time autonomy was
engaged (`runner/autonomy_on_s` above zero). Contact is inferred from the tracked poses, not
measured, so the counts are approximate: the robots touch when their centers are under
`--contact-m` apart.

- Strike: contact, the opponent inside our wedge cone, and we are outside the opponent's arc.
- Hit taken: contact while we are inside the opponent's arc.
- Danger: within `--contact-m` plus `WarpNavConfig.danger_margin_m` and inside the arc.
- Contacts closer than `WarpNavConfig.engagement_gap_s` apart merge into one event.

The contact and wedge thresholds are unvalidated against video. Check strike and hit timestamps
in Foxglove before trusting the strikes-per-hit ratio.

source scripts/activate_python.sh
python playground/rl/score_field_session.py data/recordings/<session>.mcap [...]
python playground/rl/score_field_session.py data/recordings/<session>.mcap --clips hit

- Elapsed is from the recording's first message, which Foxglove's elapsed time also uses. (at `start = recording_start_s(path)`)

