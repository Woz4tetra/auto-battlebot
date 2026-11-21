# RL nav scoring details

## Field session scoring

Score field RL sessions with the sim's episode metrics (`auto_battlebot/rl/metrics.py`).

Reads both robots' field poses from `/status/tracks` and scores only the time autonomy was engaged (`runner/autonomy_on_s` above zero). Contact is inferred from the tracked poses, not measured, so the counts are approximate: the robots touch when their centers are under `--contact-m` apart.

- Strike: contact, the opponent inside our wedge cone, and we are outside the opponent's arc.
- Hit taken: contact while we are inside the opponent's arc.
- Danger: within `--contact-m` plus `WarpNavConfig.danger_margin_m` and inside the arc.
- Contacts closer than `WarpNavConfig.engagement_gap_s` apart merge into one event.

The contact and wedge thresholds are unvalidated against video. Check strike and hit timestamps in Foxglove before trusting the strikes-per-hit ratio.

```
    source scripts/activate_python.sh
    python -m auto_battlebot.rl.score_field_session data/recordings/<session>.mcap [...]
    python -m auto_battlebot.rl.score_field_session data/recordings/<session>.mcap --clips hit
```
