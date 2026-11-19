# Opponent no-push-through rule

Date is the commit that added the rule (`ec263233`, 2026-10-17). The measurements were written in the code comment at that commit and are not re-run here.

- Bug: the kinematic opponent is a mocap body with infinite mass. Driven into our wedge it could not ride up it, so it pried our chassis over.
- Measurement: an idle robot flipped in 66 of 80 early-ending stage C episodes.
- Fix: cap the opponent's closing speed so it arrives at contact and goes no further (`CONTACT_PRESS_M`). Contact is the two footprint boxes' extents along the line between centres.
- Earlier rule: two circles. It stopped the opponent 15 cm off a nose-on robot, out of contact for good.
- Test: `tests/python/test_rl_core.py::test_opponent_does_not_push_through_us`.
- Rule description: `docs/reference/control/rl_opponents.md`.
