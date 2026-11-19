# RL policy hazard barrier (lockstep episodes)

`RlPolicyNavigation` can run `MotionProfileNavigation`'s hazard velocity barrier (`HazardAvoidance::limit_command`) on the policy's command every tick. It is off by default, so a policy trained to see hazards only as observations drives unedited. The barrier needs the top-level `[plant]` table for the stopping envelope.

Mrs Buff Mk3's policies still drove into holes in about 2 of 128 lockstep episodes after the dense near-hole penalty (queue jobs 62 and 64). The motion profile with this barrier dropped none.
