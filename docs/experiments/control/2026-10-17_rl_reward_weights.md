# RL reward weights

Measurements behind the `WarpNavConfig.reward` defaults (`auto_battlebot/rl/envs/warp_nav.py`). Key meanings are in `docs/reference/control/rl_training_config.md`.

- `hole_near_per_s` is 0.0 by default. Mrs Buff Mk3's stage C fine-tune with `hole` -30 still fell in on 0.05 to 0.07 of training episodes, mostly with no hit taken. A terminal penalty alone gives little credit to the steps that drove there.
- `still_per_s` is -0.5. The 2026-10-17 policy parked for whole episodes in a facing opponent's front arc, waiting for it to turn (`docs/experiments/control/rl_navigation_report.md`), and waiting cost nothing. 20 s parked now costs as much as one hit taken.
- `command_change` is -0.05, 5x the plan's -0.01. A stage C evaluation (2026-10-17) found the deterministic policy moving its linear command 0.11 per 20 ms step, 0.18 before a tip, and flipping in 40% of episodes.
