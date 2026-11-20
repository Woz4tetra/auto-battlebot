# RL toy run configs

`training/rl/train_nav_policy.py` reads the TOMLs in `playground/rl_toy/configs/`. Field meanings are in `docs/reference/control/rl_training_config.md`; the training loop is in `docs/reference/control/rl_training.md`. Each file is one arm of the toy viability test, all against a static opponent with 2048 eval episodes at seed 12345.

| File | Arm |
| --- | --- |
| `ppo_static.toml` | Questions 1 and 2: PPO on true state, 4096 worlds, 1200 s budget. |
| `ppo_static_sensing.toml` | Question 3: PPO with `sensing = true`, which turns on `MeasurementModel`, `OurEkfErrorModel` and the opponent filter mirror, parameterized by `simulation/sensing_models/provisional_rl.toml`. 2400 s budget. |
| `fasttd3_static.toml` | Question 4: FastTD3 on true state, same scenarios and budget as `ppo_static.toml`. |
| `pursuit_static.toml` | Scripted baseline: `PursuitNavigation`'s law with the Mr Stabs Mk2 profile gains (max linear command 0.25). |
| `pursuit_fast_static.toml` | Scripted baseline at full speed: the same law with `max_linear_command = 1.0`. |
