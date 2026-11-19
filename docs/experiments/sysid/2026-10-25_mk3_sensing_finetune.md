# Mk3 sensing fine-tune, 2026-10-25

Notes on training the Mrs Buff Mk3 navigation policy with the fitted EKF error model.

## Details

### `rl/envs/warp_nav.py`

#### Fine-tuning with the Mk3 sensing model

The Mrs Buff Mk3 profile does not use `MK3_SENSING`. Most of its heading error comes from the
sim's own spin-rate mismatch (the filter's jig plant spins 21% faster than the MuJoCo plant),
and two fine-tunes on it (queue jobs 60 and 61, 2026-10-25) stopped attacking: lockstep hits per
hit taken fell from 0.33 to 0.02. Use it once the MuJoCo fit matches the jig plant's spin.
