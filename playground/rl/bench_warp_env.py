"""Policy steps per second of the Warp environment at 2,048, 4,096 and 8,192 worlds."""

from __future__ import annotations

import argparse
import time

import torch

from auto_battlebot.rl.envs.warp_nav import WarpNavConfig, WarpNavEnv


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawTextHelpFormatter)
    ap.add_argument("--worlds", type=int, nargs="*", default=[2048, 4096, 8192])
    ap.add_argument("--stages", nargs="*", default=["A", "D"])
    ap.add_argument("--steps", type=int, default=60)
    args = ap.parse_args()
    for stage in args.stages:
        for worlds in args.worlds:
            env = WarpNavEnv(WarpNavConfig(worlds=worlds, stage=stage), "cuda", seed=1)
            env.reset(1)
            env.desynchronize()
            for _ in range(10):
                env.step(torch.rand(worlds, 2, device="cuda") * 2 - 1)
            torch.cuda.synchronize()
            start = time.time()
            for _ in range(args.steps):
                env.step(torch.rand(worlds, 2, device="cuda") * 2 - 1)
            torch.cuda.synchronize()
            rate = args.steps * worlds / (time.time() - start)
            print(f"stage {stage} worlds {worlds:5d}: {rate:9,.0f} policy steps/s", flush=True)
            del env
            torch.cuda.empty_cache()


if __name__ == "__main__":
    main()
