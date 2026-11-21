# Forward step test

## Forward step test (`playground/nav_eval/step_test.py`)

Nav scoring plan section 0.2. The user drives 0.5, 0.6 and 0.7 forward steps from rest on a soft surface with the slew limit off, recorded to MCAP. This finds each step in the robot's telemetry (`/robot/espnow_telemetry`), measures the nose lift from the BNO055 pitch, replays the recorded stick tape through the closed-loop MuJoCo sim at a grid of armature scales (`auto_battlebot.mujoco_sim.step_response`), and picks the scale that matches. Then the verdict: if the steps nearest 0.6 lift the nose past `--lift-limit-deg`, keep a slew limit and report the loosest one that holds a full-stick step under the limit in the refit sim.

A step: the stick rests (|linear| under 0.05 for 0.5 s), then rises and holds at least 0.25 for 0.4 s. Its size is the median linear over the held part. Steps with |angular| above 0.1 are flagged as turning and left out of the fit.

Pitch is `orientation_y`, degrees, in the sim's convention (rest nose-down, a lifted nose reads smaller). The BNO055 fused pitch drifts in hard spins, which straight steps avoid.

```
    source scripts/activate_python.sh
    python playground/nav_eval/step_test.py data/recordings/<step_test>.mcap --out-dir /tmp/steps
    # the sim side alone, on ideal steps, to see what the current fit predicts
    python playground/nav_eval/step_test.py --ideal 0.5 0.6 0.7 --out-dir /tmp/steps
```

The wheelie is a sharp threshold in slew rate (about 8-12/s at armature 0.45, 4-8/s at 1.10).
