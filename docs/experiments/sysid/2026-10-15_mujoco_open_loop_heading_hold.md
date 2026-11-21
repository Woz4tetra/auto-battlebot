# MuJoCo open-loop heading hold, 2026-10-15

Open-loop replay drives the plant from the logged per-motor commands, which are not what the robot's heading hold would have commanded. The firmware mixer in closed loop is the alternative, and these notes describe how the two checks in `mujoco_sim/validate.py` are set up.

## Firmware mixer

The firmware mixer takes stick percents, the sensed heading and gyro rate, and returns per-motor percents. In the current firmware the heading and gyro only change when a BNO055 sample lands, every sample period, so the yaw loop runs on samples alone. The PID output goes through the differential mixer: it adds to the left wheel and takes off the right.

Seeding the yaw loop's integral with the whole logged PID output double-counts the feedforward in a turn: a spin on 2026-10-15 then ran 40% faster than the robot did. Resuming seeds only the logged output minus its feedforward and proportional terms.

## Firmware closed loop (check 5)

On CPU MuJoCo, drive the fitted drivetrain from the logged sticks through the firmware mixer with the sim's heading, and compare its outputs with the logged ones.

Only windows where the turn stick stays inside the 1% threshold (straight holds, where the heading hold acts) are used. The sim heading is offset to match the logged BNO055 heading at the window start, so only the change in heading drives the PID.

## Closed-loop replay (check 7)

On CPU MuJoCo, replay each window from the logged sticks through the current firmware mixer, with the yaw loop fed the sim's heading and gyro.

Open-loop replay drives the plant with the logged per-motor commands, which the yaw loop computed against the real robot. Where the loop was holding a heading against a disturbance the plant does not have, the replayed correction turns the sim instead: on 2026-10-15 the heading hold backed up straight with 1.12 to 1.6x more left than right command, and the sim spun up to 400 deg. Here the loop corrects the sim's own heading, as it did the robot's.

Windows recorded before the yaw loop, or without firmware columns, replay open loop.
