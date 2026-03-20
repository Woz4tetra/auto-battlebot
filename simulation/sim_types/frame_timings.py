from __future__ import annotations

from dataclasses import dataclass


@dataclass
class FrameTimings:
    """Cumulative timings for profiling the serve loop."""

    recv: float = 0.0
    physics: float = 0.0
    render: float = 0.0
    process: float = 0.0
    send: float = 0.0
    count: int = 0

    def record(
        self,
        t_recv: float,
        t_physics: float,
        t_render: float,
        t_process: float,
        t_send: float,
    ) -> None:
        self.recv += t_recv
        self.physics += t_physics
        self.render += t_render
        self.process += t_process
        self.send += t_send
        self.count += 1

    def report(self, physics_steps: int, frame_total: float) -> str:
        n = self.count
        return (
            f"[frame {n}] avg ms/frame: "
            f"recv={1000 * self.recv / n:.1f}  "
            f"physics={1000 * self.physics / n:.1f} ({physics_steps} steps)  "
            f"render={1000 * self.render / n:.1f}  "
            f"process={1000 * self.process / n:.1f}  "
            f"send={1000 * self.send / n:.1f}  "
            f"total={1000 * frame_total:.1f}"
        )
