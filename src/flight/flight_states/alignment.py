from __future__ import annotations

import asyncio

from flight.flight_states.common import AlignPolicyInput, compute_xy_from_vision
from flight.state import StaleVisionMode


async def run(controller: "FlightController") -> None:
    controller.logger.info("[FLIGHT] Alignment phase started")

    dt = 1.0 / controller.cfg.runtime.loop_hz
    loop = asyncio.get_running_loop()
    end_time = loop.time() + controller.cfg.alignment.timeout_s
    next_log = loop.time()
    
    aligned_start_time = None
    descending = False

    while loop.time() < end_time:
        tick_start = loop.time()
        out = compute_xy_from_vision(
            AlignPolicyInput(
                sample=controller.latest_vision_sample,
                vision_age_s=controller.get_vision_age_s(),
                dt=dt,
                max_speed_mps=controller.cfg.runtime.max_speed_mps,
                sample_ttl_s=controller.cfg.ipc.sample_ttl_s,
                drop_to_zero_after_s=controller.cfg.ipc.drop_to_zero_after_s,
            ),
            state=controller.vision_state,
            pid=controller.pid_controller,
        )

        if out.mode == StaleVisionMode.FRESH:
            controller.vision_state.last_valid_target_seen_time = loop.time()

        controller.vision_state.target_visible = out.target_visible
        
        is_aligned = (out.target_visible and out.err_x_px is not None and out.err_y_px is not None 
                      and abs(out.err_x_px) < 20 and abs(out.err_y_px) < 20)
        
        if is_aligned:
            if aligned_start_time is None:
                aligned_start_time = loop.time()
            elif not descending and (loop.time() - aligned_start_time) >= 5.0:
                descending = True
                controller.logger.info("[ALIGN] Aligned for 5s, starting descent")
        else:
            aligned_start_time = None
            descending = False
        
        cmd = out.cmd
        if descending:
            cmd.vz = -0.1
        
        controller.set_desired_velocity(cmd)
        controller.publish_control_status()

        if loop.time() >= next_log:
            controller.logger.info(
                "[ALIGN] mode=%s visible=%s alt=%.2f vx=%.2f vy=%.2f vz=%.2f err_x=%s err_y=%s aligned=%s",
                out.mode.value,
                controller.vision_state.target_visible,
                controller.telemetry.get_altitude(),
                cmd.vx,
                cmd.vy,
                cmd.vz,
                "None" if out.err_x_px is None else f"{out.err_x_px:.1f}",
                "None" if out.err_y_px is None else f"{out.err_y_px:.1f}",
                is_aligned,
            )
            next_log = loop.time() + 0.5

        remaining = dt - (loop.time() - tick_start)
        if remaining > 0:
            await asyncio.sleep(remaining)


from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from flight.controller import FlightController
