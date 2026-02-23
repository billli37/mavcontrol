import asyncio

from constants import LANDING_DESCENT_SPEED_MPS, LANDING_DESCENT_TIMEOUT_S, LANDING_SWITCH_ALT_M, LOOP_HZ


async def run_pid_landing(controller):
    print("[FLIGHT] PID landing phase started")

    dt = 1.0 / LOOP_HZ
    loop = asyncio.get_running_loop()
    end_time = loop.time() + LANDING_DESCENT_TIMEOUT_S

    while loop.time() < end_time:
        tick_start = loop.time()

        offset = await controller.vision.get_offset(dt=dt)

        current_alt_m = await controller.get_altitude()
        if current_alt_m is not None and current_alt_m <= LANDING_SWITCH_ALT_M:
            await controller.set_velocity_body(0.0, 0.0, 0.0, 0.0)
            print(f"[PID_LANDING] Switch altitude reached ({current_alt_m:.2f}m)")
            break

        await controller.set_velocity_body(offset.vx, offset.vy, LANDING_DESCENT_SPEED_MPS, 0.0)

        controller.vision.render(
            offset,
            status_lines=[
                "phase=PID_LANDING",
                f"alt={controller._fmt_alt(current_alt_m)} switch={LANDING_SWITCH_ALT_M:.2f}m",
                f"vz={LANDING_DESCENT_SPEED_MPS:.2f}",
            ],
        )

        print(
            f"[PID_LANDING] visible={offset.target_visible} alt={current_alt_m} "
            f"vx={offset.vx:.2f} vy={offset.vy:.2f} vz={LANDING_DESCENT_SPEED_MPS:.2f}"
        )

        remaining = dt - (loop.time() - tick_start)
        if remaining > 0:
            await asyncio.sleep(remaining)

    await controller.set_velocity_body(0.0, 0.0, 0.0, 0.0)
    raise RuntimeError("[PID_LANDING] timeout: switch altitude not reached before descent timeout")
