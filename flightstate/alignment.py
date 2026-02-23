import asyncio

from constants import ALIGNMENT_TIMEOUT_S, LOOP_HZ


async def run_alignment(controller):
    print("[FLIGHT] Alignment phase started")

    dt = 1.0 / LOOP_HZ
    loop = asyncio.get_running_loop()
    end_time = loop.time() + ALIGNMENT_TIMEOUT_S

    while loop.time() < end_time:
        tick_start = loop.time()

        offset = await controller.vision.get_offset(dt=dt)
        current_alt_m = await controller.get_altitude()
        await controller.set_velocity_body(offset.vx, offset.vy, 0.0, 0.0)

        controller.vision.render(
            offset,
            status_lines=[
                "phase=ALIGN",
                f"alt={controller._fmt_alt(current_alt_m)}",
            ],
        )

        print(f"[ALIGN] visible={offset.target_visible} vx={offset.vx:.2f} vy={offset.vy:.2f}")

        remaining = dt - (loop.time() - tick_start)
        if remaining > 0:
            await asyncio.sleep(remaining)

    await controller.set_velocity_body(0.0, 0.0, 0.0, 0.0)
    print("[ALIGN] timeout reached; holding position and continuing")
