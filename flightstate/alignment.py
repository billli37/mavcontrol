import asyncio

from constants import ALIGNMENT_TIMEOUT_S, LOOP_HZ


async def run_alignment(controller):
    print("[FLIGHT] Alignment phase started")

    dt = 1.0 / LOOP_HZ
    loop = asyncio.get_running_loop()
    end_time = loop.time() + ALIGNMENT_TIMEOUT_S

    hold_position = None

    while loop.time() < end_time:
        tick_start = loop.time()

        offset = await controller.vision.get_offset(dt=dt)
        
        if offset.target_visible:
            await controller.set_velocity_body(offset.vx, offset.vy, 0.0, 0.0)
            hold_position = None
        else:
            if hold_position is None:
                n, e, d = await controller.get_position_ned()
                hold_position = (n, e, d)
                print(f"[ALIGN] Target lost, holding position: N={n:.2f} E={e:.2f} D={d:.2f}")
            await controller.set_position_ned(hold_position[0], hold_position[1], hold_position[2])

        controller.vision.render(
            offset,
            status_lines=[
                "phase=ALIGN",
                f"alt=none",
            ],
        )

        print(f"[ALIGN] visible={offset.target_visible} vx={offset.vx:.2f} vy={offset.vy:.2f}")

        remaining = dt - (loop.time() - tick_start)
        if remaining > 0:
            await asyncio.sleep(remaining)

    if hold_position is None:
        n, e, d = await controller.get_position_ned()
        hold_position = (n, e, d)
    await controller.set_position_ned(hold_position[0], hold_position[1], hold_position[2])
    print("[ALIGN] timeout reached; holding position and continuing")
