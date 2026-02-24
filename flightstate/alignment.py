import asyncio

from constants import ALIGNMENT_TIMEOUT_S, CUTOFF_ALT_M, LOOP_HZ


async def run_alignment(controller):
    print("[FLIGHT] Alignment phase started")

    dt = 1.0 / LOOP_HZ
    loop = asyncio.get_running_loop()
    end_time = loop.time() + ALIGNMENT_TIMEOUT_S

    aligned_start_time = None
    descending = False

    while loop.time() < end_time:
        tick_start = loop.time()

        offset = await controller.vision.get_offset(dt=dt)
        alt = await controller.get_altitude()
        
        if descending and alt <= CUTOFF_ALT_M:
            print(f"[ALIGN] Cutoff altitude reached ({alt:.2f}m), disarming")
            await controller.drone.action.disarm()
            break
        
        is_aligned = offset.target_visible and abs(offset.err_x) < 20 and abs(offset.err_y) < 20
        
        if is_aligned:
            if aligned_start_time is None:
                aligned_start_time = loop.time()
            elif not descending and (loop.time() - aligned_start_time) >= 5.0:
                descending = True
                print("[ALIGN] Aligned for 5s, starting descent")
        else:
            aligned_start_time = None
            descending = False
        
        vz = -0.1 if descending else 0.0
        await controller.set_velocity_body(offset.vx, offset.vy, vz, 0.0)

        controller.vision.render(
            offset,
            status_lines=[
                "phase=ALIGN",
                f"alt=none",
            ],
        )

        print(f"[ALIGN] visible={offset.target_visible} vx={offset.vx:.2f} vy={offset.vy:.2f} vz={vz:.2f} aligned={is_aligned}")

        remaining = dt - (loop.time() - tick_start)
        if remaining > 0:
            await asyncio.sleep(remaining)
