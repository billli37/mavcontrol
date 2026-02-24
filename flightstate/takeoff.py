import asyncio

from constants import ANGLE_LANDING, LOOP_HZ, TAKEOFF_ALT_M, TAKEOFF_ALT_TOL_M, TAKEOFF_TIMEOUT_S


async def run_takeoff(controller):
    print(f"[FLIGHT] Takeoff phase started target={TAKEOFF_ALT_M:.2f}m")
    controller.servo_mgr.move(ANGLE_LANDING, "DOWN")

    # Capture starting position
    start_n, start_e, _ = await controller.get_position_ned()
    print(f"[TAKEOFF] Starting position: N={start_n:.2f} E={start_e:.2f}")

    dt = 1.0 / LOOP_HZ
    loop = asyncio.get_running_loop()
    start_time = loop.time()

    while True:
        if loop.time() - start_time > TAKEOFF_TIMEOUT_S:
            raise RuntimeError("[TAKEOFF] timeout: altitude target not reached")
        
        await controller.set_position_ned(start_n, start_e, -TAKEOFF_ALT_M)

        alt_m = await controller.get_altitude()
        alt_err = None if alt_m is None else TAKEOFF_ALT_M - alt_m
        if alt_err is not None and abs(alt_err) <= TAKEOFF_ALT_TOL_M:
            print(f"[TAKEOFF] Altitude reached ({alt_m:.2f}m)")
            break

        print(f"[TAKEOFF] alt={alt_m} alt_err={alt_err}")
        await asyncio.sleep(dt)
