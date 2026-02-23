#!/usr/bin/env python3

import asyncio

from mavsdk import System
from mavsdk.offboard import OffboardError, PositionNedYaw, VelocityBodyYawspeed

from constants import (
    ALTITUDE_SAMPLE_TIMEOUT_S,
    CONNECTION_STRING,
    PWM_CHANNEL,
    PWM_CHIP,
    STEPS_0_DEG,
    STEPS_180_DEG,
)
from flightstate.alignment import run_alignment
from flightstate.landing import run_landing
from flightstate.pid_landing import run_pid_landing
from flightstate.takeoff import run_takeoff
from servo import ServoManager
from vision.controller import VisionController


class FlightController:
    def __init__(self):
        self.drone = System()
        self.offboard_started = False
        self.armed = False
        self.land_command_sent = False

        self.relative_alt_m = None
        self._altitude_task = None

        self.servo_mgr = ServoManager(
            PWM_CHIP=PWM_CHIP,
            PWM_CHANNEL=PWM_CHANNEL,
            STEPS_0_DEG=STEPS_0_DEG,
            STEPS_180_DEG=STEPS_180_DEG,
        )
        self.vision = VisionController(preview_enabled=True)

    async def connect_and_wait_ready(self):
        print(f"[MAVSDK] Connecting to {CONNECTION_STRING}")
        await self.drone.connect(system_address=CONNECTION_STRING)

        async for state in self.drone.core.connection_state():
            if state.is_connected:
                print("[MAVSDK] Connected")
                break

        print("[MAVSDK] Waiting for sensor calibration")
        async for health in self.drone.telemetry.health():
            if (
                health.is_gyrometer_calibration_ok
                and health.is_accelerometer_calibration_ok
                and health.is_magnetometer_calibration_ok
            ):
                print("[MAVSDK] Sensors OK")
                break

        print("[MAVSDK] Waiting for local position")
        async for health in self.drone.telemetry.health():
            if health.is_local_position_ok:
                print("[MAVSDK] Local position OK")
                break

    async def set_velocity_body(self, vx: float, vy: float, vz: float = 0.0, yaw_rate: float = 0.0):
        await self.drone.offboard.set_velocity_body(VelocityBodyYawspeed(vx, vy, vz, yaw_rate))

    async def set_position_ned(self, north_m: float, east_m: float, down_m: float):
        await self.drone.offboard.set_position_ned(PositionNedYaw(north_m, east_m, down_m, 0.0))

    async def arm_and_start_offboard(self):
        print("[FLIGHT] Arming")
        await self.drone.action.arm()
        self.armed = True

        await self.set_position_ned(0.0, 0.0, 0.0)

        try:
            await self.drone.offboard.start()
            self.offboard_started = True
            print("[FLIGHT] Offboard started")
        except OffboardError as exc:
            raise RuntimeError(f"Offboard start failed: {exc}") from exc

    async def _altitude_tracking_loop(self):
        try:
            async for pos_vel in self.drone.telemetry.position_velocity_ned():
                self.relative_alt_m = -float(pos_vel.position.down_m)
        except asyncio.CancelledError:
            raise
        except Exception as exc:
            print(f"[ALT] altitude tracking stopped: {exc}")

    async def _start_altitude_tracking(self):
        if self._altitude_task is None or self._altitude_task.done():
            self._altitude_task = asyncio.create_task(self._altitude_tracking_loop())

    async def _stop_altitude_tracking(self):
        if self._altitude_task is None:
            return

        self._altitude_task.cancel()
        try:
            await self._altitude_task
        except asyncio.CancelledError:
            pass
        finally:
            self._altitude_task = None

    async def _wait_for_altitude_sample(self, timeout_s: float):
        loop = asyncio.get_running_loop()
        start = loop.time()
        while self.relative_alt_m is None:
            if loop.time() - start > timeout_s:
                raise RuntimeError("[ALT] timeout waiting for first altitude sample")
            await asyncio.sleep(0.05)

    @staticmethod
    def _fmt_alt(alt_m):
        return "None" if alt_m is None else f"{alt_m:.2f}m"

    async def land_and_shutdown(self):
        try:
            if self.offboard_started:
                await self.set_velocity_body(0.0, 0.0, 0.0, 0.0)
        except Exception:
            pass

        if self.offboard_started:
            try:
                print("[FLIGHT] Stopping offboard")
                await self.drone.offboard.stop()
            except Exception as exc:
                print(f"[FLIGHT] Offboard stop failed: {exc}")
            finally:
                self.offboard_started = False

        if self.armed and not self.land_command_sent:
            await run_landing(self)

        await self._stop_altitude_tracking()
        self.servo_mgr.close()
        await self.vision.stop()

    async def run(self):
        await self.vision.start()

        try:
            await self.connect_and_wait_ready()
            await self._start_altitude_tracking()
            await self._wait_for_altitude_sample(ALTITUDE_SAMPLE_TIMEOUT_S)
            await self.arm_and_start_offboard()
            await run_takeoff(self)
            await run_alignment(self)
            await run_pid_landing(self)
            await run_landing(self)
        except KeyboardInterrupt:
            print("[SYSTEM] Keyboard interrupt")
        except Exception as exc:
            print(f"[SYSTEM] Error: {exc}")
        finally:
            await self.land_and_shutdown()
