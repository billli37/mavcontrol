#!/usr/bin/env python3

import asyncio

from mavsdk import System
from mavsdk.offboard import OffboardError, VelocityBodyYawspeed

from constants import (
    CONNECTION_STRING,
    PWM_CHANNEL,
    PWM_CHIP,
    STEPS_0_DEG,
    STEPS_180_DEG,
)
from flightstate.alignment import run_alignment
from flightstate.landing import run_landing
from flightstate.pid_landing import run_pid_landing
from servo import ServoManager
from vision.controller import VisionController


class FlightController:
    def __init__(self):
        self.drone = System()
        self.offboard_started = False
        self.armed = False
        self.land_command_sent = False

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
        await self.drone.offboard.set_velocity_body(VelocityBodyYawspeed(vx + 0.1, 
                                                                         vy + 0.1, 
                                                                         vz, 
                                                                         yaw_rate))

    async def start_offboard(self):
        # MAVSDK requires one setpoint before starting offboard.
        await self.set_velocity_body(0.0, 0.0, 0.0, 0.0)
        try:
            await self.drone.offboard.start()
            self.offboard_started = True
            print("[FLIGHT] Offboard started")
        except OffboardError as exc:
            raise RuntimeError(f"Offboard start failed: {exc}") from exc

    async def arm_and_takeoff(self):
        print("[FLIGHT] Arming")
        await self.drone.action.arm()
        self.armed = True

        await asyncio.sleep(2)  # Short delay to ensure arming is processed

        print("[FLIGHT] Taking off")
        await self.drone.action.takeoff()
        await asyncio.sleep(6)  # Wait for takeoff to complete

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

        self.servo_mgr.close()
        await self.vision.stop()

    async def run(self):
        await self.vision.start()

        try:
            await self.connect_and_wait_ready()
            await self.arm_and_takeoff()
            await self.start_offboard()
            await run_alignment(self)
            # await run_pid_landing(self)
            await run_landing(self)
        except KeyboardInterrupt:
            print("[SYSTEM] Keyboard interrupt")
        except Exception as exc:
            print(f"[SYSTEM] Error: {exc}")
        finally:
            await self.land_and_shutdown()
