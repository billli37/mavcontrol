#!/usr/bin/env python3

import asyncio
import numpy as np

from mavsdk import System
from mavsdk.offboard import OffboardError, VelocityBodyYawspeed

from camera import Camera
from color import ColorDetector
from pid import PID
from renderer import PreviewRenderer
from servo import ServoManager
from constants import (
    CONNECTION_STRING,
    TAKEOFF_ALT_M,
    TAKEOFF_ALT_TOL_M,
    TAKEOFF_HOLD_TIME_S,
    TAKEOFF_TIMEOUT_S,
    ALIGNMENT_TIMEOUT_S,
    LOOP_HZ,
    MAX_SPEED_MPS,
    ALIGN_PID_KP,
    ALIGN_PID_KI,
    ALIGN_PID_KD,
    ALT_PID_KP,
    ALT_PID_KI,
    ALT_PID_KD,
    ALTITUDE_SAMPLE_TIMEOUT_S,
    MAX_CLIMB_MPS,
    MAX_DESCENT_MPS,
    LANDING_SWITCH_ALT_M,
    LANDING_DESCENT_TARGET_M,
    LANDING_DESCENT_TIMEOUT_S,
    PWM_CHANNEL,
    PWM_CHIP,
    STEPS_0_DEG,
    STEPS_180_DEG,
    ANGLE_LANDING,
)


def clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


class FlightController:
    def __init__(self):
        self.drone = System()
        self.offboard_started = False
        self.armed = False
        self.target_visible = False
        self.land_command_sent = False

        self.relative_alt_m = None
        self._altitude_task = None

        self.servo_mgr = ServoManager(
            PWM_CHIP=PWM_CHIP,
            PWM_CHANNEL=PWM_CHANNEL,
            STEPS_0_DEG=STEPS_0_DEG,
            STEPS_180_DEG=STEPS_180_DEG,
            ANGLE_LANDING=ANGLE_LANDING,
        )

        self.color_det = ColorDetector()
        self.pid_controller = PID(
            kp=ALIGN_PID_KP,
            ki=ALIGN_PID_KI,
            kd=ALIGN_PID_KD,
            target=(0.0, 0.0),
        )
        self.pid_controller.vmax = MAX_SPEED_MPS

        self.alt_pid = PID(
            kp=ALT_PID_KP,
            ki=ALT_PID_KI,
            kd=ALT_PID_KD,
            target=(TAKEOFF_ALT_M,),
        )

        self.camera = Camera()
        self.renderer = PreviewRenderer(window_name="Red Target Detection", enabled=True)

    async def _track_relative_altitude(self):
        try:
            async for pos_vel in self.drone.telemetry.position_velocity_ned():
                alt_m = float(-pos_vel.position.down_m)
                if np.isfinite(alt_m):
                    self.relative_alt_m = alt_m
        except asyncio.CancelledError:
            raise
        except Exception as exc:
            print(f"[ALT] Telemetry task error: {exc}")

    async def _start_altitude_tracking(self):
        if self._altitude_task is None:
            self._altitude_task = asyncio.create_task(self._track_relative_altitude())

    async def _wait_for_altitude_sample(self, timeout_s: float):
        loop = asyncio.get_running_loop()
        start_time = loop.time()
        while loop.time() - start_time <= timeout_s:
            if self.relative_alt_m is not None:
                print(f"[ALT] First local altitude sample: {self.relative_alt_m:.3f}m")
                return
            await asyncio.sleep(0.05)
        raise RuntimeError(f"[ALT] timeout: no local altitude sample within {timeout_s:.1f}s")

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
        await self.drone.offboard.set_velocity_body(
            VelocityBodyYawspeed(vx, vy, vz, yaw_rate)
        )

    async def arm_and_start_offboard(self):
        print("[FLIGHT] Arming")
        await self.drone.action.arm()
        self.armed = True

        await self.set_velocity_body(0.0, 0.0, 0.0, 0.0)
        try:
            await self.drone.offboard.start()
            self.offboard_started = True
            print("[FLIGHT] Offboard started")
        except OffboardError as exc:
            raise RuntimeError(f"Offboard start failed: {exc}") from exc

    async def _compute_xy_from_color(self, frame, dt: float):
        vx = 0.0
        vy = 0.0
        err_x_px = None
        err_y_px = None
        red_mask = None
        centroid = None
        contour = None
        contour_area = 0.0

        if frame is None:
            self.target_visible = False
            self.pid_controller.reset(target=(0.0, 0.0))
            return (
                self.target_visible,
                err_x_px,
                err_y_px,
                vx,
                vy,
                red_mask,
                centroid,
                contour,
                contour_area,
            )

        (
            err_x_px,
            err_y_px,
            red_mask,
            centroid,
            contour,
            contour_area,
        ) = await asyncio.to_thread(self.color_det.detect_with_debug, frame)

        if err_x_px is not None and err_y_px is not None:
            self.target_visible = True
            measurement = np.array([-err_x_px, -err_y_px], dtype=float)
            pid_out = self.pid_controller.update(measurement, dt)
            vx = clamp(float(-pid_out[1]), -MAX_SPEED_MPS, MAX_SPEED_MPS)
            vy = clamp(float(pid_out[0]), -MAX_SPEED_MPS, MAX_SPEED_MPS)
        else:
            self.target_visible = False
            self.pid_controller.reset(target=(0.0, 0.0))
            contour_area = contour_area if contour_area is not None else 0.0

        return (
            self.target_visible,
            err_x_px,
            err_y_px,
            vx,
            vy,
            red_mask,
            centroid,
            contour,
            contour_area,
        )

    def _compute_vz_from_altitude(self, current_alt_m, target_alt_m: float, dt: float):
        if current_alt_m is None:
            self.alt_pid.reset(target=(target_alt_m,))
            return 0.0, None

        self.alt_pid.target = np.asarray([target_alt_m], dtype=float)
        pid_out = self.alt_pid.update(np.asarray([current_alt_m], dtype=float), dt)

        vz = float(-pid_out[0])
        vz = clamp(vz, -MAX_CLIMB_MPS, MAX_DESCENT_MPS)
        alt_err = float(target_alt_m - current_alt_m)
        return vz, alt_err

    def _render_status_lines(self, phase: str, alt_m, alt_err, vz: float):
        alt_text = "None" if alt_m is None else f"{alt_m:.2f}m"
        err_text = "None" if alt_err is None else f"{alt_err:.2f}m"
        return [
            f"phase={phase}",
            f"alt={alt_text} alt_err={err_text} vz={vz:.2f}",
        ]

    async def takeoff_phase(self):
        print(f"[FLIGHT] Takeoff phase started target={TAKEOFF_ALT_M:.2f}m")
        self.servo_mgr.move(ANGLE_LANDING, "DOWN")

        dt = 1.0 / LOOP_HZ
        hold_accum_s = 0.0
        loop = asyncio.get_running_loop()
        start_time = loop.time()

        while True:
            if loop.time() - start_time > TAKEOFF_TIMEOUT_S:
                raise RuntimeError("[TAKEOFF] timeout: altitude target not reached")

            tick_start = loop.time()
            frame = await self.camera.capture_frame()

            (
                _,
                err_x_px,
                err_y_px,
                vx,
                vy,
                red_mask,
                centroid,
                contour,
                contour_area,
            ) = await self._compute_xy_from_color(frame, dt)

            vz, alt_err = self._compute_vz_from_altitude(self.relative_alt_m, TAKEOFF_ALT_M, dt)

            if alt_err is not None and abs(alt_err) <= TAKEOFF_ALT_TOL_M:
                hold_accum_s += dt
            else:
                hold_accum_s = 0.0

            if frame is not None:
                self.renderer.render(
                    frame,
                    self.target_visible,
                    err_x_px,
                    err_y_px,
                    vx,
                    vy,
                    red_mask,
                    centroid,
                    contour,
                    contour_area,
                    status_lines=self._render_status_lines("TAKEOFF", self.relative_alt_m, alt_err, vz),
                )

            await self.set_velocity_body(vx, vy, vz, 0.0)
            print(
                f"[TAKEOFF] visible={self.target_visible} alt={self.relative_alt_m} "
                f"alt_err={alt_err} vx={vx:.2f} vy={vy:.2f} vz={vz:.2f}"
            )

            if hold_accum_s >= TAKEOFF_HOLD_TIME_S:
                print("[TAKEOFF] Altitude reached and stabilized")
                break

            remaining = dt - (loop.time() - tick_start)
            if remaining > 0:
                await asyncio.sleep(remaining)

    async def alignment_phase(self):
        print("[FLIGHT] Alignment phase started")
        dt = 1.0 / LOOP_HZ
        loop = asyncio.get_running_loop()
        end_time = loop.time() + ALIGNMENT_TIMEOUT_S

        while loop.time() < end_time:
            tick_start = loop.time()
            frame = await self.camera.capture_frame()

            (
                _,
                err_x_px,
                err_y_px,
                vx,
                vy,
                red_mask,
                centroid,
                contour,
                contour_area,
            ) = await self._compute_xy_from_color(frame, dt)

            if frame is not None:
                self.renderer.render(
                    frame,
                    self.target_visible,
                    err_x_px,
                    err_y_px,
                    vx,
                    vy,
                    red_mask,
                    centroid,
                    contour,
                    contour_area,
                    status_lines=self._render_status_lines("ALIGN", self.relative_alt_m, None, 0.0),
                )

            await self.set_velocity_body(vx, vy, 0.0, 0.0)
            print(f"[ALIGN] visible={self.target_visible} vx={vx:.2f} vy={vy:.2f}")

            remaining = dt - (loop.time() - tick_start)
            if remaining > 0:
                await asyncio.sleep(remaining)

    async def landing_phase(self):
        print(f"[FLIGHT] Landing descent phase started target={LANDING_DESCENT_TARGET_M:.2f}m")
        dt = 1.0 / LOOP_HZ
        loop = asyncio.get_running_loop()
        end_time = loop.time() + LANDING_DESCENT_TIMEOUT_S

        while loop.time() < end_time:
            tick_start = loop.time()
            frame = await self.camera.capture_frame()

            (
                _,
                err_x_px,
                err_y_px,
                vx,
                vy,
                red_mask,
                centroid,
                contour,
                contour_area,
            ) = await self._compute_xy_from_color(frame, dt)

            current_alt_m = self.relative_alt_m
            alt_err = None

            if current_alt_m is None:
                vz = 0.0
                self.alt_pid.reset(target=(LANDING_DESCENT_TARGET_M,))
            elif current_alt_m <= LANDING_DESCENT_TARGET_M:
                vz = 0.0
                alt_err = LANDING_DESCENT_TARGET_M - current_alt_m
            else:
                vz, alt_err = self._compute_vz_from_altitude(current_alt_m, LANDING_DESCENT_TARGET_M, dt)
                vz = max(0.0, vz)

            if frame is not None:
                self.renderer.render(
                    frame,
                    self.target_visible,
                    err_x_px,
                    err_y_px,
                    vx,
                    vy,
                    red_mask,
                    centroid,
                    contour,
                    contour_area,
                    status_lines=self._render_status_lines("LAND_DESCENT", current_alt_m, alt_err, vz),
                )

            await self.set_velocity_body(vx, vy, vz, 0.0)
            print(
                f"[LAND_DESCENT] visible={self.target_visible} alt={current_alt_m} "
                f"alt_err={alt_err} vx={vx:.2f} vy={vy:.2f} vz={vz:.2f}"
            )

            if current_alt_m is not None and current_alt_m <= LANDING_SWITCH_ALT_M:
                print(f"[LAND_DESCENT] Switch altitude reached ({current_alt_m:.2f}m)")
                break

            remaining = dt - (loop.time() - tick_start)
            if remaining > 0:
                await asyncio.sleep(remaining)

        await self.set_velocity_body(0.0, 0.0, 0.0, 0.0)

        if self.armed and not self.land_command_sent:
            try:
                print("[FLIGHT] Landing")
                await self.drone.action.land()
                self.land_command_sent = True
            except Exception as exc:
                print(f"[FLIGHT] Landing command failed: {exc}")

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
            try:
                print("[FLIGHT] Landing")
                await self.drone.action.land()
                self.land_command_sent = True
            except Exception as exc:
                print(f"[FLIGHT] Landing command failed: {exc}")

        await self._stop_altitude_tracking()
        self.renderer.close_window()
        self.servo_mgr.close()
        await self.camera.stop_camera()

    async def run(self):
        await self.camera.start_camera()
        self.renderer.init_window()
        try:
            await self.connect_and_wait_ready()
            await self._start_altitude_tracking()
            await self._wait_for_altitude_sample(ALTITUDE_SAMPLE_TIMEOUT_S)
            # await self.arm_and_start_offboard()
            # await self.takeoff_phase()
            # await self.alignment_phase()
            # await self.landing_phase()
        except KeyboardInterrupt:
            print("[SYSTEM] Keyboard interrupt")
        except Exception as exc:
            print(f"[SYSTEM] Error: {exc}")
        finally:
            await self.land_and_shutdown()
            self.renderer.close_window()
