from __future__ import annotations

import asyncio

from comms.protocol import VisionSample
from flight.flight_states import alignment, takeoff
from flight.offboard import OffboardPublisher
from flight.pid import PID
from flight.state import ControlState, MissionPhase, VelocityCommand, VisionState


class FlightController:
    def __init__(self, cfg, logger, flight_link, telemetry, vision_receiver, status_sender, actuator):
        self.cfg = cfg
        self.logger = logger
        self.flight_link = flight_link
        self.telemetry = telemetry
        self.vision_receiver = vision_receiver
        self.status_sender = status_sender
        self.actuator = actuator

        self.state = ControlState()
        self.vision_state = VisionState()

        self.pid_controller = PID(
            kp=self.cfg.alignment.pid_kp,
            ki=self.cfg.alignment.pid_ki,
            kd=self.cfg.alignment.pid_kd,
            target=(0.0, 0.0),
        )
        self.pid_controller.vmax = self.cfg.runtime.max_speed_mps

        self.latest_vision_sample: VisionSample | None = None
        self.latest_vision_rx_time: float | None = None
        self.latest_vision_seq = -1

        self._desired_velocity_cmd = VelocityCommand()
        self._offboard_publisher = OffboardPublisher(
            loop_hz=self.cfg.runtime.loop_hz,
            send_velocity_body=self.flight_link.set_velocity_body,
            get_desired_cmd=self.get_desired_velocity,
            get_vision_age_s=self.get_vision_age_s,
            logger=self.logger,
        )

    def _on_vision_sample(self, sample: VisionSample) -> None:
        if sample.seq <= self.latest_vision_seq:
            return
        self.latest_vision_seq = sample.seq
        self.latest_vision_sample = sample
        self.latest_vision_rx_time = asyncio.get_running_loop().time()

    def set_desired_velocity(self, cmd: VelocityCommand) -> None:
        self._desired_velocity_cmd = cmd

    def get_desired_velocity(self) -> VelocityCommand:
        return self._desired_velocity_cmd

    def get_vision_age_s(self) -> float | None:
        if self.latest_vision_rx_time is None:
            return None
        return asyncio.get_running_loop().time() - self.latest_vision_rx_time

    def publish_control_status(self) -> None:
        cmd = self.get_desired_velocity()
        self.status_sender.publish(
            phase=self.state.phase.value,
            cmd=cmd,
            alt_m=self.telemetry.get_altitude(),
            target_visible=self.vision_state.target_visible,
        )

    async def _connect_arm_offboard(self) -> None:
        await self.flight_link.connect_and_wait_ready(self.cfg.drone.connection_string)
        await self.telemetry.start()
        await self.vision_receiver.start(on_sample=self._on_vision_sample)

        self.logger.info("[FLIGHT] Arming")
        await self.flight_link.arm()
        self.state.armed = True

        self.set_desired_velocity(VelocityCommand())
        await self.flight_link.set_velocity_body(self.get_desired_velocity())
        await self.flight_link.offboard_start()
        self.state.offboard_started = True
        self.logger.info("[FLIGHT] Offboard started")

        await self._offboard_publisher.start()

    async def run(self) -> None:
        try:
            await self._connect_arm_offboard()

            self.state.phase = MissionPhase.TAKEOFF
            await takeoff.run(self)

            self.state.phase = MissionPhase.ALIGN
            await alignment.run(self)
        except KeyboardInterrupt:
            self.logger.info("[SYSTEM] Keyboard interrupt")
        except Exception as exc:
            self.logger.exception("[SYSTEM] Error: %s", exc)
        finally:
            await self.shutdown()

    async def shutdown(self) -> None:
        self.state.phase = MissionPhase.SHUTDOWN
        self.set_desired_velocity(VelocityCommand())

        try:
            await asyncio.sleep(0.1)
        except Exception:
            pass

        await self._offboard_publisher.stop()

        if self.state.offboard_started:
            try:
                await self.flight_link.set_velocity_body(VelocityCommand())
                self.logger.info("[FLIGHT] Stopping offboard")
                await self.flight_link.offboard_stop()
            except Exception as exc:
                self.logger.error("[FLIGHT] Offboard stop failed: %s", exc)
            finally:
                self.state.offboard_started = False

        await self.telemetry.stop()
        self.vision_receiver.stop()
        self.status_sender.close()
        self.actuator.close()
