import asyncio
from dataclasses import dataclass

import numpy as np

from constants import ALIGN_PID_KD, ALIGN_PID_KI, ALIGN_PID_KP, LOOP_HZ, MAX_SPEED_MPS
from vision.camera import Camera
from vision.color import ColorDetector
from vision.pid import PID
from vision.renderer import PreviewRenderer


def clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


@dataclass
class VisionOffset:
    frame: object
    target_visible: bool
    err_x_px: float | None
    err_y_px: float | None
    vx: float
    vy: float
    red_mask: object
    centroid: tuple[int, int] | None
    contour: object
    contour_area: float


class VisionController:
    def __init__(self, preview_enabled: bool = True):
        self.camera = Camera()
        self.detector = ColorDetector()
        self.renderer = PreviewRenderer(window_name="Red Target Detection", enabled=preview_enabled)

        self.pid = PID(
            kp=ALIGN_PID_KP,
            ki=ALIGN_PID_KI,
            kd=ALIGN_PID_KD,
            target=(0.0, 0.0),
        )
        self.pid.vmax = MAX_SPEED_MPS
        self.target_visible = False

    async def start(self):
        await self.camera.start_camera()

    async def stop(self):
        self.renderer.close_window()
        await self.camera.stop_camera()

    async def get_offset(self, dt: float | None = None) -> VisionOffset:
        if dt is None:
            dt = 1.0 / LOOP_HZ

        frame = await self.camera.capture_frame()
        if frame is None:
            self.target_visible = False
            self.pid.reset(target=(0.0, 0.0))
            return VisionOffset(
                frame=None,
                target_visible=False,
                err_x_px=None,
                err_y_px=None,
                vx=0.0,
                vy=0.0,
                red_mask=None,
                centroid=None,
                contour=None,
                contour_area=0.0,
            )

        (
            err_x_px,
            err_y_px,
            red_mask,
            centroid,
            contour,
            contour_area,
        ) = await asyncio.to_thread(self.detector.detect_with_debug, frame)

        vx = 0.0
        vy = 0.0

        if err_x_px is not None and err_y_px is not None:
            self.target_visible = True
            measurement = np.array([-err_x_px, -err_y_px], dtype=float)
            pid_out = self.pid.update(measurement, dt)
            vx = clamp(float(-pid_out[1]), -MAX_SPEED_MPS, MAX_SPEED_MPS)
            vy = clamp(float(pid_out[0]), -MAX_SPEED_MPS, MAX_SPEED_MPS)
        else:
            self.target_visible = False
            self.pid.reset(target=(0.0, 0.0))
            contour_area = contour_area if contour_area is not None else 0.0

        return VisionOffset(
            frame=frame,
            target_visible=self.target_visible,
            err_x_px=err_x_px,
            err_y_px=err_y_px,
            vx=vx,
            vy=vy,
            red_mask=red_mask,
            centroid=centroid,
            contour=contour,
            contour_area=float(contour_area),
        )

    def render(self, offset: VisionOffset, status_lines=None):
        if offset.frame is None:
            return

        self.renderer.render(
            offset.frame,
            offset.target_visible,
            offset.err_x_px,
            offset.err_y_px,
            offset.vx,
            offset.vy,
            offset.red_mask,
            offset.centroid,
            offset.contour,
            offset.contour_area,
            status_lines=status_lines,
        )
