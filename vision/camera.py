import asyncio
from picamera2 import Picamera2

class Camera:
    def __init__(self):
        self.picam2 = None
        self.camera_started = False
        self.capture_error_reported = False

    async def start_camera(self):
        self.picam2 = Picamera2()
        config = self.picam2.create_video_configuration(
            main={"size": (640, 480), "format": "BGR888"},
            controls={"FrameDurationLimits": (33333, 33333)},
        )
        self.picam2.configure(config)
        self.picam2.start()
        self.camera_started = True
        print("[VISION] Camera started")

    async def stop_camera(self):
        if self.picam2 and self.camera_started:
            self.picam2.stop()
            self.camera_started = False
            print("[VISION] Camera stopped")

    async def capture_frame(self):
        if not self.picam2 or not self.camera_started:
            return None
        try:
            frame = await asyncio.to_thread(self.picam2.capture_array, "main")
            self.capture_error_reported = False
            return frame
        except Exception as exc:
            if not self.capture_error_reported:
                print(f"[VISION] Camera frame capture failed: {exc}")
                self.capture_error_reported = True
            return None
