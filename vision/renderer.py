import cv2

from constants import FRAME_COLOR_ORDER


class PreviewRenderer:
    def __init__(self, window_name="Red Target Detection", enabled=True):
        self.enabled = enabled
        self.window_name = window_name
        self.window_open = False
        self.frame_color_order = str(FRAME_COLOR_ORDER).upper()
        self.user_closed = False

    def init_window(self):
        if not self.enabled or self.window_open or self.user_closed:
            return

        try:
            cv2.namedWindow(self.window_name, cv2.WINDOW_NORMAL)
            self.window_open = True
        except Exception as exc:
            self.window_open = False
            print(f"[VISION] Preview init failed: {exc}")

    def close_window(self):
        if not self.window_open:
            return

        try:
            cv2.destroyWindow(self.window_name)
        except Exception as exc:
            print(f"[VISION] Preview close warning: {exc}")
        finally:
            self.window_open = False

    def render(
        self,
        frame,
        target_visible,
        err_x,
        err_y,
        vx,
        vy,
        red_mask,
        centroid,
        contour,
        contour_area,
        status_lines=None,
    ):
        if not self.enabled or frame is None:
            return

        if not self.window_open:
            self.init_window()
            if not self.window_open:
                return

        if self.frame_color_order == "RGB":
            preview = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
        else:
            preview = frame.copy()

        h, w = preview.shape[:2]
        center = (w // 2, h // 2)
        cv2.drawMarker(
            preview,
            center,
            (0, 255, 255),
            markerType=cv2.MARKER_CROSS,
            markerSize=20,
            thickness=2,
        )

        if contour is not None:
            try:
                cv2.drawContours(preview, [contour], -1, (0, 255, 0), 2)
            except Exception:
                pass

        if centroid is not None:
            try:
                cv2.circle(preview, centroid, 6, (0, 255, 0), -1)
            except Exception:
                pass

        if red_mask is not None:
            try:
                cv2.putText(
                    preview,
                    "red_mask: ON",
                    (10, 60),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.5,
                    (255, 255, 255),
                    1,
                    cv2.LINE_AA,
                )
            except Exception:
                pass

        status = "visible" if target_visible else "not_visible"
        ex = "None" if err_x is None else f"{err_x:.1f}px"
        ey = "None" if err_y is None else f"{err_y:.1f}px"
        overlay = f"{status} err_x={ex} err_y={ey} area={contour_area:.0f} vx={vx:.2f} vy={vy:.2f}"
        cv2.putText(
            preview,
            overlay,
            (10, 30),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.6,
            (0, 255, 0) if target_visible else (0, 0, 255),
            2,
            cv2.LINE_AA,
        )

        if status_lines:
            y = 90
            for line in status_lines:
                cv2.putText(
                    preview,
                    str(line),
                    (10, y),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.5,
                    (255, 255, 255),
                    1,
                    cv2.LINE_AA,
                )
                y += 22

        try:
            cv2.imshow(self.window_name, preview)
            key = cv2.waitKey(1) & 0xFF
            if key == ord("q"):
                self.user_closed = True
                self.close_window()
                print("[VISION] Preview closed by user")
        except Exception as exc:
            self.close_window()
            print(f"[VISION] Preview render failed, will retry: {exc}")
