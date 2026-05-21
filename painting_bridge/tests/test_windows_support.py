import os
import queue
import sys
import types

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, os.path.join(ROOT, "camera_tracking"))
sys.path.insert(0, os.path.join(ROOT, "painting_bridge"))

try:
    import cv2  # noqa: F401
except ModuleNotFoundError:
    fake_cv2 = types.SimpleNamespace(
        CAP_DSHOW=700,
        CAP_PROP_FOURCC=6,
        CAP_PROP_FRAME_WIDTH=3,
        CAP_PROP_FRAME_HEIGHT=4,
        CAP_PROP_FPS=5,
        CAP_PROP_BUFFERSIZE=38,
        CAP_PROP_BACKEND=42,
        VideoWriter_fourcc=lambda *args: 1196444237,
        VideoCapture=lambda *_args, **_kwargs: None,
    )
    sys.modules["cv2"] = fake_cv2

import camera  # noqa: E402
import windows_hotkey_reader  # noqa: E402


class FakeCapture:
    def __init__(self, opened):
        self._opened = opened
        self.released = False
        self.set_calls = []

    def isOpened(self):
        return self._opened

    def set(self, prop, value):
        self.set_calls.append((prop, value))
        return True

    def get(self, prop):
        if prop == camera.cv2.CAP_PROP_FRAME_WIDTH:
            return 1280
        if prop == camera.cv2.CAP_PROP_FRAME_HEIGHT:
            return 800
        if prop == camera.cv2.CAP_PROP_FPS:
            return 120
        if hasattr(camera.cv2, "CAP_PROP_BACKEND") and prop == camera.cv2.CAP_PROP_BACKEND:
            return camera.cv2.CAP_DSHOW
        return 0

    def release(self):
        self.released = True


def _cam_cfg(**overrides):
    cfg = {
        "device": "auto",
        "width": 1280,
        "height": 800,
        "fps": 120,
        "fourcc": "MJPG",
        "manual_exposure": False,
        "probe_max_index": 2,
    }
    cfg.update(overrides)
    return {"camera": cfg}


def test_windows_auto_probe_prefers_external_directshow_name():
    calls = []
    original_open = camera._open_capture
    original_devices = camera.windows_directshow_devices

    def fake_open(index, backend):
        calls.append((index, backend))
        return FakeCapture(index == 2)

    try:
        camera._open_capture = fake_open
        camera.windows_directshow_devices = lambda: [
            "Surface Camera Front",
            "Surface Camera Rear",
            "HD Pro Webcam C920",
        ]
        cap, index = camera._open_first_windows_camera(_cam_cfg()["camera"])
    finally:
        camera._open_capture = original_open
        camera.windows_directshow_devices = original_devices

    assert index == 2
    assert cap.isOpened()
    assert calls == [(2, camera.cv2.CAP_DSHOW)]


def test_windows_auto_probe_falls_back_to_index_order():
    calls = []
    original_open = camera._open_capture
    original_devices = camera.windows_directshow_devices

    def fake_open(index, backend):
        calls.append((index, backend))
        return FakeCapture(index == 1)

    try:
        camera._open_capture = fake_open
        camera.windows_directshow_devices = lambda: []
        cap, index = camera._open_first_windows_camera(_cam_cfg()["camera"])
    finally:
        camera._open_capture = original_open
        camera.windows_directshow_devices = original_devices

    assert index == 1
    assert cap.isOpened()
    assert calls == [(1, camera.cv2.CAP_DSHOW)]


def test_windows_explicit_numeric_index_opens_directly():
    calls = []
    original_open = camera._open_capture
    original_is_windows = camera._is_windows

    def fake_open(index, backend):
        calls.append((index, backend))
        return FakeCapture(True)

    try:
        camera._open_capture = fake_open
        camera._is_windows = lambda: True
        _cap, label = camera.open_camera(_cam_cfg(device="2"))
    finally:
        camera._open_capture = original_open
        camera._is_windows = original_is_windows

    assert calls == [(2, camera.cv2.CAP_DSHOW)]
    assert "index 2" in label


def test_windows_auto_failure_is_clear():
    original = camera._open_capture

    def fake_open(_index, _backend):
        return FakeCapture(False)

    try:
        camera._open_capture = fake_open
        try:
            camera._open_first_windows_camera(_cam_cfg()["camera"])
        except RuntimeError as e:
            assert "failed to open a USB camera on Windows" in str(e)
        else:
            raise AssertionError("expected RuntimeError")
    finally:
        camera._open_capture = original


def test_windows_space_hotkey_tokens():
    q = queue.Queue()
    windows_hotkey_reader._on_press(type("K", (), {"char": " "})(), q)
    windows_hotkey_reader._on_release(type("K", (), {"char": " "})(), q)
    windows_hotkey_reader._on_press(type("K", (), {"char": "x"})(), q)

    assert q.get_nowait() == "sol_press"
    assert q.get_nowait() == "sol_release"
    assert q.empty()


if __name__ == "__main__":
    import traceback
    failed = 0
    tests = [(n, fn) for n, fn in globals().items()
             if n.startswith("test_") and callable(fn)]
    for name, fn in tests:
        try:
            fn()
            print(f"PASS {name}")
        except Exception:
            failed += 1
            print(f"FAIL {name}")
            traceback.print_exc()
    print(f"\n{len(tests) - failed}/{len(tests)} passed")
    sys.exit(1 if failed else 0)
