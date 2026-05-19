# camera_tracking — Arducam two-tag AprilTag pose

Calibrate the **Arducam UC-844 Rev B** (OV9281, 1 MP global-shutter mono) and
track a handheld **AprilTag** relative to a fixed **reference tag** — the steps
toward replacing the Quest 2 as the painting-rig input.

A fixed reference tag defines the control frame; a moving tag is reported
relative to it. Because the result is a tag-to-tag *relative* pose, **it does
not depend on where the camera is** — the camera can be bumped or moved and the
numbers stay put. Placing the reference tag is the only "calibration".

**Built:** intrinsic calibration + live two-tag pose readout.
**Not built (next phase):** a `camera_reader.py` feeding `painting_bridge` as a
drop-in for `quest_reader.py`.

## Files
| file               | purpose                                                |
|--------------------|--------------------------------------------------------|
| `config.yaml`      | camera, checkerboard, calibration, AprilTag settings   |
| `camera.py`        | shared: device auto-detect, open, exposure, threaded frame grab |
| `tag_detector.py`  | shared AprilTag detection + pose (`TagDetector`)       |
| `calibrate.py`     | checkerboard intrinsic calibration → `calibration.npz` |
| `apriltag_pose.py` | live two-tag pose readout                              |

Requires only what is already in `.venv`: `opencv-python` 4.12, `numpy`,
`pyyaml`. `device: auto` also needs `v4l2-ctl` (`v4l-utils`).

## 1. Calibrate the camera

Print a checkerboard. The defaults in `config.yaml` are **8×6 inner corners,
22.83 mm squares** — `cols`/`rows` count *inner* corners (where four squares
meet), not squares. Measure the printed squares and update `square_mm` if they
differ.

```bash
.venv/bin/python camera_tracking/calibrate.py
```

- `SPACE` captures a view (only when the grid is drawn green/locked).
- Capture **15–25 views**: centre, all four frame corners, near, far, tilted.
- `C` calibrates, `Q` quits.

Saved to `camera_tracking/calibration.npz`. Aim for **RMS < 0.5 px**.

## 2. Print two tags and set the config

Family is **tag36h11**. Generate two tags with **different ids**, e.g. id 0 and
id 1:

```bash
.venv/bin/python -c "import cv2; d=cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11); \
[cv2.imwrite(f'tag{i}.png', cv2.aruco.generateImageMarker(d,i,600)) for i in (0,1)]"
```

Print both, **measure each tag's outer black-square edge with calipers**, and
set them in `config.yaml`:

```yaml
apriltag:
  reference: { id: 0, size_mm: <measured> }   # the fixed tag
  moving:    { id: 1, size_mm: <measured> }   # the handheld tag
```

**Lay the reference tag flat** (on the table / workpiece), face up, with its
printed **top edge pointing away from you**. Its tag frame then becomes the
control frame:

```
+X = left / right     +Y = away from you     +Z = up (perpendicular to the tag)
```

If a sign comes out reversed, rotate the reference tag flat until the live
readout matches what you want.

## 3. Read the moving tag in the reference frame

```bash
.venv/bin/python camera_tracking/apriltag_pose.py
```

With both tags in view it prints, at ~10 Hz, the moving tag's pose **in the
reference frame**:

```
ref-frame  X=  +42.1 Y= -18.7 Z= +93.4 mm  rpy=( +1.2, -4.0,+178.5)  quat=(...)
```

Move the moving tag right → `X` changes; away from you → `Y` rises; up → `Z`
rises. If a tag is hidden the readout says which one — it never shows stale
numbers.

## Tips
- Mount the reference tag where it is seen at a **slight angle**, not perfectly
  head-on — a square viewed straight-on has a pose-flip ambiguity that can make
  its orientation jump.
- `camera.manual_exposure: true` locks a short exposure via `v4l2-ctl` so a
  moving tag stays crisp. Lower `exposure_time` if tags blur.
- If `Z` (or any distance) is off by a constant ratio, the cause is almost
  always a wrong `size_mm` or `checkerboard.square_mm`.
- `detect_decimate: 2` detects on a half-res image (~3× faster, shorter range).
  Set it to `1` if a distant tag stops being detected.
- `device: auto` matches the `Arducam OV9281` v4l2 name; set an explicit
  `/dev/videoN` if you have more than one Arducam.
