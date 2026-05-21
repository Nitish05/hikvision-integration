"""Optional evdev source for a USB HID macro keypad (e.g. QINIZX 2-key).

Runs as a daemon thread, grabs the device so its keys do not leak into
whichever window happens to be focused, and feeds the same token queue
the stdin hotkeys use. Designed to plug the trigger gap on the camera
(AprilTag) control source, which has no physical button input — but it
also works alongside the Teensy/Quest sources (the bridge ORs all
solenoid request sources together).

Tokens emitted on tok_q (consumed by bridge.run's hotkey drain):
  "sol_press"    momentary mode, key down on solenoid_key
  "sol_release"  momentary mode, key up on solenoid_key
  "solenoid"     toggle mode, key down on solenoid_key (same as spacebar)
  "toggle"       key down on recorder_key (same as 'r' on stdin)
"""
from __future__ import annotations

import logging
import select
import threading
from typing import Optional

log = logging.getLogger("bridge.keypad")

try:
    import evdev
    from evdev import ecodes
except Exception:                   # Linux-only / not installed
    evdev = None
    ecodes = None


def _resolve_keycode(name) -> Optional[int]:
    if name is None or ecodes is None:
        return None
    if isinstance(name, int):
        return int(name)
    s = str(name).strip().upper()
    if not s.startswith("KEY_") and not s.startswith("BTN_"):
        s = "KEY_" + s
    try:
        return int(ecodes.ecodes[s])
    except KeyError:
        log.warning("unknown evdev key name %r — ignoring", name)
        return None


def _find_devices(match_name: Optional[str],
                  vendor_id: Optional[int],
                  product_id: Optional[int],
                  required_keys: Optional[list] = None) -> list:
    """All evdev devices matching the (non-None) filters AND advertising
    at least one of the required key codes in their EV_KEY capability
    set. Returns a possibly-empty list; caller logs and runs without the
    keypad if empty.

    A single physical macropad (e.g. the QINIZX 8808:6601) often
    enumerates as several evdev nodes — keyboard interface, mouse
    interface, generic HID — and we cannot predict which one the
    remapped key will fire on. Open all of the candidates and listen to
    all of them; capability-filtering keeps the mouse-only interface
    out of the grab list."""
    if evdev is None:
        return []
    name_l = match_name.lower() if match_name else None
    required = [k for k in (required_keys or []) if k is not None]
    out: list = []
    for p in evdev.list_devices():
        try:
            d = evdev.InputDevice(p)
        except Exception:
            continue
        info = d.info
        if name_l and name_l not in d.name.lower():
            d.close(); continue
        if vendor_id is not None and int(info.vendor) != int(vendor_id):
            d.close(); continue
        if product_id is not None and int(info.product) != int(product_id):
            d.close(); continue
        if required:
            caps = d.capabilities().get(ecodes.EV_KEY, [])
            if not any(k in caps for k in required):
                d.close(); continue
        out.append(d)
    return out


def list_devices_for_cli() -> int:
    """Print every visible evdev device — path, name, VID:PID, and a tag
    on devices whose capabilities advertise KEY_SPACE (the most likely
    keypad candidates after firmware-remapping to space). Returns 0 on
    success, 2 if evdev is missing, 3 if no devices are visible (likely
    a permission problem)."""
    if evdev is None:
        print("python-evdev is not installed. pip install evdev")
        return 2
    paths = evdev.list_devices()
    if not paths:
        print("No /dev/input/event* devices visible.")
        print("You are probably not in the 'input' group. Fix with:")
        print("  sudo usermod -aG input $USER")
        print("then LOG OUT and log back in (group membership applies to "
              "new sessions only). Verify with: id | grep input")
        return 3
    print("path                   VID:PID    has_SPACE  name")
    print("-" * 72)
    for p in paths:
        try:
            d = evdev.InputDevice(p)
        except Exception as e:
            print(f"{p:22s} (open failed: {e})")
            continue
        try:
            caps = d.capabilities().get(ecodes.EV_KEY, [])
            has_space = ecodes.KEY_SPACE in caps
            tag = "  yes    " if has_space else "  no     "
            print(f"{d.path:22s} {d.info.vendor:04x}:{d.info.product:04x}  "
                  f"{tag}  {d.name!r}")
        finally:
            d.close()
    return 0


def start_keypad_reader(cfg: dict,
                        tok_q,
                        stop_event: threading.Event) -> Optional[threading.Thread]:
    """Start the keypad reader thread. Returns the Thread, or None when
    the keypad is disabled, evdev is missing, or the device isn't found.
    Always returns cleanly so the bridge can run with or without the
    keypad attached."""
    if not cfg or not cfg.get("enabled"):
        return None
    if evdev is None:
        log.warning("keypad enabled in config but python-evdev is not "
                    "installed (Linux-only; pip install evdev)")
        return None

    sol_code = _resolve_keycode(cfg.get("solenoid_key", "KEY_A"))
    rec_code = _resolve_keycode(cfg.get("recorder_key", "KEY_B"))
    grab = bool(cfg.get("grab", True))
    momentary = bool(cfg.get("momentary", True))

    devs = _find_devices(
        match_name=cfg.get("match_name"),
        vendor_id=cfg.get("vendor_id"),
        product_id=cfg.get("product_id"),
        required_keys=[sol_code, rec_code],
    )
    if not devs:
        log.warning("keypad not found (match_name=%r vid=%s pid=%s) — "
                    "bridge will run without it",
                    cfg.get("match_name"),
                    cfg.get("vendor_id"), cfg.get("product_id"))
        return None

    sol_name = ecodes.KEY.get(sol_code) if sol_code is not None else None
    rec_name = ecodes.KEY.get(rec_code) if rec_code is not None else None
    log.info("keypad: %d iface(s) [%s]  sol=%s rec=%s grab=%s momentary=%s",
             len(devs), ", ".join(d.path for d in devs),
             sol_name, rec_name, grab, momentary)

    t = threading.Thread(
        target=_run, name="keypad-reader", daemon=True,
        args=(devs, sol_code, rec_code, momentary, grab, tok_q, stop_event),
    )
    t.start()
    return t


def _run(devs: list, sol_code, rec_code, momentary, grab, tok_q,
         stop_event: threading.Event) -> None:
    grabbed = []
    if grab:
        for d in devs:
            try:
                d.grab()
                grabbed.append(d)
            except Exception as e:
                log.warning("keypad grab() failed on %s: %s — that iface's "
                            "keys may leak to the focused window", d.path, e)
    fd_to_dev = {d.fd: d for d in devs}
    try:
        while not stop_event.is_set() and fd_to_dev:
            r, _, _ = select.select(list(fd_to_dev.keys()), [], [], 0.2)
            if not r:
                continue
            for fd in r:
                d = fd_to_dev.get(fd)
                if d is None:
                    continue
                try:
                    for ev in d.read():
                        if ev.type != ecodes.EV_KEY:
                            continue
                        # value: 1=down, 0=up, 2=autorepeat (ignored)
                        if sol_code is not None and ev.code == sol_code:
                            if momentary:
                                if ev.value == 1:
                                    tok_q.put("sol_press")
                                elif ev.value == 0:
                                    tok_q.put("sol_release")
                            else:
                                if ev.value == 1:
                                    tok_q.put("solenoid")  # spacebar-style toggle
                        elif rec_code is not None and ev.code == rec_code:
                            if ev.value == 1:
                                tok_q.put("toggle")
                except OSError as e:
                    log.warning("keypad %s read failed (unplugged?): %s",
                                d.path, e)
                    fd_to_dev.pop(fd, None)
        if not fd_to_dev:
            log.warning("all keypad interfaces gone; reader exiting")
    finally:
        for d in grabbed:
            try:
                d.ungrab()
            except Exception:
                pass
        for d in devs:
            try:
                d.close()
            except Exception:
                pass
