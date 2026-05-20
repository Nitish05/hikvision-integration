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


def _find_device(match_name: Optional[str],
                 vendor_id: Optional[int],
                 product_id: Optional[int]):
    """First device matching all of the (non-None) filters. None if no
    match — caller logs and runs without the keypad."""
    if evdev is None:
        return None
    name_l = match_name.lower() if match_name else None
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
        return d
    return None


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

    dev = _find_device(
        match_name=cfg.get("match_name"),
        vendor_id=cfg.get("vendor_id"),
        product_id=cfg.get("product_id"),
    )
    if dev is None:
        log.warning("keypad not found (match_name=%r vid=%s pid=%s) — "
                    "bridge will run without it",
                    cfg.get("match_name"),
                    cfg.get("vendor_id"), cfg.get("product_id"))
        return None

    sol_code = _resolve_keycode(cfg.get("solenoid_key", "KEY_A"))
    rec_code = _resolve_keycode(cfg.get("recorder_key", "KEY_B"))
    grab = bool(cfg.get("grab", True))
    momentary = bool(cfg.get("momentary", True))

    sol_name = ecodes.KEY.get(sol_code) if sol_code is not None else None
    rec_name = ecodes.KEY.get(rec_code) if rec_code is not None else None
    log.info("keypad: %r @ %s  sol=%s rec=%s grab=%s momentary=%s",
             dev.name, dev.path, sol_name, rec_name, grab, momentary)

    t = threading.Thread(
        target=_run, name="keypad-reader", daemon=True,
        args=(dev, sol_code, rec_code, momentary, grab, tok_q, stop_event),
    )
    t.start()
    return t


def _run(dev, sol_code, rec_code, momentary, grab, tok_q,
         stop_event: threading.Event) -> None:
    if grab:
        try:
            dev.grab()
        except Exception as e:
            log.warning("keypad grab() failed: %s — keys will leak to the "
                        "focused window", e)
            grab = False
    try:
        fd = dev.fd
        while not stop_event.is_set():
            r, _, _ = select.select([fd], [], [], 0.2)
            if not r:
                continue
            try:
                for ev in dev.read():
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
                                tok_q.put("solenoid")    # spacebar-style toggle
                    elif rec_code is not None and ev.code == rec_code:
                        if ev.value == 1:
                            tok_q.put("toggle")
            except OSError as e:
                log.warning("keypad read failed (unplugged?): %s", e)
                break
    finally:
        try:
            if grab:
                dev.ungrab()
        except Exception:
            pass
        try:
            dev.close()
        except Exception:
            pass
