"""Windows global keyboard hotkey source for bridge solenoid control.

Uses pynput's non-blocking keyboard listener so SPACE can trigger the solenoid
even when the bridge terminal is not focused. Events are not suppressed: the
focused app still receives normal Space key input.
"""

from __future__ import annotations

import logging
import queue
from typing import Optional

log = logging.getLogger("bridge.windows_hotkey")

try:
    from pynput import keyboard
except Exception:
    keyboard = None


def _is_space(key) -> bool:
    if keyboard is not None and key == keyboard.Key.space:
        return True
    return getattr(key, "char", None) == " "


def _on_press(key, tok_q: "queue.Queue[str]") -> None:
    if _is_space(key):
        tok_q.put("sol_press")


def _on_release(key, tok_q: "queue.Queue[str]") -> None:
    if _is_space(key):
        tok_q.put("sol_release")


def start_windows_hotkey_reader(tok_q: "queue.Queue[str]") -> Optional[object]:
    """Start a non-blocking global Space listener.

    Returns the pynput listener, or None if pynput is unavailable. The caller
    should call stop() during shutdown when a listener is returned.
    """
    if keyboard is None:
        log.warning("pynput is not installed; Windows global SPACE hotkey disabled")
        return None

    listener = keyboard.Listener(
        on_press=lambda key, *args: _on_press(key, tok_q),
        on_release=lambda key, *args: _on_release(key, tok_q),
        suppress=False,
    )
    listener.daemon = True
    listener.start()
    log.info("Windows global SPACE hotkey enabled (hold-to-spray)")
    return listener
