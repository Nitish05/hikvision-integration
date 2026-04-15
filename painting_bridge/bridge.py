#!/usr/bin/env python3
"""Real-time Cartesian streaming bridge: Teensy painting handle -> FR5.

Architecture (see ../../plans for full rationale):
  Reader thread (blocking serial)  ---queue(maxsize=1)--->  main servo thread
                                                            (125 Hz deadline-scheduled ServoCart)

Safety invariant: the first target sent equals the robot's current TCP
pose so the arm does NOT jump on ServoMoveStart. All subsequent targets
are tcp_start + scale * (handle_now - handle_ref), with host-side EMA,
per-cycle delta clamp, and a hard workspace box.
"""

from __future__ import annotations

import argparse
import logging
import os
import queue
import signal
import sys
import threading
import time
from typing import Optional

import yaml

from teensy_reader import HandleSample, TeensyReader, resolve_port
from fr5_servo import FR5Servo  # noqa: F401 — FR5Servo.IK_FAIL used below
from recording import TrajectoryRecorder
from safety import (
    check_reach,
    clamp_delta,
    clamp_workspace,
    ema,
    is_finite,
    normalize_orientation,
)

log = logging.getLogger("bridge")

HERE = os.path.dirname(os.path.abspath(__file__))


def load_config(path: str) -> dict:
    with open(path, "r") as f:
        return yaml.safe_load(f)


def _stdin_reader(q: "queue.Queue[str]") -> None:
    """Blocking readline loop in a daemon thread. 'r' + Enter toggles the
    recorder; 'q' + Enter is reserved for future shutdown hotkey. Silently
    exits on EOF (non-TTY invocation like nohup/systemd)."""
    try:
        for line in sys.stdin:
            tok = line.strip().lower()
            if tok == "r":
                q.put("toggle")
    except Exception:
        pass


def _wait_for_clean_sample(q: "queue.Queue[HandleSample]",
                           timeout_s: float = 5.0) -> HandleSample:
    """Drain until we get a trilat-OK sample or time out."""
    t0 = time.monotonic()
    last: Optional[HandleSample] = None
    while time.monotonic() - t0 < timeout_s:
        try:
            s = q.get(timeout=0.1)
        except queue.Empty:
            continue
        last = s
        if s.trilat_ok and is_finite(s.pose):
            return s
    if last is None:
        raise RuntimeError("no sample from Teensy within 5 s")
    raise RuntimeError("no clean (trilat-OK) sample from Teensy within 5 s")


def run(cfg: dict, dry_run: bool, port_override: Optional[str]) -> int:
    # --- Setup serial + reader thread ---
    port = resolve_port(port_override or cfg["serial"]["port"])
    log.info("using serial port: %s", port)
    sample_q: "queue.Queue[HandleSample]" = queue.Queue(maxsize=1)
    reader = TeensyReader(
        port=port,
        baud=cfg["serial"]["baud"],
        out_q=sample_q,
        read_timeout_s=cfg["serial"]["read_timeout_s"],
    )
    reader.start()

    # --- Setup robot ---
    servo = FR5Servo(
        ip=cfg["robot"]["ip"],
        cmd_period_s=cfg["robot"]["cmd_period_s"],
        pos_gain=cfg["robot"]["pos_gain"],
        filter_t=cfg["robot"]["filter_t"],
        dry_run=dry_run,
        use_servoj=cfg["robot"].get("use_servoj", True),
    )

    # --- Graceful shutdown plumbing ---
    stop_flag = {"stop": False}

    def _handle_sig(signum, _frame):
        log.warning("signal %d received, stopping", signum)
        stop_flag["stop"] = True

    signal.signal(signal.SIGINT, _handle_sig)
    signal.signal(signal.SIGTERM, _handle_sig)

    exit_code = 0
    try:
        tcp_start = servo.preflight()
        first = _wait_for_clean_sample(sample_q)
        handle_ref = list(first.pose)
        log.info("anchored: tcp_start=%s handle_ref=%s",
                 [round(v, 3) for v in tcp_start],
                 [round(v, 3) for v in handle_ref])

        servo.begin()

        cmd_period = cfg["robot"].get("loop_period_s", cfg["robot"]["cmd_period_s"])
        scale_xyz = cfg["mapping"]["scale_xyz"]
        scale_rot = cfg["mapping"]["scale_rot"]
        a_xyz = cfg["filter"]["ema_alpha_xyz"]
        a_rot = cfg["filter"]["ema_alpha_rot"]
        max_dmm = cfg["safety"]["max_delta_mm_per_cycle"]
        max_ddeg = cfg["safety"]["max_delta_deg_per_cycle"]
        box_mm = cfg["safety"]["workspace_box_mm"]
        box_rot = cfg["safety"]["workspace_rot_deg"]
        stream_timeout_s = cfg["safety"]["stream_timeout_ms"] / 1000.0
        trilat_need = int(cfg["safety"]["trilat_recover_samples"])
        reach_radius = float(cfg["safety"].get(
            "workspace_reach_radius_mm", min(box_mm)))
        ik_cooldown_s = float(cfg["safety"].get(
            "ik_fail_cooldown_ms", 200)) / 1000.0
        sol_cfg = cfg.get("solenoid", {}) or {}
        sol_enabled = bool(sol_cfg.get("enabled", True))
        sol_do_id = int(sol_cfg.get("do_id", 0))

        rec_cfg = cfg.get("recording", {}) or {}
        rec_enabled = bool(rec_cfg.get("enabled", False))
        recorder: Optional[TrajectoryRecorder] = None
        tok_q: "queue.Queue[str]" = queue.Queue()
        if rec_enabled:
            recorder = TrajectoryRecorder(
                servo=servo,
                period_s=float(rec_cfg.get("period_s", cmd_period)),
                out_dir=rec_cfg.get("dir", "./recordings"),
                filename_prefix=rec_cfg.get("filename_prefix", "run"),
                tool=int(rec_cfg.get("tool", 0)),
            )
            threading.Thread(
                target=_stdin_reader, args=(tok_q,), daemon=True).start()
            log.info("recording ready — press 'r' + Enter to toggle")

        last_target = list(tcp_start)
        filtered = list(first.pose)
        last_sample: HandleSample = first
        last_servo_sample_seq = -1
        trilat_clean_streak = trilat_need
        n_sent = n_skipped = n_clamped = n_ik_fail = n_hold = 0
        t_last_ik_warn = 0.0
        last_do_state: Optional[bool] = None      # edge tracker for solenoid
        t_next = time.perf_counter() + cmd_period
        t_log = time.monotonic()

        while not stop_flag["stop"]:
            # Deadline scheduling. If we fell behind, skip ahead instead of
            # backlogging ServoCart calls.
            now = time.perf_counter()
            if now < t_next:
                time.sleep(max(0.0, t_next - now))
            t_next += cmd_period
            if time.perf_counter() - t_next > cmd_period * 5:
                t_next = time.perf_counter() + cmd_period

            # Snapshot the latest sample (non-blocking).
            try:
                s = sample_q.get_nowait()
                last_sample = s
            except queue.Empty:
                pass

            # Recorder hotkey: drain any pending 'r' toggles.
            if recorder is not None:
                try:
                    while True:
                        tok = tok_q.get_nowait()
                        if tok == "toggle":
                            if recorder.is_recording():
                                recorder.stop()
                            else:
                                recorder.start()
                except queue.Empty:
                    pass

            # Solenoid — edge-triggered. Always-on gate: run before any of the
            # motion hold paths so the valve tracks the switch even while the
            # arm is paused for trilat/IK/watchdog recovery.
            if sol_enabled and last_sample.button is not None \
                    and last_sample.button != last_do_state:
                err_do = servo.set_do(sol_do_id, 1 if last_sample.button else 0)
                if err_do == 0:
                    last_do_state = last_sample.button
                    servo.last_do_state = bool(last_sample.button)
                    log.info("solenoid -> %s", "ON" if last_sample.button else "OFF")
                else:
                    log.warning("SetDO err=%s (target=%s)", err_do,
                                int(bool(last_sample.button)))

            # Stream-loss watchdog: hold position, treat next sample as new anchor.
            if time.monotonic() - last_sample.t_mono > stream_timeout_s:
                n_skipped += 1
                n_hold += 1
                last_servo_sample_seq = -1  # force re-anchor on recovery
                trilat_clean_streak = 0
                continue

            # Skip any sample we've already acted on (nothing new to do this cycle).
            if last_sample.seq == last_servo_sample_seq:
                continue

            # Trilat recovery gate.
            if not last_sample.trilat_ok or not is_finite(last_sample.pose):
                trilat_clean_streak = 0
                n_hold += 1
                continue
            trilat_clean_streak = min(trilat_clean_streak + 1, trilat_need)
            if trilat_clean_streak < trilat_need:
                n_hold += 1
                continue

            # Rezero: re-anchor without moving the robot.
            if last_sample.rezero or last_servo_sample_seq == -1:
                err, tcp_now = servo.robot.GetActualTCPPose()
                if err == 0 and is_finite(tcp_now):
                    tcp_start = list(tcp_now)
                handle_ref = list(last_sample.pose)
                filtered = list(last_sample.pose)
                last_target = list(tcp_start)
                last_servo_sample_seq = last_sample.seq
                log.info("re-anchored at seq=%d tcp=%s",
                         last_sample.seq, [round(v, 3) for v in tcp_start])
                continue
            last_servo_sample_seq = last_sample.seq

            # Host-side EMA filter.
            filtered = ema(filtered, last_sample.pose, a_xyz, a_rot)

            # Compose absolute target in base frame.
            target = [0.0] * 6
            for i in range(3):
                target[i] = tcp_start[i] + scale_xyz * (filtered[i] - handle_ref[i])
            for i in range(3, 6):
                target[i] = tcp_start[i] + scale_rot * (filtered[i] - handle_ref[i])

            # Clamp the continuous targets.
            pre_clamp = list(target)
            target = clamp_workspace(target, tcp_start, box_mm, box_rot)
            target = clamp_delta(target, last_target, max_dmm, max_ddeg)
            if target != pre_clamp:
                n_clamped += 1

            # Spherical reach gate — catches diagonal excursions the box misses
            # and keeps obviously-unsolvable targets from reaching the IK.
            if not check_reach(target, tcp_start, reach_radius):
                n_hold += 1
                now_m = time.monotonic()
                if now_m - t_last_ik_warn >= ik_cooldown_s:
                    t_last_ik_warn = now_m
                    log.warning("reach gate: target %s outside %.1f mm of anchor — holding",
                                [round(v, 2) for v in target[:3]], reach_radius)
                continue

            # normalize_orientation is applied LAST, only here. All clamps
            # above run in continuous-angle space; wrapping earlier would
            # break clamp_workspace (wrong range) and clamp_delta (340 deg
            # spurious jumps across the ±180 seam).
            err = servo.send(normalize_orientation(target))
            if err == FR5Servo.IK_FAIL:
                n_ik_fail += 1
                n_hold += 1
                now_m = time.monotonic()
                if now_m - t_last_ik_warn >= ik_cooldown_s:
                    t_last_ik_warn = now_m
                    log.warning("IK failed: target=%s — holding (ik_fail=%d)",
                                [round(v, 2) for v in target], n_ik_fail)
                continue
            if err != 0:
                log.error("Servo send err=%d target=%s; emergency stopping",
                          err, [round(v, 2) for v in target])
                stop_flag["stop"] = True
                exit_code = 2
                break
            last_target = target
            n_sent += 1

            # Periodic stats (1 Hz).
            if time.monotonic() - t_log > 1.0:
                t_log = time.monotonic()
                log.info("sent=%d skipped=%d clamped=%d ik_fail=%d hold=%d "
                         "retry_ok=%d retry_fail=%d target=%s",
                         n_sent, n_skipped, n_clamped, n_ik_fail, n_hold,
                         servo.ik_retry_ok, servo.ik_retry_fail,
                         [round(v, 2) for v in target])

    except Exception as e:
        log.exception("fatal: %s", e)
        exit_code = 1
    finally:
        # Finalize recorder first so the file header's N is correct even if
        # the shutdown path raises below.
        try:
            if 'recorder' in locals() and recorder is not None:
                recorder.stop()
        except Exception:
            pass
        # Best-effort close the solenoid so the valve doesn't hang open if the
        # process exits with the switch held. Uses configured do_id (defaults
        # to 0) so it works even if preflight never ran.
        try:
            _sol_cfg = (cfg.get("solenoid") or {})
            if _sol_cfg.get("enabled", True):
                servo.set_do(int(_sol_cfg.get("do_id", 0)), 0)
        except Exception:
            pass
        try:
            servo.emergency_stop()
        except Exception:
            pass
        reader.stop()
        reader.join(timeout=1.0)
        log.info("bridge exiting code=%d", exit_code)
    return exit_code


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--config", default=os.path.join(HERE, "config.yaml"))
    ap.add_argument("--serial", default=None,
                    help="override serial port (default: config.yaml or auto)")
    ap.add_argument("--dry-run", action="store_true",
                    help="use MockRobot instead of connecting to an FR5")
    ap.add_argument("--log-level", default="INFO")
    args = ap.parse_args()

    logging.basicConfig(
        level=getattr(logging, args.log_level.upper()),
        format="%(asctime)s.%(msecs)03d %(name)s %(levelname)s %(message)s",
        datefmt="%H:%M:%S",
    )
    cfg = load_config(args.config)
    return run(cfg, dry_run=args.dry_run, port_override=args.serial)


if __name__ == "__main__":
    sys.exit(main())
