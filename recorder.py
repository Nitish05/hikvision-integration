import tkinter as tk
from tkinter import ttk
from fairino import Robot
from tkinter import filedialog
import threading
import time
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg, NavigationToolbar2Tk
from matplotlib.figure import Figure
from mpl_toolkits.mplot3d import Axes3D
import os

DEBUG = True
IP = '192.168.57.2'
CYCLETIME = 0.008  # 8ms cycle time
TOOL = 0
USER = 0
MOVEJ_VEL = 10.0

FRTYPE = 5
REACH = {3: 622, 5: 922, 10: 1400, 16: 1034, 20: 1854, 30: 1403}

# I/O channels
DI_SWITCH = 0  # DI0 = switch input
DO_SOLENOID = 0  # DO0 = solenoid output


class RecorderApp:
    def __init__(self, root):
        self.root = root
        self.root.title("Fairino FR5 - Trajectory + Solenoid Recorder")

        # Connect to robot
        self.Cobot = Robot.RPC(IP)
        print(self.Cobot.GetSDKVersion())
        self.Cobot.connect_to_robot()
        self.Cobot.Mode(1)

        self.tracking = False
        self.cycle_time = 0
        self.positions = []  # [x, y, z, rx, ry, rz, trigger]
        self.last_do_state = -1  # track DO state to avoid redundant SetDO calls
        self.loaded_filename = tk.StringVar(value="No File loaded")

        self.root.columnconfigure(0, weight=1)
        self.root.rowconfigure(0, weight=1)

        # --- Buttons ---
        btn_frame = ttk.Frame(self.root)
        btn_frame.grid(row=1, column=0, padx=10, pady=10)
        ttk.Button(btn_frame, text="Play (TPD)", command=self.run_tpd).pack(side="left", padx=5)
        ttk.Button(btn_frame, text="Play (Servo+DO)", command=self.run_servo_playback).pack(side="left", padx=5)

        self.load_button = ttk.Button(self.root, text="Load", command=self.load_positions_from_file_v2)
        self.load_button.grid(row=2, column=0, padx=10, pady=5)

        self.loaded_file_label = ttk.Label(self.root, textvariable=self.loaded_filename)
        self.loaded_file_label.grid(row=3, column=0, padx=10, pady=5)

        # --- 3D Plot ---
        self.figure = Figure(figsize=(5, 5), dpi=100)
        self.ax = self.figure.add_subplot(111, projection='3d')
        self.ax.set_title("Robot Path")
        self.ax.set_xlabel("X (mm)")
        self.ax.set_ylabel("Y (mm)")
        self.ax.set_zlabel("Z (mm)")
        self.canvas = FigureCanvasTkAgg(self.figure, master=self.root)
        self.canvas.get_tk_widget().grid(row=4, column=0, padx=10, pady=10)

        self.toolbar_frame = ttk.Frame(self.root)
        self.toolbar_frame.grid(row=5, column=0, padx=10, pady=5, sticky="ew")
        self.toolbar = NavigationToolbar2Tk(self.canvas, self.toolbar_frame)
        self.toolbar.update()

        if DEBUG:
            # Tracking button
            self.track_button = ttk.Button(self.root, text="Start Tracking", command=self.toggle_tracking)
            self.track_button.grid(row=6, column=0, padx=10, pady=10)

            # Save button
            self.save_button = ttk.Button(self.root, text="Save", command=self.convert_points_to_FRTPDfile)
            self.save_button.grid(row=7, column=0, padx=10, pady=5)

            # --- I/O Status Frame ---
            io_frame = ttk.LabelFrame(self.root, text="I/O Status", padding=10)
            io_frame.grid(row=8, column=0, padx=10, pady=5, sticky="ew")

            # DI0 indicator
            ttk.Label(io_frame, text="Switch (DI0):", font=("Consolas", 10)).grid(row=0, column=0, padx=5)
            self.di_canvas = tk.Canvas(io_frame, width=28, height=28, highlightthickness=0)
            self.di_canvas.grid(row=0, column=1, padx=5)
            self._draw_indicator(self.di_canvas, None)

            # DO0 indicator
            ttk.Label(io_frame, text="Solenoid (DO0):", font=("Consolas", 10)).grid(row=0, column=2, padx=5)
            self.do_canvas = tk.Canvas(io_frame, width=28, height=28, highlightthickness=0)
            self.do_canvas.grid(row=0, column=3, padx=5)
            self._draw_indicator(self.do_canvas, None)

            # Recording stats
            self.stats_var = tk.StringVar(value="")
            ttk.Label(io_frame, textvariable=self.stats_var, font=("Consolas", 9)).grid(row=0, column=4, padx=10)

        # Start I/O polling for indicators
        self._poll_io()

    def _draw_indicator(self, canvas, level):
        canvas.delete("all")
        if level is None:
            color = "#6b7280"
        elif level:
            color = "#22c55e"
        else:
            color = "#ef4444"
        canvas.create_oval(2, 2, 26, 26, fill=color, outline="")

    def _poll_io(self):
        """Update I/O indicators on the main thread."""
        if DEBUG:
            try:
                pkg = self.Cobot.robot_state_pkg
                di_level = (pkg.cl_dgt_input_l >> DI_SWITCH) & 1
                do_level = (pkg.cl_dgt_output_l >> DO_SOLENOID) & 1
                self._draw_indicator(self.di_canvas, di_level)
                self._draw_indicator(self.do_canvas, do_level)
            except (TypeError, AttributeError):
                pass  # State package not yet initialized

            if self.tracking:
                trigger_count = sum(1 for p in self.positions if len(p) > 6 and p[6] == 1)
                self.stats_var.set(f"Points: {len(self.positions)} | Triggers: {trigger_count}")

        self.root.after(100, self._poll_io)

    def update_plot(self):
        self.ax.clear()
        self.ax.set_title("Robot Path")
        self.ax.set_xlabel("X (mm)")
        self.ax.set_ylabel("Y (mm)")
        self.ax.set_zlabel("Z (mm)")
        self.ax.set_xlim([-REACH[FRTYPE], REACH[FRTYPE]])
        self.ax.set_ylim([-REACH[FRTYPE], REACH[FRTYPE]])
        self.ax.set_zlim([-REACH[FRTYPE], REACH[FRTYPE]])

        if self.positions:
            xs, ys, zs = zip(*[pos[:3] for pos in self.positions])
            self.ax.plot(xs, ys, zs, label="Path", marker='o', markersize=2, linewidth=1)

            # Highlight trigger points in red
            trigger_pts = [p for p in self.positions if len(p) > 6 and p[6] == 1]
            if trigger_pts:
                txs, tys, tzs = zip(*[p[:3] for p in trigger_pts])
                self.ax.scatter(txs, tys, tzs, color='red', s=20, label="Trigger", zorder=5)

            self.ax.legend()

        self.canvas.draw()

    def toggle_tracking(self):
        if not self.tracking:
            self.tracking = True
            self.track_button.config(text="Stop Tracking")
            self.positions = []
            self.last_do_state = -1
            self.Cobot.Mode(1)  # Manual mode for drag teaching
            time.sleep(0.5)
            self.Cobot.DragTeachSwitch(1)
            threading.Thread(target=self.track_movement, daemon=True).start()
        else:
            self.tracking = False
            self.cycle_time = CYCLETIME
            self.Cobot.DragTeachSwitch(0)
            # Turn off solenoid when stopping
            self.Cobot.SetDO(id=DO_SOLENOID, status=0)
            self.last_do_state = 0
            self.track_button.config(text="Start Tracking")
            self.update_plot()
            trigger_count = sum(1 for p in self.positions if len(p) > 6 and p[6] == 1)
            print(f"Recording stopped: {len(self.positions)} points, {trigger_count} trigger points")

    def track_movement(self):
        while self.tracking:
            result = self.Cobot.GetActualTCPPose()
            if isinstance(result, tuple) and len(result) == 2 and isinstance(result[1], list) and len(result[1]) == 6:
                xyz_angles = result[1]

                # Read DI0 from state package (fast, no RPC overhead)
                try:
                    di_level = (self.Cobot.robot_state_pkg.cl_dgt_input_l >> DI_SWITCH) & 1
                except (TypeError, AttributeError):
                    di_level = 0

                # Mirror DI0 -> DO0 (only send command on state change)
                if di_level != self.last_do_state:
                    self.Cobot.SetDO(id=DO_SOLENOID, status=di_level)
                    self.last_do_state = di_level

                # Store position + trigger state
                self.positions.append(xyz_angles + [di_level])

            time.sleep(CYCLETIME)

    def convert_points_to_FRTPDfile(self):
        if not self.positions:
            print("No positions to save")
            return

        filename = filedialog.asksaveasfilename(
            defaultextension=".txt",
            filetypes=[("Text files", "*.txt"), ("All files", "*.*")]
        )
        if not filename:
            return

        number_of_points = len(self.positions)
        periodtime = self.cycle_time if self.cycle_time > 0 else CYCLETIME
        tool = TOOL
        diconfig = 1  # bit 0 = DI0
        doconfig = 1  # bit 0 = DO0

        with open(filename, "w") as file:
            file.write(f"{number_of_points},1,{int(periodtime * 1000)},{tool},{diconfig},{doconfig}\n")

            for pos in self.positions:
                desc_pos = pos[:6]
                trigger = int(pos[6]) if len(pos) > 6 else 0

                error, joint_pos = self.Cobot.GetInverseKin(0, desc_pos, -1)
                if error != 0:
                    print(f"Error calculating joint pos: {error}")
                    continue

                full_line = joint_pos + desc_pos
                line = ",".join(f"{value:.4f}" for value in full_line)
                file.write(f"{line},{5},{trigger},{17}\n")

        print(f"Trajectory saved to {filename} ({number_of_points} points)")

        self.Cobot.TrajectoryJUpLoad(filePath=filename)
        print(f"Trajectory uploaded to robot.")
        self.loaded_filename.set(f"Saved: {filename}")

    def load_positions_from_file_v2(self):
        filename = filedialog.askopenfilename(
            filetypes=[("Text files", "*.txt"), ("All files", "*.*")]
        )
        if not filename:
            self.loaded_filename.set("No File loaded")
            return

        with open(filename, "r") as file:
            lines = file.readlines()

        if len(lines) < 2:
            print("File contains no point data")
            return

        self.positions = []
        for line in lines[1:]:
            if not line.strip():
                continue
            parts = line.strip().split(",")
            if len(parts) >= 13:
                # joints[0:6] + cartesian[6:12] + extra[12:]
                xyz_rxryrz = list(map(float, parts[6:12]))
                # Try to read trigger state from the DO field (index 13)
                trigger = 0
                if len(parts) >= 14:
                    try:
                        trigger = int(float(parts[13]))
                    except (ValueError, IndexError):
                        trigger = 0
                self.positions.append(xyz_rxryrz + [trigger])

        self.loaded_filename.set(f"Opened: {filename}")
        trigger_count = sum(1 for p in self.positions if len(p) > 6 and p[6] == 1)
        print(f"{len(self.positions)} points loaded ({trigger_count} trigger points)")

        self.Cobot.TrajectoryJUpLoad(filePath=filename)
        print(f"Trajectory uploaded to robot.")
        self.update_plot()

    def run_tpd(self):
        """Playback using MoveTPD (robot handles motion) + manual DO firing."""
        if self.loaded_filename.get() == "No File loaded":
            print("No file loaded")
            return

        full_path = self.loaded_filename.get()
        # Extract just the filename
        for prefix in ("Opened: ", "Saved: "):
            if full_path.startswith(prefix):
                full_path = full_path[len(prefix):]

        filename = os.path.basename(full_path)
        name_only = os.path.splitext(filename)[0]

        print(f"Running TPD: traj/{name_only}")

        self.Cobot.Mode(0)  # Switch to auto mode for playback
        time.sleep(0.5)

        rtn = self.Cobot.LoadTPD(f"traj/{name_only}")
        print(f"LoadTPD: {rtn}")

        error, start_pose = self.Cobot.GetTPDStartPose(f"traj/{name_only}")
        print(f"Start pose: {start_pose}")

        # Move to start position
        error, joint_pos = self.Cobot.GetInverseKin(0, start_pose, -1)
        if error != 0:
            print(f"Error calculating start joint pos: {error}")
            return

        error = self.Cobot.MoveJ(joint_pos, TOOL, USER, vel=MOVEJ_VEL)
        if error != 0:
            print(f"Error moving to start: {error}")
            return

        if not self.wait_for_motion_complete(timeout=10):
            print("Timeout reaching start position")
            return

        # Play trajectory — MoveTPD handles motion, DO is embedded in the TPD file
        self.Cobot.MoveTPD(f"traj/{name_only}", 1, 100)
        print("Playback complete")

    def run_servo_playback(self):
        """Alternative playback using ServoJ with manual DO control."""
        if not self.positions:
            print("No positions loaded")
            return

        self.Cobot.Mode(0)  # Switch to auto mode for playback
        time.sleep(0.5)

        start_point = self.positions[0][:6]
        error, joint_pos = self.Cobot.GetInverseKin(0, start_point, -1)
        if error != 0:
            print(f"Error calculating start joint pos: {error}")
            return

        error = self.Cobot.MoveJ(joint_pos, TOOL, USER, vel=MOVEJ_VEL)
        if error != 0:
            print(f"Error moving to start: {error}")
            return

        if not self.wait_for_motion_complete(timeout=10):
            print("Timeout reaching start position")
            return

        error = self.Cobot.ServoMoveStart()
        if error != 0:
            print(f"Error starting servo mode: {error}")
            return

        last_trigger = -1
        for i in range(1, len(self.positions)):
            full = self.positions[i]
            pos = full[:6]
            trigger = int(full[6]) if len(full) > 6 else 0

            # Fire solenoid on state change
            if trigger != last_trigger:
                self.Cobot.SetDO(id=DO_SOLENOID, status=trigger)
                last_trigger = trigger

            error, jp = self.Cobot.GetInverseKin(0, pos, -1)
            if error != 0:
                continue

            error = self.Cobot.ServoJ(joint_pos=jp, axisPos=[0.0,0.0,0.0,0.0], cmdT=CYCLETIME)
            if error != 0:
                print(f"ServoJ error: {error}")
                break

            time.sleep(0.001)

        # Turn off solenoid at end
        self.Cobot.SetDO(id=DO_SOLENOID, status=0)

        error = self.Cobot.ServoMoveEnd()
        if error != 0:
            print(f"Error ending servo mode: {error}")

        print("Servo playback complete")

    def wait_for_motion_complete(self, timeout=10):
        start_time = time.time()
        while time.time() - start_time < timeout:
            error, state = self.Cobot.GetRobotMotionDone()
            error, length = self.Cobot.GetMotionQueueLength()
            if error != 0:
                print(f"Error getting motion status: {error}")
                return False
            if state == 1 and length == 0:
                return True
            time.sleep(0.1)
        return False


if __name__ == "__main__":
    root = tk.Tk()
    app = RecorderApp(root)
    root.mainloop()
