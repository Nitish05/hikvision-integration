import tkinter as tk
from tkinter import ttk
from fairino import Robot
import time

IP = '192.168.57.2'
POLL_MS = 100  # poll every 100ms

DI_DO_MAP = {0: 0}  # DI0 triggers DO0

COLOR_HIGH = "#22c55e"
COLOR_LOW = "#ef4444"
COLOR_UNKNOWN = "#6b7280"


class IOMonitor:
    def __init__(self, root):
        self.root = root
        self.root.title("Fairino FR5 - Digital I/O Monitor")

        self.do_states = {i: 0 for i in range(16)}
        self.tdo_states = {i: 0 for i in range(2)}

        self.status_var = tk.StringVar(value="Connecting...")
        self.passthrough_var = tk.BooleanVar(value=False)

        self._build_ui()

        try:
            self.robot = Robot.RPC(IP)
            self.robot.connect_to_robot()
            self.status_var.set("Connected")
        except Exception as e:
            self.status_var.set(f"Connection failed: {e}")
            self.robot = None

        # Start polling on the main thread via root.after
        if self.robot:
            self.root.after(500, self._poll)

    def _build_ui(self):
        di_frame = ttk.LabelFrame(self.root, text="Digital Inputs (DI0-DI15)", padding=10)
        di_frame.grid(row=0, column=0, padx=10, pady=10, sticky="nsew")
        self.di_canvases = {}
        for i in range(16):
            r, c = divmod(i, 8)
            f = ttk.Frame(di_frame)
            f.grid(row=r, column=c, padx=4, pady=4)
            tk.Label(f, text=f"DI{i}", font=("Consolas", 10)).pack()
            cv = tk.Canvas(f, width=28, height=28, highlightthickness=0)
            cv.pack()
            self._draw(cv, None)
            self.di_canvases[i] = cv

        tdi_frame = ttk.LabelFrame(self.root, text="Tool Digital Inputs (TDI0-TDI1)", padding=10)
        tdi_frame.grid(row=1, column=0, padx=10, pady=5, sticky="nsew")
        self.tdi_canvases = {}
        for i in range(2):
            f = ttk.Frame(tdi_frame)
            f.grid(row=0, column=i, padx=4, pady=4)
            tk.Label(f, text=f"TDI{i}", font=("Consolas", 10)).pack()
            cv = tk.Canvas(f, width=28, height=28, highlightthickness=0)
            cv.pack()
            self._draw(cv, None)
            self.tdi_canvases[i] = cv

        do_frame = ttk.LabelFrame(self.root, text="Digital Outputs (DO0-DO15)", padding=10)
        do_frame.grid(row=2, column=0, padx=10, pady=10, sticky="nsew")
        self.do_canvases = {}
        for i in range(16):
            r, c = divmod(i, 8)
            f = ttk.Frame(do_frame)
            f.grid(row=r, column=c, padx=4, pady=4)
            tk.Label(f, text=f"DO{i}", font=("Consolas", 10)).pack()
            cv = tk.Canvas(f, width=28, height=28, highlightthickness=0)
            cv.pack()
            self._draw(cv, None)
            self.do_canvases[i] = cv
            ttk.Button(f, text="Toggle", width=6,
                       command=lambda ch=i: self._toggle_do(ch)).pack(pady=2)

        tdo_frame = ttk.LabelFrame(self.root, text="Tool Digital Outputs (TDO0-TDO1)", padding=10)
        tdo_frame.grid(row=3, column=0, padx=10, pady=5, sticky="nsew")
        self.tdo_canvases = {}
        for i in range(2):
            f = ttk.Frame(tdo_frame)
            f.grid(row=0, column=i, padx=4, pady=4)
            tk.Label(f, text=f"TDO{i}", font=("Consolas", 10)).pack()
            cv = tk.Canvas(f, width=28, height=28, highlightthickness=0)
            cv.pack()
            self._draw(cv, None)
            self.tdo_canvases[i] = cv
            ttk.Button(f, text="Toggle", width=6,
                       command=lambda ch=i: self._toggle_tool_do(ch)).pack(pady=2)

        ctrl_frame = ttk.LabelFrame(self.root, text="Control", padding=10)
        ctrl_frame.grid(row=4, column=0, padx=10, pady=10, sticky="nsew")
        di_ch = list(DI_DO_MAP.keys())[0]
        do_ch = list(DI_DO_MAP.values())[0]
        ttk.Checkbutton(
            ctrl_frame,
            text=f"DI->DO passthrough (DI{di_ch} -> DO{do_ch})",
            variable=self.passthrough_var
        ).pack(anchor="w")

        ttk.Label(self.root, textvariable=self.status_var, relief="sunken", anchor="w").grid(
            row=5, column=0, sticky="ew", padx=5, pady=5
        )

    def _draw(self, canvas, level):
        canvas.delete("all")
        if level is None:
            color = COLOR_UNKNOWN
        elif level:
            color = COLOR_HIGH
        else:
            color = COLOR_LOW
        canvas.create_oval(2, 2, 26, 26, fill=color, outline="")

    def _toggle_do(self, channel):
        new_state = 1 - self.do_states.get(channel, 0)
        self.do_states[channel] = new_state
        self._draw(self.do_canvases[channel], new_state)
        try:
            self.robot.SetDO(id=channel, status=new_state)
        except Exception as e:
            print(f"SetDO error: {e}")

    def _toggle_tool_do(self, channel):
        new_state = 1 - self.tdo_states.get(channel, 0)
        self.tdo_states[channel] = new_state
        self._draw(self.tdo_canvases[channel], new_state)
        try:
            self.robot.SetToolDO(id=channel, status=new_state)
        except Exception as e:
            print(f"SetToolDO error: {e}")

    def _poll(self):
        """Read all I/O from state package and update UI directly on main thread."""
        try:
            pkg = self.robot.robot_state_pkg

            # DI0-DI7
            for i in range(8):
                level = (pkg.cl_dgt_input_l >> i) & 1
                self._draw(self.di_canvases[i], level)

            # DI8-DI15
            for i in range(8):
                level = (pkg.cl_dgt_input_h >> i) & 1
                self._draw(self.di_canvases[i + 8], level)

            # Tool DI (bits 1-2 per SDK)
            for i in range(2):
                level = (pkg.tl_dgt_input_l >> (i + 1)) & 1
                self._draw(self.tdi_canvases[i], level)

            # DO0-DO7
            for i in range(8):
                level = (pkg.cl_dgt_output_l >> i) & 1
                self.do_states[i] = level
                self._draw(self.do_canvases[i], level)

            # DO8-DO15
            for i in range(8):
                level = (pkg.cl_dgt_output_h >> i) & 1
                self.do_states[i + 8] = level
                self._draw(self.do_canvases[i + 8], level)

            # Tool DO
            for i in range(2):
                level = (pkg.tl_dgt_output_l >> i) & 1
                self.tdo_states[i] = level
                self._draw(self.tdo_canvases[i], level)

            # Passthrough
            if self.passthrough_var.get():
                for di_ch, do_ch in DI_DO_MAP.items():
                    di_level = (pkg.cl_dgt_input_l >> di_ch) & 1
                    do_level = (pkg.cl_dgt_output_l >> do_ch) & 1
                    if di_level != do_level:
                        self.robot.SetDO(id=do_ch, status=di_level)

            self.status_var.set("OK")

        except Exception as e:
            self.status_var.set(f"Error: {e}")

        # Schedule next poll
        self.root.after(POLL_MS, self._poll)


if __name__ == "__main__":
    root = tk.Tk()
    app = IOMonitor(root)
    root.mainloop()
