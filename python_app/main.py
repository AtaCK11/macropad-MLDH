#!/usr/bin/env python3
"""
MLDH Macropad Configurator

A small cross‑platform Python GUI to configure your macropad over USB CDC using
its text command protocol:
  - show
  - set key <row 0..2> <col 0..2> <macro>
  - set enc <0|1> <cw|ccw> <macro>
  - save
  - resetdefaults

It auto‑detects your device by USB string descriptors:
  Manufacturer:  MalDuhaFoundation
  Product:       MLDH_Macropad_r1
  Serial:        911112

New in this version:
  - Each of the 3×3 keys has a text field **and a Select button** (radio).
  - A **Virtual Keyboard** pops up from the Quick Macros section to choose
    key combos (e.g., Ctrl+Shift+A, Alt+F4). The combo is inserted into the
    currently selected key cell.

Tested with Python 3.9+ and pyserial.
"""
import threading
import time
from dataclasses import dataclass
from typing import Optional, List, Tuple

import tkinter as tk
from tkinter import ttk, messagebox

import serial
import serial.tools.list_ports

MANUFACTURER = "MalDuhaFoundation"
PRODUCT = "MLDH_Macropad_r1"
SERIAL_NUMBER = "911112"
PROMPT_BYTES = b">"
READ_TIMEOUT = 0.2  # seconds per poll


@dataclass
class PortInfo:
    device: str
    description: str
    manufacturer: Optional[str]
    product: Optional[str]
    serial_number: Optional[str]


class MacropadClient:
    def __init__(self):
        self.ser: Optional[serial.Serial] = None
        self.lock = threading.Lock()

    # ---- Port discovery ----
    @staticmethod
    def list_ports() -> List[PortInfo]:
        ports = []
        for p in serial.tools.list_ports.comports():
            ports.append(
                PortInfo(
                    device=p.device,
                    description=p.description or "",
                    manufacturer=getattr(p, "manufacturer", None),
                    product=getattr(p, "product", None),
                    serial_number=getattr(p, "serial_number", None),
                )
            )
        return ports

    @staticmethod
    def find_macropad_port() -> Optional[str]:
        for p in MacropadClient.list_ports():
            if (
                (p.manufacturer and MANUFACTURER in p.manufacturer)
                or (p.product and PRODUCT in p.product)
                or (p.serial_number and SERIAL_NUMBER in p.serial_number)
            ):
                return p.device
        # Fallback: common CDC names (Linux/macOS)
        for p in MacropadClient.list_ports():
            if any(x in p.device for x in ("ttyACM", "tty.usbmodem", "ttyUSB")):
                return p.device
        return None

    # ---- Serial helpers ----
    def connect(self, port: Optional[str] = None, baud: int = 115200) -> None:
        if self.ser and self.ser.is_open:
            return
        if port is None:
            port = self.find_macropad_port()
        if not port:
            raise RuntimeError("Macropad not found. Select a port manually.")
        self.ser = serial.Serial(port=port, baudrate=baud, timeout=READ_TIMEOUT)
        # Give the device a moment after opening the port
        time.sleep(0.25)
        self.flush()

    def disconnect(self) -> None:
        if self.ser:
            try:
                self.ser.close()
            finally:
                self.ser = None

    def flush(self) -> None:
        if not self.ser:
            return
        with self.lock:
            self.ser.reset_input_buffer()
            self.ser.reset_output_buffer()

    def _read_until_prompt(self, timeout: float = 2.0) -> bytes:
        if not self.ser:
            return b""
        deadline = time.time() + timeout
        buf = bytearray()
        while time.time() < deadline:
            with self.lock:
                chunk = self.ser.read(1024)
            if chunk:
                buf.extend(chunk)
                # Stop if we see a prompt on any line
                if PROMPT_BYTES in buf:
                    break
            else:
                time.sleep(0.01)
        return bytes(buf)

    def send_command(self, cmd: str, wait_for_prompt: bool = True, timeout: float = 2.0) -> str:
        if not self.ser or not self.ser.is_open:
            raise RuntimeError("Not connected")
        # Ensure trailing newline
        if not cmd.endswith(""):
            cmd += ""
        with self.lock:
            self.ser.write(cmd.encode("utf-8"))
            self.ser.flush()
        if wait_for_prompt:
            out = self._read_until_prompt(timeout)
            return out.decode("utf-8", errors="replace")
        return ""

    # Convenience wrappers
    def show(self) -> str:
        return self.send_command("show")

    def set_key(self, row: int, col: int, macro: str) -> str:
        return self.send_command(f"set key {row} {col} {macro}")

    def set_enc(self, idx: int, direction: str, macro: str) -> str:
        if direction not in ("cw", "ccw"):
            raise ValueError("direction must be 'cw' or 'ccw'")
        return self.send_command(f"set enc {idx} {direction} {macro}")

    def save(self) -> str:
        return self.send_command("save")

    def resetdefaults(self) -> str:
        return self.send_command("resetdefaults")


class App(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title("MLDH Macropad Configurator")
        self.geometry("980x720")
        self.client = MacropadClient()

        # Selected key index in the 3x3 grid (-1: none)
        self.selected_key_index = tk.IntVar(value=-1)

        self._build_ui()
        self._refresh_ports()

    # ---- UI ----
    def _build_ui(self):
        root = ttk.Frame(self, padding=10)
        root.pack(fill=tk.BOTH, expand=True)

        # Connection row
        conn = ttk.Frame(root)
        conn.pack(fill=tk.X)
        ttk.Label(conn, text="Serial Port:").pack(side=tk.LEFT)
        self.port_var = tk.StringVar()
        self.port_combo = ttk.Combobox(conn, textvariable=self.port_var, width=40)
        self.port_combo.pack(side=tk.LEFT, padx=6)
        ttk.Button(conn, text="Refresh", command=self._refresh_ports).pack(side=tk.LEFT)
        ttk.Button(conn, text="Auto‑detect", command=self._auto_detect).pack(side=tk.LEFT, padx=(6,0))
        self.connect_btn = ttk.Button(conn, text="Connect", command=self._connect)
        self.connect_btn.pack(side=tk.LEFT, padx=(12,0))
        self.disconnect_btn = ttk.Button(conn, text="Disconnect", command=self._disconnect, state=tk.DISABLED)
        self.disconnect_btn.pack(side=tk.LEFT, padx=6)

        # Selected label
        selbar = ttk.Frame(root)
        selbar.pack(fill=tk.X, pady=(8,4))
        ttk.Label(selbar, text="Selected key:").pack(side=tk.LEFT)
        self.selected_label = ttk.Label(selbar, text="(none)")
        self.selected_label.pack(side=tk.LEFT, padx=6)
        self.selected_key_index.trace_add("write", lambda *_: self._update_selected_label())

        # Help bar
        help_bar = ttk.Frame(root)
        help_bar.pack(fill=tk.X, pady=(0,4))
        ttk.Label(help_bar, text="Macros e.g.: 'ctrl+c', 'win+shift+s', 'alt+f4', or hex '0xMM:0xKK'/'0xKK'").pack(side=tk.LEFT)

        # Grid for keys (3x3) — each cell: Entry + Select radio
        grid = ttk.LabelFrame(root, text="Keys (3 × 3) – click Select, then use Virtual Keyboard below")
        grid.pack(fill=tk.X, pady=6)
        self.key_vars = [[tk.StringVar() for _ in range(3)] for _ in range(3)]
        self.key_entries: List[List[ttk.Entry]] = [[None]*3 for _ in range(3)]  # type: ignore

        for r in range(3):
            rowf = ttk.Frame(grid)
            rowf.pack(fill=tk.X, pady=2)
            for c in range(3):
                cell = ttk.Frame(rowf, padding=6, borderwidth=1, relief="groove")
                cell.pack(side=tk.LEFT, expand=True, fill=tk.X, padx=4)
                ttk.Label(cell, text=f"Key {r},{c}").pack(anchor=tk.W)
                ent = ttk.Entry(cell, textvariable=self.key_vars[r][c], width=24)
                ent.pack(fill=tk.X, pady=(2,2))
                # Auto-select when entry focused
                idx = r*3 + c
                ent.bind("<FocusIn>", lambda _e, i=idx: self.selected_key_index.set(i))
                rb = ttk.Radiobutton(cell, text="Select", value=idx, variable=self.selected_key_index)
                rb.pack(anchor=tk.W)
                self.key_entries[r][c] = ent

        # Encoder section
        encf = ttk.LabelFrame(root, text="Encoders")
        encf.pack(fill=tk.X, pady=6)
        self.enc_vars = {
            (0, "cw"): tk.StringVar(),
            (0, "ccw"): tk.StringVar(),
            (1, "cw"): tk.StringVar(),
            (1, "ccw"): tk.StringVar(),
        }
        for idx in (0, 1):
            row = ttk.Frame(encf)
            row.pack(fill=tk.X, pady=2)
            ttk.Label(row, text=f"Encoder {idx} CW:").pack(side=tk.LEFT)
            ttk.Entry(row, textvariable=self.enc_vars[(idx, "cw")], width=22).pack(side=tk.LEFT, padx=4)
            ttk.Label(row, text=f"CCW:").pack(side=tk.LEFT)
            ttk.Entry(row, textvariable=self.enc_vars[(idx, "ccw")], width=22).pack(side=tk.LEFT, padx=4)

        # Quick macros + Virtual Keyboard
        quick = ttk.LabelFrame(root, text="Quick macros & Virtual Keyboard")
        quick.pack(fill=tk.X, pady=6)
        ttk.Button(quick, text="Ctrl+C", command=lambda: self._insert_macro("ctrl+c")).pack(side=tk.LEFT, padx=4)
        ttk.Button(quick, text="Win+Shift+S", command=lambda: self._insert_macro("win+shift+s")).pack(side=tk.LEFT, padx=4)
        ttk.Button(quick, text="Alt+F4", command=lambda: self._insert_macro("alt+f4")).pack(side=tk.LEFT, padx=4)
        ttk.Button(quick, text="Hex 0x1D:0x06 (Ctrl+Z)", command=lambda: self._insert_macro("0x1D:0x06")).pack(side=tk.LEFT, padx=4)
        ttk.Separator(quick, orient=tk.VERTICAL).pack(side=tk.LEFT, fill=tk.Y, padx=8)
        ttk.Button(quick, text="Open Virtual Keyboard…", command=self._open_keyboard_dialog).pack(side=tk.LEFT)

        # Actions
        actions = ttk.Frame(root)
        actions.pack(fill=tk.X, pady=6)
        ttk.Button(actions, text="Write All", command=self._write_all).pack(side=tk.LEFT)
        ttk.Button(actions, text="Save to Device", command=self._save).pack(side=tk.LEFT, padx=4)
        ttk.Button(actions, text="Show Config", command=self._show).pack(side=tk.LEFT, padx=4)
        ttk.Button(actions, text="Reset Defaults", command=self._reset).pack(side=tk.LEFT, padx=4)

     # Log output
        logf = ttk.LabelFrame(root, text="Device log")
        logf.pack(fill=tk.BOTH, expand=True, pady=(8,0))
        self.log = tk.Text(logf, height=12)
        self.log.pack(fill=tk.BOTH, expand=True)

    # ---- UI helpers ----
    def _update_selected_label(self):
        idx = self.selected_key_index.get()
        if idx < 0:
            self.selected_label.configure(text="(none)")
        else:
            r, c = divmod(idx, 3)
            self.selected_label.configure(text=f"Row {r}, Col {c}")

    def _log(self, text: str):
        self.log.insert(tk.END, text + "")
        self.log.see(tk.END)

    def _refresh_ports(self):
        ports = self.client.list_ports()
        items = [f"{p.device} — {p.description}" for p in ports]
        self.port_combo["values"] = items
        if not items:
            self.port_var.set("")
        else:
            # Prefer auto-detected port
            auto = self.client.find_macropad_port()
            if auto:
                for i, p in enumerate(ports):
                    if p.device == auto:
                        self.port_combo.current(i)
                        break

    def _auto_detect(self):
        port = self.client.find_macropad_port()
        if port:
            for i, p in enumerate(self.client.list_ports()):
                if p.device == port:
                    self.port_combo.current(i)
                    self._log(f"Auto‑detected port: {port}")
                    return
        messagebox.showwarning("Not found", "Couldn't auto‑detect the macropad.")

    def _selected_port(self) -> Optional[str]:
        label = self.port_var.get()
        if not label:
            return None
        return label.split(" — ")[0]

    def _connect(self):
        try:
            port = self._selected_port()
            self.client.connect(port)
            self._log(f"Connected to {self.client.ser.port}")
            self.connect_btn.configure(state=tk.DISABLED)
            self.disconnect_btn.configure(state=tk.NORMAL)
            # Ask for help to warm up the prompt
            out = self.client.send_command("help")
            self._log(out.strip())
        except Exception as e:
            messagebox.showerror("Connection error", str(e))

    def _disconnect(self):
        self.client.disconnect()
        self._log("Disconnected")
        self.connect_btn.configure(state=tk.NORMAL)
        self.disconnect_btn.configure(state=tk.DISABLED)

    def _apply_combo_to_selected(self, combo: str):
        idx = self.selected_key_index.get()
        if idx < 0:
            messagebox.showwarning("No key selected", "Select a key in the 3×3 grid first.")
            return
        r, c = divmod(idx, 3)
        self.key_vars[r][c].set(combo)

    def _insert_macro(self, macro: str):
        # Insert into selected key cell if any; otherwise, currently focused entry
        idx = self.selected_key_index.get()
        if idx >= 0:
            r, c = divmod(idx, 3)
            self.key_vars[r][c].set(macro)
            return
        w = self.focus_get()
        if isinstance(w, ttk.Entry):
            w.insert(tk.INSERT, macro)
        else:
            self.key_vars[0][0].set(macro)

    # ---- Virtual Keyboard ----
    def _open_keyboard_dialog(self):
        dlg = tk.Toplevel(self)
        dlg.title("Virtual Keyboard – choose a combo")
        dlg.transient(self)
        dlg.grab_set()

        body = ttk.Frame(dlg, padding=10)
        body.pack(fill=tk.BOTH, expand=True)

        # Modifier checkboxes
        mods_frame = ttk.LabelFrame(body, text="Modifiers")
        mods_frame.pack(fill=tk.X)
        mod_vars = {
            "ctrl": tk.BooleanVar(value=False),
            "shift": tk.BooleanVar(value=False),
            "alt": tk.BooleanVar(value=False),
            "win": tk.BooleanVar(value=False),
        }
        for name in ("ctrl", "shift", "alt", "win"):
            ttk.Checkbutton(mods_frame, text=name.capitalize(), variable=mod_vars[name]).pack(side=tk.LEFT, padx=6)

        # Main key selection
        key_frame = ttk.LabelFrame(body, text="Keys")
        key_frame.pack(fill=tk.BOTH, expand=True, pady=(8,0))
        chosen = tk.StringVar(value="")

        def set_key(k: str):
            chosen.set(k)
            update_label()

        # Rows: numbers, letters, function keys, misc
        row1 = ttk.Frame(key_frame); row1.pack(fill=tk.X, pady=2)
        for k in list("1234567890"):
            ttk.Button(row1, text=k, width=3, command=lambda x=k: set_key(x)).pack(side=tk.LEFT, padx=2)

        # letters A-Z
        import string
        row2 = ttk.Frame(key_frame); row2.pack(fill=tk.X, pady=2)
        for k in string.ascii_uppercase:
            ttk.Button(row2, text=k, width=3, command=lambda x=k.lower(): set_key(x)).pack(side=tk.LEFT, padx=2)

        # function keys
        row3 = ttk.Frame(key_frame); row3.pack(fill=tk.X, pady=2)
        for i in range(1, 13):
            ttk.Button(row3, text=f"F{i}", width=4, command=lambda x=f"f{i}": set_key(x)).pack(side=tk.LEFT, padx=2)

        # Misc keys
        misc = ["tab","enter","esc","space","backspace","delete","home","end","pgup","pgdn","left","right","up","down"]
        row4 = ttk.Frame(key_frame); row4.pack(fill=tk.X, pady=2)
        for k in misc:
            ttk.Button(row4, text=k.title(), width=9, command=lambda x=k: set_key(x)).pack(side=tk.LEFT, padx=2)

        # Current combo preview
        preview = ttk.Frame(body); preview.pack(fill=tk.X, pady=8)
        ttk.Label(preview, text="Current combo:").pack(side=tk.LEFT)
        combo_lbl = ttk.Label(preview, text="(none)")
        combo_lbl.pack(side=tk.LEFT, padx=6)

        def build_combo() -> str:
            parts = [name for name in ("ctrl","shift","alt","win") if mod_vars[name].get()]
            k = chosen.get()
            if k:
                parts.append(k)
            return "+".join(parts) if parts else ""

        def update_label():
            combo_lbl.configure(text=build_combo() or "(none)")

        btns = ttk.Frame(body); btns.pack(fill=tk.X)
        def on_clear():
            for v in mod_vars.values():
                v.set(False)
            chosen.set("")
            update_label()
        ttk.Button(btns, text="Clear", command=on_clear).pack(side=tk.LEFT)
        ttk.Button(btns, text="Insert into selected key", command=lambda: (self._apply_combo_to_selected(build_combo()), dlg.destroy())).pack(side=tk.RIGHT)
        ttk.Button(btns, text="Cancel", command=dlg.destroy).pack(side=tk.RIGHT, padx=4)

        update_label()
        dlg.wait_window()

    # ---- Device actions ----
    def _write_all(self):
        if not self.client.ser:
            messagebox.showwarning("Not connected", "Connect to the device first.")
            return
        # Keys
        for r in range(3):
            for c in range(3):
                val = self.key_vars[r][c].get().strip()
                if val:
                    out = self.client.set_key(r, c, val)
                    self._log(out.strip())
        # Encoders
        for idx in (0, 1):
            for d in ("cw", "ccw"):
                val = self.enc_vars[(idx, d)].get().strip()
                if val:
                    out = self.client.set_enc(idx, d, val)
                    self._log(out.strip())
        self._log("Write completed. Consider clicking 'Save to Device'.")

    def _save(self):
        if not self.client.ser:
            messagebox.showwarning("Not connected", "Connect to the device first.")
            return
        out = self.client.save()
        self._log(out.strip())

    def _reset(self):
        if not self.client.ser:
            messagebox.showwarning("Not connected", "Connect to the device first.")
            return
        if messagebox.askyesno("Reset defaults", "Reset device to factory defaults?"):
            out = self.client.resetdefaults()
            self._log(out.strip())

    def _show(self):
        if not self.client.ser:
            messagebox.showwarning("Not connected", "Connect to the device first.")
            return
        out = self.client.show()
        self._log(out.strip())


if __name__ == "__main__":
    try:
        App().mainloop()
    except KeyboardInterrupt:
        pass
