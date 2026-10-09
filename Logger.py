#!/usr/bin/env python3
"""
USB-CDC binary log viewer
=========================

Reads fixed-size LogMessage records from a (virtual) COM port and shows them
in a filterable, colour-coded table.

    pip install pyserial
    python log_viewer.py COM5
    python log_viewer.py /dev/ttyACM0

Record layout (little-endian), matching the C++ struct:

    uint32_t              timeStamp   ms since start-up
    Severity              severity    0 = Info, 1 = Warning, 2 = Error,
                                      3 = Heartbeat (dashboard only, never shown in the log)
    std::array<char, 32>  module
    std::array<char,128>  message
    std::array<char, 32>  fileName
    uint16_t              line

The size of `Severity` and the struct packing depend on your firmware:

    enum class Severity : uint8_t   -> --severity-size 1   (default)
    enum class Severity            -> --severity-size 4   (plain int enum)
    __attribute__((packed))        -> add --packed

    natural alignment, 1-byte enum = 200 bytes
    natural alignment, 4-byte enum = 204 bytes
    packed,            1-byte enum = 199 bytes
"""
from __future__ import annotations

import argparse
import bisect
import os
import queue
import struct
import sys
import threading
import time
import tkinter as tk
from dataclasses import dataclass, field
from tkinter import font as tkfont
from tkinter import ttk

import serial
from serial.tools import list_ports

# --------------------------------------------------------------------------- #
# Settings
# --------------------------------------------------------------------------- #
DEFAULT_PORT = "COM3"          # e.g. "/dev/ttyACM0" on Linux, "/dev/cu.usbmodem1101" on macOS
DEFAULT_BAUD = 115200          # irrelevant for USB CDC, but pyserial wants a value
MAX_MESSAGES = 50_000          # oldest messages are discarded beyond this
SCAN_INTERVAL_S = 0.5          # how often we look for the port while it is absent
UI_POLL_MS = 30                # how often the GUI drains the receive queue
MAX_PER_TICK = 5000            # max messages handled per GUI tick
MAX_RESYNC_GAP_MS = 24 * 3600 * 1000   # max forward timestamp step between records when re-syncing
MAX_BACKSTEP_MS = 1000                 # tolerated out-of-order timestamp (records from different threads)
RESYNC_RECORDS = 4                     # consecutive valid records needed to accept a record boundary

# Severity -> (name, text colour).
# Pure #FFFF00 is unreadable on white, so "yellow" is a dark amber.
SEVERITIES: dict[int, tuple[str, str]] = {
    0: ("Info", "#000000"),
    1: ("Warning", "#B08000"),
    2: ("Error", "#D00000"),
}
HEARTBEAT = 3                      # severity 3: module heartbeat -> dashboard, never in the log
VALID_SEVERITIES = set(SEVERITIES) | {HEARTBEAT}

BG = "#f4f5f7"
ROW_EVEN = "#ffffff"
ROW_ODD = "#f6f7f9"
SELECT_BG = "#cfe3ff"
OK_COLOR = "#2e9e4f"
WAIT_COLOR = "#e0a100"

# Dashboard
CARD_BG = "#ffffff"
CARD_BORDER = "#d5d9e0"
SELECT_BORDER = "#2563eb"
NEUTRAL_COLOR = "#9ca3af"
ERROR_COLOR = SEVERITIES[2][1]
DASH_HEIGHT = 210              # initial height of the dashboard pane (drag the divider to resize)
CARD_MIN_W = 330               # cards are laid out in as many columns as fit
DASH_REFRESH_MS = 250
HB_STALE_FACTOR = 3            # a heartbeat is "silent" after this many missed periods ...
HB_STALE_MIN_S = 1.5           # ... but never sooner than this many seconds
# What identifies one heartbeat inside a module:
#   "location" = the source file:line that emits it (message text may change, e.g. carry a state)
#   "message"  = the message text (use this if several heartbeats share one logging call site)
BEAT_KEY = "location"


# --------------------------------------------------------------------------- #
# Record decoding
# --------------------------------------------------------------------------- #
@dataclass
class LogMessage:
    timestamp_ms: int
    severity: int
    module: str
    message: str
    file: str
    line: int
    seq: int = 0  # running number assigned by the GUI


def location_of(m: LogMessage) -> str:
    return f"{m.file}:{m.line}" if m.file else ""


@dataclass
class Beat:
    """One heartbeat source inside a module."""
    message: str = ""            # latest heartbeat text (may carry state information)
    count: int = 0
    last_device_ms: int = 0
    last_seen: float = 0.0       # host time.monotonic() of the last beat
    period_ms: float = 0.0       # smoothed interval between beats (device clock)

    def update(self, message: str, device_ms: int, now: float) -> None:
        if self.count:
            step = device_ms - self.last_device_ms
            if 0 < step < 60_000:            # ignore resets / duplicate timestamps
                self.period_ms = step if not self.period_ms else 0.8 * self.period_ms + 0.2 * step
        self.message = message
        self.count += 1
        self.last_device_ms = device_ms
        self.last_seen = now

    def silent_for(self, now: float) -> float | None:
        """Seconds of silence if the beat is overdue, otherwise None."""
        age = now - self.last_seen
        limit = max(HB_STALE_MIN_S, HB_STALE_FACTOR * self.period_ms / 1000)
        return age if age > limit else None


@dataclass
class ModuleStats:
    name: str
    beats: dict = field(default_factory=dict)                 # beat key -> Beat
    counts: list = field(default_factory=lambda: [0, 0, 0])   # Info / Warning / Error
    last_problem: LogMessage | None = None                    # last warning or error

    def beat_from(self, m: LogMessage, now: float) -> None:
        key = location_of(m) if BEAT_KEY == "location" else m.message
        beat = self.beats.get(key)
        if beat is None:
            beat = self.beats[key] = Beat()
        beat.update(m.message, m.timestamp_ms, now)


# Bytes we accept inside a text field (printable ASCII, UTF-8 continuation bytes, \t \r \n)
_TEXT_OK = bytes([9, 10, 13] + list(range(32, 127)) + list(range(128, 256)))


def _is_text(raw: bytes) -> bool:
    head = raw.split(b"\0", 1)[0]
    return not head.translate(None, _TEXT_OK)


def _cstr(raw: bytes) -> str:
    return raw.split(b"\0", 1)[0].decode("utf-8", errors="replace")


class RecordCodec:
    """Builds the struct format and turns raw bytes into LogMessage objects."""

    def __init__(self, severity_size: int = 1, packed: bool = False):
        sev = {1: "B", 2: "H", 4: "I"}[severity_size]
        if packed:
            fmt = f"<I{sev}32s128s32sH"
        else:
            # Natural alignment, as the compiler would lay it out.
            offset = 4 + severity_size + 32 + 128 + 32
            pad_line = offset % 2                      # uint16_t line needs 2-byte alignment
            offset += pad_line + 2
            tail = -offset % max(4, severity_size)     # struct size is a multiple of its alignment
            fmt = f"<I{sev}32s128s32s{'x' * pad_line}H{'x' * tail}"
        self.struct = struct.Struct(fmt)
        self.size = self.struct.size

    def parse(self, raw: bytes, strict: bool = False) -> LogMessage | None:
        """
        Decode one record. Returns None if it does not look like a valid record.

        `strict` is used while hunting for the record boundary after connecting
        mid-stream: zero padding makes a shifted window look like a valid (empty)
        record, so we additionally require a non-empty, NUL-terminated module,
        a non-empty message and a NUL-terminated file name.
        """
        ts, sev, module, message, filename, line = self.struct.unpack(raw)
        if sev not in VALID_SEVERITIES:
            return None
        if not (_is_text(module) and _is_text(message) and _is_text(filename)):
            return None
        if strict and not (module[0] != 0 and b"\0" in module and (message[0] != 0 or sev == HEARTBEAT) and b"\0" in filename):
            return None
        return LogMessage(
            timestamp_ms=ts,
            severity=sev,
            module=_cstr(module),
            message=_cstr(message).replace("\r", "").replace("\n", " ").rstrip(),
            file=_cstr(filename),
            line=line,
        )


def format_uptime(ms: int) -> str:
    """12345678 -> '03:25:45.678' (days are prefixed only when needed)."""
    s, ms = divmod(ms, 1000)
    m, s = divmod(s, 60)
    h, m = divmod(m, 60)
    d, h = divmod(h, 24)
    prefix = f"{d}d " if d else ""
    return f"{prefix}{h:02d}:{m:02d}:{s:02d}.{ms:03d}"


# --------------------------------------------------------------------------- #
# Serial reader thread (auto-connect / auto-reconnect)
# --------------------------------------------------------------------------- #
class SerialReader(threading.Thread):
    """
    Watches for the port to appear, opens it, decodes records and pushes
    them to `self.queue`. If the device disappears it goes back to waiting.

    Queue items:
        ("msg", LogMessage)
        ("status", "connected" | "waiting", text)
        ("skipped", n_bytes)
    """

    def __init__(self, port: str, baud: int, codec: RecordCodec):
        super().__init__(daemon=True)
        self.port = port            # may be changed from the GUI thread at any time
        self.baud = baud
        self.codec = codec
        self.queue: queue.Queue = queue.Queue()
        self._stop_evt = threading.Event()
        self._last_status: tuple[str, str] | None = None
        self._synced = False

    def stop(self) -> None:
        self._stop_evt.set()

    # -- helpers ----------------------------------------------------------- #
    def _status(self, state: str, text: str) -> None:
        if (state, text) != self._last_status:
            self._last_status = (state, text)
            self.queue.put(("status", state, text))

    @staticmethod
    def _port_present(name: str) -> bool:
        if not name:
            return False
        wanted = name.lower()
        try:
            if any(p.device.lower() == wanted for p in list_ports.comports()):
                return True
        except Exception:
            pass
        return os.path.exists(name)

    @staticmethod
    def _close(ser: serial.Serial | None) -> None:
        try:
            if ser is not None:
                ser.close()
        except Exception:
            pass

    def _drain(self, buf: bytearray) -> None:
        """
        Decode as many records as possible from `buf`.

        A USB CDC stream has no framing, so after connecting (or after a corrupt
        record) we have to find the record boundary ourselves. While "unsynced"
        we slide forward one byte at a time until RESYNC_RECORDS consecutive records
        decode cleanly with advancing timestamps. Once synced,
        records are simply taken every `size` bytes.
        """
        size = self.codec.size
        skipped = 0
        while True:
            if self._synced:
                if len(buf) < size:
                    break
                msg = self.codec.parse(bytes(buf[:size]))
                if msg is not None:
                    del buf[:size]
                    self.queue.put(("msg", msg))
                    continue
                self._synced = False          # corrupt record -> look for the boundary again

            if len(buf) < RESYNC_RECORDS * size:
                break
            recs = []
            for i in range(RESYNC_RECORDS):
                rec = self.codec.parse(bytes(buf[i * size:(i + 1) * size]), strict=True)
                if rec is None:
                    break
                recs.append(rec)
            if len(recs) == RESYNC_RECORDS and self._plausible_sequence(recs):
                self._synced = True
            else:
                del buf[0]
                skipped += 1
        if skipped:
            self.queue.put(("skipped", skipped))

    @staticmethod
    def _plausible_sequence(recs: list[LogMessage]) -> bool:
        """
        Timestamps of real, consecutive records advance (allowing tiny out-of-order steps).
        A wrongly aligned window usually reads constant bytes (e.g. zero padding) as the
        'timestamp', so requiring overall progress rejects it even when every field looks valid.
        """
        ts = [r.timestamp_ms for r in recs]
        steps = [b - a for a, b in zip(ts, ts[1:])]
        return all(-MAX_BACKSTEP_MS <= d <= MAX_RESYNC_GAP_MS for d in steps) and ts[-1] > ts[0]

    def _on_idle(self, buf: bytearray) -> None:
        """No data for a moment. A buffer holding whole records only means we are aligned."""
        if not self._synced and buf and len(buf) % self.codec.size == 0:
            self._synced = True
            self._drain(buf)

    # -- main loop --------------------------------------------------------- #
    def run(self) -> None:
        ser: serial.Serial | None = None
        connected_port = ""
        buf = bytearray()

        while not self._stop_evt.is_set():
            port = self.port

            if ser is not None and port != connected_port:   # user picked another port
                self._close(ser)
                ser = None

            if ser is None:
                if not self._port_present(port):
                    self._status("waiting", f"Waiting for {port or '(no port set)'} …")
                    self._stop_evt.wait(SCAN_INTERVAL_S)
                    continue
                try:
                    ser = serial.Serial(port, self.baud, timeout=0.1)
                except (serial.SerialException, OSError, ValueError) as exc:
                    self._status("waiting", f"Cannot open {port}: {exc}")
                    self._stop_evt.wait(1.0)
                    continue
                connected_port = port
                buf.clear()
                self._synced = False
                self._status("connected", f"Connected to {port}")

            try:
                data = ser.read(ser.in_waiting or 1)
            except (serial.SerialException, OSError):
                # Device unplugged / reset: go back to waiting for it.
                self._close(ser)
                ser = None
                continue

            if data:
                buf.extend(data)
                self._drain(buf)
            else:
                self._on_idle(buf)

        self._close(ser)


# --------------------------------------------------------------------------- #
# GUI
# --------------------------------------------------------------------------- #
def pick_font(candidates: tuple[str, ...], fallback: str) -> str:
    available = set(tkfont.families())
    for name in candidates:
        if name in available:
            return name
    return fallback


class ModuleCard(tk.Frame):
    """Dashboard tile for one module: heartbeat health, log counters, last problem."""

    WRAP = 290

    def __init__(self, parent, name: str, ui_font: str, on_click):
        super().__init__(parent, bg=CARD_BORDER, padx=2, pady=2)
        self.name = name
        self._ui = ui_font
        self._on_click = on_click
        self._beat_labels: dict[str, tk.Label] = {}

        inner = tk.Frame(self, bg=CARD_BG)
        inner.pack(fill="both", expand=True)
        self.stripe = tk.Frame(inner, width=6, bg=NEUTRAL_COLOR)
        self.stripe.pack(side="left", fill="y")
        body = tk.Frame(inner, bg=CARD_BG, padx=10, pady=8)
        body.pack(side="left", fill="both", expand=True)

        head = tk.Frame(body, bg=CARD_BG)
        head.pack(fill="x")
        tk.Label(head, text=name or "(none)", bg=CARD_BG, font=(ui_font, 11, "bold"), anchor="w").pack(side="left")
        self.status = tk.Label(head, bg=CARD_BG, font=(ui_font, 9, "bold"))
        self.status.pack(side="right")

        self.beats_box = tk.Frame(body, bg=CARD_BG)
        self.beats_box.pack(fill="x", pady=(6, 4))

        counts = tk.Frame(body, bg=CARD_BG)
        counts.pack(fill="x")
        self.count_labels = []
        for _ in SEVERITIES:
            lbl = tk.Label(counts, bg=CARD_BG, font=(ui_font, 9))
            lbl.pack(side="left", padx=(0, 12))
            self.count_labels.append(lbl)

        self.problem = tk.Label(body, bg=CARD_BG, font=(ui_font, 9), anchor="w", justify="left",
                                wraplength=self.WRAP)
        self.problem.pack(fill="x", pady=(4, 0))
        self._bind_click(self)

    def _bind_click(self, widget) -> None:
        widget.configure(cursor="hand2")
        widget.bind("<Button-1>", lambda _e: self._on_click(self.name))
        for child in widget.winfo_children():
            self._bind_click(child)

    def refresh(self, stats: ModuleStats, now: float, connected: bool, selected: bool) -> None:
        self.configure(bg=SELECT_BORDER if selected else CARD_BORDER)

        states = []
        for key, beat in stats.beats.items():
            lbl = self._beat_labels.get(key)
            if lbl is None:
                lbl = tk.Label(self.beats_box, bg=CARD_BG, font=(self._ui, 10), anchor="w",
                               justify="left", wraplength=self.WRAP)
                lbl.pack(fill="x")
                self._bind_click(lbl)
                self._beat_labels[key] = lbl
            silent = beat.silent_for(now)
            if not connected:
                state, detail, colour = "offline", "offline", NEUTRAL_COLOR
            elif silent is not None:
                state, detail, colour = "silent", f"silent {silent:.1f} s", ERROR_COLOR
            else:
                state, colour = "alive", OK_COLOR
                detail = f"{beat.period_ms:.0f} ms" if beat.period_ms else "…"
            states.append(state)
            lbl.configure(text=f"●  {beat.message or 'heartbeat'}   ·   {detail}", fg=colour)

        if not states:
            status, colour = "NO HEARTBEAT", NEUTRAL_COLOR
        elif "silent" in states:
            status, colour = "SILENT", ERROR_COLOR
        elif "offline" in states:
            status, colour = "OFFLINE", NEUTRAL_COLOR
        else:
            status, colour = "ALIVE", OK_COLOR
        self.status.configure(text=status, fg=colour)
        self.stripe.configure(bg=colour)

        for sev, lbl in enumerate(self.count_labels):
            name, sev_colour = SEVERITIES[sev]
            n = stats.counts[sev]
            lbl.configure(text=f"{name} {n:,}", fg=sev_colour if n else NEUTRAL_COLOR)

        p = stats.last_problem
        if p is None:
            self.problem.configure(text="")
        else:
            msg = p.message if len(p.message) <= 80 else p.message[:77] + "…"
            icon = "✖" if p.severity == 2 else "▲"
            self.problem.configure(text=f"{icon} {msg}", fg=SEVERITIES[p.severity][1])


class LogViewer(tk.Tk):
    COLUMNS = (
        # id, heading, width, anchor, stretch
        ("time", "Time", 120, "w", False),
        ("level", "Level", 80, "center", False),
        ("module", "Module", 140, "w", False),
        ("message", "Message", 640, "w", True),
        ("location", "Location", 220, "w", False),
    )

    def __init__(self, reader: SerialReader):
        super().__init__()
        self.reader = reader

        self.messages: list[LogMessage] = []
        self.modules: list[str] = []          # sorted, parallel to the listbox rows
        self._mod_set: set[str] = set()
        self._seq = 0
        self._shown = 0
        self._skipped = 0
        self._search_job: str | None = None
        self.stats: dict[str, ModuleStats] = {}
        self.cards: dict[str, ModuleCard] = {}
        self._connected = False
        self._dash_cols = 1

        # Snapshot of the active filter (so we don't query Tk variables per message)
        self._f_sev: set[int] = set(SEVERITIES)
        self._f_mods: set[str] = set()
        self._f_query = ""

        self.title("Log Viewer")
        self.geometry("1250x720")
        self.minsize(850, 400)

        self._build_style()
        self._build_toolbar()
        self._build_statusbar()
        self._build_body()

        self.protocol("WM_DELETE_WINDOW", self._on_close)
        self.reader.start()
        self.after(UI_POLL_MS, self._poll)
        self.after(DASH_REFRESH_MS, self._refresh_dashboard)

    # ---- construction ---------------------------------------------------- #
    def _build_style(self) -> None:
        self.configure(bg=BG)
        self.ui_font = pick_font(("Segoe UI", "SF Pro Text", "Helvetica Neue", "DejaVu Sans", "Ubuntu"), "TkDefaultFont")
        self.mono_font = pick_font(("Cascadia Mono", "Consolas", "Menlo", "DejaVu Sans Mono", "Courier New"), "TkFixedFont")

        s = ttk.Style(self)
        s.theme_use("clam")
        for w in ("TFrame", "TLabel", "TCheckbutton"):
            s.configure(w, background=BG, font=(self.ui_font, 10))
        s.configure("TButton", font=(self.ui_font, 10), padding=(12, 4))
        s.configure("Section.TLabel", foreground="#6b7280", font=(self.ui_font, 8, "bold"))
        s.configure("Status.TLabel", foreground="#4b5563", font=(self.ui_font, 9))
        for sev, (name, colour) in SEVERITIES.items():
            s.configure(f"{name}.TCheckbutton", foreground=colour, font=(self.ui_font, 10, "bold"))

        s.configure("Treeview", font=(self.mono_font, 10), rowheight=22,
                    background=ROW_EVEN, fieldbackground=ROW_EVEN, borderwidth=0)
        s.configure("Treeview.Heading", font=(self.ui_font, 9, "bold"), background="#e4e7ec",
                    foreground="#374151", relief="flat", padding=(8, 5))
        # Explicit map: also avoids the Tk 8.6.9 bug where tag colours are ignored.
        s.map("Treeview", background=[("selected", SELECT_BG)], foreground=[("disabled", "#a0a0a0")])
        s.map("Treeview.Heading", background=[("active", "#d7dbe2")])

    def _build_toolbar(self) -> None:
        bar = ttk.Frame(self, padding=(12, 10, 12, 6))
        bar.pack(side="top", fill="x")

        ttk.Label(bar, text="Port").pack(side="left")
        self.port_var = tk.StringVar(value=self.reader.port)
        self.port_box = ttk.Combobox(bar, textvariable=self.port_var, width=20,
                                     postcommand=self._refresh_ports)
        self.port_box.pack(side="left", padx=(8, 14))
        self.port_box.bind("<Return>", self._on_port_changed)
        self.port_box.bind("<<ComboboxSelected>>", self._on_port_changed)

        self.status_dot = tk.Label(bar, text="●", fg=WAIT_COLOR, bg=BG, font=(self.ui_font, 12))
        self.status_dot.pack(side="left")
        self.status_lbl = ttk.Label(bar, text="Starting …")
        self.status_lbl.pack(side="left", padx=(4, 0))

        ttk.Button(bar, text="Clear", command=self.clear).pack(side="right")
        self.autoscroll = tk.BooleanVar(value=True)
        ttk.Checkbutton(bar, text="Auto-scroll", variable=self.autoscroll,
                        command=self._on_autoscroll).pack(side="right", padx=14)

    def _build_statusbar(self) -> None:
        bar = ttk.Frame(self, padding=(12, 4, 12, 8))
        bar.pack(side="bottom", fill="x")
        self.count_lbl = ttk.Label(bar, text="", style="Status.TLabel")
        self.count_lbl.pack(side="left")

    def _build_body(self) -> None:
        body = ttk.Frame(self)
        body.pack(side="top", fill="both", expand=True)

        # ---- sidebar (filters)
        side = ttk.Frame(body, padding=(12, 6, 8, 6), width=220)
        side.pack(side="left", fill="y")
        side.pack_propagate(False)

        ttk.Label(side, text="SEVERITY", style="Section.TLabel").pack(anchor="w", pady=(0, 4))
        self.sev_vars: dict[int, tk.BooleanVar] = {}
        for sev, (name, _) in SEVERITIES.items():
            var = tk.BooleanVar(value=True)
            self.sev_vars[sev] = var
            ttk.Checkbutton(side, text=name, variable=var, style=f"{name}.TCheckbutton",
                            command=self._on_filter_changed).pack(anchor="w", pady=1)

        head = ttk.Frame(side)
        head.pack(fill="x", pady=(16, 4))
        ttk.Label(head, text="MODULES", style="Section.TLabel").pack(side="left")
        for text, sel in (("None", False), ("All", True)):
            lbl = tk.Label(head, text=text, fg="#2563eb", bg=BG, cursor="hand2",
                           font=(self.ui_font, 9, "underline"))
            lbl.pack(side="right", padx=(8, 0))
            lbl.bind("<Button-1>", lambda _e, s=sel: self._select_modules(s))

        list_frame = ttk.Frame(side)
        list_frame.pack(fill="both", expand=True)
        self.mod_list = tk.Listbox(
            list_frame, selectmode="multiple", exportselection=False, activestyle="none",
            font=(self.ui_font, 10), borderwidth=0, highlightthickness=1,
            highlightbackground="#c8ccd2", highlightcolor="#9ca3af",
            selectbackground=SELECT_BG, selectforeground="#000000", background="#ffffff",
        )
        lsb = ttk.Scrollbar(list_frame, orient="vertical", command=self.mod_list.yview)
        self.mod_list.configure(yscrollcommand=lsb.set)
        self.mod_list.pack(side="left", fill="both", expand=True)
        lsb.pack(side="right", fill="y")
        self.mod_list.bind("<<ListboxSelect>>", lambda _e: self._on_filter_changed())

        ttk.Label(side, text="SEARCH", style="Section.TLabel").pack(anchor="w", pady=(16, 4))
        self.search_var = tk.StringVar()
        ttk.Entry(side, textvariable=self.search_var).pack(fill="x")
        self.search_var.trace_add("write", lambda *_: self._debounce_filter())

        # ---- table
        right = ttk.Frame(body, padding=(0, 6, 12, 0))
        right.pack(side="left", fill="both", expand=True)
        paned = ttk.PanedWindow(right, orient="vertical")
        paned.pack(fill="both", expand=True)
        self._build_dashboard(paned)
        table = ttk.Frame(paned)
        paned.add(table, weight=1)
        table.rowconfigure(0, weight=1)
        table.columnconfigure(0, weight=1)

        ids = [c[0] for c in self.COLUMNS]
        self.tree = ttk.Treeview(table, columns=ids, show="headings", selectmode="extended")
        for cid, heading, width, anchor, stretch in self.COLUMNS:
            self.tree.heading(cid, text=heading, anchor="w")
            self.tree.column(cid, width=width, anchor=anchor, stretch=stretch, minwidth=50)

        vsb = ttk.Scrollbar(table, orient="vertical", command=self.tree.yview)
        hsb = ttk.Scrollbar(table, orient="horizontal", command=self.tree.xview)
        self.tree.configure(yscrollcommand=vsb.set, xscrollcommand=hsb.set)
        self.tree.grid(row=0, column=0, sticky="nsew")
        vsb.grid(row=0, column=1, sticky="ns")
        hsb.grid(row=1, column=0, sticky="ew")

        for sev, (name, colour) in SEVERITIES.items():
            font = (self.mono_font, 10, "bold") if sev == 2 else (self.mono_font, 10)
            self.tree.tag_configure(name.lower(), foreground=colour, font=font)
        self.tree.tag_configure("even", background=ROW_EVEN)
        self.tree.tag_configure("odd", background=ROW_ODD)

        self.tree.bind("<<Copy>>", self._copy_selection)
        self.tree.bind("<Control-a>", self._select_all)

        self._snapshot_filter()

    # ---- GUI event handlers ---------------------------------------------- #
    def _refresh_ports(self) -> None:
        try:
            self.port_box["values"] = [p.device for p in list_ports.comports()]
        except Exception:
            pass

    def _on_port_changed(self, _event=None) -> None:
        self.reader.port = self.port_var.get().strip()
        self.tree.focus_set()

    def _on_autoscroll(self) -> None:
        if self.autoscroll.get():
            self.tree.yview_moveto(1.0)

    def _select_modules(self, select: bool) -> None:
        if select:
            self.mod_list.selection_set(0, "end")
        else:
            self.mod_list.selection_clear(0, "end")
        self._on_filter_changed()

    def _debounce_filter(self) -> None:
        if self._search_job is not None:
            self.after_cancel(self._search_job)
        self._search_job = self.after(250, self._on_filter_changed)

    def _select_all(self, _event=None) -> str:
        self.tree.selection_set(self.tree.get_children())
        return "break"

    def _copy_selection(self, _event=None) -> str:
        selected = set(self.tree.selection())
        if selected:
            rows = [
                "\t".join(str(v) for v in self.tree.item(iid, "values"))
                for iid in self.tree.get_children() if iid in selected
            ]
            self.clipboard_clear()
            self.clipboard_append("\n".join(rows))
        return "break"

    def _on_close(self) -> None:
        self.reader.stop()
        self.destroy()

    # ---- filtering ------------------------------------------------------- #
    def _snapshot_filter(self) -> None:
        self._f_sev = {s for s, v in self.sev_vars.items() if v.get()}
        self._f_mods = {self.modules[i] for i in self.mod_list.curselection()}
        self._f_query = self.search_var.get().strip().lower()

    def _passes(self, m: LogMessage) -> bool:
        if m.severity not in self._f_sev or m.module not in self._f_mods:
            return False
        q = self._f_query
        if q and q not in m.message.lower() and q not in m.module.lower() and q not in m.file.lower():
            return False
        return True

    def _on_filter_changed(self) -> None:
        self._search_job = None
        self._rebuild()

    def _rebuild(self) -> None:
        self._snapshot_filter()
        self.tree.delete(*self.tree.get_children())
        self._shown = 0
        for m in self.messages:
            if self._passes(m):
                self._insert(m)
        if self.autoscroll.get():
            self.tree.yview_moveto(1.0)
        self._update_counts()

    # ---- data handling --------------------------------------------------- #
    def _insert(self, m: LogMessage) -> None:
        name = SEVERITIES[m.severity][0]
        location = f"{m.file}:{m.line}" if m.file else ""
        self.tree.insert(
            "", "end",
            values=(format_uptime(m.timestamp_ms), name, m.module, m.message, location),
            tags=(name.lower(), "odd" if m.seq % 2 else "even"),
        )
        self._shown += 1

    def _register_module(self, name: str) -> None:
        if name in self._mod_set:
            return
        self._mod_set.add(name)
        i = bisect.bisect_left(self.modules, name)
        self.modules.insert(i, name)
        self.mod_list.insert(i, name or "(none)")
        self.mod_list.selection_set(i)      # new modules are visible by default
        self._f_mods.add(name)
        self.stats[name] = ModuleStats(name)

    def _ingest(self, batch: list[LogMessage]) -> None:
        now = time.monotonic()
        for m in batch:
            self._register_module(m.module)             # also creates the dashboard card
            stats = self.stats[m.module]
            if m.severity == HEARTBEAT:                 # dashboard only: never stored, never logged
                if m.module not in self.cards:          # a module gets a card with its first beat
                    self._add_card(m.module)
                stats.beat_from(m, now)
                continue

            stats.counts[m.severity] += 1
            if m.severity:
                stats.last_problem = m
            self._seq += 1
            m.seq = self._seq
            self.messages.append(m)
            if self._passes(m):
                self._insert(m)

        excess = len(self.messages) - MAX_MESSAGES
        if excess > MAX_MESSAGES // 10:     # trim in chunks, not on every message
            dropped = self.messages[:excess]
            del self.messages[:excess]
            n_visible = sum(1 for m in dropped if self._passes(m))
            if n_visible:
                self.tree.delete(*self.tree.get_children()[:n_visible])
                self._shown -= n_visible

        if self.autoscroll.get():
            self.tree.yview_moveto(1.0)

    def clear(self) -> None:
        self.messages.clear()
        self.tree.delete(*self.tree.get_children())
        self._shown = 0
        self._skipped = 0
        for stats in self.stats.values():           # counters restart; heartbeat state is kept
            stats.counts = [0, 0, 0]
            stats.last_problem = None
        self._update_counts()

    # ---- dashboard ------------------------------------------------------- #
    def _build_dashboard(self, paned: ttk.PanedWindow) -> None:
        dash = ttk.Frame(paned)
        paned.add(dash, weight=0)

        head = ttk.Frame(dash)
        head.pack(fill="x", pady=(0, 4))
        ttk.Label(head, text="DASHBOARD", style="Section.TLabel").pack(side="left")
        ttk.Label(head, text="click a module to filter the log", style="Status.TLabel").pack(side="left", padx=10)

        wrap = ttk.Frame(dash)
        wrap.pack(fill="both", expand=True)
        self.dash_canvas = tk.Canvas(wrap, height=DASH_HEIGHT, width=600, bg=BG,
                                     highlightthickness=0, borderwidth=0)
        vsb = ttk.Scrollbar(wrap, orient="vertical", command=self.dash_canvas.yview)
        self.dash_canvas.configure(yscrollcommand=vsb.set)
        self.dash_canvas.pack(side="left", fill="both", expand=True)
        vsb.pack(side="right", fill="y")

        self.dash_inner = ttk.Frame(self.dash_canvas)
        self._dash_win = self.dash_canvas.create_window((0, 0), window=self.dash_inner, anchor="nw")
        self.dash_canvas.bind("<Configure>", self._on_dash_resize)
        self.dash_inner.bind("<Configure>",
                             lambda _e: self.dash_canvas.configure(scrollregion=self.dash_canvas.bbox("all")))
        self._dash_empty = ttk.Label(self.dash_inner, text="No heartbeats received yet …", style="Status.TLabel")
        self._dash_empty.grid(row=0, column=0, padx=8, pady=8)

    def _on_dash_resize(self, event) -> None:
        self.dash_canvas.itemconfigure(self._dash_win, width=event.width)
        self._reflow()

    def _add_card(self, name: str) -> None:
        self._dash_empty.grid_forget()
        self.cards[name] = ModuleCard(self.dash_inner, name, self.ui_font, self._toggle_module_filter)
        self._reflow()

    def _reflow(self) -> None:
        cols = max(1, self.dash_canvas.winfo_width() // CARD_MIN_W)
        for c in range(max(cols, self._dash_cols)):
            self.dash_inner.columnconfigure(c, weight=1 if c < cols else 0, uniform="cards" if c < cols else "")
        self._dash_cols = cols
        for i, name in enumerate(sorted(self.cards)):
            self.cards[name].grid(row=i // cols, column=i % cols, padx=6, pady=6, sticky="nsew")

    def _toggle_module_filter(self, name: str) -> None:
        """Click a card: show only that module in the log. Click again: show all modules."""
        if self._f_mods == {name}:
            self.mod_list.selection_set(0, "end")
        else:
            self.mod_list.selection_clear(0, "end")
            self.mod_list.selection_set(self.modules.index(name))
        self._on_filter_changed()

    def _refresh_dashboard(self) -> None:
        now = time.monotonic()
        for name, card in self.cards.items():
            card.refresh(self.stats[name], now, self._connected, self._f_mods == {name})
        self.after(DASH_REFRESH_MS, self._refresh_dashboard)

    def _set_status(self, state: str, text: str) -> None:
        connected = state == "connected"
        if connected and not self._connected:           # fresh connection: grace period for every heartbeat
            now = time.monotonic()
            for stats in self.stats.values():
                for beat in stats.beats.values():
                    beat.last_seen = now
        self._connected = connected
        self.status_dot.configure(fg=OK_COLOR if state == "connected" else WAIT_COLOR)
        self.status_lbl.configure(text=text)

    def _update_counts(self) -> None:
        text = f"Showing {self._shown:,} of {len(self.messages):,} messages"
        if self._skipped:
            text += f"   ·   {self._skipped:,} bytes skipped while re-syncing"
        self.count_lbl.configure(text=text)

    # ---- queue polling --------------------------------------------------- #
    def _poll(self) -> None:
        batch: list[LogMessage] = []
        try:
            for _ in range(MAX_PER_TICK):
                item = self.reader.queue.get_nowait()
                kind = item[0]
                if kind == "msg":
                    batch.append(item[1])
                elif kind == "status":
                    self._set_status(item[1], item[2])
                elif kind == "skipped":
                    self._skipped += item[1]
        except queue.Empty:
            pass

        if batch:
            self._ingest(batch)
        self._update_counts()
        self.after(UI_POLL_MS, self._poll)


# --------------------------------------------------------------------------- #
def main() -> None:
    ap = argparse.ArgumentParser(description="USB-CDC binary log viewer")
    ap.add_argument("port", nargs="?", default=DEFAULT_PORT,
                    help=f"serial port to watch (default: {DEFAULT_PORT})")
    ap.add_argument("-b", "--baud", type=int, default=DEFAULT_BAUD)
    ap.add_argument("--severity-size", type=int, choices=(1, 2, 4), default=1,
                    help="sizeof(Severity) in bytes (default: 1)")
    ap.add_argument("--packed", action="store_true",
                    help="struct is packed (no alignment padding)", default=True)
    args = ap.parse_args()

    if sys.platform == "win32":      # crisp text on high-DPI displays
        try:
            import ctypes
            ctypes.windll.shcore.SetProcessDpiAwareness(1)
        except Exception:
            pass

    codec = RecordCodec(args.severity_size, args.packed)
    reader = SerialReader(args.port, args.baud, codec)
    LogViewer(reader).mainloop()


if __name__ == "__main__":
    main()
