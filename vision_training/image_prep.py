#!/usr/bin/env python3
"""
Image Preparation Tool for RF-DETR
Dark-theme Tkinter GUI: browse images, resize, crop, grayscale, normalise, save.
Default target resolution: 1088 (configurable via the toolbar entry).
"""
from __future__ import annotations

import tkinter as tk
from tkinter import filedialog, messagebox, ttk
from pathlib import Path

from PIL import Image, ImageDraw, ImageOps, ImageTk
from matplotlib.figure import Figure
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg

# ---------------------------------------------------------------------------
# Palette
# ---------------------------------------------------------------------------
BG_ROOT   = "#12121e"
BG_PANEL  = "#1a1a2e"
BG_OPS    = "#13132a"
BG_CARD   = "#0f3460"
ACCENT    = "#e94560"
GREEN     = "#00b86b"
TEXT_HI   = "#eaeaea"
TEXT_LO   = "#7777aa"
BTN_BASE  = "#1f4068"
BTN_HOVER = "#2a5c90"
BTN_ACT   = "#c23152"
CANVAS_BG = "#0d0d1a"

SUPPORTED_EXTS = {".jpg", ".jpeg", ".png", ".bmp", ".tiff", ".tif", ".webp"}


# ---------------------------------------------------------------------------
# Widget helpers
# ---------------------------------------------------------------------------
def _btn(parent: tk.Widget, text: str, cmd,
         accent: bool = False, green: bool = False,
         width: int | None = None) -> tk.Button:
    bg   = ACCENT if accent else (GREEN if green else BTN_BASE)
    h_bg = BTN_ACT if accent else ("#009055" if green else BTN_HOVER)
    kw: dict = dict(text=text, command=cmd, bg=bg, fg=TEXT_HI,
                    activebackground=BTN_ACT, activeforeground=TEXT_HI,
                    relief=tk.FLAT, font=("Segoe UI", 9, "bold"),
                    padx=12, pady=6, cursor="hand2", bd=0)
    if width:
        kw["width"] = width
    b = tk.Button(parent, **kw)
    b.bind("<Enter>", lambda _e: b.configure(bg=h_bg))
    b.bind("<Leave>", lambda _e: b.configure(bg=bg))
    return b


def _sep(parent: tk.Widget):
    tk.Frame(parent, bg="#2e2e55", width=1).pack(side=tk.LEFT, fill=tk.Y, padx=7, pady=3)


# ---------------------------------------------------------------------------
# App
# ---------------------------------------------------------------------------
class ImagePrepApp:
    def __init__(self, root: tk.Tk):
        self.root = root
        root.title("RF-DETR Image Prep")
        root.configure(bg=BG_ROOT)
        root.minsize(1060, 680)

        # Image state
        self._images: list[Path] = []
        self._index   = 0
        self._original: Image.Image | None = None  # never mutated after load/save
        self._working:  Image.Image | None = None  # editable copy

        # Canvas geometry (set in _render)
        self._scale = 1.0
        self._off_x = 0
        self._off_y = 0
        self._tkimg: ImageTk.PhotoImage | None = None

        # Fixed crop state: res×res drag-to-position
        self._crop_active      = False
        self._crop_x           = 0
        self._crop_y           = 0
        self._drag_anchor:     tuple[int, int] | None = None
        self._drag_crop_start: tuple[int, int] | None = None

        # Free crop state: arbitrary rectangle drawn by user
        self._free_active  = False
        self._free_start:  tuple[int, int] | None = None
        self._free_end:    tuple[int, int] | None = None

        # Histogram window state
        self._hist_win:    tk.Toplevel | None = None
        self._hist_fig:    Figure | None = None
        self._hist_mpl:    FigureCanvasTkAgg | None = None
        self._hist_before: Image.Image | None = None  # snapshot before equalization

        self._build_ui()
        self._refresh_ui()

    # -----------------------------------------------------------------------
    # UI build
    # -----------------------------------------------------------------------
    def _build_ui(self):
        self._build_toolbar()
        self._build_infobar()
        self._build_canvas()
        self._build_navbar()
        self._build_opsbar()
        self._build_statusbar()
        self._bind_keys()

    def _build_toolbar(self):
        bar = tk.Frame(self.root, bg=BG_PANEL, pady=8, padx=14)
        bar.pack(fill=tk.X)

        self._btn_open = _btn(bar, "📂  Open Directory", self._open_dir)
        self._btn_open.pack(side=tk.LEFT, padx=(0, 14))

        self._dir_lbl = tk.Label(bar, text="No directory selected",
                                  bg=BG_PANEL, fg=TEXT_LO, font=("Segoe UI", 9))
        self._dir_lbl.pack(side=tk.LEFT, fill=tk.X, expand=True)

        # Resolution entry
        rf = tk.Frame(bar, bg=BG_PANEL)
        rf.pack(side=tk.RIGHT)
        tk.Label(rf, text="Resolution:", bg=BG_PANEL, fg=TEXT_LO,
                 font=("Segoe UI", 9)).pack(side=tk.LEFT, padx=(0, 5))
        self._res_var = tk.StringVar(value="1088")
        self._res_var.trace_add("write", lambda *_: self._update_btn_labels())
        tk.Entry(rf, textvariable=self._res_var, width=6,
                 bg=BG_CARD, fg=TEXT_HI, insertbackground=TEXT_HI,
                 relief=tk.FLAT, font=("Segoe UI", 10, "bold"),
                 justify=tk.CENTER).pack(side=tk.LEFT, padx=(0, 3))
        tk.Label(rf, text="px", bg=BG_PANEL, fg=TEXT_LO,
                 font=("Segoe UI", 9)).pack(side=tk.LEFT)

    def _build_infobar(self):
        bar = tk.Frame(self.root, bg=BG_ROOT, pady=4, padx=14)
        bar.pack(fill=tk.X)
        self._fname_lbl = tk.Label(bar, text="", bg=BG_ROOT, fg=TEXT_HI,
                                    font=("Segoe UI", 10, "bold"), anchor=tk.W)
        self._fname_lbl.pack(side=tk.LEFT, fill=tk.X, expand=True)
        self._size_lbl = tk.Label(bar, text="", bg=BG_ROOT, fg=TEXT_LO,
                                   font=("Segoe UI", 9))
        self._size_lbl.pack(side=tk.RIGHT, padx=(0, 20))
        self._nav_lbl = tk.Label(bar, text="", bg=BG_ROOT, fg=TEXT_LO,
                                  font=("Segoe UI", 9))
        self._nav_lbl.pack(side=tk.RIGHT, padx=(0, 8))

    def _build_canvas(self):
        f = tk.Frame(self.root, bg=BG_ROOT, padx=12, pady=2)
        f.pack(fill=tk.BOTH, expand=True)
        self._canvas = tk.Canvas(f, bg=CANVAS_BG, highlightthickness=0, cursor="crosshair")
        self._canvas.pack(fill=tk.BOTH, expand=True)
        self._canvas.bind("<Configure>",       self._on_canvas_resize)
        self._canvas.bind("<ButtonPress-1>",   self._on_press)
        self._canvas.bind("<B1-Motion>",       self._on_drag)
        self._canvas.bind("<ButtonRelease-1>", self._on_release)
        self._canvas.bind("<Motion>",          self._on_motion)
        self._canvas.bind("<Leave>",           lambda _e: self._clear_crosshair())

    def _build_navbar(self):
        """Navigation row + Save / Save Copy buttons."""
        bar = tk.Frame(self.root, bg=BG_PANEL, pady=8, padx=14)
        bar.pack(fill=tk.X)

        self._btn_prev = _btn(bar, "◀  Prev", self._prev, width=8)
        self._btn_prev.pack(side=tk.LEFT, padx=(0, 4))
        self._btn_next = _btn(bar, "Next  ▶", self._next, width=8)
        self._btn_next.pack(side=tk.LEFT)

        tk.Label(bar,
                 text="← A / → D navigate  |  R resize  |  C fixed-crop  |  F free-crop  |  S save  |  Esc cancel",
                 bg=BG_PANEL, fg=TEXT_LO, font=("Segoe UI", 8)).pack(side=tk.LEFT, padx=16)

        # Save buttons — right side, Save Copy first so Save (overwrite) is outermost
        self._btn_save = _btn(bar, "💾  Save (overwrite)", self._do_save, accent=True)
        self._btn_save.pack(side=tk.RIGHT, padx=(6, 0))

        self._btn_save_copy = _btn(bar, "📋  Save Copy…", self._do_save_copy)
        self._btn_save_copy.pack(side=tk.RIGHT, padx=(0, 4))

    def _build_opsbar(self):
        """Operations row: resize | crop | image adjustments."""
        bar = tk.Frame(self.root, bg=BG_OPS, pady=8, padx=14)
        bar.pack(fill=tk.X)

        # -- Resize --
        self._btn_resize = _btn(bar, "Resize → 1088²", self._do_resize)
        self._btn_resize.pack(side=tk.LEFT, padx=(0, 4))
        self._btn_resize_ratio = _btn(bar, "Resize (ratio) → 1088", self._do_resize_ratio)
        self._btn_resize_ratio.pack(side=tk.LEFT, padx=(0, 4))

        _sep(bar)

        # -- Crop --
        self._btn_crop = _btn(bar, "Crop → 1088²", self._toggle_crop)
        self._btn_crop.pack(side=tk.LEFT, padx=(0, 4))
        self._btn_free = _btn(bar, "✂  Free Crop", self._toggle_free_crop)
        self._btn_free.pack(side=tk.LEFT, padx=(0, 4))
        self._btn_apply = _btn(bar, "✓  Apply", self._do_apply, green=True)
        self._btn_apply.pack(side=tk.LEFT, padx=(0, 4))
        self._btn_reset = _btn(bar, "↺  Reset", self._do_reset)
        self._btn_reset.pack(side=tk.LEFT, padx=(0, 4))

        _sep(bar)

        # -- Image adjustments --
        self._btn_gray = _btn(bar, "⬛  Grayscale", self._do_grayscale)
        self._btn_gray.pack(side=tk.LEFT, padx=(0, 4))
        self._btn_norm = _btn(bar, "◈  Normalise", self._do_normalise)
        self._btn_norm.pack(side=tk.LEFT, padx=(0, 4))

        _sep(bar)

        # -- Histogram --
        self._btn_hist = _btn(bar, "📊  Histogram", self._show_histogram)
        self._btn_hist.pack(side=tk.LEFT, padx=(0, 4))
        self._btn_hist_eq = _btn(bar, "⊜  Hist Equalise", self._do_hist_equalize)
        self._btn_hist_eq.pack(side=tk.LEFT, padx=(0, 4))

        self._update_btn_labels()

    def _build_statusbar(self):
        self._status_var = tk.StringVar(value="Open a directory to begin.")
        tk.Label(self.root, textvariable=self._status_var,
                 bg="#0d0d18", fg=TEXT_LO, font=("Segoe UI", 8),
                 anchor=tk.W, padx=14, pady=3).pack(fill=tk.X)

    def _bind_keys(self):
        self.root.bind("<Left>",   lambda _e: self._prev())
        self.root.bind("<Right>",  lambda _e: self._next())
        self.root.bind("a",        lambda _e: self._prev())
        self.root.bind("A",        lambda _e: self._prev())
        self.root.bind("d",        lambda _e: self._next())
        self.root.bind("D",        lambda _e: self._next())
        self.root.bind("r",        lambda _e: self._do_resize())
        self.root.bind("R",        lambda _e: self._do_resize())
        self.root.bind("c",        lambda _e: self._toggle_crop())
        self.root.bind("C",        lambda _e: self._toggle_crop())
        self.root.bind("f",        lambda _e: self._toggle_free_crop())
        self.root.bind("F",        lambda _e: self._toggle_free_crop())
        self.root.bind("s",        lambda _e: self._do_save())
        self.root.bind("S",        lambda _e: self._do_save())
        self.root.bind("<Escape>", lambda _e: self._cancel_all())

    # -----------------------------------------------------------------------
    # Button label sync
    # -----------------------------------------------------------------------
    def _update_btn_labels(self):
        res = self._res_str()
        self._btn_resize.configure(text=f"Resize → {res}²")
        self._btn_resize_ratio.configure(text=f"Resize (ratio) → {res}")
        self._btn_crop.configure(
            text=f"{'✓ ' if self._crop_active else ''}Crop → {res}²",
            bg=ACCENT if self._crop_active else BTN_BASE,
        )
        self._btn_free.configure(
            text=f"{'✓ ' if self._free_active else ''}✂  Free Crop",
            bg=ACCENT if self._free_active else BTN_BASE,
        )
        free_ready = self._free_active and bool(self._free_start and self._free_end)
        can_apply  = self._crop_active or free_ready
        self._btn_apply.configure(
            text="✓  Apply Crop"      if self._crop_active else
                 "✓  Apply Free Crop" if free_ready else "✓  Apply",
            state=tk.NORMAL if can_apply else tk.DISABLED,
            bg=GREEN if can_apply else BTN_BASE,
        )

    # -----------------------------------------------------------------------
    # Navigation
    # -----------------------------------------------------------------------
    def _open_dir(self):
        d = filedialog.askdirectory(title="Select image directory")
        if not d:
            return
        p = Path(d)
        imgs = sorted(f for f in p.iterdir() if f.suffix.lower() in SUPPORTED_EXTS)
        if not imgs:
            messagebox.showwarning("No images", f"No supported images in:\n{d}")
            return
        self._images = imgs
        self._index  = 0
        self._dir_lbl.configure(text=str(p), fg=TEXT_HI)
        self._load()

    def _prev(self):
        if not self._images:
            return
        self._index = (self._index - 1) % len(self._images)
        self._load()

    def _next(self):
        if not self._images:
            return
        self._index = (self._index + 1) % len(self._images)
        self._load()

    def _load(self):
        path = self._images[self._index]
        try:
            img = Image.open(path).convert("RGB")
        except Exception as exc:
            messagebox.showerror("Load error", str(exc))
            return
        self._original    = img
        self._working     = img.copy()
        self._hist_before = None
        self._cancel_all(render=False)
        self._render()
        self._set_status(f"Loaded: {path.name}")

    # -----------------------------------------------------------------------
    # Operations
    # -----------------------------------------------------------------------
    def _res_str(self) -> str:
        return self._res_var.get().strip() or "1088"

    def _target_res(self) -> int:
        try:
            return max(1, int(self._res_str()))
        except ValueError:
            return 1088

    # -- Resize --

    def _do_resize(self):
        """Stretch to res×res square (may distort aspect ratio)."""
        if not self._working:
            return
        res = self._target_res()
        self._working = self._working.resize((res, res), Image.LANCZOS)
        self._cancel_all(render=False)
        self._render()
        self._set_status(f"Resized to {res}×{res}. Press S to save.")

    def _do_resize_ratio(self):
        """Scale so the longest side = res; shorter side proportional."""
        if not self._working:
            return
        res = self._target_res()
        iw, ih = self._working.size
        if iw >= ih:
            nw, nh = res, max(1, round(ih * res / iw))
        else:
            nw, nh = max(1, round(iw * res / ih)), res
        self._working = self._working.resize((nw, nh), Image.LANCZOS)
        self._cancel_all(render=False)
        self._render()
        self._set_status(f"Resized to {nw}×{nh} (aspect ratio kept). Press S to save.")

    # -- Fixed crop --

    def _toggle_crop(self):
        if not self._working:
            return
        if self._crop_active:
            self._cancel_all()
            return
        res = self._target_res()
        iw, ih = self._working.size
        if iw < res or ih < res:
            messagebox.showwarning(
                "Image too small",
                f"Image ({iw}×{ih}) is smaller than {res}×{res}.\n"
                "Resize first or lower the target resolution.",
            )
            return
        self._free_active = False
        self._free_start  = None
        self._free_end    = None
        self._crop_active = True
        self._crop_x = (iw - res) // 2
        self._crop_y = (ih - res) // 2
        self._update_btn_labels()
        self._render()
        self._set_status(
            f"Drag the {res}×{res} window. ✓ Apply to commit, S to apply+save. Esc to cancel."
        )

    def _apply_fixed_crop(self):
        res = self._target_res()
        x0, y0 = self._crop_x, self._crop_y
        self._working = self._working.crop((x0, y0, x0 + res, y0 + res))

    # -- Free crop --

    def _toggle_free_crop(self):
        if not self._working:
            return
        if self._free_active:
            self._cancel_all()
            return
        self._crop_active = False
        self._free_active = True
        self._free_start  = None
        self._free_end    = None
        self._update_btn_labels()
        self._canvas.delete("free_crop")
        self._set_status("Drag to draw a crop region. ✓ Apply to commit, S to apply+save. Esc to cancel.")

    def _apply_free_crop(self):
        if not self._free_start or not self._free_end:
            return
        iw, ih = self._working.size
        x0 = max(0, min(self._free_start[0], self._free_end[0]))
        y0 = max(0, min(self._free_start[1], self._free_end[1]))
        x1 = min(iw, max(self._free_start[0], self._free_end[0]))
        y1 = min(ih, max(self._free_start[1], self._free_end[1]))
        if x1 > x0 and y1 > y0:
            self._working = self._working.crop((x0, y0, x1, y1))

    # -- Apply --

    def _do_apply(self):
        if not self._working:
            return
        if self._crop_active:
            self._apply_fixed_crop()
            self._cancel_all(render=False)
            self._render()
            w, h = self._working.size
            self._set_status(f"Crop applied → {w}×{h}px. Continue editing or press S to save.")
        elif self._free_active and self._free_start and self._free_end:
            self._apply_free_crop()
            self._cancel_all(render=False)
            self._render()
            w, h = self._working.size
            self._set_status(f"Free crop applied → {w}×{h}px. Continue editing or press S to save.")

    # -- Image adjustments --

    def _do_grayscale(self):
        if not self._working:
            return
        self._working = self._working.convert("L").convert("RGB")
        self._cancel_all(render=False)
        self._render()
        self._set_status("Converted to grayscale. Press S to save.")

    def _do_normalise(self):
        """Per-channel linear stretch: maps [min, max] → [0, 255] (autocontrast)."""
        if not self._working:
            return
        self._working = ImageOps.autocontrast(self._working, cutoff=0)
        self._cancel_all(render=False)
        self._render()
        self._set_status("Normalised (per-channel autocontrast). Press S to save.")

    # -- Histogram --

    def _show_histogram(self, after_img: Image.Image | None = None):
        """Open or refresh the histogram window.
        If _hist_before is set, shows a before/after comparison; otherwise single plot."""
        if not self._working:
            return

        # Create window if needed
        if self._hist_win is None or not self._hist_win.winfo_exists():
            self._hist_win = tk.Toplevel(self.root)
            self._hist_win.configure(bg=BG_ROOT)
            self._hist_win.geometry("700x340")
            self._hist_win.resizable(True, True)
            self._hist_fig = Figure(facecolor="#12121e")
            self._hist_mpl = FigureCanvasTkAgg(self._hist_fig, master=self._hist_win)
            self._hist_mpl.get_tk_widget().pack(fill=tk.BOTH, expand=True,
                                                 padx=6, pady=6)

        dual = self._hist_before is not None
        self._hist_win.title(
            "Histogram — Before vs After Equalisation" if dual else "Histogram"
        )
        self._hist_fig.clear()

        if dual:
            axes = [self._hist_fig.add_subplot(1, 2, 1),
                    self._hist_fig.add_subplot(1, 2, 2)]
            pairs = [(self._hist_before, "Before"), (self._working, "After Equalisation")]
        else:
            axes = [self._hist_fig.add_subplot(1, 1, 1)]
            pairs = [(self._working, "Histogram")]

        ch_colors = [("#e94560", "R"), ("#00b86b", "G"), ("#4488ff", "B")]
        for ax, (img, title) in zip(axes, pairs):
            ax.set_facecolor("#1a1a2e")
            ax.set_title(title, color=TEXT_HI, fontsize=9, pad=6)
            ax.tick_params(colors=TEXT_LO, labelsize=7)
            for spine in ax.spines.values():
                spine.set_color("#333355")
            ax.set_xlim(0, 255)
            ax.set_xlabel("Pixel value", color=TEXT_LO, fontsize=8)
            ax.set_ylabel("Count", color=TEXT_LO, fontsize=8)
            r_hist, g_hist, b_hist = [img.split()[i].histogram() for i in range(3)]
            if r_hist == g_hist == b_hist:
                # Grayscale image: single luminance line
                ax.plot(range(256), r_hist, color="#cccccc", linewidth=1.5,
                        alpha=0.9, label="Luminance")
            else:
                for i, (color, name) in enumerate(ch_colors):
                    ax.plot(range(256), [r_hist, g_hist, b_hist][i],
                            color=color, linewidth=1, alpha=0.85, label=name)
            ax.legend(facecolor="#1a1a2e", edgecolor="#333355",
                      labelcolor=TEXT_HI, fontsize=8)

        self._hist_fig.tight_layout(pad=1.5)
        self._hist_mpl.draw()
        self._hist_win.lift()

    def _do_hist_equalize(self):
        """Apply per-channel histogram equalisation and show before/after histogram."""
        if not self._working:
            return
        self._hist_before = self._working.copy()
        self._working = ImageOps.equalize(self._working)
        self._cancel_all(render=False)
        self._render()
        self._show_histogram()
        self._set_status("Histogram equalised. Press S to save.")

    # -- Reset --

    def _do_reset(self):
        if not self._original:
            return
        self._working = self._original.copy()
        self._hist_before = None
        self._cancel_all()
        self._set_status("Reset to original.")

    # -- Save (overwrite) --

    def _do_save(self):
        if not self._working or not self._images:
            return
        if self._crop_active:
            self._apply_fixed_crop()
        elif self._free_active and self._free_start and self._free_end:
            self._apply_free_crop()
        path = self._images[self._index]
        try:
            kw: dict = {}
            if path.suffix.lower() in (".jpg", ".jpeg"):
                kw = {"quality": 95, "subsampling": 0}
            self._working.save(path, **kw)
            self._original = self._working.copy()
        except Exception as exc:
            messagebox.showerror("Save failed", str(exc))
            return
        self._cancel_all(render=False)
        self._render()
        self._set_status(f"Saved (overwrite) → {path.name}")

    # -- Save Copy --

    def _do_save_copy(self):
        if not self._working or not self._images:
            return
        # Apply any pending crop before saving copy
        working = self._working
        if self._crop_active:
            res = self._target_res()
            x0, y0 = self._crop_x, self._crop_y
            working = working.crop((x0, y0, x0 + res, y0 + res))
        elif self._free_active and self._free_start and self._free_end:
            iw, ih = working.size
            x0 = max(0, min(self._free_start[0], self._free_end[0]))
            y0 = max(0, min(self._free_start[1], self._free_end[1]))
            x1 = min(iw, max(self._free_start[0], self._free_end[0]))
            y1 = min(ih, max(self._free_start[1], self._free_end[1]))
            if x1 > x0 and y1 > y0:
                working = working.crop((x0, y0, x1, y1))

        src = self._images[self._index]
        init_file = src.stem + "_copy" + src.suffix
        dest = filedialog.asksaveasfilename(
            title="Save Copy As",
            initialdir=str(src.parent),
            initialfile=init_file,
            filetypes=[
                ("JPEG", "*.jpg *.jpeg"),
                ("PNG",  "*.png"),
                ("All images", "*.jpg *.jpeg *.png *.bmp *.tiff *.tif *.webp"),
            ],
            defaultextension=src.suffix,
        )
        if not dest:
            return
        dest_path = Path(dest)
        try:
            kw: dict = {}
            if dest_path.suffix.lower() in (".jpg", ".jpeg"):
                kw = {"quality": 95, "subsampling": 0}
            working.save(dest_path, **kw)
            self._set_status(f"Copy saved → {dest_path.name}")
        except Exception as exc:
            messagebox.showerror("Save Copy failed", str(exc))

    # -- Cancel everything --

    def _cancel_all(self, render: bool = True):
        self._crop_active      = False
        self._crop_x           = 0
        self._crop_y           = 0
        self._drag_anchor      = None
        self._drag_crop_start  = None
        self._free_active      = False
        self._free_start       = None
        self._free_end         = None
        self._canvas.delete("free_crop")
        self._canvas.delete("crosshair")
        self._update_btn_labels()
        if render:
            self._render()

    # -----------------------------------------------------------------------
    # Rendering
    # -----------------------------------------------------------------------
    def _on_canvas_resize(self, _event):
        self._render()

    def _render(self):
        if not self._working:
            self._canvas.delete("all")
            return
        cw = self._canvas.winfo_width()
        ch = self._canvas.winfo_height()
        if cw < 2 or ch < 2:
            return

        iw, ih  = self._working.size
        scale   = min(cw / iw, ch / ih)
        self._scale = scale
        dw, dh  = int(iw * scale), int(ih * scale)
        self._off_x = (cw - dw) // 2
        self._off_y = (ch - dh) // 2

        display = self._working.resize((dw, dh), Image.LANCZOS)

        if self._crop_active:
            display = self._draw_fixed_overlay(display)

        self._tkimg = ImageTk.PhotoImage(display)
        self._canvas.delete("all")
        self._canvas.create_image(self._off_x, self._off_y, anchor=tk.NW, image=self._tkimg)

        if self._free_active and self._free_start and self._free_end:
            self._draw_free_items()

        self._update_info()
        self._refresh_ui()

    def _draw_fixed_overlay(self, display: Image.Image) -> Image.Image:
        res = self._target_res()
        s   = self._scale
        cx  = int(self._crop_x * s)
        cy  = int(self._crop_y * s)
        cr  = int(res * s)

        shade = Image.new("RGBA", display.size, (0, 0, 0, 130))
        mask  = Image.new("L",    display.size, 255)
        ImageDraw.Draw(mask).rectangle([cx, cy, cx + cr - 1, cy + cr - 1], fill=0)
        base = display.convert("RGBA")
        base.paste(shade, mask=mask)
        display = base.convert("RGB")

        draw = ImageDraw.Draw(display)
        draw.rectangle([cx, cy, cx + cr - 1, cy + cr - 1],
                       outline="#e94560", width=max(2, int(s)))
        hl = max(12, int(20 * s))
        hw = max(2, int(3 * s))
        for ax, ay, bx, by, ex, ey in [
            (cx,      cy,      cx+hl,      cy,      cx,      cy+hl),
            (cx+cr-1, cy,      cx+cr-1-hl, cy,      cx+cr-1, cy+hl),
            (cx,      cy+cr-1, cx+hl,      cy+cr-1, cx,      cy+cr-1-hl),
            (cx+cr-1, cy+cr-1, cx+cr-1-hl, cy+cr-1, cx+cr-1, cy+cr-1-hl),
        ]:
            draw.line([(ax, ay), (bx, by)], fill="#ffffff", width=hw)
            draw.line([(ax, ay), (ex, ey)], fill="#ffffff", width=hw)

        lbl = f"{res}×{res}"
        tx, ty = cx + 6, cy + 6
        draw.rectangle([tx-2, ty-2, tx + 7*len(lbl), ty + 14], fill=(0, 0, 0, 170))
        draw.text((tx, ty), lbl, fill="#ffffff")
        return display

    def _draw_free_items(self):
        x0i, y0i = self._free_start
        x1i, y1i = self._free_end

        rx0 = int(min(x0i, x1i) * self._scale) + self._off_x
        ry0 = int(min(y0i, y1i) * self._scale) + self._off_y
        rx1 = int(max(x0i, x1i) * self._scale) + self._off_x
        ry1 = int(max(y0i, y1i) * self._scale) + self._off_y
        pw  = abs(x1i - x0i)
        ph  = abs(y1i - y0i)

        self._canvas.delete("free_crop")
        self._canvas.create_rectangle(rx0-1, ry0-1, rx1+1, ry1+1,
                                       outline="#000000", width=3, tags="free_crop")
        self._canvas.create_rectangle(rx0, ry0, rx1, ry1,
                                       outline="#00ff88", width=2,
                                       dash=(8, 4), tags="free_crop")
        lbl = f"{pw} × {ph} px"
        tw  = 9 * len(lbl) + 8
        self._canvas.create_rectangle(rx0, ry0, rx0 + tw, ry0 + 20,
                                       fill="#000000", outline="", tags="free_crop")
        self._canvas.create_text(rx0 + 4, ry0 + 10, text=lbl,
                                  fill="#00ff88", anchor=tk.W,
                                  font=("Segoe UI", 8, "bold"), tags="free_crop")

    # -----------------------------------------------------------------------
    # Mouse events
    # -----------------------------------------------------------------------
    def _canvas_to_image(self, cx: int, cy: int) -> tuple[int, int]:
        iw, ih = self._working.size
        ix = max(0, min(round((cx - self._off_x) / self._scale), iw))
        iy = max(0, min(round((cy - self._off_y) / self._scale), ih))
        return ix, iy

    def _on_motion(self, event):
        if self._free_active:
            self._draw_crosshair(event.x, event.y)
        else:
            self._clear_crosshair()

    def _draw_crosshair(self, x: int, y: int):
        cw = self._canvas.winfo_width()
        ch = self._canvas.winfo_height()
        self._canvas.delete("crosshair")
        dash = (6, 5)
        # Horizontal line
        self._canvas.create_line(0, y, cw, y,
                                  fill="#00ff88", dash=dash, width=1,
                                  tags="crosshair")
        # Vertical line
        self._canvas.create_line(x, 0, x, ch,
                                  fill="#00ff88", dash=dash, width=1,
                                  tags="crosshair")
        # Small centre circle at intersection
        r = 4
        self._canvas.create_oval(x - r, y - r, x + r, y + r,
                                  outline="#00ff88", width=1,
                                  tags="crosshair")

    def _clear_crosshair(self):
        self._canvas.delete("crosshair")

    def _on_press(self, event):
        if self._crop_active:
            self._drag_anchor     = (event.x, event.y)
            self._drag_crop_start = (self._crop_x, self._crop_y)
        elif self._free_active:
            ix, iy = self._canvas_to_image(event.x, event.y)
            self._free_start = (ix, iy)
            self._free_end   = (ix, iy)

    def _on_drag(self, event):
        if self._crop_active and self._drag_anchor:
            res    = self._target_res()
            iw, ih = self._working.size
            dx     = round((event.x - self._drag_anchor[0]) / self._scale)
            dy     = round((event.y - self._drag_anchor[1]) / self._scale)
            self._crop_x = max(0, min(self._drag_crop_start[0] + dx, iw - res))
            self._crop_y = max(0, min(self._drag_crop_start[1] + dy, ih - res))
            self._render()
        elif self._free_active and self._free_start is not None:
            ix, iy = self._canvas_to_image(event.x, event.y)
            self._free_end = (ix, iy)
            self._canvas.delete("free_crop")
            self._draw_free_items()
            self._update_btn_labels()

    def _on_release(self, _event):
        self._drag_anchor     = None
        self._drag_crop_start = None

    # -----------------------------------------------------------------------
    # Info / state
    # -----------------------------------------------------------------------
    def _update_info(self):
        if not self._working or not self._images:
            return
        path = self._images[self._index]
        self._fname_lbl.configure(text=path.name)
        self._nav_lbl.configure(text=f"{self._index + 1} / {len(self._images)}")
        ww, wh = self._working.size
        ow, oh = self._original.size
        if (ww, wh) != (ow, oh):
            self._size_lbl.configure(text=f"{ow}×{oh} → {ww}×{wh}", fg=ACCENT)
        else:
            self._size_lbl.configure(text=f"{ww}×{wh}", fg=TEXT_LO)

    def _refresh_ui(self):
        has = bool(self._images)
        for b in (self._btn_prev, self._btn_next,
                  self._btn_resize, self._btn_resize_ratio,
                  self._btn_crop, self._btn_free, self._btn_reset,
                  self._btn_gray, self._btn_norm,
                  self._btn_hist, self._btn_hist_eq,
                  self._btn_save_copy, self._btn_save):
            b.configure(state=tk.NORMAL if has else tk.DISABLED)
        if not has:
            self._btn_apply.configure(state=tk.DISABLED)

    def _set_status(self, msg: str):
        self._status_var.set(msg)


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------
def main():
    root = tk.Tk()
    root.geometry("1200x800")
    root.title("RF-DETR Image Prep")
    ttk.Style(root).theme_use("clam")
    ImagePrepApp(root)
    root.mainloop()


if __name__ == "__main__":
    main()
