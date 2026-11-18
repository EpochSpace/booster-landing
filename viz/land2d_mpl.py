"""Real-time 2D view of a pdg powered-descent landing, drawn with matplotlib.

Same picture as ``land2d.py`` but rendered on the CPU through Tk, which is what
you want when the GPU cannot be trusted -- a virtual adapter (VMware SVGA,
Parallels, RDP) typically leaves the Rerun viewer window transparent or black.

    python viz/land2d_mpl.py --data out --speed 1.0

The replay is paced against the wall clock and *drops* frames when the renderer
falls behind, so a second of playback is a second of flight whatever the frame
rate turns out to be.
"""

from __future__ import annotations

import argparse
import math
import sys
import time
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))

import matplotlib                                        # noqa: E402
from flight_data import (                                # noqa: E402
    C_AUX, C_BG, C_FLAME, C_FLAME_CORE, C_FLOWN, C_GHOST, C_GROUND, C_HULL,
    C_HULL_DARK, C_LIMIT, C_PAD, C_PAD_MARK, C_PANEL, C_PLAN, C_TEXT, C_WARN,
    Booster, leg_deployment_track, load_run, mpl_color, pad_geometry, scene_bounds,
)

HULL = mpl_color(C_HULL)
HULL_DARK = mpl_color(C_HULL_DARK)
FLAME = mpl_color(C_FLAME)
FLAME_CORE = mpl_color(C_FLAME_CORE)
PLAN = mpl_color(C_PLAN)
FLOWN = mpl_color(C_FLOWN)
PAD = mpl_color(C_PAD)
PAD_MARK = mpl_color(C_PAD_MARK)
GROUND = mpl_color(C_GROUND)
LIMIT = mpl_color(C_LIMIT)
GHOST = mpl_color(C_GHOST)
AUX = mpl_color(C_AUX)
BG = mpl_color(C_BG)
PANEL = mpl_color(C_PANEL)
TEXT = mpl_color(C_TEXT)
WARN = mpl_color(C_WARN)


def style_axes(ax, title: str | None = None) -> None:
    ax.set_facecolor(PANEL)
    ax.tick_params(colors=TEXT, labelsize=8)
    ax.grid(True, color=(1, 1, 1, 0.06), lw=0.8)
    ax.set_axisbelow(True)
    for spine in ax.spines.values():
        spine.set_color((1, 1, 1, 0.12))
    if title:
        ax.set_title(title, color=TEXT, fontsize=9, loc="left", pad=4)


class Viewer:
    """Blitted matplotlib replay of one landing."""

    def __init__(self, meta, flight, plan, boost: Booster, *, follow: float,
                 leg_alt: float, zoom_at: float = 0.0, zoom_to: float = 0.0,
                 zoom_pause: float = 0.0):
        self.meta, self.flight, self.plan, self.boost = meta, flight, plan, boost
        self.leg_alt = leg_alt
        self.leg_frac = leg_deployment_track(flight.alt, leg_alt)
        self.follow = follow            # 0 = whole trajectory, >0 = window in metres
        self.n = len(flight.t)
        self.closed = False

        # ---- camera ------------------------------------------------------
        x_lo, x_hi, y_lo, y_hi = scene_bounds(flight, plan)
        self.full_bounds = (x_lo, x_hi, y_lo, y_hi)
        # A square window keeps the equal-aspect axes box a constant shape, so
        # the panel is never re-laid-out mid-replay however far the camera zooms.
        self.wide = max(x_hi - x_lo, y_hi - y_lo)
        self.scene_cx = 0.5 * (x_lo + x_hi)
        self.scene_cy = 0.5 * (y_lo + y_hi)

        # Auto-zoom earns its keep only on a flight with a real wide phase to
        # zoom out of: a 500 m hop has nothing to pull back to, and forcing it
        # would just crop the trajectory from the first frame.
        self.zoom_at, self.zoom_to = zoom_at, zoom_to
        alt = np.maximum(flight.alt, 0.0)
        # The flight only has to *reach* above the trigger, not start there. Every
        # flight this viewer had seen until the RTLS profile began at its highest
        # point and came down, so testing the first sample was the same thing as
        # testing the highest one. A flight that lifts off from the pad starts at
        # zero altitude, and on that test the camera simply switched itself off.
        self.auto_zoom = (follow <= 0.0 and zoom_at > 0.0 and zoom_to > 0.0
                          and float(alt.max()) > zoom_at
                          and self.wide > 4.0 * zoom_to)
        # Altitude drives the trigger, forced monotone so a wobble near the
        # threshold cannot fire it twice -- but only over the *final* descent.
        # Running the minimum from sample 0 would have the ascent's own zero
        # altitude latched in before the stage ever left the pad, firing the
        # zoom on the launch and leaving it closed for the rest of the flight.
        # So the window starts at the last moment the stage was above the
        # trigger, which for a descent-only flight is the same sample as before.
        above = np.flatnonzero(alt > zoom_at)
        self.zoom_alt = np.full(alt.shape, np.inf)
        if above.size:
            s0 = int(above[-1])
            self.zoom_alt[s0:] = np.minimum.accumulate(alt[s0:])
        self.moving = follow > 0.0 or self.auto_zoom

        # The zoom is a move in its own right, not something smeared over the
        # descent: at the trigger altitude an altitude-driven camera is still
        # fully wide, so there would be nothing to watch. The flight holds while
        # the camera travels, then real time resumes.
        self.zoom_pause = zoom_pause
        self.zoom_state = "close" if not self.auto_zoom else "wide"
        self.zoom_s = 0.0 if self.auto_zoom else 1.0

        # the plan resampled onto the flight clock, for the ghost and the error.
        # A producer whose guidance runs in the loop has no plan at all.
        self.has_plan = plan is not None
        if self.has_plan:
            self.ref_d = np.interp(flight.t, plan.t, plan.downrange)
            self.ref_a = np.interp(flight.t, plan.t, plan.alt)
            ref_c = np.interp(flight.t, plan.t, plan.crossrange)
            self.err = np.sqrt((flight.downrange - self.ref_d) ** 2
                               + (flight.alt - self.ref_a) ** 2
                               + (flight.crossrange - ref_c) ** 2)
        else:
            self.ref_d = self.ref_a = self.err = None

        # phase spans, when the producer reports them (donor core)
        self.phases = meta.get("phases", [])

        self._build_figure()

    # -- camera -----------------------------------------------------------
    def _window(self, i: int) -> tuple[float, float, float]:
        """Centre and half-size of the view at sample i, always 1:1."""
        if self.follow > 0.0:
            half = 0.5 * self.follow
            cx = float(self.flight.xy[i][0])
            cy = max(float(self.flight.xy[i][1]), 0.42 * self.follow)
            return cx, cy, half

        if not self.auto_zoom:
            x_lo, x_hi, y_lo, y_hi = self.full_bounds
            return self.scene_cx, self.scene_cy, 0.5 * max(x_hi - x_lo, y_hi - y_lo)

        # Blend between the whole-scene framing and a chase window. Geometric on
        # the size -- linear in metres would crawl for most of the move and then
        # snap -- and smoothstepped so it eases in and out rather than starting
        # and stopping dead.
        s = float(np.clip(self.zoom_s, 0.0, 1.0))
        s = s * s * (3.0 - 2.0 * s)
        width = self.wide * (self.zoom_to / self.wide) ** s
        half = 0.5 * width

        cx = (1.0 - s) * self.scene_cx + s * float(self.flight.xy[i][0])
        cy_close = max(float(self.flight.xy[i][1]), 0.42 * width)
        cy = (1.0 - s) * self.scene_cy + s * cy_close
        return cx, cy, half

    @staticmethod
    def _nice_scale(width: float) -> float:
        """Longest round bar that still fits in a quarter of the window.

        A powers-of-ten ladder is too coarse for a camera that zooms smoothly:
        the bar's share of the frame drifts from 8 % up past 50 %, then snaps
        back by a factor of ten. Stepping 1-2-5 keeps it between about a tenth
        and a quarter the whole way down.
        """
        budget = max(width, 1e-6) / 4.0
        decade = 10.0 ** math.floor(math.log10(budget))
        for step in (5.0, 2.0, 1.0):
            if step * decade <= budget:
                return step * decade
        return decade / 2.0

    @staticmethod
    def _scale_label(length: float) -> str:
        if length >= 1000.0:
            return f"{length / 1000.0:g} km"
        if length >= 1.0:
            return f"{length:g} m"
        return f"{length * 100.0:g} cm"

    # -- figure -----------------------------------------------------------
    def _build_figure(self) -> None:
        import matplotlib.pyplot as plt
        from matplotlib.collections import LineCollection
        from matplotlib.gridspec import GridSpec
        from matplotlib.patches import Polygon

        self.plt = plt
        self.fig = plt.figure(figsize=(15.5, 8.6), facecolor=BG)
        self.fig.canvas.manager.set_window_title("booster landing — pdg + SCvx")
        gs = GridSpec(4, 2, figure=self.fig, width_ratios=[2.15, 1.0],
                      left=0.045, right=0.985, top=0.955, bottom=0.06,
                      wspace=0.16, hspace=0.42)

        ax = self.fig.add_subplot(gs[:, 0])
        self.ax = ax
        style_axes(ax, "landing — side view")
        ax.set_xlabel("downrange [m]", color=TEXT, fontsize=8)
        ax.set_ylabel("altitude [m]", color=TEXT, fontsize=8)
        x_lo, x_hi, y_lo, y_hi = self.full_bounds
        if self.moving:
            # Start on the camera's own first window: with an equal aspect the
            # axes box is laid out from these limits, and a 62 km descent over
            # 5 km of ground would shape it into a sliver.
            cx, cy, half = self._window(0)
            ax.set_xlim(cx - half, cx + half)
            ax.set_ylim(cy - half, cy + half)
        else:
            ax.set_xlim(x_lo, x_hi)
            ax.set_ylim(y_lo, y_hi)

        # A moving camera is always 1:1 -- its window is square, so the box keeps
        # its shape. A fixed wide view keeps true scale too, except for a
        # trajectory far taller than it is wide (a 62 km entry over 5 km of
        # ground), where equal aspect would squeeze the panel into a sliver.
        ratio = (y_hi - y_lo) / max(x_hi - x_lo, 1e-9)
        self.true_scale = self.moving or 0.25 < ratio < 4.0
        ax.set_aspect("equal" if self.true_scale else "auto", adjustable="box")
        self.exaggeration = 1.0
        if not self.true_scale:
            # Say by how much, and say it on the plot: at these ratios a 14 m
            # lateral wander over 500 m of descent is drawn as a dramatic S, and
            # it is very easy to read that as the vehicle misbehaving.
            pos = ax.get_position()
            fw, fh = self.fig.get_size_inches()
            m_per_in_x = (x_hi - x_lo) / max(pos.width * fw, 1e-9)
            m_per_in_y = (y_hi - y_lo) / max(pos.height * fh, 1e-9)
            self.exaggeration = m_per_in_y / max(m_per_in_x, 1e-9)
            ax.set_title(
                f"landing - side view   NOT TO SCALE: horizontal exaggerated "
                f"{self.exaggeration:.0f}x (use --follow for true shape)",
                color=WARN, fontsize=9, loc="left", pad=4)

        # ---- static scenery ---------------------------------------------
        # In follow mode the axis limits move every frame, which would leave a
        # baked-in background stale -- so the scenery is kept as a list and
        # redrawn as an ordinary artist alongside the booster.
        self.scene_static = []
        self.scene_static.append(ax.axhline(0.0, color=GROUND, lw=2.0, zorder=1))
        outline, marks, bullseye = pad_geometry()
        self.scene_static += ax.plot(outline[:, 0], outline[:, 1], color=PAD,
                                     lw=1.8, zorder=2)
        for m in marks:
            self.scene_static += ax.plot(m[:, 0], m[:, 1], color=PAD, lw=1.1, zorder=2)
        for b in bullseye:
            self.scene_static += ax.plot(b[:, 0], b[:, 1], color=PAD_MARK, lw=2.0,
                                         zorder=2)

        gs_deg = self.meta["limits"]["glide_slope_deg"]
        if gs_deg > 0.0:
            reach = max(abs(x_lo), abs(x_hi)) + 120.0
            tan_gs = np.tan(np.radians(gs_deg))
            for sign in (-1.0, 1.0):
                self.scene_static += ax.plot(
                    [0.0, sign * reach], [0.0, tan_gs * reach], color=LIMIT,
                    lw=0.9, ls=(0, (6, 4)), zorder=1,
                    label=f"glide slope {gs_deg:.0f}°" if sign > 0 else None)

        if self.has_plan:
            self.scene_static += ax.plot(self.plan.downrange, self.plan.alt,
                                         color=PLAN, lw=1.4, label="SCvx plan",
                                         zorder=3)
        # a donor run has neither a plan nor a glide-slope cone: nothing to label
        if ax.get_legend_handles_labels()[1]:
            leg = ax.legend(loc="upper left", fontsize=8, facecolor=PANEL,
                            edgecolor=(1, 1, 1, 0.12), labelcolor=TEXT,
                            framealpha=0.9)
            leg.set_zorder(20)
            self.scene_static.append(leg)

        # A moving camera has no use for fixed ticks, and dropping them keeps the
        # blitted background independent of the limits. A scale bar replaces them.
        self.scale_bar = None
        if self.moving:
            ax.set_xticks([])
            ax.set_yticks([])
            ax.grid(False)
            self.scale_len = self._nice_scale(2.0 * self._window(0)[2])
            self.scale_bar, = ax.plot([], [], color=PAD_MARK, lw=2.2, zorder=21)
            self.scale_text = ax.text(0, 0, "", color=PAD_MARK, fontsize=9,
                                      family="monospace", ha="center", va="bottom",
                                      zorder=21)

        # ---- dynamic artists --------------------------------------------
        self.trail, = ax.plot([], [], color=FLOWN, lw=1.6, zorder=4)
        self.ghost, = ax.plot([], [], marker="o", ms=6, color=GHOST, ls="none",
                              zorder=5)
        blank = np.zeros((3, 2))
        self.plume_outer = Polygon(blank, closed=True, facecolor=(*FLAME, 0.45),
                                   edgecolor=(*FLAME, 0.95), lw=1.2, zorder=6)
        self.plume_inner = Polygon(blank, closed=True, facecolor=(*FLAME_CORE, 0.85),
                                   edgecolor="none", zorder=6.5)
        self.hull_poly = Polygon(blank, closed=True, facecolor=(*HULL_DARK, 0.40),
                                 edgecolor=HULL, lw=1.7, zorder=8)
        # the interstage is the black composite section; it is what makes a
        # Falcon read as a Falcon at a glance
        self.interstage_poly = Polygon(blank, closed=True, facecolor=(0.09, 0.10, 0.13),
                                       edgecolor=HULL_DARK, lw=1.0, zorder=8.5)
        for patch in (self.plume_outer, self.plume_inner, self.hull_poly,
                      self.interstage_poly):
            ax.add_patch(patch)

        self.band_lc = LineCollection([], colors=[HULL_DARK], linewidths=[1.0],
                                      zorder=9)
        self.fins_lc = LineCollection([], colors=[HULL_DARK], linewidths=[1.2],
                                      zorder=7)
        self.legs_lc = LineCollection([], colors=[HULL_DARK], linewidths=[1.5],
                                      zorder=7)
        for lc in (self.band_lc, self.fins_lc, self.legs_lc):
            ax.add_collection(lc)

        self.hud = ax.text(0.985, 0.975, "", transform=ax.transAxes, ha="right",
                           va="top", family="monospace", fontsize=10.5,
                           color=PAD_MARK, zorder=21,
                           bbox=dict(facecolor=(*BG, 0.82), edgecolor=(1, 1, 1, 0.14),
                                     boxstyle="round,pad=0.5"))
        # bottom-left, clear of the pad at the centre of the frame
        self.status = ax.text(0.018, 0.022, "", transform=ax.transAxes, ha="left",
                              va="bottom", family="monospace", fontsize=11.5,
                              color=FLOWN, zorder=21,
                              bbox=dict(facecolor=(*BG, 0.82),
                                        edgecolor=(1, 1, 1, 0.14),
                                        boxstyle="round,pad=0.35"))

        # ---- telemetry ---------------------------------------------------
        self.plots = []
        specs = [
            ("altitude / range [m]", [
                ("altitude", self.flight.alt, FLOWN, 1.8),
                ("downrange", self.flight.downrange, PLAN, 1.4),
                ("crossrange", self.flight.crossrange, AUX, 1.2)]),
            ("velocity [m/s]", [
                ("speed", self.flight.speed, FLOWN, 1.8),
                ("vertical", self.flight.vspeed, PLAN, 1.4)]),
            ("thrust [kN]", [
                ("commanded", self.flight.thrust / 1e3, FLAME, 1.8)]),
            ("attitude [deg]", [
                ("tilt", self.flight.tilt, FLOWN, 1.8),
                ("gimbal", self.flight.gimbal, PLAN, 1.4),
                ("AoA", self.flight.aoa, AUX, 1.2)]),
        ]
        t = self.flight.t
        for row, (title, series) in enumerate(specs):
            pax = self.fig.add_subplot(gs[row, 1])
            style_axes(pax, title)
            pax.set_xlim(t[0], t[-1])
            lines = []
            stack = []
            for name, y, color, lw in series:
                ln, = pax.plot([], [], color=color, lw=lw, label=name)
                lines.append((ln, y))
                stack.append(y)
            if title.startswith("thrust"):
                veh = self.meta["vehicle"]
                for lvl in (veh["t_min"] / 1e3, veh["t_max"] / 1e3):
                    pax.axhline(lvl, color=LIMIT, lw=0.9, ls=(0, (5, 4)))
                    stack.append(np.array([lvl]))
            if title.startswith("attitude"):
                lim = self.meta["limits"]
                for lvl in (lim["tilt_max_deg"], lim["gimbal_max_deg"],
                            lim["aoa_max_deg"]):
                    pax.axhline(lvl, color=LIMIT, lw=0.8, ls=(0, (5, 4)))
                stack.append(np.array([lim["gimbal_max_deg"] * 1.15]))
            flat = np.concatenate([np.asarray(s, dtype=float).ravel() for s in stack])
            lo, hi = np.nanmin(flat), np.nanmax(flat)
            pad = 0.08 * max(hi - lo, 1e-6)
            pax.set_ylim(lo - pad, hi + pad)
            cursor = pax.axvline(t[0], color=PAD_MARK, lw=0.9, alpha=0.55)
            pax.legend(loc="upper right", fontsize=7, facecolor=PANEL, ncol=3,
                       edgecolor=(1, 1, 1, 0.12), labelcolor=TEXT, framealpha=0.85)
            if row == len(specs) - 1:
                pax.set_xlabel("time [s]", color=TEXT, fontsize=8)
            self.plots.append((pax, lines, cursor))

        self.fig.canvas.mpl_connect("close_event", self._on_close)

    def _on_close(self, _event) -> None:
        self.closed = True

    # -- drawing ----------------------------------------------------------
    def _dynamic_artists(self):
        # Whenever the camera moves -- chase or auto-zoom -- the scenery and the
        # scale bar move with it, so they cannot live in the blitted background.
        if self.moving:
            yield from self.scene_static
            yield self.scale_bar
            yield self.scale_text
        yield from (self.trail, self.ghost, self.plume_outer, self.plume_inner,
                    self.hull_poly, self.interstage_poly, self.band_lc,
                    self.fins_lc, self.legs_lc, self.hud, self.status)
        for _pax, lines, cursor in self.plots:
            for ln, _y in lines:
                yield ln
            yield cursor

    def _update(self, i: int) -> None:
        f, b = self.flight, self.boost
        com, body = f.xy[i], f.body_dir[i]
        dep = float(self.leg_frac[i])

        outline, *details = b.hull(com, body)
        self.hull_poly.set_xy(outline)
        self.band_lc.set_segments(details + b.engine(com, body))
        self.interstage_poly.set_xy(b.interstage(com, body))
        self.fins_lc.set_segments(b.grid_fins(com, body))
        self.legs_lc.set_segments(b.legs(com, body, dep))

        plume = b.plume(com, body, f.thrust_dir[i], f.thrust[i])
        if plume:
            self.plume_outer.set_xy(plume[0])
            self.plume_inner.set_xy(plume[1])
        self.plume_outer.set_visible(bool(plume))
        self.plume_inner.set_visible(bool(plume))

        self.trail.set_data(f.downrange[: i + 1], f.alt[: i + 1])
        if self.has_plan:
            self.ghost.set_data([self.ref_d[i]], [self.ref_a[i]])

        veh = self.meta["vehicle"]
        thr_pct = 100.0 * f.thrust[i] / veh["t_max"]
        self.hud.set_text(
            f"T+{f.t[i]:6.2f} s\n"
            f"alt   {f.alt[i]:7.1f} m\n"
            f"range {f.downrange[i]:7.1f} m\n"
            f"speed {f.speed[i]:7.1f} m/s\n"
            f"vs    {f.vspeed[i]:7.1f} m/s\n"
            f"thrust{thr_pct:6.1f} %\n"
            f"tilt  {f.tilt[i]:7.2f} °\n"
            f"prop  {f.mass[i] - veh['m_dry']:7.0f} kg"
            + (f"\nerr   {self.err[i]:7.2f} m" if self.has_plan else "")
        )
        if i >= self.n - 1:
            td = self.meta["touchdown"]
            self.status.set_text(
                f"TOUCHDOWN   {td['lateral_error_m']:.2f} m lateral   "
                f"{td['descent_rate_mps']:.2f} m/s   {td['tilt_deg']:.2f}° tilt")
            self.status.set_color(FLOWN if td["descent_rate_mps"] < 3.0 else LIMIT)
        elif dep > 0.02:
            self.status.set_text("LEGS DEPLOYED" if dep > 0.98 else "LEGS DEPLOYING")
            self.status.set_color(TEXT)
        elif self.phases:
            now = float(f.t[i])
            name = next((p["name"] for p in self.phases
                         if p["t_start"] <= now <= p["t_end"]), "")
            self.status.set_text(name.upper())
            self.status.set_color(TEXT)
        else:
            self.status.set_text("")

        for _pax, lines, cursor in self.plots:
            for ln, y in lines:
                ln.set_data(f.t[: i + 1], y[: i + 1])
            cursor.set_xdata([f.t[i], f.t[i]])

        if self.moving:
            cx, cy, half = self._window(i)
            width = 2.0 * half
            x0, y0 = cx - half, cy - half
            self.ax.set_xlim(x0, cx + half)
            self.ax.set_ylim(y0, cy + half)
            # the bar keeps a round length as the window shrinks
            self.scale_len = self._nice_scale(width)
            # top-left, below the legend: the pad and the plume own the bottom
            bx = x0 + 0.05 * width
            by = y0 + 0.80 * width
            self.scale_bar.set_data([bx, bx + self.scale_len], [by, by])
            self.scale_text.set_position((bx + 0.5 * self.scale_len,
                                          by + 0.012 * width))
            self.scale_text.set_text(self._scale_label(self.scale_len))

    # -- playback ---------------------------------------------------------
    def _blit(self, i: int, bg) -> None:
        self._update(i)
        self.fig.canvas.restore_region(bg)
        for art in self._dynamic_artists():
            art.axes.draw_artist(art)
        self.fig.canvas.blit(self.fig.bbox)

    def run(self, speed: float, loop: bool) -> None:
        self.plt.show(block=False)
        # Hide everything that moves before capturing the background, otherwise
        # follow mode bakes the scenery in at its starting position.
        hidden = list(self._dynamic_artists()) if self.moving else []
        for art in hidden:
            art.set_visible(False)
        self.fig.canvas.draw()
        bg = self.fig.canvas.copy_from_bbox(self.fig.bbox)
        for art in hidden:
            art.set_visible(True)

        t = self.flight.t
        frames = 0
        held = 0.0
        wall0 = time.perf_counter()
        while not self.closed:
            wall0 = time.perf_counter()
            last = -1
            if self.auto_zoom:                  # --loop replays the move too
                self.zoom_state, self.zoom_s = "wide", 0.0
            while not self.closed:
                now = time.perf_counter()

                # ---- the camera move, with the flight held ----------------
                if self.zoom_state == "zooming":
                    self.zoom_s = min(1.0, (now - hold_t0) / self.zoom_pause)
                    self._blit(hold_i, bg)
                    frames += 1
                    if self.zoom_s >= 1.0:
                        self.zoom_state = "close"
                        # hand the held time back so the rest stays real time
                        wall0 += now - hold_t0
                        held += now - hold_t0
                        last = -1
                    self.fig.canvas.flush_events()
                    time.sleep(0.015)
                    continue

                elapsed = (now - wall0) * (speed if speed > 0 else 1e9)
                i = int(np.searchsorted(t, elapsed, side="right")) - 1
                i = max(0, min(i, self.n - 1))

                if (self.zoom_state == "wide"
                        and self.zoom_alt[i] <= self.zoom_at):
                    if self.zoom_pause > 0.0:
                        self.zoom_state, hold_i, hold_t0 = "zooming", i, now
                        continue
                    self.zoom_state, self.zoom_s, last = "close", 1.0, -1

                if i != last:
                    self._blit(i, bg)
                    frames += 1
                    last = i
                self.fig.canvas.flush_events()
                if i >= self.n - 1:
                    break
                time.sleep(0.002)
            if not loop or self.closed:
                break
            time.sleep(1.2)

        wall = time.perf_counter() - wall0 + held
        note = f", {held:.1f} s of it holding for the zoom" if held else ""
        print(f"rendered {frames} frames in {wall:.1f} s wall "
              f"({frames / max(wall, 1e-6):.0f} fps, {self.n} samples available"
              f"{note})")
        if not self.closed:
            print("replay finished — close the window to exit")
            self.plt.show(block=True)


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--data", default="out", help="directory written by pdg_flight")
    ap.add_argument("--speed", type=float, default=1.0,
                    help="playback rate (1.0 = real time, 0 = as fast as possible)")
    ap.add_argument("--vehicle-scale", type=float, default=1.0,
                    help="draw the booster larger than life so it stays readable")
    ap.add_argument("--follow", type=float, default=0.0, metavar="METRES",
                    help="track the booster in a fixed window this wide for the "
                         "whole replay, instead of the adaptive camera")
    ap.add_argument("--zoom-at", type=float, default=2000.0, metavar="METRES",
                    help="altitude at which the camera starts closing in "
                         "(default 2000; 0 disables the adaptive camera)")
    ap.add_argument("--zoom-to", type=float, default=200.0, metavar="METRES",
                    help="window width it closes down to (default 200)")
    ap.add_argument("--zoom-pause", type=float, default=1.5, metavar="SECONDS",
                    help="hold the flight for this long while the camera closes "
                         "in (default 1.5; 0 cuts straight to the close view)")
    ap.add_argument("--leg-deploy-alt", type=float, default=None,
                    help="altitude at which the legs start swinging out [m]; "
                         "defaults to the value in meta.json, else 140")
    ap.add_argument("--loop", action="store_true", help="replay continuously")
    ap.add_argument("--backend", default="TkAgg",
                    help="matplotlib backend (default TkAgg, CPU rendered)")
    ap.add_argument("--save", metavar="FILE.mp4",
                    help="render to a video file instead of showing a window")
    ap.add_argument("--fps", type=int, default=30, help="frame rate for --save")
    args = ap.parse_args()

    matplotlib.use("Agg" if args.save else args.backend)

    meta, flight, plan = load_run(Path(args.data))
    boost = Booster(meta, args.vehicle_scale)

    g = meta["guidance"]
    if g.get("solver"):
        print(f"{meta.get('source', 'run')} / {meta.get('scenario', '?')}: "
              f"{g['tf']:.1f} s, {g['fuel_used_kg']:.0f} kg propellant, "
              f"guidance: {g['solver']}")
    else:
        print(f"SCvx plan: tf = {g['tf']:.2f} s, fuel {g['fuel_used_kg']:.1f} kg, "
              f"{g['scvx_iterations']} iterations")

    leg_alt = args.leg_deploy_alt
    if leg_alt is None:
        leg_alt = meta["vehicle"].get("leg_deploy_alt_m", 140.0)
    viewer = Viewer(meta, flight, plan, boost, follow=args.follow, leg_alt=leg_alt,
                    zoom_at=args.zoom_at, zoom_to=args.zoom_to,
                    zoom_pause=args.zoom_pause)
    if viewer.auto_zoom:
        how = (f"pausing {args.zoom_pause:.1f} s to close in"
               if args.zoom_pause > 0 else "cutting straight in")
        print(f"camera: whole trajectory at 1:1 down to {args.zoom_at:.0f} m, "
              f"then {how} to a {args.zoom_to:.0f} m window")
    elif viewer.follow > 0.0:
        print(f"camera: chase, fixed {args.follow:.0f} m window")

    if args.save:
        save_video(viewer, Path(args.save), args.fps)
        return

    print(f"replaying {len(flight.t)} samples at {args.speed}x on {matplotlib.get_backend()}")
    viewer.run(args.speed, args.loop)

    td = meta["touchdown"]
    label = td.get("speed_label", "descent rate")
    print(f"touchdown at t = {td['time']:.2f} s: lateral {td['lateral_error_m']:.2f} m, "
          f"{label} {td['descent_rate_mps']:.2f} m/s, tilt {td['tilt_deg']:.2f} deg")


def save_video(viewer: Viewer, path: Path, fps: int) -> None:
    from matplotlib import animation

    t = viewer.flight.t
    stamps = np.arange(t[0], t[-1], 1.0 / fps)
    idx = np.clip(np.searchsorted(t, stamps, side="right") - 1, 0, viewer.n - 1)
    idx = np.append(idx, viewer.n - 1)

    writer_name = "ffmpeg" if animation.writers.is_available("ffmpeg") else "pillow"
    if writer_name == "pillow":
        if path.suffix.lower() != ".gif":
            path = path.with_suffix(".gif")
            print("ffmpeg not available — writing an animated GIF instead")
        # Pillow quantises every frame into one palette and falls over on long
        # animations; a 132 s entry at 30 fps is 4000 frames. Thin them out.
        cap = 300
        if len(idx) > cap:
            idx = idx[np.linspace(0, len(idx) - 1, cap).astype(int)]
            fps = max(1, int(round(cap / (t[-1] - t[0]))))
            print(f"GIF capped at {cap} frames ({fps} fps); "
                  f"install ffmpeg for a full-rate video")

    # There is no wall clock here, so the camera move becomes real frames: the
    # flight index repeats while the zoom progresses, which is the same hold the
    # live replay does.
    plan_frames: list[tuple[int, float]] = []
    zoomed = not viewer.auto_zoom
    for k in idx:
        if not zoomed and viewer.zoom_alt[k] <= viewer.zoom_at:
            hold = max(1, int(round(viewer.zoom_pause * fps)))
            plan_frames += [(int(k), (j + 1) / hold) for j in range(hold)]
            zoomed = True
        plan_frames.append((int(k), 1.0 if zoomed else 0.0))

    def frame(k):
        i, zoom_s = plan_frames[k]
        viewer.zoom_s = zoom_s
        viewer._update(i)
        return list(viewer._dynamic_artists())

    anim = animation.FuncAnimation(viewer.fig, frame, frames=len(plan_frames),
                                   blit=False)
    anim.save(str(path), writer=writer_name, fps=fps,
              savefig_kwargs={"facecolor": BG})
    print(f"wrote {path} ({len(plan_frames)} frames at {fps} fps)")


if __name__ == "__main__":
    main()
