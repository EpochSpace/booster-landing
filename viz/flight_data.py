"""Shared reading, projection and vehicle geometry for the landing viewers.

``pdg_flight`` writes a flight log, the SCvx plan and a metadata file. Both
viewers -- the Rerun one (``land2d.py``) and the matplotlib one
(``land2d_mpl.py``) -- read them through here, so the booster is drawn from one
set of equations no matter which renderer is in front of it.
"""

from __future__ import annotations

import json
import math
from pathlib import Path

import numpy as np

DEG = 180.0 / math.pi

# ---------------------------------------------------------------------------
# palette, 0-255 RGB (Rerun takes these directly; mpl_color converts)
# ---------------------------------------------------------------------------
C_HULL = (214, 219, 226)
C_HULL_DARK = (120, 128, 140)
C_FLAME = (255, 149, 41)
C_FLAME_CORE = (255, 232, 168)
C_PLAN = (86, 180, 233)
C_FLOWN = (255, 214, 92)
C_PAD = (150, 158, 170)
C_PAD_MARK = (233, 239, 247)
C_GROUND = (74, 80, 90)
C_LIMIT = (196, 78, 82)
C_GHOST = (86, 180, 233, 120)
C_AUX = (160, 120, 220)
C_BG = (16, 18, 24)
C_PANEL = (22, 25, 32)
C_TEXT = (154, 163, 176)
C_WARN = (236, 180, 96)


def mpl_color(c) -> tuple:
    """0-255 RGB(A) -> matplotlib 0-1 tuple."""
    return tuple(v / 255.0 for v in c)


# ---------------------------------------------------------------------------
# data loading
# ---------------------------------------------------------------------------
COLUMNS = (
    "t m rx ry rz vx vy vz q0 q1 q2 q3 wx wy wz Tx Ty Tz Tmag tilt_deg speed"
).split()


def load_csv(path: Path) -> dict[str, np.ndarray]:
    raw = np.loadtxt(path, delimiter=",", skiprows=1, ndmin=2)
    if raw.shape[1] != len(COLUMNS):
        raise SystemExit(f"{path}: expected {len(COLUMNS)} columns, got {raw.shape[1]}")
    return {name: raw[:, i] for i, name in enumerate(COLUMNS)}


def quat_rotate(q: np.ndarray, v: np.ndarray) -> np.ndarray:
    """Rotate v (N,3) body -> inertial by q (N,4), scalar first."""
    w, x, y, z = q[:, 0], q[:, 1], q[:, 2], q[:, 3]
    qv = np.stack([x, y, z], axis=1)
    t = 2.0 * np.cross(qv, v)
    return v + w[:, None] * t + np.cross(qv, t)


class Track:
    """A trajectory projected onto the downrange/altitude plane."""

    def __init__(self, data: dict[str, np.ndarray], e_h: np.ndarray):
        self.t = data["t"]
        r = np.stack([data["rx"], data["ry"], data["rz"]], axis=1)
        v = np.stack([data["vx"], data["vy"], data["vz"]], axis=1)
        q = np.stack([data["q0"], data["q1"], data["q2"], data["q3"]], axis=1)
        u_b = np.stack([data["Tx"], data["Ty"], data["Tz"]], axis=1)

        e_c = np.cross(np.array([0.0, 0.0, 1.0]), e_h)  # cross-range axis

        self.alt = r[:, 2]
        self.downrange = r @ e_h
        self.crossrange = r @ e_c
        self.vspeed = v[:, 2]
        self.speed = data["speed"]
        self.mass = data["m"]
        self.thrust = data["Tmag"]
        self.tilt = data["tilt_deg"]

        # body +z and thrust, rotated into the inertial frame then projected
        b_i = quat_rotate(q, np.tile(np.array([0.0, 0.0, 1.0]), (len(q), 1)))
        t_i = quat_rotate(q, u_b)
        self.body_dir = np.stack([b_i @ e_h, b_i[:, 2]], axis=1)
        self.thrust_dir = np.stack([t_i @ e_h, t_i[:, 2]], axis=1)

        # signed in-plane tilt and the gimbal deflection off the body axis
        self.tilt_signed = np.arctan2(self.body_dir[:, 0], self.body_dir[:, 1]) * DEG
        self.gimbal = np.arctan2(np.hypot(u_b[:, 0], u_b[:, 1]),
                                 np.maximum(u_b[:, 2], 1e-9)) * DEG

        # angle of attack: between -v and the body axis (undefined at rest)
        vn = np.linalg.norm(v, axis=1)
        safe = np.maximum(vn, 1e-6)
        cos_aoa = np.einsum("ij,ij->i", -v / safe[:, None], b_i)
        self.aoa = np.degrees(np.arccos(np.clip(cos_aoa, -1.0, 1.0)))
        self.aoa[vn < 1.0] = np.nan

        self.xy = np.stack([self.downrange, self.alt], axis=1)


def load_run(data: Path) -> tuple[dict, "Track", "Track | None"]:
    """Read meta.json, flight.csv and, if present, reference.csv.

    The plan is optional: a producer whose guidance solves in the loop (the
    donor core, via ``bridge/bls_flight.py``) has no precomputed trajectory to
    compare against, and writes no reference.csv. The viewers then drop the
    plan line, the ghost marker and the tracking error.
    """
    meta = json.loads((data / "meta.json").read_text())
    flight_raw = load_csv(data / "flight.csv")

    # downrange axis: the horizontal direction the booster starts out along
    r0 = np.array([flight_raw["rx"][0], flight_raw["ry"][0], 0.0])
    if np.linalg.norm(r0) < 1e-6:
        r0 = np.array([1.0, 0.0, 0.0])
    e_h = r0 / np.linalg.norm(r0)

    ref = data / "reference.csv"
    plan = Track(load_csv(ref), e_h) if ref.exists() else None
    return meta, Track(flight_raw, e_h), plan


# ---------------------------------------------------------------------------
# vehicle drawing
# ---------------------------------------------------------------------------
class Booster:
    """Falcon-9-class first stage in the (downrange, altitude) plane, in metres.

    Everything is built from two numbers the producer reports -- overall length
    and diameter -- so the same code draws pdg's 41.2 m stage and the donor's
    47.7 m one at their true proportions. A landing booster is a very slender
    object: 13 diameters long, with grid fins at the very top and legs that
    splay to five diameters at the bottom. Drawing it as a fat rectangle loses
    all of that, so the parts are laid out at their real stations.
    """

    # stations as a fraction of overall length, and widths in half-diameters
    F_SKIRT = 0.045          # engine bay, slightly narrower than the tank
    F_INTERSTAGE = 0.135     # black composite section at the top
    F_FIN = 0.945            # grid fin mounting station
    F_LEG_HINGE = 0.055      # main leg hinge
    F_LEG_PUSHROD = 0.135    # pushrod upper hinge
    LEG_SPAN_W = 4.58        # gives an 18 m footpad span, as on Falcon 9
    BELL_H_W = 1.4           # engine bell height, in half-diameters

    def __init__(self, meta: dict, scale: float):
        veh = meta["vehicle"]
        self.length = veh["length_m"] * scale
        self.half_w = 0.5 * veh["diameter_m"] * scale
        self.below = veh["engine_offset_m"] * scale       # engine gimbal point to CoM
        self.above = self.length - self.below
        self.t_max = veh["t_max"]
        self.scale = scale

    # -- frame ------------------------------------------------------------
    def _axes(self, com: np.ndarray, body: np.ndarray):
        """Base point, axial unit vector and lateral unit vector.

        The base -- the nozzle exit plane -- sits on the logged point. pdg's
        state is the centre of mass and its touchdown condition is ``r_z <= 0``,
        i.e. the CoM reaching ground level, because that model carries no vehicle
        height; drawing the hull around the CoM would bury the stage in the deck.
        Anchoring the base there instead puts the stage on the pad exactly when
        the guidance says altitude is zero. For donor data the logged height is
        already the vehicle base, so the same rule is exact. Every plotted number
        stays the raw logged value.
        """
        b = body / max(np.linalg.norm(body), 1e-9)
        p = np.array([b[1], -b[0]])
        return np.asarray(com, dtype=float), b, p

    def _pt(self, base, b, p, h, lat):
        """A point h metres up the axis and lat metres to the side."""
        return base + h * b + lat * p

    def _poly(self, base, b, p, pts):
        return np.array([self._pt(base, b, p, h, lat) for h, lat in pts])

    # -- structure --------------------------------------------------------
    def hull(self, com, body) -> list[np.ndarray]:
        """Outline first, then detail lines: callers may fill the outline."""
        base, b, p = self._axes(com, body)
        L, w = self.length, self.half_w
        skirt = self.F_SKIRT * L
        inter = (1.0 - self.F_INTERSTAGE) * L
        chamfer = 0.012 * L

        outline = self._poly(base, b, p, [
            (0.0, -0.80 * w),               # engine bay, tucked in
            (skirt, -w),                    # flare out to tank diameter
            (L - chamfer, -w),
            (L, -0.94 * w),                 # small chamfer at the interstage lip
            (L, 0.94 * w),
            (L - chamfer, w),
            (skirt, w),
            (0.0, 0.80 * w),
            (0.0, -0.80 * w),
        ])

        details = [
            # interstage joint -- the black section starts here
            self._poly(base, b, p, [(inter, -w), (inter, w)]),
            # octaweb joint
            self._poly(base, b, p, [(skirt, -w), (skirt, w)]),
            # raceway running the length of the tank on one side
            self._poly(base, b, p, [(skirt + 0.02 * L, 0.62 * w),
                                    (inter - 0.02 * L, 0.62 * w)]),
        ]
        return [outline] + details

    def interstage(self, com, body) -> np.ndarray:
        """The dark composite section at the top, as a fillable polygon."""
        base, b, p = self._axes(com, body)
        L, w = self.length, self.half_w
        inter = (1.0 - self.F_INTERSTAGE) * L
        chamfer = 0.012 * L
        return self._poly(base, b, p, [
            (inter, -w), (L - chamfer, -w), (L, -0.94 * w),
            (L, 0.94 * w), (L - chamfer, w), (inter, w), (inter, -w),
        ])

    def engine(self, com, body) -> list[np.ndarray]:
        """Merlin bell, drawn inside the engine bay above the exit plane."""
        base, b, p = self._axes(com, body)
        w, hb = self.half_w, self.BELL_H_W * self.half_w
        return [self._poly(base, b, p, [
            (0.0, -0.46 * w), (hb, -0.17 * w), (hb, 0.17 * w), (0.0, 0.46 * w),
        ])]

    def grid_fins(self, com, body) -> list[np.ndarray]:
        """Two of the four fins show in a side view, as lattice panels."""
        base, b, p = self._axes(com, body)
        L, w = self.length, self.half_w
        station = self.F_FIN * L
        span, chord = 0.9 * w, 1.15 * w
        out = []
        for s in (-1.0, 1.0):
            root, tip = s * w, s * (w + span)
            lo, hi = station - 0.5 * chord, station + 0.5 * chord
            out.append(self._poly(base, b, p, [
                (lo, root), (lo, tip), (hi, tip), (hi, root), (lo, root)]))
            # lattice: the fins are grids, and that reads even at small scale
            for f in (0.33, 0.67):
                out.append(self._poly(base, b, p, [
                    (lo + f * chord, root), (lo + f * chord, tip)]))
                out.append(self._poly(base, b, p, [
                    (lo, root + f * (tip - root)), (hi, root + f * (tip - root))]))
        return out

    def legs(self, com, body, deployed: float) -> list[np.ndarray]:
        """Four legs, two visible: main strut, pushrod and footpad each side.

        ``deployed`` runs 0 (folded along the hull) to 1 (out at full span).
        """
        base, b, p = self._axes(com, body)
        L, w, d = self.length, self.half_w, float(np.clip(deployed, 0.0, 1.0))
        hinge_h = self.F_LEG_HINGE * L
        push_h = self.F_LEG_PUSHROD * L

        # stowed the foot sits high and tight; deployed it is out and just below
        # the exit plane, so the pads are what touches the deck
        foot_lat = (1.05 * w) + d * (self.LEG_SPAN_W - 1.05) * w
        foot_h = (1.0 - d) * (hinge_h - 0.6 * w) - d * 0.25 * w
        pad = 0.34 * w

        out = []
        for s in (-1.0, 1.0):
            hinge = (hinge_h, s * w)
            foot = (foot_h, s * foot_lat)
            # main strut
            out.append(self._poly(base, b, p, [hinge, foot]))
            # pushrod, meeting the strut part way down
            mid = (hinge[0] + 0.55 * (foot[0] - hinge[0]),
                   hinge[1] + 0.55 * (foot[1] - hinge[1]))
            out.append(self._poly(base, b, p, [(push_h, s * 0.85 * w), mid]))
            # footpad
            out.append(self._poly(base, b, p, [
                (foot[0], foot[1] - s * pad), (foot[0], foot[1] + s * pad)]))
        return out

    def plume(self, com, body, thrust_dir, thrust_n) -> list[np.ndarray]:
        """Exhaust cone from the engine, opposite the thrust vector."""
        frac = float(np.clip(thrust_n / self.t_max, 0.0, 1.0))
        if frac <= 1e-3:
            return []
        base, b, _ = self._axes(com, body)
        nozzle = base
        d = np.asarray(thrust_dir, dtype=float)
        d = d / max(np.linalg.norm(d), 1e-9)
        e = -d                                    # exhaust points down the thrust
        n = np.array([e[1], -e[0]])
        w = self.half_w
        length = (2.5 + 11.0 * frac) * w
        flare = (0.45 + 0.65 * frac) * w
        outer = np.array([
            nozzle - flare * n,
            nozzle + e * length * 0.62 - flare * 1.55 * n,
            nozzle + e * length,
            nozzle + e * length * 0.62 + flare * 1.55 * n,
            nozzle + flare * n,
        ])
        inner = np.array([
            nozzle - flare * 0.45 * n,
            nozzle + e * length * 0.5,
            nozzle + flare * 0.45 * n,
        ])
        return [outer, inner]


def leg_deployment(alt: float, leg_alt: float) -> float:
    """Leg swing-out fraction at a given altitude."""
    return float(np.clip((leg_alt - alt) / max(leg_alt * 0.55, 1.0), 0.0, 1.0))


def leg_deployment_track(alt: np.ndarray, leg_alt: float) -> np.ndarray:
    """Leg swing-out fraction per sample, deployed only on the final descent.

    Altitude alone is not enough once a flight can go *up* through the trigger.
    A booster lifts off with its legs stowed, and on an RTLS profile it passes
    250 m twice -- once at about eight seconds into the ascent, once on the way
    home. Keying off altitude alone puts the legs out on the launch pad.

    So the legs arm at the last moment the stage was above ``leg_alt``, which on
    a descent-only flight is before the first sample and leaves the behaviour
    exactly as it was.
    """
    alt = np.asarray(alt, dtype=float)
    frac = np.clip((leg_alt - alt) / max(leg_alt * 0.55, 1.0), 0.0, 1.0)
    above = np.flatnonzero(alt > leg_alt)
    if above.size:
        frac[: int(above[-1]) + 1] = 0.0
    return frac


def scene_bounds(flight: Track, plan: "Track | None") -> tuple[float, float, float, float]:
    tracks = [flight] if plan is None else [flight, plan]
    lo = min(t.downrange.min() for t in tracks)
    hi = max(t.downrange.max() for t in tracks)
    top = max(t.alt.max() for t in tracks)
    # margins scale with the flight: a 62 km entry and a 500 m hop both need room
    pad = max(0.08 * (hi - lo), 0.02 * top, 90.0)
    return min(lo, -80.0) - pad, max(hi, 80.0) + pad, -0.02 * top - 40.0, top * 1.10 + 40.0


# pad geometry, shared so both viewers draw the same deck
PAD_HALF_WIDTH = 45.0
PAD_DECK_HEIGHT = 3.2


def pad_geometry():
    """Deck outline, edge markings and bullseye, in (downrange, altitude)."""
    half, deck = PAD_HALF_WIDTH, PAD_DECK_HEIGHT
    outline = np.array([[-half, 0.0], [-half, deck], [half, deck], [half, 0.0],
                        [-half, 0.0]])
    marks = [np.array([[x, deck], [x, deck + 4.0]])
             for x in (-half, -half / 2, half / 2, half)]
    bullseye = [np.array([[-14.0, deck + 0.4], [14.0, deck + 0.4]]),
                np.array([[-9.0, deck + 0.4], [9.0, deck + 0.4]])]
    return outline, marks, bullseye
