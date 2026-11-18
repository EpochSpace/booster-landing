"""Ascent, boostback and coast for a solo first stage flying back to its pad.

This is the half of an RTLS flight that happens above the atmosphere, plus the
ascent that gets there. It owns everything from liftoff to the **entry
interface** on the way down; below that the donor core takes over, because that
part is aerodynamic and the donor already models it (US76 atmosphere, body and
grid-fin aerodynamics, leg contact). See ``docs/task-rtls.md``, *Architecture:
the split at 70 km* -- the interface actually sits at 65 km, for a reason that is
about the donor's guidance rather than the physics; see ``ENTRY_INTERFACE_ALT``.

Why the split is legitimate. Measured against US76 at 1579 m/s, dynamic
pressure is 386 Pa at 60 km, 23 Pa at 80 km and 0.7 Pa at 100 km -- the last of
which is a drag deceleration of order 1e-5 g on an 80 t stage, about 0.2 % of
the thrust acceleration. Thrust-and-gravity dynamics are therefore physically
valid across the boostback and the coast. The ascent is *not* in that regime, so
this module carries a US76 atmosphere and a drag model for it; ``self_test()``
checks the atmosphere against the published table.

Frame
-----
Planar, Earth-centred, **non-rotating**. State is ``[X, Z, VX, VZ, m]`` with the
pad at ``(0, R_EARTH)``, so ``+Z`` is up at the pad and ``+X`` is downrange.
Gravity is central, ``-mu r / |r|^3``, with ``mu`` pinned so that surface gravity
is exactly ``G0``. Two deliberate omissions, both stated rather than hidden:

* **No Earth rotation.** A due-east launch from 28.5 deg would start with
  408 m/s of eastward ground speed and pick up a Coriolis cross-range. The
  flight here is planar by construction and the donor's frame is flat and
  non-rotating too, so adding it would have to stop at the handover anyway.
* **No cross-range.** Everything is in one plane, which is what a side view
  shows and what the donor's ``lat`` is.

Curvature *is* modelled: over 70 km of downrange the Earth drops 384 m, which is
small against 130 km of altitude but not against a landing.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field

import numpy as np

# ---------------------------------------------------------------------------
# World
# ---------------------------------------------------------------------------

G0 = 9.80665
R_EARTH = 6_371_000.0            # m, mean radius, for gravity and the ground track
R_GEOP = 6_356_766.0             # m, geopotential radius -- the donor's, for US76
MU = G0 * R_EARTH ** 2           # pinned so g(0) == G0 exactly
R_AIR = 287.053
GAMMA_AIR = 1.4

# US76, seven base layers up to 84.852 km geopotential (NASA-TM-X-74335 table I)
_H_B = np.array([0.0, 11_000.0, 20_000.0, 32_000.0, 47_000.0, 51_000.0, 71_000.0])
_L_B = np.array([-0.0065, 0.0, 0.001, 0.0028, 0.0, -0.0028, -0.002])
_T_B = np.array([288.15, 216.65, 216.65, 228.65, 270.65, 270.65, 214.65])
_P_B = np.array([101_325.0, 22_632.06, 5474.889, 868.0187, 110.9063, 66.93887, 3.956420])
_H_TOP = 84_852.0                # top of the US76 layer stack, geopotential

# Above 86 km geometric -- which is exactly where the layer stack ends, since
# 84.852 km geopotential *is* 86 km geometric -- US76 stops being a lapse-rate
# model: molecular weight varies and the temperature profile is tabulated rather
# than piecewise-linear. Density is therefore log-interpolated from the
# published table instead of extrapolated, which matters only for getting the
# right *zero*: at 100 km this is 0.7 Pa of dynamic pressure at 1579 m/s,
# the figure the task quotes, and an exponential tail overstates it by 20 %.
_H_UPPER = np.array([86_000.0, 90_000.0, 95_000.0, 100_000.0, 110_000.0,
                     120_000.0, 130_000.0, 150_000.0, 180_000.0])
_RHO_UPPER = np.array([6.958e-6, 3.416e-6, 1.393e-6, 5.604e-7, 9.708e-8,
                       2.222e-8, 8.152e-9, 2.076e-9, 5.194e-10])
_T_UPPER = np.array([186.87, 186.87, 186.87, 195.08, 240.00, 360.00,
                     469.27, 634.39, 790.65])


def atmosphere(h: float) -> tuple[float, float, float]:
    """US76 density, pressure and speed of sound at geometric altitude ``h`` [m].

    Two regimes, split at 86 km geometric: the seven-layer lapse-rate stack
    below, the tabulated upper atmosphere above. See ``self_test`` for the
    check against the published table and against the task's own dynamic
    pressures.
    """
    if h <= 0.0:
        h = 0.0
    if h >= _H_UPPER[0]:
        if h >= _H_UPPER[-1]:
            # a plain exponential past 180 km; rho is already 5e-10 there
            rho = float(_RHO_UPPER[-1]) * math.exp(-(h - _H_UPPER[-1]) / 40_000.0)
            t = float(_T_UPPER[-1])
        else:
            rho = float(math.exp(np.interp(h, _H_UPPER, np.log(_RHO_UPPER))))
            t = float(np.interp(h, _H_UPPER, _T_UPPER))
        return rho, rho * R_AIR * t, math.sqrt(GAMMA_AIR * R_AIR * t)
    hg = R_GEOP * h / (R_GEOP + h)          # geometric -> geopotential
    t, p, _ = _layer(hg)
    return p / (R_AIR * t), p, math.sqrt(GAMMA_AIR * R_AIR * t)


def _layer(hg: float) -> tuple[float, float, int]:
    b = int(np.searchsorted(_H_B, hg, side="right") - 1)
    b = max(0, min(b, len(_H_B) - 1))
    dh = hg - _H_B[b]
    lb, tb, pb = _L_B[b], _T_B[b], _P_B[b]
    if abs(lb) < 1e-12:
        t = tb
        p = pb * math.exp(-G0 * dh / (R_AIR * tb))
    else:
        t = tb + lb * dh
        p = pb * (tb / t) ** (G0 / (R_AIR * lb))
    return t, p, b


# ---------------------------------------------------------------------------
# Vehicle
# ---------------------------------------------------------------------------

ENG_T_SL = 845_000.0             # N, one Merlin 1D at sea level [official]
ENG_ISP_SL = 282.0               # s [official]
ENG_ISP_VAC = 311.0              # s [official]
ENG_MDOT = ENG_T_SL / (G0 * ENG_ISP_SL)          # 305.6 kg/s [derived]
ENG_T_VAC = ENG_MDOT * G0 * ENG_ISP_VAC          # 931.9 kN [derived]
ENG_AE = (ENG_T_VAC - ENG_T_SL) / 101_325.0      # 0.859 m^2 [derived]
ENG_THR_MIN = 0.40               # the donor's floor; see the note in AscentConfig
ENG_COUNT = 9

VEH_DRY = 25_600.0               # kg, bare 22.2 t plus legs and grid fins
VEH_DIA = 3.66                   # m
VEH_AREF = math.pi * (0.5 * VEH_DIA) ** 2        # 10.52 m^2, the donor's reference area
VEH_STAGE_LEN = 42.6             # m, stage without the interstage
PROP_MAX = 410_900.0             # kg, a full stage-1 load (287.4 LOX + 123.5 RP-1)

# The nose fairing. Flying solo the stage has an open interstage where the second
# stage used to be, so the ascent is flown with a cone fitted over it: 3.66 m
# base, 6.0 m long, a 17 deg half-angle -- an ordinary launch-vehicle nose. It
# changes the ascent drag and nothing else, and the task flags it as not
# critical. Length below is only used to report the ascent configuration.
NOSE_LEN = 6.0
NOSE_HALF_ANGLE_DEG = math.degrees(math.atan(0.5 * VEH_DIA / NOSE_LEN))

# ---- Cd, nose-first, with the fairing on (work item 1) ---------------------
# A pointed cone-cylinder at zero incidence: ~0.30 subsonic, a transonic peak
# just past M 1, decaying through supersonic. Referenced to VEH_AREF, so the
# peak Cd.S is 5.8 m^2.
#
# This is NOT the coefficient the return flies on, and that is the whole
# resolution of work item 1. The notebook's Cd 0.5 on 12.57 m^2 and the expert
# review's Cd ~1.0 disagree by 1.7x because they describe **different
# attitudes**. A returning booster flies engines-first: a flat, separated base
# leading, which is a bluff body, not a cone. The donor already carries that as
# a Mach table, CA 0.85..1.40 on the same 10.52 m^2 (core/dynamics.c), i.e.
# Cd.S 8.9..14.7 m^2 -- it sides with the review, and since the donor owns
# everything below the entry interface, the descent coefficient is settled by
# the code that flies it. What is left for this module to choose is the
# *ascent* coefficient, nose-first, which is the table below.
_CD_M = np.array([0.0, 0.60, 0.80, 0.90, 1.05, 1.20, 1.50, 2.00, 3.00, 5.00, 8.00])
_CD_A = np.array([0.30, 0.30, 0.32, 0.38, 0.55, 0.52, 0.45, 0.38, 0.30, 0.26, 0.24])

# Engines-first, for the short stretch of our own coast that is still inside
# measurable air (60-70 km). The donor's own CA table, so the two halves of the
# flight do not disagree across the handover.
_CD_M_RETRO = np.array([0.0, 0.6, 0.9, 1.1, 1.5, 2.0, 3.0, 5.0, 8.0])
_CD_A_RETRO = np.array([0.85, 0.88, 1.10, 1.40, 1.25, 1.10, 0.95, 0.92, 0.90])


def cd_ascent(mach: float) -> float:
    return float(np.interp(mach, _CD_M, _CD_A))


def cd_retro(mach: float) -> float:
    return float(np.interp(mach, _CD_M_RETRO, _CD_A_RETRO))


# ---------------------------------------------------------------------------
# Geometry helpers
# ---------------------------------------------------------------------------

def altitude(x: float, z: float) -> float:
    return math.hypot(x, z) - R_EARTH


def downrange(x: float, z: float) -> float:
    """Arc distance along the surface from the pad, signed, +ve downrange."""
    return R_EARTH * math.atan2(x, z)


def local_frame(x: float, z: float) -> tuple[np.ndarray, np.ndarray]:
    """Local up and local downrange unit vectors at position ``(x, z)``."""
    r = math.hypot(x, z)
    up = np.array([x / r, z / r])
    east = np.array([z / r, -x / r])
    return up, east


def to_local(x: float, z: float, vx: float, vz: float) -> tuple[float, float, float, float]:
    """Inertial planar state -> the donor's flat frame: (range, alt, v_range, v_up)."""
    up, east = local_frame(x, z)
    v = np.array([vx, vz])
    return downrange(x, z), altitude(x, z), float(v @ east), float(v @ up)


# ---------------------------------------------------------------------------
# Equations of motion
# ---------------------------------------------------------------------------

@dataclass
class Thrusting:
    """What the engines are doing: how many, at what throttle, pointing where.

    ``direction`` is a callable ``(t, y) -> unit vector`` in the planar inertial
    frame. ``n_eng`` of zero means the engines are off and the mass is frozen.
    """
    n_eng: int
    throttle: float
    direction: object = None
    retro_drag: bool = False


def _accel(t: float, y: np.ndarray, cmd: Thrusting) -> np.ndarray:
    x, z, vx, vz, m = y
    r = math.hypot(x, z)
    h = r - R_EARTH

    # central gravity
    a = np.array([-MU * x / r ** 3, -MU * z / r ** 3])

    rho, p_amb, a_snd = atmosphere(h)
    speed = math.hypot(vx, vz)

    # drag, opposing the velocity
    if rho > 1e-12 and speed > 1e-6:
        mach = speed / a_snd
        cd = cd_retro(mach) if cmd.retro_drag else cd_ascent(mach)
        q = 0.5 * rho * speed * speed
        d = q * VEH_AREF * cd
        a -= (d / m) * np.array([vx / speed, vz / speed])

    mdot = 0.0
    # An empty stage does not thrust. Without this a solver that overshoots the
    # burn duration integrates straight through the dry mass into negative
    # propellant, where thrust/m diverges through zero and the trajectory it
    # returns is not wrong so much as meaningless -- and, being meaningless, it
    # can still bracket a root and be mistaken for an answer.
    if m <= VEH_DRY:
        return np.array([vx, vz, a[0], a[1], 0.0])
    if cmd.n_eng > 0 and cmd.throttle > 0.0:
        # pressure-corrected thrust, exactly as the donor derives it
        t_eng = ENG_T_VAC - ENG_AE * p_amb
        thrust = cmd.n_eng * cmd.throttle * t_eng
        u = cmd.direction(t, y)
        a += (thrust / m) * u
        mdot = cmd.n_eng * cmd.throttle * ENG_MDOT

    return np.array([vx, vz, a[0], a[1], -mdot])


def integrate(y0: np.ndarray, cmd: Thrusting, dt: float, t0: float,
              stop) -> tuple[np.ndarray, np.ndarray]:
    """RK4 to the first sample where ``stop(t, y)`` is true, with a bisected end.

    Returns ``(t, Y)`` with ``Y`` shaped ``(n, 5)``. The final sample is refined
    onto the stop condition by bisection, so a cutoff on velocity or an altitude
    crossing lands on the condition rather than on the step after it.
    """
    ts, ys = [t0], [np.asarray(y0, dtype=float)]
    t, y = t0, np.asarray(y0, dtype=float)
    guard = 0
    while not stop(t, y):
        guard += 1
        if guard > 2_000_000:
            raise RuntimeError("integrate: no stop condition reached")
        k1 = _accel(t, y, cmd)
        k2 = _accel(t + 0.5 * dt, y + 0.5 * dt * k1, cmd)
        k3 = _accel(t + 0.5 * dt, y + 0.5 * dt * k2, cmd)
        k4 = _accel(t + dt, y + dt * k3, cmd)
        y_next = y + (dt / 6.0) * (k1 + 2 * k2 + 2 * k3 + k4)
        t_next = t + dt

        if stop(t_next, y_next):
            # bisect the last step onto the condition
            lo, hi = 0.0, dt
            for _ in range(40):
                mid = 0.5 * (lo + hi)
                k1 = _accel(t, y, cmd)
                k2 = _accel(t + 0.5 * mid, y + 0.5 * mid * k1, cmd)
                k3 = _accel(t + 0.5 * mid, y + 0.5 * mid * k2, cmd)
                k4 = _accel(t + mid, y + mid * k3, cmd)
                y_mid = y + (mid / 6.0) * (k1 + 2 * k2 + 2 * k3 + k4)
                if stop(t + mid, y_mid):
                    hi = mid
                else:
                    lo = mid
            k1 = _accel(t, y, cmd)
            k2 = _accel(t + 0.5 * hi, y + 0.5 * hi * k1, cmd)
            k3 = _accel(t + 0.5 * hi, y + 0.5 * hi * k2, cmd)
            k4 = _accel(t + hi, y + hi * k3, cmd)
            y_next, t_next = y + (hi / 6.0) * (k1 + 2 * k2 + 2 * k3 + k4), t + hi

        t, y = t_next, y_next
        ts.append(t)
        ys.append(y)
    return np.array(ts), np.array(ys)


# ---------------------------------------------------------------------------
# Ascent
# ---------------------------------------------------------------------------

@dataclass
class AscentConfig:
    """The ascent's fixed choices; the solver moves ``AscentParams`` inside these.

    **Engine count and throttle.** A solo stage is light -- it lifts off at
    around 200 t rather than 541 t -- so nine Merlins at full thrust would give
    it a liftoff T/W near 4, which is not a launch vehicle. The count is
    therefore chosen, not assumed: ``size_engines`` picks the smallest cluster
    whose liftoff T/W clears ``tw_min``, which for this vehicle is three
    engines at full throttle. Running all nine deep-throttled instead would need
    41 % throttle, and while the donor's ``ENG_THR_MIN`` is 0.40, the published
    Merlin 1D floor is nearer 57 % and deep throttling costs Isp that this model
    does not carry. Three engines at 100 % avoids both objections and is the
    same cluster the boostback and entry burns use.
    """
    t_vertical: float = 8.0          # s of vertical rise before the pitch kick
    t_ramp: float = 12.0             # s to swing the kick in
    h_free: float = 40_000.0         # m: above this the stage may steer off the
                                     # velocity vector (q < 300 Pa, so no load penalty)
    aoa_max_deg: float = 25.0        # cap on that off-velocity bias
    a_cap_lo: float = 1.25           # g, bounds on the free axial-acceleration cap
    a_cap_hi: float = 4.00
    tw_min: float = 1.35             # minimum full-thrust liftoff thrust-to-weight
    n_eng: int = 0                   # 0 = size it from tw_min
    dt: float = 0.05


@dataclass
class AscentParams:
    m0: float                        # kg, liftoff mass (dry + propellant loaded)
    t_pitch: float                   # s, when the kick starts
    kick_deg: float                  # deg off vertical at the end of the ramp
    bias_deg: float                  # deg off the velocity vector above h_free
    a_cap_g: float = 4.0             # axial acceleration held by throttling


@dataclass
class AscentResult:
    t: np.ndarray
    y: np.ndarray
    throttle: np.ndarray
    thrust: np.ndarray
    pitch: np.ndarray                # thrust direction, deg from local vertical
    n_eng: int
    params: AscentParams
    cutoff: dict = field(default_factory=dict)
    max_q: float = 0.0
    max_g: float = 0.0
    losses: dict = field(default_factory=dict)


def size_engines(m0: float, cfg: AscentConfig) -> int:
    if cfg.n_eng:
        return cfg.n_eng
    for n in (1, 2, 3, 4, 5, 9):
        if n * ENG_T_SL >= cfg.tw_min * m0 * G0:
            return n
    return 9


def _ascent_direction(p: AscentParams, cfg: AscentConfig):
    """Thrust unit vector: vertical, then a pitch kick, then a gravity turn.

    Zero angle of attack through the dense atmosphere -- the stage flies where it
    points, which is what keeps the airframe loads honest -- and a constant bias
    off the velocity vector allowed only above ``h_free``, where the dynamic
    pressure is under a few hundred pascals and steering is free. That bias is
    the one degree of freedom the gravity turn does not have, and without it the
    three cutoff targets cannot all be met.
    """
    kick = math.radians(p.kick_deg)
    bias = math.radians(p.bias_deg)

    def direction(t: float, y: np.ndarray) -> np.ndarray:
        x, z, vx, vz, _ = y
        up, east = local_frame(x, z)
        if t < p.t_pitch:
            return up
        speed = math.hypot(vx, vz)
        if t < p.t_pitch + cfg.t_ramp or speed < 1.0:
            s = min(1.0, (t - p.t_pitch) / max(cfg.t_ramp, 1e-6))
            s = s * s * (3.0 - 2.0 * s)
            ang = kick * s
            return math.cos(ang) * up + math.sin(ang) * east
        vhat = np.array([vx / speed, vz / speed])
        if altitude(x, z) < cfg.h_free or abs(bias) < 1e-9:
            return vhat
        # rotate the velocity direction by `bias` towards downrange
        n = np.array([vhat[1], -vhat[0]])          # +90 deg -> downrange side
        if n @ east < 0:
            n = -n
        return math.cos(bias) * vhat + math.sin(bias) * n

    return direction


def _throttle_for(m: float, n_eng: int, p_amb: float, a_cap_g: float) -> float:
    """Throttle to hold the axial acceleration cap, within the engine's floor.

    This is the knob that sets how *long* the ascent takes, and therefore how
    lofted it is, which turns out to be the whole difficulty of the solo ascent.
    A light stage on nine -- or even four -- Merlins reaches 1579 m/s in about
    110 s and is only 54 km up when it does. CRS-9 took 143 s to the same speed
    because it was dragging a 107.5 t upper stack, and the extra 33 s of climb is
    most of the missing 6 km. Without an upper stage the only way to buy that
    time back is to throttle, so the cap is a free variable of the solve rather
    than a fixed limit, and the value it settles on is a result worth reporting.
    """
    t_eng = ENG_T_VAC - ENG_AE * p_amb
    want = a_cap_g * G0 * m / (n_eng * t_eng)
    return float(min(1.0, max(ENG_THR_MIN, want)))


def fly_ascent(p: AscentParams, cfg: AscentConfig, v_cutoff: float) -> AscentResult:
    """Integrate the ascent to the first time the speed reaches ``v_cutoff``."""
    n_eng = size_engines(p.m0, cfg)
    direction = _ascent_direction(p, cfg)

    ts = [0.0]
    ys = [np.array([0.0, R_EARTH, 0.0, 0.0, p.m0])]
    thr, tmag, pitch = [], [], []
    t, y = 0.0, ys[0]
    dt = cfg.dt
    max_q = 0.0
    max_g = 0.0
    loss_grav = 0.0
    loss_drag = 0.0
    dv_ideal = 0.0
    prop = p.m0 - VEH_DRY

    while True:
        x, z, vx, vz, m = y
        h = altitude(x, z)
        rho, p_amb, a_snd = atmosphere(h)
        speed = math.hypot(vx, vz)

        throttle = _throttle_for(m, n_eng, p_amb, p.a_cap_g)
        if m - VEH_DRY <= 0.0:
            raise RuntimeError("ascent ran the tanks dry before cutoff")
        cmd = Thrusting(n_eng, throttle, direction)

        t_eng = ENG_T_VAC - ENG_AE * p_amb
        thrust = n_eng * throttle * t_eng
        u = direction(t, y)
        up, _ = local_frame(x, z)

        thr.append(throttle)
        tmag.append(thrust)
        pitch.append(math.degrees(math.acos(float(np.clip(u @ up, -1.0, 1.0)))))

        q = 0.5 * rho * speed * speed
        max_q = max(max_q, q)
        max_g = max(max_g, thrust / (m * G0))

        # loss bookkeeping, integrated the standard way
        if speed > 1.0:
            vhat = np.array([vx / speed, vz / speed])
            sin_gamma = float(vhat @ up)
        else:
            sin_gamma = 1.0
        g_here = MU / (x * x + z * z)
        loss_grav += g_here * sin_gamma * dt
        loss_drag += (q * VEH_AREF * cd_ascent(speed / a_snd) / m) * dt
        dv_ideal += (thrust / m) * dt

        if speed >= v_cutoff:
            break

        k1 = _accel(t, y, cmd)
        k2 = _accel(t + 0.5 * dt, y + 0.5 * dt * k1, cmd)
        k3 = _accel(t + 0.5 * dt, y + 0.5 * dt * k2, cmd)
        k4 = _accel(t + dt, y + dt * k3, cmd)
        y_next = y + (dt / 6.0) * (k1 + 2 * k2 + 2 * k3 + k4)

        if math.hypot(y_next[2], y_next[3]) >= v_cutoff:
            lo, hi = 0.0, dt
            for _ in range(40):
                mid = 0.5 * (lo + hi)
                k1 = _accel(t, y, cmd)
                k2 = _accel(t + 0.5 * mid, y + 0.5 * mid * k1, cmd)
                k3 = _accel(t + 0.5 * mid, y + 0.5 * mid * k2, cmd)
                k4 = _accel(t + mid, y + mid * k3, cmd)
                y_mid = y + (mid / 6.0) * (k1 + 2 * k2 + 2 * k3 + k4)
                if math.hypot(y_mid[2], y_mid[3]) >= v_cutoff:
                    hi = mid
                else:
                    lo = mid
            k1 = _accel(t, y, cmd)
            k2 = _accel(t + 0.5 * hi, y + 0.5 * hi * k1, cmd)
            k3 = _accel(t + 0.5 * hi, y + 0.5 * hi * k2, cmd)
            k4 = _accel(t + hi, y + hi * k3, cmd)
            y_next, dt_last = y + (hi / 6.0) * (k1 + 2 * k2 + 2 * k3 + k4), hi
            t = t + dt_last
            y = y_next
            ts.append(t)
            ys.append(y)
            # one final sample's worth of command, so the arrays line up
            x, z, vx, vz, m = y
            _, p_amb, _ = atmosphere(altitude(x, z))
            throttle = _throttle_for(m, n_eng, p_amb, p.a_cap_g)
            thr.append(throttle)
            tmag.append(n_eng * throttle * (ENG_T_VAC - ENG_AE * p_amb))
            u = direction(t, y)
            up, _ = local_frame(x, z)
            pitch.append(math.degrees(math.acos(float(np.clip(u @ up, -1.0, 1.0)))))
            break

        t, y = t + dt, y_next
        ts.append(t)
        ys.append(y)
        if t > 600.0:
            raise RuntimeError("ascent did not reach cutoff velocity in 600 s")

    t_arr, y_arr = np.array(ts), np.array(ys)
    x, z, vx, vz, m = y_arr[-1]
    dr, h, v_dr, v_up = to_local(x, z, vx, vz)
    gamma = math.degrees(math.atan2(v_up, v_dr))
    res = AscentResult(
        t=t_arr, y=y_arr, throttle=np.array(thr), thrust=np.array(tmag),
        pitch=np.array(pitch), n_eng=n_eng, params=p,
        cutoff={
            "t": float(t_arr[-1]), "alt_m": h, "downrange_m": dr,
            "speed_mps": math.hypot(vx, vz), "v_downrange_mps": v_dr,
            "v_up_mps": v_up, "mass_kg": m, "propellant_left_kg": m - VEH_DRY,
            "flight_path_deg": gamma,
        },
        max_q=max_q, max_g=max_g,
        losses={
            "dv_ideal_mps": dv_ideal, "gravity_mps": loss_grav,
            "drag_mps": loss_drag,
            "propellant_used_kg": p.m0 - m,
            "propellant_loaded_kg": prop,
        },
    )
    return res


def solve_ascent(target_alt: float, target_speed: float, target_downrange: float,
                 target_cutoff_mass: float, cfg: AscentConfig | None = None,
                 guess: AscentParams | None = None,
                 target_gamma_deg: float | None = None,
                 verbose: bool = False) -> AscentResult:
    """Find the ascent that arrives at the required cutoff state.

    Four unknowns -- propellant load, when to pitch, how hard, and the
    above-atmosphere steering bias -- against three conditions evaluated at the
    instant the speed reaches ``target_speed``: altitude, downrange, and the mass
    left in the tanks. The mass condition is what makes the return budget close;
    it is supplied by the caller, which sizes it from the return burns.

    The propellant actually spent is an *output* of this solve, not an input,
    which is the point the task makes about a solo stage: it is not CRS-9's
    ascent and should not be reported as one.
    """
    from scipy.optimize import least_squares

    cfg = cfg or AscentConfig()
    g = guess or AscentParams(m0=215_000.0, t_pitch=12.0, kick_deg=8.0,
                              bias_deg=0.0, a_cap_g=2.0)
    scale = np.array([1e5, 10.0, 10.0, 10.0, 1.0])

    # solve on a coarse step, then fly the answer properly
    coarse = AscentConfig(**{**cfg.__dict__, "dt": max(cfg.dt, 0.1)})

    def unpack(u: np.ndarray) -> AscentParams:
        return AscentParams(m0=u[0] * scale[0], t_pitch=u[1] * scale[1],
                            kick_deg=u[2] * scale[2], bias_deg=u[3] * scale[3],
                            a_cap_g=u[4] * scale[4])

    def residual(u: np.ndarray) -> np.ndarray:
        p = unpack(u)
        try:
            r = fly_ascent(p, coarse, target_speed)
        except RuntimeError:
            return np.array([50.0, 50.0, 50.0, 0.0])
        c = r.cutoff
        # The fourth residual is the one free direction the cutoff spec leaves
        # open. Altitude, speed and downrange do NOT pin the flight path angle,
        # and the flight path angle is what sets apogee -- see the gamma sweep in
        # `rtls_flight.py --sweep-gamma`. With no gamma asked for, the solve
        # simply prefers to steer as little as possible.
        if target_gamma_deg is None:
            fourth = 0.02 * p.bias_deg
        else:
            fourth = (c["flight_path_deg"] - target_gamma_deg) / 5.0
        return np.array([
            (c["alt_m"] - target_alt) / 1000.0,
            (c["downrange_m"] - target_downrange) / 1000.0,
            (c["mass_kg"] - target_cutoff_mass) / 1000.0,
            fourth,
        ])

    u0 = np.array([g.m0, g.t_pitch, g.kick_deg, g.bias_deg, g.a_cap_g]) / scale
    lo = np.array([60_000.0, 2.0, 0.5, -cfg.aoa_max_deg, cfg.a_cap_lo]) / scale
    hi = np.array([VEH_DRY + PROP_MAX, 60.0, 30.0, cfg.aoa_max_deg,
                   cfg.a_cap_hi]) / scale
    u0 = np.clip(u0, lo, hi)
    sol = least_squares(residual, u0, bounds=(lo, hi), xtol=1e-12, ftol=1e-12,
                        diff_step=3e-3, verbose=2 if verbose else 0)
    return fly_ascent(unpack(sol.x), cfg, target_speed)


# ---------------------------------------------------------------------------
# Boostback and coast
# ---------------------------------------------------------------------------

BOOSTBACK_ENGINES = 3

# Where our dynamics stop and the donor's begin. The task says "~70 km" and the
# vacuum argument is made at 70; the handover actually happens at 65, and the
# reason is the donor rather than the physics.
#
# The donor's entry burn is armed and cut on *predicted peak dynamic pressure*.
# Handed a stage at 70 km, where q is 62 Pa, the prediction falls below the cut
# threshold while about 20 t is still aboard, and the burn stops early. That
# leaves a 55 t stage for a landing burn flown on one Merlin, which nets
# 5.4 m/s^2 against gravity and therefore needs 13 km to stop -- which is
# exactly where the landing burn then ignites, and it runs the tank dry on the
# way down. Handed the same stage at 65 km the entry burn runs to its 7 t fuel
# floor instead, and the landing burn ignites at 2.7 km on a 33 t stage, which
# is the regime the donor was built and tested in.
#
# 65 km costs nothing in validity: q there is about 130 Pa, so drag on the
# returning stage is 0.03 m/s^2 -- a few hundredths of a percent of the entry
# burn's thrust acceleration, and two orders below where the vacuum
# approximation starts to matter. The cliff is between 68 and 66 km; 65 sits
# clear of it, and every altitude from 66 km down lands at 2.6-4.3 m/s.
ENTRY_INTERFACE_ALT = 65_000.0


SETTLE_S = 8.0                   # s of RCS-only coast before the engines relight
RCS_NOZZLES = 2
RCS_FORCE = 500.0                # N per nozzle, the donor's [estimate]
RCS_Z = 40.5                     # m above the base
GIMBAL_MAX_DEG = 5.0


@dataclass
class BoostbackResult:
    t_settle: float
    t_slew: float
    t_burn: float
    aim_deg: float
    coast_t: np.ndarray
    coast_y: np.ndarray               # the settling coast before relight
    burn_t: np.ndarray
    burn_y: np.ndarray
    ball_t: np.ndarray
    ball_y: np.ndarray                # boostback cutoff -> entry interface
    apogee_m: float = 0.0
    max_downrange_m: float = 0.0
    handover: dict = field(default_factory=dict)
    impact_downrange_m: float = 0.0
    propellant_kg: float = 0.0
    dv_mps: float = 0.0
    turnaround: dict = field(default_factory=dict)


def pitch_authority(mass: float, thrust: float) -> tuple[float, float]:
    """Pitch angular acceleration available from the RCS alone, and with thrust.

    Above the atmosphere the grid fins are worth nothing, so there are only two
    sources, and they differ by a factor of fifty:

    * **Cold gas**, always available: ``n F (z_rcs - z_com) / J``.
    * **The engine gimbal**, only while the engines are lit:
      ``T sin(delta_max) z_com / J``.

    Same torque model as ``bls_flight.pitch_authority``, minus the fin term.
    """
    inertia, com = transverse_inertia(mass)
    m_rcs = RCS_NOZZLES * RCS_FORCE * abs(RCS_Z - com)
    m_gimbal = thrust * math.sin(math.radians(GIMBAL_MAX_DEG)) * com
    return m_rcs / inertia, (m_rcs + m_gimbal) / inertia


def slew_time(angle_rad: float, alpha: float) -> float:
    """Bang-bang: accelerate half way, decelerate the rest."""
    return 2.0 * math.sqrt(max(angle_rad, 0.0) / max(alpha, 1e-9))


def _turn_and_hold(aim_deg: float, u0: np.ndarray, t_ign: float, alpha: float):
    """Thrust direction that slews from ``u0`` to the boostback attitude and holds.

    **This is the turnaround, and it is flown by the engines, not by the cold gas.**
    Giving the whole 150 deg flip to the RCS is the obvious model and it is
    wrong by a factor of eight: two 500 N nozzles at the 40.5 m station against
    6.2e6 kg m^2 give 0.31 deg/s^2, so a bang-bang flip would take 48 s, during
    which the stage travels another 49 km downrange. That alone pushes maximum
    downrange to 97 km, and it is also slower than the real vehicle -- CRS-9's
    MECO-to-boostback gap was 37 s *including stage separation*.

    What actually happens is that the stage settles its propellant, relights, and
    lets the gimbal finish the turn: three Merlins at 5 deg through a 6.9 m arm
    give 15 deg/s^2, fifty times the cold gas, so the same 150 deg takes about
    6 s. The burn and the turn are one manoeuvre, which is why the direction
    slews *during* the burn rather than before it -- the thrust genuinely does
    point the wrong way for the first few seconds, and the solver integrates
    that rather than assuming it away.
    """
    a = math.radians(aim_deg)
    u_tgt = np.array([math.cos(a), math.sin(a)])
    # signed angle from u0 to u_tgt, taken the short way round
    ang = math.atan2(u0[0] * u_tgt[1] - u0[1] * u_tgt[0], u0 @ u_tgt)
    total = abs(ang)
    sign = 1.0 if ang >= 0.0 else -1.0
    t_s = slew_time(total, alpha)
    theta0 = math.atan2(u0[1], u0[0])

    def direction(t: float, _y: np.ndarray) -> np.ndarray:
        dt = t - t_ign
        if dt >= t_s:
            return u_tgt
        half = 0.5 * t_s
        if dt <= half:
            done = 0.5 * alpha * dt * dt
        else:
            done = total - 0.5 * alpha * (t_s - dt) ** 2
        th = theta0 + sign * min(done, total)
        return np.array([math.cos(th), math.sin(th)])

    return direction, t_s


def transverse_inertia(mass: float) -> tuple[float, float]:
    """Pitch inertia about the CoM and the CoM height -- the donor's mass model.

    Thin-walled shell for the dry stage plus the propellant as a solid column,
    both carried to the combined centre of mass by the parallel-axis theorem.
    Kept identical to ``bls_flight.transverse_inertia`` so the attitude story
    does not change across the handover.
    """
    prop = max(mass - VEH_DRY, 0.0)
    tank_area = 9.9
    rho_prop = 1048.0
    dry_comz = 12.4
    veh_len = 47.7
    radius = 0.5 * VEH_DIA
    r2_col = tank_area / math.pi
    h_col = prop / (rho_prop * tank_area)
    z_col = 1.6 + 0.5 * h_col
    com = (VEH_DRY * dry_comz + prop * z_col) / max(mass, 1.0)
    i_dry = VEH_DRY * (6.0 * radius ** 2 + veh_len ** 2) / 12.0
    i_col = prop * (3.0 * r2_col + h_col ** 2) / 12.0
    inertia = (i_dry + VEH_DRY * (dry_comz - com) ** 2
               + i_col + prop * (z_col - com) ** 2)
    return inertia, com


def _ballistic_to_ground(y0: np.ndarray, t0: float, dt: float = 0.25) -> float:
    """Downrange where an unpowered arc from ``y0`` would reach the ground.

    Vacuum-continued below the entry interface on purpose: this is the *aim
    point*, the quantity the boostback is targeting, not a prediction of where
    the stage lands. Everything below the interface belongs to the donor, whose
    atmosphere shortens the arc; that difference is absorbed by biasing this aim
    point against the donor's own touchdown, in ``rtls_flight.py``.
    """
    cmd = Thrusting(0, 0.0, None, retro_drag=True)

    def stop(_t, y):
        return altitude(y[0], y[1]) <= 0.0

    # guard: an arc that never comes down would spin forever
    def stop_guarded(t, y):
        return stop(t, y) or t - t0 > 3000.0

    _, ys = integrate(y0, cmd, dt, t0, stop_guarded)
    return downrange(ys[-1][0], ys[-1][1])


def fly_boostback(cutoff: np.ndarray, t_cutoff: float, aim_deg: float,
                  t_burn: float, t_settle: float = SETTLE_S,
                  dt: float = 0.1,
                  interface_alt: float = ENTRY_INTERFACE_ALT) -> BoostbackResult:
    """Settle, relight, turn and burn, then fall to the entry interface."""
    # 1. propellant-settling coast: engines off, RCS starts the turn but at
    #    0.31 deg/s^2 it gets barely 5 deg in 8 s, so this is effectively a coast
    coast_cmd = Thrusting(0, 0.0, None, retro_drag=False)
    ct, cy = integrate(cutoff, coast_cmd, dt, t_cutoff,
                       lambda t, _y: t >= t_cutoff + t_settle)

    # 2. relight: three engines at full throttle, the gimbal turning the stage
    #    onto the boostback attitude while the burn is already running
    y1 = cy[-1]
    t1 = ct[-1]
    speed1 = math.hypot(y1[2], y1[3])
    u0 = (np.array([y1[2], y1[3]]) / speed1 if speed1 > 1.0
          else np.array(local_frame(y1[0], y1[1])[0]))
    thrust1 = BOOSTBACK_ENGINES * (ENG_T_VAC - ENG_AE * atmosphere(
        altitude(y1[0], y1[1]))[1])
    alpha_rcs, alpha_lit = pitch_authority(y1[4], thrust1)
    u_aim = np.array([math.cos(math.radians(aim_deg)), math.sin(math.radians(aim_deg))])
    turn_rad = math.acos(float(np.clip(u0 @ u_aim, -1.0, 1.0)))
    direction, t_slew = _turn_and_hold(aim_deg, u0, t1, alpha_lit)
    burn_cmd = Thrusting(BOOSTBACK_ENGINES, 1.0, direction, retro_drag=True)
    bt, by = integrate(y1, burn_cmd, dt, t1, lambda t, _y: t >= t1 + t_burn)

    # 3. ballistic, up through apogee and back down to the entry interface
    y2 = by[-1]
    ball_cmd = Thrusting(0, 0.0, None, retro_drag=True)
    t2 = bt[-1]

    def at_interface(t, y):
        return altitude(y[0], y[1]) <= interface_alt and t > t2 + 5.0

    def guarded(t, y):
        return at_interface(t, y) or t - t2 > 3000.0

    st, sy = integrate(y2, ball_cmd, dt, t2, guarded)

    alts = np.array([altitude(a, b) for a, b in zip(sy[:, 0], sy[:, 1])])
    drs = np.array([downrange(a, b) for a, b in zip(sy[:, 0], sy[:, 1])])
    x, z, vx, vz, m = sy[-1]
    dr, h, v_dr, v_up = to_local(x, z, vx, vz)

    res = BoostbackResult(
        t_settle=t_settle, t_slew=t_slew, t_burn=t_burn, aim_deg=aim_deg,
        coast_t=ct, coast_y=cy, burn_t=bt, burn_y=by, ball_t=st, ball_y=sy,
        turnaround={
            "settle_s": t_settle,
            "slew_s": t_slew,
            "total_s": t_settle + t_slew,
            "slew_deg": math.degrees(turn_rad),
            "rcs_alpha_deg_s2": math.degrees(alpha_rcs),
            "lit_alpha_deg_s2": math.degrees(alpha_lit),
            # what the same turn would cost on cold gas alone -- the model this
            # replaces, kept so the difference stays visible
            "rcs_only_flip_s": slew_time(turn_rad, alpha_rcs),
        },
        apogee_m=float(alts.max()),
        max_downrange_m=float(max(drs.max(), np.array(
            [downrange(a, b) for a, b in zip(by[:, 0], by[:, 1])]).max())),
        handover={
            "t": float(st[-1]), "alt_m": h, "downrange_m": dr,
            "v_downrange_mps": v_dr, "v_up_mps": v_up,
            "speed_mps": math.hypot(vx, vz), "mass_kg": m,
            "propellant_left_kg": m - VEH_DRY,
        },
        impact_downrange_m=_ballistic_to_ground(sy[-1], float(st[-1])),
        propellant_kg=float(by[0][4] - by[-1][4]),
    )
    isp_v = ENG_ISP_VAC
    res.dv_mps = isp_v * G0 * math.log(by[0][4] / by[-1][4])
    return res


def solve_boostback(cutoff: np.ndarray, t_cutoff: float, aim_downrange: float,
                    dt: float = 0.1, verbose: bool = False,
                    interface_alt: float = ENTRY_INTERFACE_ALT) -> BoostbackResult:
    """The boostback as a boundary-value problem, solved for minimum propellant.

    This is a *targeting* problem, not a landing problem, which is why pdg's SCvx
    formulation does not apply: its terminal condition is "at the origin with
    near-zero velocity", and the stage at boostback cutoff is 100 km up and
    travelling at a kilometre a second. What is actually being solved is the
    return leg of the notebook's phase 2, posed on the cutoff state:

        given the state at engine cutoff, find a burn (attitude, duration) whose
        ballistic continuation reaches the launch site.

    One equality constraint -- the ballistic impact point -- against two
    unknowns, so there is a one-parameter family of solutions and the spare
    freedom buys the cheapest of them. For each attitude the burn duration that
    hits the aim point is found by bisection; the attitude is then optimised for
    minimum burn time, which for a fixed throttle and engine count is minimum
    propellant. Apogee is left alone by this, so it stays an independent check
    against CRS-9's 128.70 km.
    """
    from scipy.optimize import brentq, minimize_scalar

    cache: dict[tuple[float, float], BoostbackResult] = {}

    def fly(aim_deg: float, t_burn: float) -> BoostbackResult:
        key = (round(aim_deg, 6), round(t_burn, 6))
        if key not in cache:
            cache[key] = fly_boostback(cutoff, t_cutoff, aim_deg, t_burn, dt=dt,
                                       interface_alt=interface_alt)
        return cache[key]

    def miss(aim_deg: float, t_burn: float) -> float:
        return fly(aim_deg, t_burn).impact_downrange_m - aim_downrange

    def burn_for(aim_deg: float) -> float | None:
        """The burn at this attitude whose arc lands on the aim point.

        Monotone in one direction, which is what makes this cheap: at a fixed
        attitude a longer burn always leaves the stage further up-range, so the
        miss decreases through zero exactly once and the two ends of the bracket
        settle it. A linear walk over the same interval costs forty integrations
        instead of a dozen, and this is called a few hundred times per solve.

        **The upper end of the bracket is the propellant, not a round number.**
        A fixed 90 s would have three engines burn 82 t out of a stage that is
        carrying 61, so the search spends most of its time exploring negative
        propellant mass -- where the dynamics are not merely wrong but violently
        so, since ``thrust/m`` diverges through zero. The bracket therefore ends
        just short of a dry tank.
        """
        lo = 1.0
        hi = 0.98 * (cutoff[4] - VEH_DRY) / (BOOSTBACK_ENGINES * ENG_MDOT)
        if hi <= lo:
            return None
        try:
            f_lo, f_hi = miss(aim_deg, lo), miss(aim_deg, hi)
        except (RuntimeError, ValueError):
            return None
        if f_lo * f_hi > 0.0:
            return None                  # this attitude cannot reach the target
        try:
            return brentq(lambda tb: miss(aim_deg, tb), lo, hi, xtol=1e-3)
        except (ValueError, RuntimeError):
            return None

    def cost(aim_deg: float) -> float:
        tb = burn_for(aim_deg)
        if tb is None:
            return 1e6
        return tb

    # The attitude window starts at 180 deg -- straight back up-range -- and
    # runs to "back and steeply down". **Anything below 180 is excluded, and
    # that exclusion is doing real work.**
    #
    # Below 180 the thrust has an upward component, and the solver loves it: the
    # cheapest lofted solution costs 33.32 t against 33.29 t for the cheapest
    # retrograde one, a difference of thirty kilograms. For that thirty
    # kilograms it throws apogee from 132 km to 204 km and hands the stage to
    # the donor at 1585 m/s of descent rate instead of 1090. The minimum-fuel
    # objective cannot see that, because the cost it is minimising stops at
    # boostback cutoff and the bill arrives later, in the entry burn.
    #
    # The constraint is not a fudge to make apogee come out right. A boostback
    # burn is a *retrograde* manoeuvre -- it exists to reverse the downrange
    # velocity, and pitching above the horizon to add energy upward is not
    # boosting back, it is lofting. Excluding it costs 0.1 % of the propellant
    # and buys an entry state the donor can actually fly.
    grid = np.arange(180.0, 260.0, 10.0)
    scores = [(cost(a), a) for a in grid]
    scores.sort()
    if scores[0][0] >= 1e6:
        raise RuntimeError(
            "no retrograde fixed-attitude boostback reaches the aim point -- the "
            "stage is too heavy or too fast at cutoff for three engines to "
            "reverse it with the propellant aboard")
    best = scores[0][1]
    sol = minimize_scalar(cost, bracket=None,
                          bounds=(max(180.0, best - 11.0), min(259.0, best + 11.0)),
                          method="bounded", options={"xatol": 0.05})
    aim_deg = float(sol.x)
    t_burn = burn_for(aim_deg)
    if t_burn is None:
        aim_deg, t_burn = best, burn_for(best)
    if verbose:
        print(f"  boostback: attitude {aim_deg:.2f} deg, burn {t_burn:.2f} s")
    return fly_boostback(cutoff, t_cutoff, aim_deg, t_burn, dt=dt,
                         interface_alt=interface_alt)


# ---------------------------------------------------------------------------
# Self-check
# ---------------------------------------------------------------------------

def self_test(verbose: bool = True) -> bool:
    """Check the atmosphere against US76 and the dynamic pressures the task quotes."""
    ok = True
    # US76 table I, density [kg/m^3], indexed by GEOMETRIC altitude -- the layer
    # stack's own base heights are geopotential, so the two differ by 1.4 % at
    # 86 km and quoting the wrong one looks like a 17 % model error.
    table = [(0, 1.2250), (5_000, 0.73643), (10_000, 0.41351), (20_000, 0.088910),
             (30_000, 0.018410), (40_000, 0.0039957), (50_000, 0.0010269),
             (60_000, 3.0968e-4), (70_000, 8.2829e-5), (86_000, 6.958e-6),
             (100_000, 5.604e-7), (120_000, 2.222e-8)]
    for h, rho_ref in table:
        rho, _, _ = atmosphere(float(h))
        err = abs(rho - rho_ref) / rho_ref
        flag = "ok " if err < 0.02 else "BAD"
        if err >= 0.02:
            ok = False
        if verbose:
            print(f"  {flag} rho({h/1000:6.2f} km) = {rho:.6g} vs US76 {rho_ref:.6g} "
                  f"({100*err:.2f} %)")

    # the task's own table: dynamic pressure at 1579 m/s, and drag on an 80 t stage
    if verbose:
        print("  dynamic pressure at 1579 m/s (task's Architecture table):")
    for h, q_ref in ((60_000, 386.0), (80_000, 23.0), (100_000, 0.7)):
        rho, _, _ = atmosphere(float(h))
        q = 0.5 * rho * 1579.0 ** 2
        a_drag = q * VEH_AREF * cd_retro(5.0) / 80_000.0
        if verbose:
            print(f"    {h/1000:5.0f} km: q = {q:8.2f} Pa (task says {q_ref:6.1f}), "
                  f"drag on 80 t = {a_drag:.5f} m/s^2")
    return ok


if __name__ == "__main__":
    print("rtls_trajectory self-check")
    print(f"  nose fairing: {NOSE_LEN:.1f} m cone, {NOSE_HALF_ANGLE_DEG:.1f} deg "
          f"half-angle, Cd.S peak {max(_CD_A) * VEH_AREF:.2f} m^2 at M 1.05")
    print(f"  engines-first (donor table): Cd.S "
          f"{min(_CD_A_RETRO) * VEH_AREF:.2f}..{max(_CD_A_RETRO) * VEH_AREF:.2f} m^2")
    raise SystemExit(0 if self_test() else 1)
