"""Fly the donor simulator from stage separation and write viewer telemetry.

`pdg` models thrust and gravity only, so it cannot represent a return from
separation altitude: at 62 km and 1.5 km/s the atmosphere does most of the
braking, and an engine-only solution would need roughly 3 km/s of dV against
the ~1.2 km/s the stage carries. `booster-lander-simulator` has the physics that
is missing -- US76 atmosphere, body and grid-fin aerodynamics, leg contact --
and ships an `entry` scenario that starts at separation and flies the whole
profile: entry burn, unpowered aero descent, landing burn.

This runs that core and converts its trajectory log into the same three files
`pdg_flight` writes, so the existing 2D viewers replay it unchanged.

    python bridge/bls_flight.py --scenario entry --out out_entry
    python viz/land2d_mpl.py --data out_entry --speed 4 --follow 400

Three things are reconstructed rather than read, because the core's text log does
not carry them:

* **Thrust magnitude** comes from the mass flow, ``T = -dm/dt * g0 * Isp``, with
  Isp blended between the sea-level and vacuum figures by ambient pressure. That
  is better than scaling the logged throttle fraction, which does not say how
  many engines are lit -- three for the entry burn, one to land.
* **The sign of the range**, because ``lat`` is a distance and cannot go
  negative -- see ``unfold_radius``.
* **The lean direction.** The log carries the tilt magnitude only, never which
  side it leans. Thrust acts along the body axis, so the sign of the lateral
  acceleration gives the sign of the lean; with the engine off there is no such
  evidence and the last decision is held.

That last one used to produce the worst artefact in the whole pipeline. An
inferred sign flips between one sample and the next, which drew the stage
snapping through 35 degrees in 50 ms -- 347 deg/s, twelve times anything the
vehicle ever did. A stage changes attitude with torque and inertia like anything
else, so the command is now flown through the agility it actually has:
``pitch_authority`` sums the engine gimbal, the grid fins and the RCS against
the pitch inertia, and ``slew_limit`` tracks the command time-optimally within
that, additionally capped by the logged body rate. The peak now lands just under
what the core itself recorded -- 20.2 deg/s against 28.6 -- instead of far past
it.
"""

from __future__ import annotations

import argparse
import json
import math
import re
import subprocess
import sys
from pathlib import Path

import numpy as np

# Merlin-1D class figures from the donor's core/constants.h
ENG_T_SL = 845_000.0
ENG_ISP_SL = 282.0
ENG_ISP_VAC = 311.0
ENG_MDOT_100 = ENG_T_SL / (9.80665 * ENG_ISP_SL)
ENG_T_VAC = ENG_MDOT_100 * 9.80665 * ENG_ISP_VAC   # 932 kN, as core/constants.h derives it
ENG_THR_MIN = 0.40
ENTRY_ENGINES = 3
G0 = 9.80665
VEH_LEN = 47.7
VEH_DIA = 3.66
VEH_RADIUS = 0.5 * VEH_DIA
VEH_DRY = 25600.0          # kg, incl. legs and fins
VEH_DRY_COMZ = 12.4        # m above the base
TANK_AREA = 9.9            # m^2 column cross-section
PROP_BASE_Z = 1.6          # m, RP-1 column base
PROP_RHO = 1048.0          # kg/m^3, LOX+RP-1 mixture at the nominal O/F 2.56
ENG_GIMBAL_MAX_DEG = 5.0
RCS_FORCE = 500.0          # N per nozzle
RCS_Z = 40.5               # m above the base
RCS_NOZZLES = 2            # firing as a couple
FIN_Z = 45.0               # m above the base
FIN_AREA = 2.4             # m^2 per fin
FIN_CNA = 3.0              # /rad
FIN_DEFL_MAX_DEG = 20.0
FIN_PITCH_PAIR = 2         # of the four fins, one pair works in pitch
LEG_DEPLOY_H = 250.0

LINE = re.compile(
    r"t=\s*(?P<t>[-\d.]+)\s+h=\s*(?P<h>[-\d.]+)\s+vz=\s*(?P<vz>[-\d.]+)\s+"
    r"thr=\s*(?P<thr>[-\d.]+)\s+tilt=\s*(?P<tilt>[-\d.]+)\s+lat=\s*(?P<lat>[-\d.]+)\s+"
    r"vrad=\s*(?P<vrad>[-\d.]+)\s+qbar=\s*(?P<qbar>[-\d.]+)\s+wperp=\s*(?P<wperp>[-\d.]+)\s+"
    r"wind=\s*(?P<wind>[-\d.]+)\s+m=\s*(?P<m>[-\d.]+)\s+ph=\s*(?P<ph>\d+)"
)
RESULT = re.compile(
    r"RESULT:\s+(?P<verdict>\w+)\s+fault=(?P<fault>\S+)\s+td_v=(?P<td_v>[-\d.]+)\s+m/s\s+"
    r"lat=(?P<lat>[-\d.]+)\s+m\s+tilt=(?P<tilt>[-\d.]+)\s+deg\s+fuel=(?P<fuel>[-\d.]+)\s+kg\s+"
    r"t=(?P<t>[-\d.]+)\s+s\s+maxq=(?P<maxq>[-\d.]+)"
)
HEADER = re.compile(r"scenario=(?P<scen>\S+)\s+seed=(?P<seed>\d+)\s+run=(?P<run>\d+)\s+"
                    r"h0=(?P<h0>[-\d.]+)\s+m\s+vz0=(?P<vz0>[-\d.]+)")

# core/state.h: PH_INIT, PH_COAST, PH_ENTRY_BURN, PH_AERO, PH_LANDING_BURN,
# PH_TOUCHDOWN, PH_SETTLING, PH_LANDED, PH_TIPPED, PH_CRASHED, ...
PHASE_NAMES = {
    0: "init", 1: "coast", 2: "entry burn", 3: "aero descent",
    4: "landing burn", 5: "touchdown", 6: "settling", 7: "landed",
    8: "tipped", 9: "crashed", 10: "fuel depleted", 11: "structural failure",
    12: "thermal failure", 13: "loss of control",
}
PH_TOUCHDOWN = 5


def run_headless_record(exe: Path, scenario: str, seed: int, run: int,
                        extra: list[str]) -> dict:
    """The authoritative touchdown record, from the core's own summary CSV.

    The --verbose RESULT line is thinner and, for tilt, reports the *settled*
    attitude rather than the one at impact -- which is what the verdict is
    actually graded on. The headless CSV carries both, plus leg crush. The sim
    is deterministic per (seed, run), so this is the same flight.
    """
    import csv
    import tempfile

    with tempfile.TemporaryDirectory() as tmp:
        csv_path = Path(tmp) / "run.csv"
        cmd = [str(exe), "--headless", "--scenario", scenario, "--seed", str(seed),
               "--runs", str(run), "--out", str(csv_path), *extra]
        proc = subprocess.run(cmd, capture_output=True, text=True)
        if not csv_path.exists():
            sys.stderr.write(proc.stdout + proc.stderr)
            raise SystemExit(f"booster-core --headless failed (exit {proc.returncode})")
        rows = list(csv.DictReader(csv_path.read_text().splitlines()))
    for r in rows:
        if int(r["run"]) == run:
            return r
    return rows[-1] if rows else {}


def run_core(exe: Path, scenario: str, seed: int, run: int, extra: list[str]) -> str:
    cmd = [str(exe), "--run", "--scenario", scenario, "--seed", str(seed),
           "--run", str(run), "--verbose", *extra]
    print("running:", " ".join(cmd))
    proc = subprocess.run(cmd, capture_output=True, text=True)
    if proc.returncode != 0 and not LINE.search(proc.stdout):
        sys.stderr.write(proc.stdout + proc.stderr)
        raise SystemExit(f"booster-core failed (exit {proc.returncode})")
    return proc.stdout


def parse(text: str) -> tuple[dict, np.ndarray, dict]:
    rows, head, result = [], {}, {}
    for line in text.splitlines():
        m = LINE.search(line)
        if m:
            rows.append({k: float(v) for k, v in m.groupdict().items()})
            continue
        m = HEADER.search(line)
        if m:
            head = m.groupdict()
            continue
        m = RESULT.search(line)
        if m:
            result = m.groupdict()
    if not rows:
        raise SystemExit("no trajectory lines found -- was --verbose passed?")
    keys = ["t", "h", "vz", "thr", "tilt", "lat", "vrad", "qbar", "wperp", "wind",
            "m", "ph"]
    arr = np.array([[r[k] for k in keys] for r in rows])
    return head, arr, result


def pressure_ratio(h: np.ndarray) -> np.ndarray:
    """Crude isothermal pressure ratio, only used to blend Isp between SL and vacuum."""
    return np.exp(-np.clip(h, 0.0, None) / 8500.0)


def resample(cols: dict[str, np.ndarray], hz: float) -> dict[str, np.ndarray]:
    """Interpolate the log onto a uniform grid, for smooth playback only.

    The core prints one line every 250 steps -- 0.5 s, hardcoded -- which is
    2 Hz of real data and a visibly choppy replay. Interpolating between those
    points adds no information and the HUD says so via ``log.source_rate_hz``;
    it only stops the animation stepping. Phase is a label, so it is held rather
    than blended.
    """
    t = cols["t"]
    grid = np.arange(t[0], t[-1] + 1e-9, 1.0 / hz)
    out = {}
    for k, v in cols.items():
        if k == "ph":
            idx = np.clip(np.searchsorted(t, grid, side="right") - 1, 0, len(t) - 1)
            out[k] = v[idx]
        else:
            out[k] = np.interp(grid, t, v)
    out["t"] = grid
    return out


def transverse_inertia(mass: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """Pitch inertia about the CoM, and the CoM height, from the logged mass.

    The same model the core's ``mass_props`` uses (core/dynamics.c): a thin-walled
    shell for the dry stage, ``m (6R^2 + L^2) / 12`` about its own centre, plus
    the propellant as a solid column, ``m (3r^2 + h^2) / 12``, both carried to the
    combined centre of mass by the parallel-axis theorem. The log does not split
    LOX from RP-1, so the column uses the mixture density at the nominal O/F.
    """
    prop = np.maximum(mass - VEH_DRY, 0.0)
    r2_col = TANK_AREA / math.pi
    h_col = prop / (PROP_RHO * TANK_AREA)
    z_col = PROP_BASE_Z + 0.5 * h_col

    com = (VEH_DRY * VEH_DRY_COMZ + prop * z_col) / np.maximum(mass, 1.0)

    i_dry = VEH_DRY * (6.0 * VEH_RADIUS ** 2 + VEH_LEN ** 2) / 12.0
    i_col = prop * (3.0 * r2_col + h_col ** 2) / 12.0
    inertia = (i_dry + VEH_DRY * (VEH_DRY_COMZ - com) ** 2
               + i_col + prop * (z_col - com) ** 2)
    return inertia, com


def pitch_authority(mass: np.ndarray, thrust: np.ndarray,
                    qbar: np.ndarray) -> tuple[np.ndarray, dict]:
    """Peak pitch angular acceleration the stage can actually command [rad/s^2].

    Three torque sources, each dominant in a different phase:

    * **Gimbal.** The engine swivels up to ``ENG_GIMBAL_MAX`` off the body axis,
      so the side force is ``T sin(delta)`` acting at the CoM-to-gimbal arm:
      ``M = T sin(delta_max) z_com``. Scales with thrust, so it is worth nothing
      while the engine is off.
    * **Grid fins.** A pitch pair deflects up to ``FIN_DEFL_MAX`` into the flow:
      ``M = n q S C_Na delta (z_fin - z_com)``. This is what actually flies the
      unpowered aero descent, and it scales with dynamic pressure -- nothing up
      high, everything through max q.
    * **RCS.** Cold-gas nozzles at ``RCS_Z`` firing as a couple:
      ``M = n F (z_rcs - z_com)``. Small and constant; all there is in vacuum.

    ``alpha_max = sum(M) / I_pitch``. A slew of theta radians flown bang-bang --
    accelerate half way, decelerate the rest -- then takes
    ``t = 2 sqrt(theta / alpha_max)`` and peaks at ``omega = sqrt(theta alpha_max)``.
    """
    inertia, com = transverse_inertia(mass)
    m_gimbal = thrust * math.sin(math.radians(ENG_GIMBAL_MAX_DEG)) * com
    m_fin = (FIN_PITCH_PAIR * np.maximum(qbar, 0.0) * FIN_AREA * FIN_CNA
             * math.radians(FIN_DEFL_MAX_DEG) * np.abs(FIN_Z - com))
    m_rcs = RCS_NOZZLES * RCS_FORCE * np.abs(RCS_Z - com)
    alpha = (m_gimbal + m_fin + m_rcs) / inertia
    info = {
        "inertia_kgm2": (float(inertia.min()), float(inertia.max())),
        "com_m": (float(com.min()), float(com.max())),
        "alpha_deg_s2": (float(np.degrees(alpha.min())), float(np.degrees(alpha.max()))),
        "gimbal_peak_deg_s2": float(np.degrees((m_gimbal / inertia).max())),
        "fin_peak_deg_s2": float(np.degrees((m_fin / inertia).max())),
        "rcs_only_deg_s2": float(np.degrees((m_rcs / inertia).max())),
    }
    return alpha, info


def slew_limit(target: np.ndarray, alpha: np.ndarray, t: np.ndarray,
               omega_cap: np.ndarray | None = None) -> np.ndarray:
    """Follow a commanded angle without exceeding the vehicle's own agility.

    Time-optimal double integrator: at each step aim for the fastest rate that
    can still be brought to zero exactly on target,
    ``omega_des = sign(e) sqrt(2 alpha |e|)``, and change the rate by at most
    ``alpha dt``. A step command therefore becomes a real pitch-over of
    ``2 sqrt(theta / alpha)`` seconds instead of an instant flip.

    ``omega_cap`` is the logged body rate magnitude. The in-plane component of a
    rate can never exceed the magnitude of that rate, so this is not a modelling
    choice but an exact bound, and it keeps the reconstruction honest even where
    the torque estimate is generous.
    """
    theta = np.empty_like(target)
    theta[0] = target[0]
    omega = 0.0
    for i in range(1, len(target)):
        dt = t[i] - t[i - 1]
        err = target[i] - theta[i - 1]
        a = max(float(alpha[i]), 1e-6)
        omega_des = math.copysign(math.sqrt(2.0 * a * abs(err)), err)
        if omega_cap is not None:
            w = float(omega_cap[i])
            omega_des = float(np.clip(omega_des, -w, w))
        omega += float(np.clip(omega_des - omega, -a * dt, a * dt))
        if omega_cap is not None:
            omega = float(np.clip(omega, -omega_cap[i], omega_cap[i]))
        theta[i] = theta[i - 1] + omega * dt
    return theta


def unfold_radius(lat: np.ndarray, vrad: np.ndarray, dt: float) -> np.ndarray:
    """Sign a radial distance so a pass over the pad reads as a crossing.

    The core logs ``lat = sqrt(rx^2 + ry^2)`` -- a distance, never negative --
    and ``vrad``, its rate. Plotted raw, a stage that flies over the pad and out
    the other side looks like it bounces off an invisible wall at range zero,
    which is what turns the last seconds of the descent into a zigzag.

    A pass through the origin leaves a cusp: the radial speed reverses between
    two samples with both magnitudes large, at a range smaller than the distance
    covered in one sample. A genuine radial turnaround is smooth -- vrad decays
    through zero -- and fails both tests. Returns +-1 per sample.

    This assumes the horizontal motion stays in one plane, which is what a side
    view assumes anyway; the log carries no azimuth to check it against.
    """
    sign = np.ones(len(lat))
    s = 1.0
    for i in range(1, len(lat)):
        reversed_ = vrad[i - 1] * vrad[i] < 0.0
        fast = min(abs(vrad[i - 1]), abs(vrad[i])) > 0.5
        near = min(lat[i - 1], lat[i]) < max(abs(vrad[i - 1]), abs(vrad[i])) * dt
        if reversed_ and fast and near:
            s = -s
        sign[i] = s
    return sign


def convert(arr: np.ndarray, trim_settled: bool, hz: float) -> dict[str, np.ndarray]:
    cols = {name: arr[:, i] for i, name in enumerate(
        ["t", "h", "vz", "thr", "tilt", "lat", "vrad", "qbar", "wperp", "wind",
         "m", "ph"])}

    if trim_settled:
        # from PH_TOUCHDOWN on the stage is on the deck and the legs are
        # absorbing; keep the first of those samples and drop the rest
        settled = np.flatnonzero(cols["ph"] >= PH_TOUCHDOWN)
        if settled.size > 1:
            keep = slice(0, settled[0] + 1)
            cols = {k: v[keep] for k, v in cols.items()}

    source_hz = 1.0 / float(np.median(np.diff(cols["t"])))

    # Sign the range before resampling: unfolded it passes smoothly through
    # zero, so linear interpolation is meaningful. Folded it has a corner there.
    crossings = unfold_radius(cols["lat"], cols["vrad"], 1.0 / source_hz)
    cols["lat"] = cols["lat"] * crossings
    cols["vrad"] = cols["vrad"] * crossings

    if hz > 0.0 and hz > source_hz:
        cols = resample(cols, hz)

    t, m, vrad = cols["t"], cols["m"], cols["vrad"]

    # thrust from the mass flow: accounts for how many engines are lit
    mdot = -np.gradient(m, t)
    mdot = np.clip(mdot, 0.0, None)
    isp = ENG_ISP_VAC + (ENG_ISP_SL - ENG_ISP_VAC) * pressure_ratio(cols["h"])
    thrust = mdot * G0 * isp
    thrust[cols["thr"] <= 1e-3] = 0.0

    # ---- lean angle ---------------------------------------------------
    # The log carries the tilt magnitude only, never which way it leans, so the
    # direction has to be inferred. Thrust acts along the body axis, so a stage
    # leaning towards +x accelerates towards +x: the sign of the lateral
    # acceleration is the sign of the lean. While the engine is off there is no
    # such evidence, so the last decided sign is held.
    a_lat = np.gradient(vrad, t)
    a_lat = np.convolve(a_lat, np.ones(5) / 5.0, mode="same")
    powered = thrust > 0.05 * thrust.max()
    sign = np.where(powered & (np.abs(a_lat) > 0.05), np.sign(a_lat), 0.0)
    last = 1.0
    held = np.empty_like(sign)
    for i, v in enumerate(sign):
        if v != 0.0:
            last = v
        held[i] = last
    theta_cmd = np.radians(cols["tilt"]) * held

    # That command still steps when the inferred direction reverses, and a stage
    # cannot step. Fly it through the attitude control authority it actually
    # has -- engine gimbal plus RCS against the pitch inertia -- so every change
    # takes the time the torque allows.
    alpha, authority = pitch_authority(m, thrust, cols["qbar"])
    theta = slew_limit(theta_cmd, alpha, t, omega_cap=cols["wperp"])
    rate = np.gradient(theta, t)
    authority["peak_rate_deg_s"] = float(np.degrees(np.abs(rate).max()))
    authority["logged_peak_rate_deg_s"] = float(np.degrees(cols["wperp"].max()))
    # the worst commanded step, and how long the available torque takes over it
    step = float(np.abs(np.diff(theta_cmd)).max())
    authority["largest_commanded_step_deg"] = float(np.degrees(step))
    authority["slowest_slew_s"] = float(
        2.0 * math.sqrt(step / max(float(alpha.min()), 1e-6))) if step > 0 else 0.0

    # pitch about +y takes body +z towards +x by theta
    q0 = np.cos(0.5 * theta)
    q2 = np.sin(0.5 * theta)
    zero = np.zeros_like(t)

    return {
        "t": t,
        "m": m,
        "rx": cols["lat"], "ry": zero, "rz": cols["h"],
        "vx": vrad, "vy": zero, "vz": cols["vz"],
        "q0": q0, "q1": zero, "q2": q2, "q3": zero,
        "wx": zero, "wy": np.gradient(theta, t), "wz": zero,
        # the log carries no gimbal angle, so the thrust sits on the body axis
        "Tx": zero, "Ty": zero, "Tz": thrust,
        "Tmag": thrust,
        "tilt_deg": cols["tilt"],
        "speed": np.hypot(vrad, cols["vz"]),
        "_qbar": cols["qbar"], "_phase": cols["ph"], "_wind": cols["wind"],
        "_source_hz": source_hz, "_authority": authority,
        "_theta_cmd": theta_cmd,
    }


COLUMNS = ("t m rx ry rz vx vy vz q0 q1 q2 q3 wx wy wz Tx Ty Tz Tmag tilt_deg "
           "speed").split()


def touchdown_record(data: dict, result: dict, record: dict) -> dict:
    """Merge what the core reports about the landing into the viewer schema."""
    def num(src: dict, key: str, default: float = 0.0) -> float:
        try:
            return float(src.get(key, default))
        except (TypeError, ValueError):
            return default

    verdict = record.get("verdict") or result.get("verdict", "?")
    verdict = VERDICTS.get(verdict, verdict) if verdict.isdigit() else verdict

    # td_v is the speed of the lowest point at FIRST leg contact, captured at the
    # 500 Hz integration rate -- a full 3D speed, not a descent rate, and not an
    # instant the 2 Hz trajectory log contains.
    impact_v = num(record, "td_v") or num(result, "td_v")
    # td_tilt is radians at impact; settled_tilt is where it comes to rest
    tilt_impact = math.degrees(num(record, "td_tilt")) if record else num(result, "tilt")
    tilt_settled = (math.degrees(num(record, "settled_tilt")) if record
                    else num(result, "tilt"))

    return {
        "landed": verdict in ("PERFECT", "GOOD", "HARD"),
        "verdict": verdict,
        # RESULT's t is the whole run including the settling transient; the
        # impact is where the trajectory itself ends
        "time": float(data["t"][-1]),
        "run_duration_s": num(result, "t", float(data["t"][-1])),
        "lateral_error_m": num(record, "td_lat") or num(result, "lat"),
        "descent_rate_mps": impact_v,
        "speed_label": "impact speed",
        "speed_note": "3D speed of the lowest point at first leg contact, "
                      "captured at the 500 Hz integration rate",
        "lateral_speed_mps": 0.0,
        "tilt_deg": tilt_impact,
        "settled_tilt_deg": tilt_settled,
        "max_leg_crush_frac": num(record, "max_crush"),
        "rate_deg_s": 0.0,
        "propellant_used_kg": float(data["m"][0] - data["m"][-1]),
    }


# core/state.h: enum { V_NONE, V_PERFECT, V_GOOD, V_HARD, V_TIPPED, V_CRASHED }
VERDICTS = {"0": "NONE", "1": "PERFECT", "2": "GOOD", "3": "HARD",
            "4": "TIPPED", "5": "CRASHED"}


def write_outputs(out: Path, data: dict, head: dict, result: dict,
                  scenario: str, record: dict) -> None:
    out.mkdir(parents=True, exist_ok=True)
    table = np.stack([data[c] for c in COLUMNS], axis=1)
    np.savetxt(out / "flight.csv", table, delimiter=",",
               header=",".join(COLUMNS), comments="", fmt="%.10g")

    touchdown = touchdown_record(data, result, record)

    phases = data["_phase"]
    spans = []
    for p in np.unique(phases):
        idx = np.flatnonzero(phases == p)
        spans.append({
            "phase": int(p), "name": PHASE_NAMES.get(int(p), f"phase {int(p)}"),
            "t_start": float(data["t"][idx[0]]), "t_end": float(data["t"][idx[-1]]),
            "alt_start": float(data["rz"][idx[0]]), "alt_end": float(data["rz"][idx[-1]]),
        })
    spans.sort(key=lambda s: s["t_start"])

    meta = {
        "source": "booster-lander-simulator",
        "scenario": scenario,
        "seed": int(head.get("seed", 0)),
        "vehicle": {
            "m_wet": float(data["m"][0]),
            "m_dry": float(data["m"][-1] - float(result.get("fuel", 0.0) or 0.0)),
            "isp": ENG_ISP_VAC,
            # the entry burn lights three engines, and does it high enough to
            # get vacuum thrust out of them; one engine lands
            "t_min": ENG_THR_MIN * ENG_T_SL,
            "t_max": ENTRY_ENGINES * ENG_T_VAC,
            "engine_offset_m": 0.30 * VEH_LEN,
            "length_m": VEH_LEN,
            "diameter_m": VEH_DIA,
            "leg_deploy_alt_m": LEG_DEPLOY_H,
        },
        "limits": {
            "tilt_max_deg": 90.0, "gimbal_max_deg": 5.0,
            "glide_slope_deg": 0.0, "omega_max_deg_s": 60.0,
            "aoa_trigger_mps": 0.0, "aoa_max_deg": 90.0,
        },
        "dispersions": {"note": "turbulence and dispersions are internal to the core"},
        "guidance": {
            "tf": float(data["t"][-1]),
            "fuel_used_kg": float(data["m"][0] - data["m"][-1]),
            "scvx_iterations": 0,
            "solver": "in-the-loop (donor core), no precomputed plan",
        },
        "phases": spans,
        "aero": {
            "max_qbar_pa": float(np.max(data["_qbar"])),
            "max_wind_mps": float(np.max(data["_wind"])),
        },
        "attitude": data["_authority"],
        "touchdown": touchdown,
        "log": {
            "rate_hz": 1.0 / float(np.median(np.diff(data["t"]))),
            "samples": int(len(data["t"])),
            "dt": float(np.median(np.diff(data["t"]))),
            # what the core actually printed, before interpolation for playback
            "source_rate_hz": float(data["_source_hz"]),
        },
        "has_plan": False,
    }
    (out / "meta.json").write_text(json.dumps(meta, indent=2))
    # no reference.csv: this core solves guidance in the loop, there is no plan
    stale = out / "reference.csv"
    if stale.exists():
        stale.unlink()
    return meta


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    root = Path(__file__).resolve().parent.parent
    ap.add_argument("--exe", default=str(root / "build-bls" / "bin" / "booster-core.exe"))
    ap.add_argument("--scenario", default="entry",
                    help="terminal | aero_offset | entry | asds_night | chaos")
    ap.add_argument("--seed", type=int, default=1)
    ap.add_argument("--run", type=int, default=1)
    ap.add_argument("--out", default="out_entry")
    ap.add_argument("--log", help="parse this saved --verbose log instead of running")
    ap.add_argument("--keep-settled", action="store_true",
                    help="keep the whole post-touchdown settling tail")
    ap.add_argument("--resample", type=float, default=20.0, metavar="HZ",
                    help="interpolate onto this rate for smooth playback "
                         "(default 20; 0 keeps the core's 2 Hz log cadence)")
    ap.add_argument("core_args", nargs="*",
                    help="extra flags passed straight to booster-core")
    args = ap.parse_args()

    record = {}
    if args.log:
        text = Path(args.log).read_text(errors="replace")
    else:
        exe = Path(args.exe)
        if not exe.exists():
            raise SystemExit(f"{exe} not found -- build it with:\n"
                             f"  cmake -S external/booster-lander-simulator -B build-bls "
                             f"-G Ninja -DBL_CUDA=OFF && cmake --build build-bls")
        text = run_core(exe, args.scenario, args.seed, args.run, args.core_args)
        record = run_headless_record(exe, args.scenario, args.seed, args.run,
                                     args.core_args)

    head, arr, result = parse(text)
    data = convert(arr, trim_settled=not args.keep_settled, hz=args.resample)
    meta = write_outputs(Path(args.out), data, head, result, args.scenario, record)

    lg = meta["log"]
    rate = f"{lg['source_rate_hz']:.0f} Hz logged"
    if lg["rate_hz"] > lg["source_rate_hz"] + 1e-6:
        rate += f", interpolated to {lg['rate_hz']:.0f} Hz for playback"
    print(f"\n{args.scenario}: {lg['samples']} samples over {data['t'][-1]:.1f} s "
          f"({rate}), from {data['rz'][0]/1000:.1f} km at "
          f"{abs(data['vz'][0]):.0f} m/s")
    for s in meta["phases"]:
        print(f"  {s['name']:<14} t {s['t_start']:6.1f} -> {s['t_end']:6.1f} s, "
              f"h {s['alt_start']/1000:6.2f} -> {s['alt_end']/1000:6.2f} km")
    td = meta["touchdown"]
    print(f"  max dynamic pressure {meta['aero']['max_qbar_pa']/1000:.1f} kPa")
    at = meta["attitude"]
    print(f"  pitch inertia {at['inertia_kgm2'][0]/1e6:.1f}..{at['inertia_kgm2'][1]/1e6:.1f} e6 kg m^2, "
          f"CoM {at['com_m'][0]:.1f}..{at['com_m'][1]:.1f} m above the base")
    print(f"  pitch authority: {at['alpha_deg_s2'][0]:.2f}..{at['alpha_deg_s2'][1]:.1f} deg/s^2 "
          f"(gimbal {at['gimbal_peak_deg_s2']:.1f}, fins {at['fin_peak_deg_s2']:.1f}, "
          f"RCS {at['rcs_only_deg_s2']:.2f} at peak)")
    print(f"  slew rate used {at['peak_rate_deg_s']:.1f} deg/s peak against the "
          f"logged {at['logged_peak_rate_deg_s']:.1f} deg/s; largest commanded "
          f"step {at['largest_commanded_step_deg']:.0f} deg")
    print(f"  {td['verdict']}: {td['descent_rate_mps']:.2f} m/s at first leg "
          f"contact (3D speed), {td['lateral_error_m']:.1f} m lateral, "
          f"{td['tilt_deg']:.2f} deg tilt at impact "
          f"({td['settled_tilt_deg']:.2f} deg settled), "
          f"legs crushed {100 * td['max_leg_crush_frac']:.0f} %")
    print(f"\nwrote {args.out}/flight.csv + meta.json")


if __name__ == "__main__":
    main()
