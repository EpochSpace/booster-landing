"""Real-time 2D view of a pdg powered-descent landing, streamed into Rerun.

The 6-DoF flight computed by ``pdg_flight`` is projected onto the vertical
plane that contains the initial downrange direction, and replayed against the
wall clock: a side view with the landing pad, the booster itself drawn to
scale, the engine plume, the SCvx plan and the trajectory actually flown, plus
the usual telemetry strips on a shared timeline.

    python viz/land2d.py --data out --speed 1.0

Rerun renders through the GPU. On a virtual adapter (VMware SVGA, Parallels,
RDP) the viewer window often comes up transparent or black -- use the CPU
renderer in ``land2d_mpl.py`` there.

Note on coordinates: a Rerun 2D view has +y pointing *down*, so altitude is
logged negated. Only the scene canvas is affected -- every number in the plots
is in physical units. The shared geometry lives in ``flight_data.py``.
"""

from __future__ import annotations

import argparse
import math
import os
import sys
import time
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))

import rerun as rr                                       # noqa: E402
import rerun.blueprint as rrb                            # noqa: E402
from flight_data import (                                # noqa: E402
    C_AUX, C_FLAME, C_FLAME_CORE, C_FLOWN, C_GHOST, C_GROUND, C_HULL,
    C_HULL_DARK, C_LIMIT, C_PAD, C_PAD_MARK, C_PLAN,
    Booster, Track, leg_deployment_track, load_run, pad_geometry, scene_bounds,
)


def view(points: np.ndarray) -> np.ndarray:
    """(downrange, altitude) -> Rerun 2D canvas coordinates (y down)."""
    pts = np.atleast_2d(np.asarray(points, dtype=float))
    return np.stack([pts[:, 0], -pts[:, 1]], axis=1)


# ---------------------------------------------------------------------------
# static scene
# ---------------------------------------------------------------------------
def log_scene(meta: dict, plan: Track, bounds: tuple[float, float, float, float]) -> None:
    x_lo, x_hi = bounds[0], bounds[1]

    ground = np.array([[x_lo - 200.0, 0.0], [x_hi + 200.0, 0.0]])
    rr.log("/scene/ground", rr.LineStrips2D([view(ground)], colors=[C_GROUND], radii=[1.6]),
           static=True)

    outline, marks, bullseye = pad_geometry()
    rr.log("/scene/pad/deck", rr.LineStrips2D([view(outline)], colors=[C_PAD], radii=[1.3]),
           static=True)
    rr.log("/scene/pad/marks", rr.LineStrips2D([view(m) for m in marks],
                                               colors=[C_PAD] * len(marks), radii=[0.8]),
           static=True)
    rr.log("/scene/pad/bullseye", rr.LineStrips2D([view(c) for c in bullseye],
                                                  colors=[C_PAD_MARK] * len(bullseye),
                                                  radii=[1.5]),
           static=True)
    rr.log("/scene/pad/label",
           rr.Points2D(view([[0.0, outline[1, 1]]]), colors=[C_PAD_MARK], radii=[2.0],
                       labels=["LZ"], show_labels=True),
           static=True)

    # glide-slope cone the guidance had to stay inside
    gs = meta["limits"]["glide_slope_deg"]
    if gs > 0.0:
        reach = max(abs(x_lo), abs(x_hi)) + 120.0
        tan_gs = math.tan(math.radians(gs))
        cone = [np.array([[0.0, 0.0], [s * reach, tan_gs * reach]]) for s in (-1.0, 1.0)]
        rr.log("/scene/glide_slope",
               rr.LineStrips2D([view(c) for c in cone], colors=[C_LIMIT] * 2, radii=[0.5],
                               labels=[f"glide slope {gs:.0f}°", ""],
                               show_labels=[True, False]),
               static=True)

    # the plan: SCvx reference, drawn once. A producer whose guidance runs in
    # the loop (the donor core) has none.
    if plan is not None:
        rr.log("/scene/plan",
               rr.LineStrips2D([view(plan.xy)], colors=[C_PLAN], radii=[0.9],
                               labels=["SCvx plan"], show_labels=True),
               static=True)
        rr.log("/scene/plan/nodes",
               rr.Points2D(view(plan.xy), colors=[C_PLAN], radii=[1.4]), static=True)


def log_series_styles() -> None:
    styles = {
        "/plot/altitude": ("altitude [m]", C_FLOWN),
        "/plot/downrange": ("downrange [m]", C_PLAN),
        "/plot/crossrange": ("crossrange [m]", C_AUX),
        "/plot/speed": ("speed [m/s]", C_FLOWN),
        "/plot/vspeed": ("vertical speed [m/s]", C_PLAN),
        "/plot/thrust": ("thrust [kN]", C_FLAME),
        "/plot/thrust_min": ("T min", C_LIMIT),
        "/plot/thrust_max": ("T max", C_LIMIT),
        "/plot/tilt": ("tilt [deg]", C_FLOWN),
        "/plot/tilt_limit": ("tilt limit", C_LIMIT),
        "/plot/gimbal": ("gimbal [deg]", C_PLAN),
        "/plot/gimbal_limit": ("gimbal limit", C_LIMIT),
        "/plot/aoa": ("angle of attack [deg]", C_AUX),
        "/plot/aoa_limit": ("AoA limit", C_LIMIT),
        "/plot/propellant": ("propellant [kg]", C_FLAME),
        "/plot/track_error": ("tracking error [m]", C_FLOWN),
    }
    for path, (name, color) in styles.items():
        width = 1.0 if path.endswith(("_limit", "_min", "_max")) else 2.0
        rr.log(path, rr.SeriesLines(names=[name], colors=[color], widths=[width]),
               static=True)


def log_limits(meta: dict, t0: float, t1: float) -> None:
    lim, veh = meta["limits"], meta["vehicle"]
    constants = {
        "/plot/thrust_min": veh["t_min"] / 1e3,
        "/plot/thrust_max": veh["t_max"] / 1e3,
        "/plot/tilt_limit": lim["tilt_max_deg"],
        "/plot/gimbal_limit": lim["gimbal_max_deg"],
        "/plot/aoa_limit": lim["aoa_max_deg"],
    }
    for t in (t0, t1):
        rr.set_time("sim_time", duration=t)
        for path, value in constants.items():
            rr.log(path, rr.Scalars(value))


def build_blueprint(bounds: tuple[float, float, float, float]) -> rrb.Blueprint:
    x_lo, x_hi, y_lo, y_hi = bounds
    return rrb.Blueprint(
        rrb.Horizontal(
            rrb.Spatial2DView(
                name="Landing — side view",
                origin="/scene",
                visual_bounds=rrb.VisualBounds2D(
                    x_range=[x_lo, x_hi],
                    y_range=[-y_hi, -y_lo],     # canvas y is negated altitude
                ),
                background=rrb.Background(color=(16, 18, 24)),
            ),
            rrb.Vertical(
                rrb.TimeSeriesView(
                    name="Altitude / range",
                    contents=["/plot/altitude", "/plot/downrange", "/plot/crossrange"],
                ),
                rrb.TimeSeriesView(
                    name="Velocity",
                    contents=["/plot/speed", "/plot/vspeed"],
                ),
                rrb.TimeSeriesView(
                    name="Thrust",
                    contents=["/plot/thrust", "/plot/thrust_min", "/plot/thrust_max"],
                ),
                rrb.TimeSeriesView(
                    name="Attitude",
                    contents=["/plot/tilt", "/plot/tilt_limit", "/plot/gimbal",
                              "/plot/gimbal_limit", "/plot/aoa", "/plot/aoa_limit"],
                ),
                rrb.TimeSeriesView(
                    name="Propellant / tracking",
                    contents=["/plot/propellant", "/plot/track_error"],
                ),
            ),
            column_shares=[3.0, 2.0],
        ),
        rrb.BlueprintPanel(state="collapsed"),
        rrb.SelectionPanel(state="collapsed"),
        rrb.TimePanel(state="expanded"),
    )


# ---------------------------------------------------------------------------
# playback
# ---------------------------------------------------------------------------
def replay(flight: Track, plan: Track, meta: dict, boost: Booster,
           speed: float, leg_alt: float, trail_stride: int) -> None:
    n = len(flight.t)
    m_dry = meta["vehicle"]["m_dry"]
    leg_frac = leg_deployment_track(flight.alt, leg_alt)

    # reference position resampled onto the flight clock, for the tracking error
    has_plan = plan is not None
    if has_plan:
        ref_d = np.interp(flight.t, plan.t, plan.downrange)
        ref_a = np.interp(flight.t, plan.t, plan.alt)
        ref_c = np.interp(flight.t, plan.t, plan.crossrange)
        err = np.sqrt((flight.downrange - ref_d) ** 2 + (flight.alt - ref_a) ** 2
                      + (flight.crossrange - ref_c) ** 2)

    wall0 = time.perf_counter()
    for i in range(n):
        t = float(flight.t[i])
        if speed > 0.0:
            lag = t / speed - (time.perf_counter() - wall0)
            if lag > 0.0:
                time.sleep(lag)

        rr.set_time("sim_time", duration=t)

        com = flight.xy[i]
        body = flight.body_dir[i]
        thrust_n = float(flight.thrust[i])
        deployed = float(leg_frac[i])

        parts = boost.hull(com, body) + boost.engine(com, body)
        rr.log("/scene/booster/hull",
               rr.LineStrips2D([view(s) for s in parts],
                               colors=[C_HULL] + [C_HULL_DARK] * (len(parts) - 1),
                               radii=[0.85] + [0.5] * (len(parts) - 1)))
        rr.log("/scene/booster/interstage",
               rr.LineStrips2D([view(boost.interstage(com, body))],
                               colors=[C_HULL_DARK], radii=[0.7]))
        fins = boost.grid_fins(com, body)
        rr.log("/scene/booster/fins",
               rr.LineStrips2D([view(s) for s in fins], colors=[C_HULL_DARK] * len(fins),
                               radii=[0.55]))
        legs = boost.legs(com, body, deployed)
        rr.log("/scene/booster/legs",
               rr.LineStrips2D([view(s) for s in legs], colors=[C_HULL_DARK] * len(legs),
                               radii=[0.7]))
        plume = boost.plume(com, body, flight.thrust_dir[i], thrust_n)
        if plume:
            rr.log("/scene/booster/plume",
                   rr.LineStrips2D([view(s) for s in plume],
                                   colors=[C_FLAME, C_FLAME_CORE], radii=[1.0, 1.6]))
        else:
            rr.log("/scene/booster/plume", rr.Clear(recursive=False))

        # where the plan says it should be right now
        if has_plan:
            rr.log("/scene/ghost",
                   rr.Points2D(view([[ref_d[i], ref_a[i]]]), colors=[C_GHOST],
                               radii=[3.0]))

        # trail actually flown so far
        if i % trail_stride == 0 or i == n - 1:
            rr.log("/scene/flown",
                   rr.LineStrips2D([view(flight.xy[: i + 1])], colors=[C_FLOWN], radii=[1.1]))

        rr.log("/plot/altitude", rr.Scalars(float(flight.alt[i])))
        rr.log("/plot/downrange", rr.Scalars(float(flight.downrange[i])))
        rr.log("/plot/crossrange", rr.Scalars(float(flight.crossrange[i])))
        rr.log("/plot/speed", rr.Scalars(float(flight.speed[i])))
        rr.log("/plot/vspeed", rr.Scalars(float(flight.vspeed[i])))
        rr.log("/plot/thrust", rr.Scalars(thrust_n / 1e3))
        rr.log("/plot/tilt", rr.Scalars(float(flight.tilt[i])))
        rr.log("/plot/gimbal", rr.Scalars(float(flight.gimbal[i])))
        rr.log("/plot/propellant", rr.Scalars(float(flight.mass[i] - m_dry)))
        if has_plan:
            rr.log("/plot/track_error", rr.Scalars(float(err[i])))
        if not math.isnan(flight.aoa[i]):
            rr.log("/plot/aoa", rr.Scalars(float(flight.aoa[i])))

    td = meta["touchdown"]
    rr.set_time("sim_time", duration=float(flight.t[-1]))
    verdict = (
        f"touchdown at t = {td['time']:.2f} s\n"
        f"  lateral error : {td['lateral_error_m']:.2f} m\n"
        f"  {td.get('speed_label', 'descent rate')[:14]:<14}: "
        f"{td['descent_rate_mps']:.2f} m/s\n"
        f"  lateral speed : {td['lateral_speed_mps']:.2f} m/s\n"
        f"  tilt          : {td['tilt_deg']:.2f} deg\n"
        f"  body rate     : {td['rate_deg_s']:.2f} deg/s\n"
        f"  propellant    : {td['propellant_used_kg']:.1f} kg used"
    )
    rr.log("/scene/touchdown", rr.TextLog(verdict, level=rr.TextLogLevel.INFO))
    print(verdict)


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--data", default="out", help="directory written by pdg_flight")
    ap.add_argument("--speed", type=float, default=1.0,
                    help="playback rate (1.0 = real time, 0 = as fast as possible)")
    ap.add_argument("--vehicle-scale", type=float, default=1.0,
                    help="draw the booster larger than life so it stays readable")
    ap.add_argument("--leg-deploy-alt", type=float, default=None,
                    help="altitude at which the legs start swinging out [m]; "
                         "defaults to the value in meta.json, else 140")
    ap.add_argument("--trail-stride", type=int, default=2,
                    help="redraw the flown trail every N samples")
    ap.add_argument("--save", metavar="FILE.rrd",
                    help="also write the recording to an .rrd file")
    ap.add_argument("--spawn", dest="spawn", action="store_true", default=True,
                    help="spawn the Rerun viewer (default)")
    ap.add_argument("--no-spawn", dest="spawn", action="store_false",
                    help="do not spawn a viewer (use with --save or a running viewer)")
    ap.add_argument("--connect", metavar="URL",
                    help="stream to an already running viewer, "
                         "e.g. rerun+http://127.0.0.1:9876/proxy")
    args = ap.parse_args()

    meta, flight, plan = load_run(Path(args.data))
    boost = Booster(meta, args.vehicle_scale)
    bounds = scene_bounds(flight, plan)

    # rerun-sdk ships the viewer next to the interpreter, but spawning looks it
    # up on PATH -- which misses it when this script is run through the venv's
    # python.exe without activating the environment first.
    scripts_dir = str(Path(sys.executable).parent)
    if scripts_dir not in os.environ.get("PATH", "").split(os.pathsep):
        os.environ["PATH"] = scripts_dir + os.pathsep + os.environ.get("PATH", "")

    rr.init("booster_landing", spawn=args.spawn and not args.connect)
    if args.connect:
        rr.connect_grpc(args.connect)
    if args.save:
        rr.save(args.save)
    rr.send_blueprint(build_blueprint(bounds))

    log_scene(meta, plan, bounds)
    log_series_styles()
    log_limits(meta, float(flight.t[0]), float(flight.t[-1]))

    g = meta["guidance"]
    if g.get("solver"):
        print(f"{meta.get('source', 'run')} / {meta.get('scenario', '?')}: "
              f"{g['tf']:.1f} s, {g['fuel_used_kg']:.0f} kg propellant, "
              f"guidance: {g['solver']}")
    else:
        print(f"SCvx plan: tf = {g['tf']:.2f} s, fuel {g['fuel_used_kg']:.1f} kg, "
              f"{g['scvx_iterations']} iterations")
    print(f"replaying {len(flight.t)} samples at {args.speed}x")
    leg_alt = args.leg_deploy_alt
    if leg_alt is None:
        leg_alt = meta["vehicle"].get("leg_deploy_alt_m", 140.0)
    replay(flight, plan, meta, boost, args.speed, leg_alt, max(1, args.trail_stride))


if __name__ == "__main__":
    main()
