# booster-landing

A propulsive landing simulation in the SpaceX mould: convex-optimization powered
descent guidance, flown closed loop through a 6-DoF rigid-body plant, replayed
in real time as a 2D side view with the landing pad and the booster itself.

```
external/pdg  ──►  bridge/pdg_flight.cpp  ──►  out/*.csv + meta.json  ──►  viz/land2d*.py  ──►  window
 guidance + plant      solve, fly, log            telemetry                project + replay
```

`external/` carries **one** edit, and it is described in full under *The one
donor patch* below: a `custom` scenario in the donor's `core/scenario.c`, plus
the horizontal-velocity field it needs, because no shipped scenario could
express a stage that is still flying sideways when it arrives. Every shipped
scenario is bit-identical under it. Nothing else in `external/` is touched, and
the bridges are the only other glue. `external/` itself is not committed: the
edit lives in [`patches/booster-lander-simulator.patch`](patches/booster-lander-simulator.patch),
and `setup.ps1` applies it after cloning.

[`docs/model.tex`](docs/model.tex) states every equation actually implemented —
guidance, plant, autopilot, the donor reconstruction, the RTLS trajectory model
and the projection — with the assumptions and open questions collected at the
end. It is written from the source rather than from intent, and it is the right
place to start if you are checking the physics. `docs\build.ps1` compiles it to
`docs/model.pdf` (16 pages); it finds MiKTeX by path rather than relying on
`PATH`, since a machine-scope install only reaches shells started after it.
Without a LaTeX toolchain: `winget install --id MiKTeX.MiKTeX -e`, or paste the
`.tex` into Overleaf.

[`docs/model-ru.tex`](docs/model-ru.tex) — **математическая модель на русском**,
`docs\build-ru.ps1` → `docs/model-ru.pdf` (17 страниц). Полный свод уравнений
движения на всех участках полёта, включая 6-СС динамику атмосферного спуска,
аэродинамику корпуса и решётчатых рулей, наведение тормозных импульсов входа и
посадки и модель контакта опор — то, что английская версия описывает лишь
частично. Кириллица требует кодировки T2A; глифы берутся из cm-super, первая
сборка может минуту строить метрики через METAFONT.

[`docs/task-rtls.md`](docs/task-rtls.md) specified a solo first-stage test flight
that boosts back and lands on the pad it left. It is **built** — see *The solo
RTLS flight* below. `bridge/rtls_trajectory.py` flies the half of it that nobody
here modelled before, liftoff through the boostback burn to the entry interface,
and hands the rest to the donor.

## Quick start

```powershell
.\setup.ps1      # clone the upstream repos at pinned commits, apply the patch, create .venv   (once)
.\run.ps1        # build, fly a landing, watch it land
```

`run.ps1` builds with MSVC + Ninja if needed, runs `pdg_flight.exe`, then replays
the flight paced against the wall clock.

```powershell
.\run.ps1 -Rtls -Speed 12              # the whole RTLS flight: pad to pad, 9 min
.\run.ps1 -Follow 200                  # chase camera: the booster drawn large
.\run.ps1 -Scenario entry -Speed 4     # the whole return from separation, 62 km
.\run.ps1 -Build                       # build and run the unit tests only
.\run.ps1 -Speed 0.35                  # slow motion
.\run.ps1 -R0 "3000,0,10000" -V0 "-60,0,-300" -Nodes 41 -TfGuess 70
.\run.ps1 -MisalignDeg 0.3 -ThrustScale 0.98 -ActuatorTau 0.08   # dispersed plant
.\run.ps1 -Renderer rerun              # the GPU viewer instead (see below)
```

## What you see

One window, both halves on the same clock:

- **Left — the landing, side view.** The pad deck with its markings, the glide-slope
  cone the guidance had to stay inside, the SCvx plan in blue, the trajectory
  actually flown in yellow, a ghost marker showing where the plan says the
  booster should be right now, and the booster itself drawn to scale: hull, grid
  fins, landing legs that swing out below 140 m, and an engine plume whose length
  and flare track the commanded thrust. A HUD carries the live numbers.
  The stage is laid out from the length and diameter the producer reports, so the
  Falcon-class one comes out at its real 13:1 slenderness, with the black
  interstage, lattice grid fins at the top station and legs splaying to an 18 m
  footpad span — not a rectangle with sticks on it.
- **Right — telemetry.** Altitude/downrange/crossrange, speed and vertical speed,
  thrust against its min and max bounds, and tilt/gimbal/angle-of-attack against
  their limits — each drawn only up to the current instant.

## Two renderers

**matplotlib (`viz/land2d_mpl.py`) — the default.** Pure CPU rendering through
Tk. Blitted, so it holds ~45 fps in the wide view and ~33 fps with `-Follow`;
a full redraw would manage 7. Playback is paced against the wall clock and
*drops* frames rather than falling behind, so a second of playback is always a
second of flight.

### The camera

A 41 m booster inside a 62 km descent is sub-pixel, and a fixed frame can show
either the whole flight or the landing, never both. So the camera moves: it
holds the whole trajectory while the stage is high, and when the booster drops
below `-ZoomAt` (2000 m) it closes in on a `-ZoomTo` window (200 m) and chases
the booster from there.

**The flight holds while the camera travels** — `-ZoomPause` seconds, 1.5 by
default, `0` to cut straight in. The move is worth a beat of its own: it spans a
factor of ~350 in scale, and run at playback speed it is a blur. The held time
is handed back afterwards, so everything either side stays real time and the
console reports the hold separately. Spreading the zoom across the descent
instead does not work, because at the trigger altitude an altitude-driven camera
is still fully wide — there would be nothing to watch.

The move itself is geometric in scale and smoothstepped in time, so it eases in
and out instead of crawling and then snapping. Axis ticks give way to a scale
bar that steps 1-2-5 and so keeps between a tenth and a quarter of the frame all
the way down; a powers-of-ten ladder swings from 8 % to past 50 % and then snaps
back tenfold.

**The window is square and the scale is 1:1 the whole way.** That matters for
more than looks. A fixed wide view of the `entry` profile has to compress the
vertical scale by about 15× — 5 km of ground against 69 km of altitude — or the
panel degenerates into a sliver. At that ratio the last 500 m, a real ±14 m
lateral wander of roughly 1.6° from vertical, is drawn as a dramatic S and reads
as if the vehicle were corkscrewing. The moving camera never has to distort
anything; if you do pin it (`-ZoomAt 0`) and the aspect goes bad, the title says
so and by how much.

The camera engages only where there is something to zoom out of: the flight has
to *reach* above `-ZoomAt` and the wide framing has to be several times the
target window. A 500 m hop is already close to the target, so it keeps a fixed
frame. `-Follow <metres>` overrides all of it with a plain chase camera at a
fixed window for the whole replay.

**It used to say "start above", and that was two bugs.** Every profile before
the RTLS one began at its highest point and came down, so testing the first
sample was the same thing as testing the highest one, and taking a running
minimum of altitude from `t=0` was the same thing as taking it from apogee. A
flight that lifts off from the pad breaks both at once: it starts at zero
altitude, which failed the enable test and switched the camera off entirely,
and it latched that zero into the running minimum before the stage had left the
ground, which would have fired the zoom on the launch and left it closed for the
rest of the flight. The enable test now looks at the whole trajectory, and the
running minimum starts at the last sample above the trigger — which for a
descent-only flight is the same sample as before, so nothing else changed.

**Rerun (`viz/land2d.py`) — `-Renderer rerun`.** Nicer to drive: scrub, pause
and rewind on a shared timeline. But it renders through the GPU, and **on a
virtual adapter (VMware SVGA, Parallels, RDP) its window comes up transparent or
black** — which is why matplotlib is the default here. Use it if you have real
GPU drivers.

Both read the same three files and share the projection and vehicle geometry in
`viz/flight_data.py`, so they draw the same booster.

## How high can it start?

**pdg has no atmosphere.** Its dynamics are `vdot = C(q) u / m + g` — thrust and
gravity, nothing else. Whatever slows the stage down has to come out of the
tanks, so the entry altitude is capped by the propellant budget, not by the
solver.

The stock vehicle (15 t wet, 10 t dry, Isp 300 s) carries 1193 m/s of dV and
lands from **20 km**, burning 92 % of its propellant. `--m-wet`/`--m-dry`/
`--isp`/`--thrust-max` scale the vehicle: 25 t / 11 t gives 2415 m/s and reaches
**35 km**. `pdg_flight` prints the dV budget and the unpowered arrival speed
before it solves, so an impossible case is obvious.

Above a few kilometres the default discretisation is too coarse and SCvx stalls
— that is a solver setting, not a wall. `-Nodes 41 -TfGuess <seconds>` fixes it;
without them a 5 km entry fails to converge and with them it lands at 3 cm.

### The F9-like preset

`pdg`'s defaults are the textbook Mars case: a 15 t stage with `T_min/W = 0.82`,
i.e. one that **can hover**. A Falcon-9-class first stage cannot — a single
Merlin at its 40 % throttle floor already outweighs the empty stage — and that
changes the problem qualitatively. `-Preset f9` supplies the other vehicle:

| | default | f9 |
| --- | --- | --- |
| wet / dry | 15 / 10 t | 30.1 / 25.6 t (15 % propellant) |
| Isp | 300 s | 282 s (sea level) |
| thrust | 80–250 kN | 338–845 kN (one Merlin, 40 % floor) |
| `T_min/W` at dry | 0.82, can hover | **1.35, cannot** |
| pitch inertia | 8×10⁵ kg·m² | 5.30×10⁶ (shell + column model) |
| gimbal | 12° | 5° |

Three things had to change with it, each forced by the vehicle rather than
chosen:

**Thrust margin.** A minimum-fuel plan rides `T_max` for over half the burn,
which leaves the tracker nothing to correct with — at `T_max` there is no "more".
The planner is held to `--thrust-margin` (0.85) of the envelope and the plant and
autopilot keep the rest as authority. Without it the stage arrives at 25 m/s.

**Retuned attitude gains.** pdg's `attP = 9.0` assumes its own 8×10⁵ kg·m² stage.
At 5.44×10⁶ the same error asks for 6.8× the deflection, and the loop sat on its
stop for **91 % of the flight**, commanding 30° against a 5° limit. Sizing the
loop to the authority instead — `α_max = T sin δ L / J = 0.108 rad/s²`, so
`ω_n = √(α_max/e) ≈ 1.1 rad/s` for a 5° error — gives `attP = 1.2`, `attD = 2.9`.

**Engine cutoff.** Past the end of the plan the tracker feeds forward `-mg`,
which is below `T_min`; the clip to `T_min` then turns the last metre into a
climb-away. Left alone the stage bottomed out at 0.2 m, went back up, burned its
reserve and coasted to 16 km. A stage that cannot hover has to cut, so it does —
at the plan's end or once the descent is arrested below 1.5 m, whichever comes
first, and then it falls the last half metre.

#### Where the numbers come from

Published Falcon 9 Full Thrust stage-1 figures: **433.1 t** gross, **22.2 t**
empty, **410.9 t** propellant (287.4 LOX + 123.5 RP-1), nine Merlin 1D at
**7607 kN** sea level — 845 kN each — and Isp **282 s** sea level, **311 s**
vacuum. The landed stage carries legs and grid fins, which puts it nearer 25.6 t.

**The landing reserve is not published.** Working the final burn out from the
rocket equation — about 400 m/s against a 25.6 t stage at 282 s — gives close to
4 t, so the preset carries 4.5 t. Whole-return figures quoted online (around
55 t) are the sum of three burns, and an independent check agrees: boostback
~30 t, entry burn ~20 t, landing burn ~4.5 t at 305.6 kg/s per engine.

#### Starting higher

Only the last of those three burns is a pdg problem — the other two happen in
the atmosphere, which pdg does not have. So the preset's envelope is the
landing-burn regime and nothing above it:

| start | time of flight | propellant used | touchdown |
| --- | --- | --- | --- |
| 2 km, 180 m/s | 16.7 s | 3518 of 4500 kg | 3.2 m/s |
| 3 km, 210 m/s | 20.7 s | 4177 of 4500 kg | 4.9 m/s |

```powershell
.\run.ps1 -Preset f9 -R0 "250,0,3000" -V0 "-21,0,-210" -TfGuess 26
```

3 km spends 93 % of the reserve; past that the plan runs out of propellant and
SCvx stops converging, which is the correct answer rather than a solver problem.
A stage that starts higher needs the propellant for the burns above it, so give
it some with `--m-wet` and treat the result as a hypothetical vehicle:

```powershell
.\build\pdg_flight.exe --preset f9 --m-wet 36000 --nodes 41 `
    --r0 1500,0,10000 --v0 -30,0,-300 --tf-guess 45 --tf-max 150 --t-max 200 --out out_f9_10km
```

For the real profile from separation — entry burn, aero descent, landing burn —
use the donor instead: `-Scenario entry` below flies it from 62 km with an
atmosphere, which is the only way this repository can do it honestly.

#### How it tests

Nominal lands at **3.12 m/s, 0.61 m lateral, 0.68° tilt**. Graded against the
donor's thresholds (PERFECT ≤ 2 m/s, GOOD ≤ 4, HARD ≤ 6, crash above):

| dispersion | touchdown | lateral | tilt | grade |
| --- | --- | --- | --- | --- |
| nominal | 3.12 m/s | 0.61 m | 0.68° | GOOD |
| thrust 99 % | 3.06 | 0.41 | 0.42° | GOOD |
| thrust 101 % | 4.86 | 1.28 | 1.36° | HARD |
| thrust 98 % | 5.69 | 0.45 | 0.53° | HARD |
| thrust 102 % | 6.21 | 3.14 | 3.26° | CRASH |
| Isp 98 % | 3.51 | 0.62 | 0.71° | GOOD |
| cant 0.1° | 3.35 | 2.01 | 2.04° | GOOD |
| cant 0.2° | 3.71 | 3.44 | 3.42° | HARD |
| lag 30 ms | 3.24 | 1.88 | 2.24° | GOOD |
| lag 80 ms | 3.99 | 4.91 | 5.37° | HARD |

**It is fragile, and the fragility is architectural.** ±2 % of thrust is enough
to lose it, and note that *more* thrust is worse than less: the stage arrests
higher, and everything above the deck at cutoff becomes touchdown speed through
`√(2gh)`. A single open-loop plan plus a PD tracker cannot hold a vehicle whose
landing has no hover phase to absorb timing error — there is one chance to get
the ignition point right. The fix is receding-horizon re-planning: an SCvx solve
takes about half a second, so re-solving every 1–2 s from the current state is
affordable. That is the next piece of work, not a tuning problem.

### From stage separation

Separation is ~62 km at ~1.5 km/s, and there the atmosphere does most of the
braking: an engine-only solution would need roughly 3 km/s against the ~1.2 km/s
the stage carries. pdg cannot fly that profile, and no amount of tuning changes
it.

The donor can. `booster-lander-simulator` has the missing physics — US76
atmosphere, body and grid-fin aerodynamics, leg contact — and ships an `entry`
scenario that starts at separation:

```powershell
.\run.ps1 -Scenario entry -Speed 4 -Follow 400
```

| phase | time | altitude |
| --- | --- | --- |
| entry burn (3 engines, 2.8 MN) | 0.5 → 25.5 s | 62.0 → 39.1 km |
| aero descent (engines off, grid fins) | 26 → 113 s | 39.1 → 2.7 km |
| landing burn (1 engine) | 113.5 → 131.5 s | 2.7 → 0 km |

Peak dynamic pressure 54 kPa; touchdown at 3.2 m/s, 24 m from the pad.

### Is that touchdown speed normal?

The core grades against its own thresholds (`core/constants.h`): target 1.5 m/s,
PERFECT ≤ 2, GOOD ≤ 4, HARD ≤ 6, above 6 it is a crash. Over 40 dispersed runs:

| scenario | start | mean touchdown | max | landed | grades |
| --- | --- | --- | --- | --- | --- |
| `terminal` | 2 km | 1.90 m/s | 2.74 | 95 % | 4 PERFECT, 33 GOOD, 1 HARD |
| `aero_offset` | 12 km | 3.06 m/s | 5.76 | 78 % | 7 GOOD, 24 HARD |
| `entry` | 62 km | 3.37 m/s | 5.79 | 88 % | 4 GOOD, 31 HARD |

So ~3.2 m/s from separation is typical, not an outlier, and it sits inside the
GOOD *speed* band. Those runs grade HARD on **lateral miss** (mean 13.7 m against
a 10 m limit) and **tilt**, not on descent rate. The margin is real but not
generous: 4 of 40 entry runs exceeded 6 m/s and were scored crashes.

Three things about these numbers are easy to misread, so the bridge records them
explicitly:

- `descent_rate_mps` for a donor run is the **3D speed of the lowest point at
  first leg contact**, sampled at the core's 500 Hz rate — not a vertical rate.
  `speed_label` says so and the viewers use it.
- It will not match the end of the trajectory. The visible log ends at 1.7 m/s
  and 1.3 m altitude, because contact happens *between* two 2 Hz log lines and
  the legs are already absorbing by the next one.
- `tilt_deg` is the attitude at impact (3.80°, which is what the verdict is
  graded on); `settled_tilt_deg` is where it comes to rest (0.09°). The
  `--verbose` RESULT line only prints the settled one.

pdg's 0.98 m/s is a different kind of number: `ScvxParams::vf = {0, 0, -1.0}` is
a boundary condition, so the guidance is *told* to arrive at 1 m/s and the
tracker delivers it. pdg has no contact model at all — the run stops when
`r_z <= 0`. It is also fragile: the 2 % thrust deficit case arrives at 10 m/s.
`bridge/bls_flight.py` runs that core and converts its trajectory log into the
same three files, so the viewers replay it unchanged. Other scenarios:
`terminal`, `aero_offset`, `asds_night`, `chaos`.

Three quantities are reconstructed, because the core's text log does not carry
them. **Thrust** comes from the mass flow, `T = -dm/dt · g0 · Isp` — better than
scaling the logged throttle fraction, which does not say how many engines are
lit. **The lean direction** comes from the sign of the lateral acceleration:
thrust acts along the body axis, so a stage accelerating towards +x is leaning
that way; with the engine off there is no such evidence and the last decision is
held. The tilt magnitude is exactly as logged, and the gimbal is drawn at zero
because the log omits it.

#### Attitude cannot step

An inferred sign flips between one sample and the next, and drawing that
directly had the stage snapping through 35° in 50 ms — **347 °/s**, twelve times
anything the vehicle actually did. A stage turns with torque against inertia
like anything else, so the command is now flown through the agility it has:

| source | torque | peak |
| --- | --- | --- |
| engine gimbal | `T sin δ_max · z_com` | 25.4 °/s² |
| grid fins | `n q S C_Nα δ_max (z_fin − z_com)` | 99.2 °/s² |
| RCS couple | `n F (z_rcs − z_com)` | 0.32 °/s² |

against a pitch inertia of 5.1–6.1 ×10⁶ kg·m² — the dry stage as a thin-walled
shell plus the propellant as a solid column, carried to the CoM by the
parallel-axis theorem, the same model as the core's `mass_props`. Each source
dominates its own phase: fins fly the aero descent, the gimbal the burns, RCS
the vacuum coast.

`slew_limit` then tracks the command time-optimally within `α_max` — aim for the
fastest rate that can still stop exactly on target, `ω = sign(e)·√(2α|e|)` — so a
step becomes a pitch-over of `2√(θ/α)` seconds. It is additionally capped by the
logged body rate `wperp`, since the in-plane component of a rate cannot exceed
that rate's magnitude; that one is an exact bound, not a modelling choice.

It checks out against the core's own numbers: reconstructed peak 20.2 °/s
against a logged 28.6 °/s, and every other scenario likewise just under its
logged peak. pdg data needs none of this — its attitude comes from real 6-DoF
dynamics and already peaks at a sane 11.5 °/s.

**The sign of the range.** The core logs `lat = sqrt(rx² + ry²)`, a distance,
never negative. A stage that flies over the pad and out the other side then
appears to bounce off an invisible wall at range zero, which turns the last
seconds of the descent into a zigzag. `unfold_radius()` signs it: a pass through
the origin leaves a cusp — the radial speed reverses between two samples with
both magnitudes large, at a range shorter than one sample's travel — while a
genuine radial turnaround has `vrad` decay smoothly through zero and fails both
tests. In the `entry` run this turns a corner at range 2.5 m into what actually
happened: a 14 m overshoot past the pad and a swing back to land 24 m the other
side. It assumes the horizontal motion stays in one plane, which a side view
assumes regardless; the log carries no azimuth to check against. pdg data needs
none of this — its `downrange` is a signed projection of the real 3D position.

The core prints one line every 250 steps — 0.5 s, hardcoded — so the real data
rate is 2 Hz and the raw replay steps visibly. `--resample` interpolates onto a
uniform grid for playback (20 Hz by default, `0` to keep the raw cadence), which
takes it from 7 to 23 fps. That adds no information, and `meta.json` records
`log.source_rate_hz` alongside `log.rate_hz` so the difference stays visible.

The two producers are not mixed: each is a complete simulator writing the same
telemetry format. The warning against mixing donors is about splicing equations
of motion into one plant, which nothing here does.

## The solo RTLS flight

One stage, one continuous flight, pad to pad. No second stage, no payload — not
carried, not simulated, not jettisoned. It lifts off, flies an ascent, cuts the
engines, **turns around and burns back**, coasts through apogee, re-enters, flies
an aerodynamic descent under grid fins and lands on the pad it left.

```powershell
.\run.ps1 -Rtls -Speed 12
```

That is `docs/task-rtls.md` implemented. The boostback burn is the piece that did
not exist here before, and it is the piece that makes the flight an RTLS rather
than a hop.

### The flight

| phase | t | altitude |
| --- | --- | --- |
| ascent, 4 engines | 0 → 110.1 s | 0 → 60.67 km |
| post-cutoff coast (settling) | 110.1 → 118.1 s | 60.73 → 70.06 km |
| **boostback burn, 3 engines** | 118.1 → 152.1 s | 70.12 → 106.90 km |
| ballistic coast, through apogee | 152.2 → 386.7 s | 106.95 → 64.37 km |
| entry burn, 3 engines | 386.7 → 412.2 s | 64.30 → 46.74 km |
| aero descent, grid fins | 412.2 → 529.1 s | 46.74 → 2.70 km |
| landing burn, 1 engine | 529.2 → 548.1 s | 2.69 → 0 km |

**It touches down at 2.09 m/s, 16.0 m from the pad it left, with 2.0 t still in
the tanks.** Nine minutes and eight seconds, one vehicle, 181.3 t of propellant.

| | |
| --- | --- |
| liftoff | 208.9 t, 183.3 t of propellant, four engines, T/W 1.65 |
| ascent | 122.3 t spent, capped at 2.79 g, max q 47.0 kPa |
| engine cutoff | T+109.9 s, 60.46 km, 1578.9 m/s, 35.00 km downrange, γ 50.2°, 86.59 t |
| turnaround | 8 s settling + 5.8 s slew through 132° (41 s on cold gas alone) |
| boostback | 34.0 s, 31.21 t, ΔV 1363 m/s, attitude 180.0° |
| apogee | 151.9 km at T+250 s |
| maximum downrange | 60.5 km |
| handover | T+386 s, 65 km, 9.59 km downrange, −222 m/s horizontal, 29.79 t |
| return total | 58.98 t across the three burns |

Every check the specification asks for, graded by the run itself
(`meta.json → rtls.validation`, printed at the end of a run):

| check | value | band | |
| --- | --- | --- | --- |
| cutoff altitude vs CRS-9 MECO | 60 458 m | 59 937 – 60 937 | PASS |
| cutoff speed vs CRS-9 MECO | 1578.9 m/s | 1573.9 – 1583.9 | PASS |
| cutoff downrange | 35 003 m | 30 – 40 km | PASS |
| apogee | 151.9 km | 118 – 170 km | PASS |
| maximum downrange | 60.5 km | 59 – 70 km | PASS |
| **horizontal velocity reversed** | +1010 → **−222 m/s** | sign change | **PASS** |
| boostback propellant | 31.21 t | ~30 t | PASS |
| boostback ΔV | 1363 m/s | 1300 – 1600 | PASS |
| three return burns total | 58.98 t | ~55 t | PASS |
| did not run dry | 2019 kg left | > 0 | PASS |
| **lands on the pad it left** | **16.0 m** | ≤ 30 m | **PASS** |
| impact speed | 2.09 m/s | ≤ 6 (donor grading) | PASS |

The reversal is the one that matters: the specification is explicit that "if the
boostback manoeuvre is not there, the task is not done", and the test it names is
the horizontal velocity being *reversed rather than merely reduced*. It goes from
+1010 m/s downrange at cutoff to −222 m/s at the entry interface, and the ground
track turns round at 60.5 km and comes home.

Two honest notes on those numbers. **Apogee passes its band but is not CRS-9's
128.70 km**, for a reason set out under *the cutoff spec does not pin the flight
path angle* below. And **the 16 m lateral error is the donor's accuracy, not the
aiming loop's**: the aim-point shooting converges to about that on its first pass
and then wanders by a metre either way, because 16 m is well inside the donor's
own landing dispersion — its `entry` scenario averages 12–13 m and lands 24 m off
on the seed this repository quotes.

### The split at the entry interface

```
liftoff ─► cutoff ─► boostback ─► coast ─► 65 km │ 65 km ─► entry ─► aero ─► landing
        bridge/rtls_trajectory.py                │  external/booster-lander-simulator
```

Two producers, one clock, and the handover is a **state vector** — not a splice
of equations of motion, which is the thing this repository has always refused to
do. `rtls_trajectory.py` is new code: a planar round-Earth point-mass model with
a US76 atmosphere, which owns everything down to the entry interface. Below that
the donor takes the flight, because the descent is aerodynamic and the donor
already models it and has been tested against Monte Carlo.

The split is not a convenience. Against US76 at 1579 m/s the dynamic pressure is
386 Pa at 60 km, 23 Pa at 80 km and 0.70 Pa at 100 km; on an 80 t stage the last
is a deceleration of 8×10⁻⁵ m/s², four parts in 10⁵ of the thrust acceleration.
Thrust-and-gravity dynamics are therefore *physically valid* across the boostback
and the coast, and the atmosphere only has to be right on the way back down.

The atmosphere reproduces the published US76 density to better than 0.01 % from
sea level to 120 km — `python bridge/rtls_trajectory.py` prints the check — and
reproduces all three of the dynamic pressures above exactly. Two things were
worth getting right rather than approximating:

- **Geopotential vs geometric altitude.** The layer stack's base heights are
  geopotential; published density tables are indexed by geometric altitude. Using
  one against the other looks like a 17 % model error at 86 km. (Conveniently,
  84.852 km geopotential *is* 86 km geometric, so the two regimes meet exactly.)
- **Above 86 km US76 stops being a lapse-rate model** — molecular weight varies —
  so density is log-interpolated from the table rather than extrapolated
  exponentially. An exponential tail overstates the density at 100 km by 20 %,
  and it is precisely that near-zero the vacuum argument rests on.

### Work item 1: `Cd·S`, settled

The specification flagged a disagreement: the reference notebook uses `Cd = 0.5`
on 4π = 12.57 m², the expert review suggested `Cd ≈ 1.0`, and `Cd·S` differs by
1.7×. Everything below the entry interface depends on it.

**They are not disagreeing about the same number — they describe different
attitudes.**

| | leading end | `Cd` | `Cd·S` on 10.52 m² |
| --- | --- | --- | --- |
| ascent, nose-first, fairing fitted | 17° cone | 0.30 → 0.55 at M 1.05 → 0.24 | 3.2 → **5.79** → 2.5 m² |
| return, engines-first | flat, separated base | 0.85 → 1.40 (`CA(M)`) | **8.94 → 14.73 m²** |

A returning booster flies engines-first, so the leading end is a bluff base, not
a cone — and the donor already carries exactly that, as a Mach table in
`core/dynamics.c`. It sides with the review. Since the donor owns every metre
below the entry interface, **the descent coefficient is settled by the code that
flies it**, and the notebook's figure is much closer to the *ascent* value. This
repository uses the cone table for the ascent and the donor's own table for the
short stretch of its coast that is still in measurable air, so the two halves of
the flight do not disagree across the handover.

One consequence: the specification's own *Architecture* table quotes the drag on
an 80 t stage at 60 km as 0.025 m/s², which implies `Cd·S ≈ 5.2 m²` — the
ascent-like value applied to a vehicle flying backwards. With the donor's
coefficient it is 0.047 m/s². Both are negligible against thrust, so the
conclusion stands, but the figure is low by 1.9×.

### The ascent, and why a solo stage is a different vehicle

A stage without a 107.5 t upper stack on top is light, and that turns out to be
the whole difficulty. Flown at full thrust it reaches 1579 m/s while it is still
only **54 km** up — 6.5 km short of the cutoff condition, with the steering bias
already saturated against its limit and nowhere left to go. CRS-9 reached the
same speed at 60.4 km because it was dragging the upper stack; a solo stage is
simply too good at accelerating to arrive that high that slowly.

So the axial-acceleration cap is a **free variable of the solve** rather than a
fixed limit — the stage has to be held back to fly a trajectory it could not
otherwise reach — and the value it settles on, 2.79 g, is a result rather than a
setting. The engine count is chosen the same way: the smallest cluster whose
full-thrust liftoff thrust-to-weight clears 1.35, which for this vehicle is four.
(The burn is not much *longer* for it — 109.9 s against 110.6 — it is a different
shape, climbing more steeply for the same speed.)

The pitch programme is a vertical rise, a smoothstepped kick, then a
zero-angle-of-attack gravity turn — the stage flies where it points, which is
what keeps the airframe loads honest — with a constant steering bias permitted
only above 40 km, where dynamic pressure is under a few hundred pascals and
steering is aerodynamically free. Four unknowns against three conditions at
cutoff: altitude, downrange and the mass left in the tanks. **The propellant
spent is an output, not an input**, which is the point the specification makes
about a solo stage: this is not CRS-9's ascent and is not reported as one.

#### The cutoff spec does not pin the flight path angle — and that sets apogee

Altitude, speed and downrange are three numbers. The *direction* of the velocity
is a fourth, and they do not determine it. It matters, because apogee follows
almost entirely from the vertical component. Sweeping it:

| γ at cutoff | vₓ | ascent + boostback | apogee | max downrange |
| --- | --- | --- | --- | --- |
| 49.8° (least steering) | 1020 m/s | **151.4 t** | 168.0 km | **60.8 km** |
| 43.1° | 1153 m/s | 169.9 t | 154.7 km | 67.5 km |
| 42.6° | 1162 m/s | 169.9 t | 153.8 km | 67.7 km |
| 42.4° (asked for 35°) | 1167 m/s | 170.8 t | 153.3 km | 68.3 km |

Below about 42° the steering bias saturates and the trajectory is not reachable
at all under a zero-incidence gravity turn through the atmosphere. So the
least-steering solution is also the cheapest *and* has the lowest maximum
downrange, which is why it is the default; `-Gamma` forces the others.

**CRS-9's measured 128.70 km apogee would need γ ≈ 32°**, which this ascent
cannot produce while also arriving at 60.4 km only 35 km downrange. Apogee lands
in the specified 118–170 km band but at the top of it, and that is a real
statement about the anchor numbers rather than a tuning failure: the specification's
own derived band of γ = 30–40° is not consistent with the cutoff state it asks for.

### The boostback: a targeting problem, not a landing problem

This is why `pdg`'s SCvx formulation does not apply. Its terminal condition is
"at the origin with near-zero velocity"; the stage at boostback cutoff is 100 km
up and moving at a kilometre a second. What is actually solved is the return leg
of the reference notebook's phase 2, posed on the cutoff state:

> given the state at engine cutoff, find a burn — attitude held fixed in the
> inertial frame, plus a duration — whose ballistic continuation reaches the
> launch site.

One equality constraint against two unknowns, so there is a one-parameter family
of solutions and the spare freedom buys the cheapest of them: minimise burn time
subject to hitting the aim point, which at fixed engine count and throttle is
minimum propellant. The duration is found by bisection — the miss is monotone in
burn time, since a longer burn always leaves the stage further up-range — and the
attitude by bounded scalar minimisation. **Apogee is left alone by this**, so it
stays an independent check rather than something the solve was told to hit.

#### Minimum propellant does not pick a trajectory on its own

Scanning the attitude at a fixed cutoff state shows why that needed care. The
propellant cost is almost flat and everything else is not:

| attitude | burn | propellant | apogee | v_z at the interface |
| --- | --- | --- | --- | --- |
| 172.5° | 35.60 s | **32.64 t** | 163.4 km | −1329 m/s |
| 180° | 35.85 s | 32.86 t | 144.0 km | −1184 m/s |
| 185° | 36.32 s | 33.29 t | 132.5 km | −1090 m/s |
| 195° | 38.15 s | 34.97 t | 113.1 km | −905 m/s |

Over the range where apogee moves 30 km and the entry speed moves 240 m/s, the
propellant moves **2 %**. A minimum-fuel objective cannot discriminate between
those, and left to itself it drifts below 180° into thrusting *upward* — lofting
the stage to 200 km for a saving of thirty kilograms, then handing the donor a
stage arriving 500 m/s faster. The objective cannot see that, because the cost
it minimises stops at boostback cutoff and the bill arrives later, in the entry
burn.

So the attitude is constrained to 180° or more: **the boostback thrust may not
have an upward component.** That is not a fudge to make apogee come out right —
a boostback is a *retrograde* manoeuvre, and pitching above the horizon to add
energy upward is not boosting back, it is lofting. It costs 0.1 % of the
propellant and buys an entry state the donor can fly. Apogee still falls out of
the solve rather than being targeted.

There was also a plain bug behind the first version of this table. The burn-time
bracket ran to a round 90 s, which for three Merlins is 82 t of propellant out of
a stage carrying 61 — so the search spent most of its time integrating *negative*
propellant mass, where `thrust/m` diverges through zero. The trajectories that
came back were not merely inaccurate, they were meaningless, and being
meaningless they could still bracket a root and be mistaken for an answer: four
attitudes in the middle of the useful range reported themselves unreachable. The
bracket now ends just short of a dry tank, and the dynamics refuse to thrust an
empty stage.

#### The turnaround is flown by the engines, not by the cold gas

This one modelling choice moves maximum downrange from 97 km to 61 km — from
outside the specified 59–70 km band to inside it — and it is worth spelling out
because the wrong model is the obvious one.

Out of the atmosphere the grid fins produce no torque, and with the engines off
neither does the gimbal. So the flip belongs to the RCS: two 500 N nozzles at the
40.5 m station against 6.2×10⁶ kg·m² give **0.31 °/s²**, and a bang-bang slew of
150° takes **44 s** — during which the stage travels another 49 km downrange.
That alone puts maximum downrange at 97 km. It is also slower than the real
vehicle, whose MECO-to-boostback gap on CRS-9 was 37 s *including stage
separation*, which is the clue that the model is wrong.

What actually happens is that the stage settles its propellant for 8 s, relights,
and lets the gimbal finish the turn. Three Merlins at 5° through a 6.9 m arm give
**15.7 °/s²**, fifty times the cold gas, so the same 150° takes **5.7 s**. The
burn and the turn are one manoeuvre, so the thrust direction slews *during* the
burn, and the solver integrates the fact that the thrust genuinely points the
wrong way for the first few seconds rather than assuming it away.

### The one donor patch

The specification predicted this and it was right: **every shipped scenario starts
with essentially zero horizontal velocity.** `scenario_init` gives `S_VX` and
`S_VY` a ±5 m/s dispersion and nothing else, so a stage still flying back towards
the pad at a couple of hundred metres a second could not be expressed at all.

`core/scenario.c` gains a `vx0` field on `ScenDef`, a `custom` scenario whose
initial conditions come from the command line, and a retrograde initial attitude
— because a stage arriving from a boostback is flying engines-first at some 9°
off vertical, well outside the ±2° every shipped scenario assumes. `core/main.c`
gains one `--custom` line in each of its two argument parsers.

```
booster-core --run --custom "h0=70000,vx0=-205,vz0=-1360,lateral=9083,prop0=30000,retro=1"
```

**It is behaviour-preserving, and that is checked rather than asserted.** Every
shipped scenario leaves `vx0` at zero and `retro` off, so the added terms are an
exact `+ 0.0` and a dead branch. Empirically: `--selftest` passes, and the
patched core reproduces the donor's own frozen 100-run Monte Carlo golden
(`goldens/mc/entry_s42_d012_baseline.txt`) **line for line** — 88/100 landed,
17 GOOD, 71 HARD, 12 CRASHED, same fault and crash-cause breakdown.

### What the donor will accept at the handover

Work item 5 asked to "check the donor accepts it and that its guidance engages
sensibly from there". It does not, at first, and the failure is instructive.

Handed a stage at 70 km, the donor's entry supervisor — which arms on a predicted
peak dynamic pressure and cuts on the same — gets it wrong in both directions
depending on how fast the stage arrives, and at no propellant load does it land.

**Too fast, and it over-burns.** At 1342 m/s the predictor keeps calling for
more burn, and three engines on a 60 t stage is 4.7 g, so at T+30 s of the donor
leg the stage is at 49 km *climbing at 204 m/s*, having spent 27 t. That is a
suicide burn at 50 km, not an entry burn.

**Too slow, and it under-burns — which turns out to be worse.** At 70 km the
dynamic pressure is 62 Pa, so the predicted peak falls below the cut threshold
while 20 t is still aboard and the burn stops early. That leaves a 55 t stage for
a landing burn flown on *one* Merlin, which nets 5.4 m/s² against gravity and
therefore needs 13 km to stop — and 13 km is exactly where the landing burn then
ignites, running the tank dry on the way down. Sweeping the propellant at that
handover, **every load from 26 t to 36 t crashes**, and more propellant makes it
monotonically worse: the ignition altitude climbs from 13.9 km to 17.1 km and the
impact speed from 57 m/s to 118.

The fix is not propellant, it is the handover altitude:

| interface | q there | landing burn from | result |
| --- | --- | --- | --- |
| 70.0 km | 62 Pa | 14.2 km | CRASHED, ran dry |
| 67.9 km | 84 Pa | 10.2 km | HARD, 3.50 m/s |
| **66.0 km** | **113 Pa** | **2.76 km** | **GOOD, 3.38 m/s, 7.6 m** |
| 64.0 km | 150 Pa | 2.76 km | HARD, 2.59 m/s |
| 62.0 km | 199 Pa | 2.69 km | HARD, 3.49 m/s |
| 58.0 km | 340 Pa | 2.69 km | HARD, 4.29 m/s |

Below 66 km the entry burn runs to its 7 t fuel floor instead of cutting on
dynamic pressure, and the landing burn then ignites at 2.7 km on a 33 t stage —
the regime the donor was built and tested in. **The handover is therefore at
65 km**, clear of the cliff between 68 and 66.

That costs nothing in validity. The specification's vacuum argument is made at
70 km, but it is an argument about dynamic pressure, not about a particular
altitude: at 65 km q is about 130 Pa, so drag on the returning stage is
0.03 m/s² — a few hundredths of a percent of the entry burn's thrust
acceleration, and two orders of magnitude below where the vacuum approximation
starts to matter. The task's "~70 km" is satisfied; what moved is which side of
the donor's own trigger the stage arrives on.

### The return is wind-limited, not fuel-limited

This is the most important number in the whole flight and it is not a trajectory
number at all.

The donor's `entry` scenario flies an 8 m/s surface wind with severe Dryden
turbulence (`w20 = 30` kt). Under it, this return lands or crashes **on the
propellant load alone**:

| handover propellant | 8 m/s, w20 30 | 3 m/s, w20 15 |
| --- | --- | --- |
| 29.1 t | CRASHED, 27.6 m/s | HARD, 4.18 m/s |
| 32 t | HARD, 2.76 m/s | HARD, 2.34 m/s |
| 34 t | HARD, 5.61 m/s | HARD, 2.85 m/s |

Nothing about the trajectory changed between those columns. That is the wind, and
it is the same architectural fragility this README already documents for a stage
that cannot hover: there is one chance to get the ignition point right, and a
gust near the deck spends it. RTLS makes the plan longer, so it does not improve
on its own.

The nominal flight is therefore flown at 3 m/s and moderate turbulence — the
environment the donor's own `terminal` scenario uses — and `meta.json` records
it. `--wind` and `--turb` put the rougher day back.

### Closing the loop on the pad

The boostback targets a **vacuum** ballistic impact point, because above the
entry interface that is the honest calculation. The real descent has an atmosphere, which
shortens the arc, so aiming at the pad lands the stage short of it. Rather than
model the donor's drag a second time, the aim point is corrected against the
donor itself: fly it, measure the touchdown miss, move the aim point, fly it
again. The relationship is very nearly one-to-one, so two or three passes
suffice. It is a shooting method with the donor as the plant, and it is why the
flight ends where it started rather than wherever the ballistics put it.

There is a second loop outside that one. The ascent has to arrive with enough
propellant for the return, but what the return costs depends on the mass it
arrives with — a fixed point, and it has to be posed carefully. Leaving the
flight path angle free let each pass land in a different basin, and the loop
oscillated (86.6 t, 87.1, 86.6, 87.7) with the boostback bill moving 4 t as γ
wandered between 43° and 50°. The first pass now picks the least-steering
trajectory and the rest hold that γ, solving for mass alone; and the iteration is
on the propellant *at the interface*, which is the quantity that matters. The
sensitivity between the two is not something to guess at: the rocket equation
says a kilogram at cutoff should deliver 0.63 kg to the interface, the measured
slope is 1.24 — because the ascent re-solves and the whole trajectory shifts
rather than just the mass — and a fixed gain of 1/0.63 therefore overshoots by
two and settles into a stable two-cycle. It is a secant now, which needs no
guess and converged in one pass.

### What this does not do

- **No Earth rotation and no cross-range.** A due-east launch from 28.5° starts
  with 408 m/s of eastward ground speed and picks up a Coriolis cross-range.
  Everything here is planar and non-rotating — so is the donor's frame, so
  adding it would have to stop at the handover anyway. It biases the boostback
  ΔV and the ground track; the aim-point loop absorbs the residual rather than
  accounting for it.
- **The ascent has no attitude dynamics.** It commands a thrust direction and
  the vehicle takes it. The boostback slew is rate-limited by real gimbal
  authority and the coast turn by real RCS authority, but neither is a closed
  attitude loop with an actuator model.
- **One flight, not a campaign.** This is the nominal trajectory. The donor
  leg can be dispersed (`--run N`), but there is no Monte Carlo over the whole
  profile, and given how sharply the landing responds to wind there should be.
- **The known architectural weakness is still there**, and RTLS makes it worse
  rather than better: a single open-loop plan plus a tracker, for a stage that
  cannot hover. Receding-horizon re-planning remains the answer and remains
  unwritten.

## How it fits together

`pdg` is the core, and it already agrees with itself: guidance, plant and
autopilot share one rigid-body model (`pdg/rocket.hpp`), so there are no
mismatched equations of motion to reconcile.

**Two levels, as they have to be.** SCvx solves the whole 6-DoF free-final-time
descent once, up front — 24 iterations, about half a second. That is nowhere
near fast enough for a control loop, so it is not in one: the plan is computed
once, and a PD tracking autopilot (`pdg::TrajectoryTracker`) flies it at 200 Hz,
feeding forward the reference attitude and gimbal command. `bridge/pdg_flight.cpp`
keeps that split intact.

**The flight log comes from inside the loop.** `RocketSim::fly` takes a command
callback and calls it exactly once per integrator step, at the step start —
a real controller tick. The bridge logs from there, so `flight.csv` is the
closed-loop flight and not a re-integration of it.

**Dispersions are real.** `--thrust-scale`, `--isp-scale`, `--misalign-deg` and
`--actuator-tau` perturb the *plant*, not the plan. The autopilot is not told.
Nominal touches down at 0.16 m lateral error and 0.98 m/s; a 0.3° engine cant
with 80 ms of actuator lag degrades that to a hard landing, which is the point
of having the knobs.

## Output format

`pdg_flight` writes three files into `--out`:

| file | contents |
| --- | --- |
| `reference.csv` | the SCvx plan, one row per node — omitted when the producer's guidance runs in the loop |
| `flight.csv` | the closed-loop flight at `--log-hz` (default 50) |
| `meta.json` | vehicle geometry, path limits, dispersions, touchdown statistics |

Both CSVs share one header: `t, m, r(3), v(3), q(4), omega(3), T_body(3), Tmag,
tilt_deg, speed`. Inertial frame is z-up with the pad at the origin; quaternions
are scalar-first and map body to inertial; `T` is in the body frame.

Both viewers read those three files and need nothing else — point them at any
output directory, or write your own producer against the same format.

## Two conventions worth knowing

**Rerun's 2D canvas is y-down.** Its 2D views use image coordinates, so
`land2d.py` logs altitude negated. Only that canvas is affected; every number in
the plots is in physical units, and the matplotlib viewer is y-up throughout.

**pdg's state is the centre of mass, and touchdown is `r_z <= 0`** — the CoM
reaching ground level. The model carries no vehicle height, so drawing the hull
around the CoM would bury the booster 12 m into the deck at the end. The viewers
offset the hull up the body axis by the engine arm instead, so the nozzle rests
on the pad exactly when the guidance says CoM altitude is zero. The trail and
every plotted value stay raw CoM.

## The other repositories

| repo | state | why |
| --- | --- | --- |
| `external/pdg` | **in use** — the core | builds clean with MSVC, all 5 unit tests pass |
| `rerun-sdk` | **works, not the default** | installed as a pip package into `.venv` (the monorepo is not needed). Its viewer needs a real GPU — see *Two renderers*. |
| `external/SCvx` | **runs** | `cd external/SCvx && ..\..\.venv\Scripts\python.exe freefinaltime.py` converges in 19 iterations. Kept as a cross-check on the C++ guidance, not wired into the pipeline. |
| `external/booster-lander-simulator` | **in use** — the separation-altitude path | US76 atmosphere, grid-fin and body aerodynamics, leg contact. Built CPU-only into `build-bls/`; driven through `bridge/bls_flight.py`. See *From stage separation*. |
| `external/RL_Rocket_6DOF` | **not set up** | pins `gym==0.21` and `numpy==1.21` and imports the private `scipy.spatial.transform.rotation` path, none of which exist on Python 3.12. It needs its own Dockerfile or a Python 3.9 environment. It is the alternative prototype path, not part of this pipeline. |

### If you want richer physics inside pdg


The donor's `core/` is the place to take it from — atmosphere, aerodynamics,
contact. **One donor only.** Its equations of motion and pdg's use different
quaternion and axis conventions; splicing the two plants together means weeks of
sign-hunting. Pick one source of dynamics and extend it. (Running both as
separate simulators, as this repo does, is not that: neither one's equations
ever touch the other's.)

## Requirements

- Windows, MSVC C++ toolchain (Visual Studio 2022 Build Tools, "Desktop
  development with C++"). `setup.ps1` prints the `winget` line if you need it.
- Python 3.10+. CMake and Ninja come from the virtualenv, so no separate install.

## Layout

```
docs/model.tex             the mathematical model as implemented, for review
docs/model-ru.tex          the full model in Russian, all flight segments
docs/task-rtls.md          the RTLS specification this repository now implements
bridge/pdg_flight.cpp      solve, fly closed loop, write telemetry
bridge/bls_flight.py       fly the donor core from separation, same telemetry
bridge/rtls_trajectory.py  US76, ascent, boostback and coast — our own dynamics
bridge/rtls_flight.py      the whole RTLS flight, stitched onto one clock
viz/flight_data.py         CSV loading, 2D projection, booster geometry
viz/land2d_mpl.py          real-time replay on the CPU (default renderer)
viz/land2d.py              real-time replay into Rerun (GPU)
CMakeLists.txt             adds external/pdg and builds the bridge
setup.ps1 / run.ps1        clone + venv / build + fly + view
patches/                   the donor patch, applied by setup.ps1
external/                  upstream repositories at pinned commits (not committed)
out/                       telemetry from the last run
out_rtls/                  the solo RTLS flight, pad to pad
```

