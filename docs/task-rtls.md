# Task: solo first-stage test flight with return to the launch site

**Status:** specification, not started. Written to be picked up in a fresh session.

## The two things that define this task

**1. The whole flight is flown by the first stage alone.** There is no second
stage and no payload — not carried, not simulated, not jettisoned. One vehicle
lifts off, flies the whole profile and lands. Treat it as a full-scale test
flight of the stage itself.

**2. It must perform the return manoeuvre and land back at the launch site.**
RTLS, on the pad it took off from. Not a droneship, not a downrange landing, not
a straight-up-and-down hop. The stage has to build downrange velocity, reverse
it under power, fly back and land where it started. If the boostback manoeuvre
is not there, the task is not done.

## Goal

Simulate that flight end to end and replay it in the existing 2D real-time
viewer: liftoff, ascent, engine cutoff, **boostback burn**, ballistic coast
through apogee, entry burn, aerodynamic descent under grid fins, landing burn,
touchdown on the pad.

Vehicle parameters come from the real **CRS-9** flight — an RTLS mission with
published telemetry — as set out under *Anchor numbers*.

## Scope

**In:** one stage, one continuous flight, liftoff to touchdown, on a plant that
has an atmosphere.

**Out:** second stage, orbit insertion, payload, anything about turnaround.

### What "solo" does to the ascent

A stage flying without a 107.5 t upper stack on top is a different vehicle from
the CRS-9 first stage. Its ascent will not match CRS-9's telemetry and should not
be expected to. So the ascent needs a defining condition, and the recommended one
is:

> **Fly the ascent so that engine cutoff reaches the CRS-9 MECO state —
> 60.4 km at 1579 m/s — with a downrange distance near 35 km.**

That keeps the *return* problem the real one, which is the point of the exercise,
and keeps CRS-9's return telemetry usable as a check. How much propellant that
takes, and what pitch programme gets there, falls out of the solution and will
differ from CRS-9. Say so in the results rather than presenting the ascent as a
CRS-9 reproduction.

### The nose

Flying solo, the stage has an open interstage where the second stage used to be,
so for the ascent **assume a nose fairing is fitted**. It matters for ascent drag
and nothing else — pick a reasonable cone, state the drag area used, and move on.
The user has flagged this as not critical.

It matters even less on the way back, and it is worth being explicit about why: a
returning booster flies **engines-first**, so the nose cone is at the *trailing*
end during entry and descent. Return aerodynamics are set by the engine end, the
body and the grid fins, exactly as on a normal booster return. Do not let the
fairing complicate the descent model.

## Anchor numbers

### CRS-9 telemetry (measured, from launch webcast reconstruction)

| quantity | value |
| --- | --- |
| MECO | T+143 s |
| MECO altitude | 60.437 km |
| MECO velocity | 1578.9 m/s |
| stage 1 apogee | 128.70 km |
| recovery | RTLS, Landing Zone 1 |
| payload | 2257 kg to ISS |

Source: <https://github.com/shahar603/Telemetry-Data/tree/master/SpaceX%20CRS-9>.
The same repository has per-second altitude, velocity, vertical and horizontal
velocity, downrange distance and dynamic pressure.

On a solo stage there is no staging event, so read "MECO" here as **engine
cutoff** — the state the ascent is asked to reach, not an event we reproduce.

### Vehicle (published)

| quantity | value |
| --- | --- |
| stage 1 gross | 433.1 t |
| stage 1 empty | 22.2 t (bare; with legs and grid fins nearer 25.6 t) |
| propellant | 410.9 t (287.4 LOX + 123.5 RP-1) |
| Merlin 1D, sea level | 845 kN each, 7607 kN for nine |
| Isp | 282 s sea level, 311 s vacuum |
| diameter | 3.66 m; length 42.6 m without the interstage |

### RTLS profile (mission reports and trajectory reconstructions)

| event | value | confidence |
| --- | --- | --- |
| downrange at separation | ~35 km | moderate |
| maximum downrange | 59–70 km | good (59 km measured on SAOCOM-1A) |
| boostback burn | ~T+3:00, ~30 s, 3 engines, retrograde | good |
| apogee | 118–170 km across missions | good |
| entry burn | 70 → 40 km, 3 engines | good |
| landing burn | from ~8 km, 1 engine | moderate — sources differ, some say 2–3 km |

### Derived budget (computed, self-consistent — verify, do not trust blindly)

The boostback requirement follows from the cutoff state and is therefore the
same for a solo stage as for the real one. Horizontal velocity at cutoff is
1210–1370 m/s for a flight path angle of 30–40°. Cancelling it and adding
170–230 m/s of return velocity (60–70 km in ~300 s of coast) puts the boostback
at **1400–1600 m/s**.

What *does* differ is the propellant that buys: it depends on the mass at
cutoff, which for a solo stage is an output of the ascent, not a known. For
scale, burning 30 t from an 80 t stage at Isp 311 gives 1433 m/s in 32.7 s on
three engines — the figure that closes the loop for the real CRS-9 booster. Per
burn on that vehicle: boostback ~30 t, entry ~20 t, landing ~4.5 t, total ~55 t,
consistent with the ~56 t quoted online for a whole return. A lighter solo stage
needs proportionally less for the same dV; recompute rather than copying.

## Architecture: the split at 70 km

**The boostback and the coast are a vacuum problem.** Measured against US76:

| altitude | dynamic pressure at 1579 m/s | drag on an 80 t stage |
| --- | --- | --- |
| 60 km | 386 Pa | 0.025 m/s² |
| 80 km | 23 Pa | 0.0015 m/s² |
| 100 km | 0.7 Pa | ~0 |

Drag there is 0.2 % of thrust acceleration. So thrust-and-gravity dynamics are
physically valid above roughly 70 km, and the atmosphere only has to be right on
the way back down.

That gives the division of labour:

```
liftoff ──► MECO ──► boostback ──► coast ──► 70 km   │   70 km ──► entry ──► aero ──► landing
            our own code, vacuum-valid dynamics       │   donor core, US76 + aero + contact
```

### Why this shape (option C of four considered)

- **Extending pdg to cover everything** cannot work: pdg has no atmosphere at
  all, and the descent below 70 km is aerodynamic.
- **Forking the donor and adding a boostback phase** puts everything in one
  place but is the largest change, ends the "nothing in `external/` is modified"
  rule, and requires writing new guidance inside someone else's architecture.
- **Writing a new plant with atmosphere** duplicates what the donor already has
  and has been tested against Monte Carlo.
- **This option** keeps the fork surface to a single entry point, leaves the
  tested atmospheric physics untouched, and puts the genuinely new work — the
  boostback and its guidance — in our own code, under our own tests.

### The one patch the donor needs

`core/scenario.c`, function `scenario_init`, builds the whole initial state from
a `ScenDef` record: `{name, h0, vz0, lateral, prop0, tilt0, u_ref, deck_z, w20,
fins}`. Adding a `custom` scenario fed from the command line is on the order of
20–30 lines in that one file.

**One field is missing.** Every shipped scenario starts with essentially zero
horizontal velocity — `S_VX` and `S_VY` get only a ±5 m/s dispersion. A stage
handed over at the entry interface has real horizontal velocity, so `ScenDef`
needs a `vx0` (and the CLI needs to reach it). Check this before estimating.

Once patched, the donor is driven exactly as now, through
`bridge/bls_flight.py`, which already converts its log into the viewer's
telemetry format.

## The reference notebook

`references/falcon9_to_orbit.ipynb` (Devakumar Thammisetty, 2023) solves a
three-phase optimal control problem with **mpopt** — pseudospectral collocation
over CasADi:

- **phase 0** — first stage ascent, nine engines, t 0 → 131.4 s
- **phase 1** — second stage to orbit, terminal constraint on five orbital
  elements
- **phase 2** — **booster return**, t 131.4 → 569.7 s

**Mind the label.** The markdown above the plotting cell says "Booster return to
Ship", and that is the only occurrence of the word in the notebook — there is no
droneship coordinate anywhere in it. What the code actually does is constrain the
booster to `x0`, the launch site at latitude 28.5°, in both position and
velocity. The prose says droneship; the implementation is return-to-launch-site.

This matters less than it looks. The landing target is a single constant, used
in `terminal_constraints2` and again in the glide-slope term of
`path_constraints2`. RTLS versus droneship is that constant and nothing else, so
the formulation carries over either way — but do not take the notebook's own
description as a statement of what it computes.

Phase 2 is directly relevant: it already poses the return as an optimal control
problem with

- terminal constraint: match the target position *and* velocity exactly
  (`terminal_constraints2` — six equality constraints against `x0`),
- a dynamic-pressure path constraint, `q_max = 80 kPa`,
- a glide-slope cone of 80° referenced to the same target point,
- ECI Cartesian state, spherical-Earth gravity, exponential atmosphere.

**Take from it:** the problem formulation — state, phase structure, the way the
return is posed as a boundary-value problem, the constraint set. That is the
part worth reusing, and it answers the question this project has not yet
answered, namely what the boostback guidance is actually solving.

**Do not take its parameters.** They are approximate where CRS-9 is exact:

| reference | reality | note |
| --- | --- | --- |
| Isp 340 s everywhere | 282 s SL, 311 s vac | ~10 % error on every burn |
| thrust 934 kN per engine | 845 kN SL, 932 kN vac | the reference uses the vacuum figure at sea level |
| `Sa = 4π` = 12.57 m² | body cross-section 10.52 m² | — |
| `Cd = 0.5` | the expert review suggested ≈1.0 | `Cd·S` differs by 1.7× — settle this |
| separation at t 131.4 s | CRS-9 MECO at T+143 s | reference value is an initial guess |
| booster reserve ~78 t | ~55 t derived, see above | follows from the Isp error |

Also note what the reference is and is not: it computes an **open-loop optimal
trajectory**. It is not a closed-loop 6-DoF simulation, has no attitude dynamics,
no actuator model and no contact model. It gives the plan, not the flight.

## Work items

1. **Settle `Cd·S`.** The reference notebook and the expert review disagree by a
   factor of 1.7 — see the table above. Everything downstream of the entry
   interface depends on it.
2. **Ascent, solo stage.** Fly it to the cutoff condition given under *What
   "solo" does to the ascent*: 60.4 km at 1579 m/s, ~35 km downrange, with a
   nose fairing fitted. The reference's phase 0 is the template for the
   formulation; its parameters are not. Output: the cutoff state — position,
   velocity, mass, attitude — and the propellant actually spent getting there,
   which is a result, not an input.
3. **Boostback guidance.** The new piece, and the one with real design content.
   It is a *targeting* problem, not a landing problem: null the downrange
   velocity and set up a ballistic arc whose impact point is the pad. The SCvx
   formulation in `pdg` does not solve this — its terminal condition is "at the
   origin with near-zero velocity". Either adapt the reference's phase-2
   formulation, which already poses exactly this as a boundary-value problem, or
   pose your own onto the entry-interface state.
4. **Coast.** Ballistic, vacuum. Trivial to integrate. Apogee should land in the
   118–170 km band and near 128.7 km if the cutoff state matches CRS-9's.
5. **Donor patch and handover.** As above: a `custom` scenario fed from the
   command line, plus the missing horizontal-velocity field. Define the handover
   state at ~70 km, check the donor accepts it and that its guidance engages
   sensibly from there rather than assuming a near-vertical arrival.
6. **Stitch the telemetry.** Every segment writes the existing format
   (`flight.csv` + `meta.json`, see the Output format section of the README), so
   the viewer replays the whole flight on one timeline. Phases are already
   supported — `meta.json` carries a `phases` list and the viewer shows the
   current phase name.
7. **Camera.** The flight spans ~130 km of altitude and ~70 km of ground, and
   unlike everything built so far it starts at zero altitude and climbs. The
   adaptive camera zooms *in* on a descent below a trigger altitude; it has never
   seen a flight that begins on the pad. Check it, and expect to extend it.

## Validation

Unusually well served for a project like this — use it.

- **Ascent:** CRS-9's per-second telemetry is *not* a check on our ascent, since
  a solo stage flies a different profile. What it does check is the **cutoff
  state**: 60.4 km, 1579 m/s, ~35 km downrange. Hit that and the rest of the
  comparison becomes meaningful.
- **Boostback and coast:** apogee near 128.70 km, maximum downrange 59–70 km,
  and the horizontal velocity reversed rather than merely reduced. The last one
  is the actual test that the return manoeuvre happened.
- **Entry and landing:** the donor grades these itself (PERFECT ≤ 2 m/s,
  GOOD ≤ 4, HARD ≤ 6, crash above); its `entry` scenario gives a baseline of
  3.4 m/s mean over 40 dispersed runs.
- **Propellant:** the three return burns should total near 55 t — boostback ~30,
  entry ~20, landing ~4.5 — and the stage must not run dry. The ascent figure is
  separate and is an output of work item 2.
- **The whole flight:** it ends on the pad it started from. Lateral error at
  touchdown is measured against the launch point, not against wherever the stage
  happened to come down.

## What already exists — do not rebuild it

- `bridge/pdg_flight.cpp` — SCvx guidance flown closed loop, with an `f9`
  preset (30.1 t wet, 25.6 t dry, 845 kN, 5 ° gimbal, cannot hover), a thrust
  margin, retuned attitude gains and engine cutoff. Landing burn only.
- `bridge/bls_flight.py` — runs the donor core and converts its log to the
  viewer format, including a reconstructed attitude with a physically derived
  slew limiter.
- `viz/` — the 2D real-time viewer, CPU-rendered (the machine's GPU is virtual
  and Rerun comes up transparent on it), with an adaptive camera and a
  Falcon-proportioned stage drawing.
- `docs/model.tex` — the mathematical model as implemented, for review. **It is
  one revision behind:** the `f9` preset, thrust margin, retuned gains and
  engine cutoff are not in it yet.

## Standing constraints

- **Do not modify `external/` except the one donor patch above**, and say so
  explicitly in the README when it happens — the current text claims nothing in
  `external/` is modified.
- The expert review of the model (see `docs/model.tex` section 7 and the review
  itself) found real defects that bear on this work and are still open: the
  reconstruction's mass model puts propellant in the wrong tank and applies the
  parallel-axis theorem from the wrong point, pdg's plant holds inertia and the
  thrust arm constant, and the autopilot's gimbal saturation exceeds the
  guidance envelope. None of these block starting, but the boostback is a
  large-attitude manoeuvre and the inertia model matters more there than it did
  for a landing burn.
- The known architectural weakness found in F9-preset testing: a single
  open-loop plan plus a PD tracker is fragile for a stage that cannot hover
  (±2 % thrust loses the landing). RTLS makes the plan longer, so this will not
  improve on its own. Receding-horizon re-planning is the known answer.
