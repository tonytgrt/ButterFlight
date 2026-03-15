# Task 2 Implementation Plan — Butterfly Skeleton Rig & Maneuvering Functions

---

## Overview

Task 2 implements `BFWingModel` and the joint-rotation applicator — the kinematic
backbone that drives all five maneuvering angles (Eq. 1) with sigmoid-modulated
frequency and amplitude (Eqs. 2–3). By the end of this task, the butterfly rig
should animate plausibly when scrubbing the timeline, even before aerodynamic
forces are wired up.

---

## What Cecilia Has Already Done

Before starting, note that Subtask 2.1 is **effectively complete**:

| File                        | What's there                                                                                                                                                                                                                               |
| --------------------------- | ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------ |
| `src/BFState.h`           | `BFJointId` enum (6 joints), `BFSkeleton` struct (MDagPath per joint), `BFManeuverAngles` struct (5 angles), `BFState` struct (position, velocity, phase, flapCycle, frequency, amplitude, sliding-window history vectors)         |
| `src/BFSimulateCmd.h/cpp` | `readSkeleton()` — resolves `BF_thorax` root, traverses hierarchy with `MItDag`, matches all 6 expected child joints, returns `BFSkeleton`. `doIt()` parses `-rig` flag and calls `readSkeleton`. Undo/redo stubs in place. |
| `src/BFWingModel.h/cpp`   | **Empty stubs** — these are yours to fill in.                                                                                                                                                                                       |

You should **build on** the existing `BFState` and `BFManeuverAngles` structs
and the already-resolved `BFSkeleton` from `readSkeleton()`.

---

## Subtask 2.1 — BFState Struct and Joint Enum *(DONE)*

Already implemented by Cecilia in `BFState.h`. No further work needed.

If you find you need additional fields later (e.g., per-angle frequency/amplitude
arrays instead of single scalars), extend `BFState` in place rather than creating
a new struct.

---

## Subtask 2.2 — Maneuvering Function (Eqs. 1–3)

**Goal:** Implement `BFWingModel` so that given the current velocity and time,
it computes all five maneuvering angles.

### 2.2.1 — Define parameter storage in `BFWingModel.h`

Create a struct (or use class members) to hold the per-angle parameters from
Table 3 of the paper:

```
Per-angle params (* = beta, gamma, zeta, psi, phi):
  - R_f*        frequency range (Hz)      — e.g. gamma: 0–11
  - R_thetaA*   amplitude range (degrees) — e.g. gamma: 0–150
  - phi_p*      phase offset (degrees)    — e.g. phi: -180
  - phi_m*      mean angle (degrees)      — e.g. gamma: 10
```

Recommended: a small `BFAngleParams` struct with fields `freqRange`,
`ampRange`, `phaseOffset`, `meanAngle`, and an array of 5 of them inside
`BFWingModel` initialized to the Monarch defaults from the design doc Table 1.

### 2.2.2 — Implement the sigmoid modifiers (Eqs. 2–3)

Two helper functions that return frequency and amplitude as a function of speed:

```
f*(u) = R_f* / (1 + exp(-16 * (|u| / |u_max| - 0.5)))
phiA*(u) = R_thetaA* / (1 + exp(-16 * (|u| / |u_max| - 0.5)))
```

- `|u|` = current speed (`velocity.length()`)
- `|u_max|` = maximum flight speed (parameter, default ~2.0 m/s for Monarch)
- The sigmoid saturates at `R_f*` when speed is high and approaches 0 when
  speed is near zero.

### 2.2.3 — Implement the maneuvering function (Eq. 1)

```
theta*(t) = phiA*(u) * cos(2*pi*f*(u)*t + phi_p*) + phi_m*
```

For each of the five angles, evaluate this expression using:

- The sigmoid-modulated amplitude and frequency from 2.2.2
- The phase accumulator `BFState::phase` (advanced by `dt` each frame)
- The per-angle phase offset and mean from the parameter table

Key details:

- **Frequency and amplitude are held constant within one flapping cycle**
  (from downstroke start `t0` to next downstroke `t1`). They are only
  recalculated at cycle boundaries.
- Detect cycle boundary: when `phase` crosses `1.0 / frequency`, reset it
  and increment `BFState::flapCycle`.
- At cycle boundary, recompute frequency and amplitude from current velocity
  via the sigmoid and store them in `BFState::frequency` / `amplitude`
  (later, Task 5 will smooth them with the sliding-window).

### 2.2.4 — Public API for `BFWingModel`

```cpp
class BFWingModel {
public:
    // Initialize with Monarch defaults (or accept a species preset)
    BFWingModel();

    // Compute maneuvering angles for this frame.
    // Updates state.angles, state.phase, state.flapCycle.
    void update(BFState& state, double dt);

    // Maximum flight speed (needed by sigmoid)
    double maxSpeed = 2.0;

private:
    BFAngleParams params[5]; // one per maneuvering angle
    double evalSigmoid(double speed, double range) const;
};
```

Call site in `BFSimulateCmd::doIt()` will be inside the per-frame loop:

```cpp
wingModel.update(m_state, dt);
```

---

## Subtask 2.3 — Joint-Rotation Applicator

**Goal:** After `BFWingModel::update` fills `BFState::angles`, apply those
angles as local rotations on the resolved Maya joints.

### 2.3.1 — Rotation mapping

Map each maneuvering angle to a joint and rotation axis:

| Angle                            | Joint                                | Axis        | Notes                                    |
| -------------------------------- | ------------------------------------ | ----------- | ---------------------------------------- |
| `thetaBeta` (thorax pitch)     | `BF_thorax`                        | X (pitch)   | Rotates the entire body                  |
| `thetaGamma` (forewing flap)   | `BF_forewing_L`, `BF_forewing_R` | Z (flap)    | Bilateral symmetric; negate for one side |
| `thetaZeta` (forewing feather) | `BF_forewing_L`, `BF_forewing_R` | X (feather) | Same value both sides                    |
| `thetaPsi` (forewing sweep)    | `BF_forewing_L`, `BF_forewing_R` | Y (sweep)   | Negate for one side                      |
| `thetaPhi` (abdomen rotation)  | `BF_abdomen`                       | X (pitch)   | Counter-phase to thorax                  |

Hindwings (`BF_hindwing_L`, `BF_hindwing_R`) track the forewing flap angle
`thetaGamma` only (1 DOF). Apply the same flap value on the Z axis.

### 2.3.2 — Implementation approach

Use `MFnTransform` on each joint's `MDagPath` from `BFSkeleton::joints[]`:

```cpp
#include <maya/MFnTransform.h>
#include <maya/MEulerRotation.h>

static void applyAngles(const BFSkeleton& skel, const BFManeuverAngles& angles)
{
    // Thorax pitch
    MFnTransform thoraxFn(skel.joints[kThorax]);
    thoraxFn.setRotation(MEulerRotation(
        deg2rad(angles.thetaBeta), 0.0, 0.0,
        MEulerRotation::kXYZ));

    // Forewing L — flap(Z), feather(X), sweep(Y)
    MFnTransform fwlFn(skel.joints[kForewingL]);
    fwlFn.setRotation(MEulerRotation(
        deg2rad(angles.thetaZeta),
        deg2rad(angles.thetaPsi),
        deg2rad(angles.thetaGamma),
        MEulerRotation::kXYZ));

    // Forewing R — mirror flap and sweep
    MFnTransform fwrFn(skel.joints[kForewingR]);
    fwrFn.setRotation(MEulerRotation(
        deg2rad(angles.thetaZeta),
        deg2rad(-angles.thetaPsi),
        deg2rad(-angles.thetaGamma),
        MEulerRotation::kXYZ));

    // Hindwings track forewing flap only
    MFnTransform hwlFn(skel.joints[kHindwingL]);
    hwlFn.setRotation(MEulerRotation(0, 0, deg2rad(angles.thetaGamma),
        MEulerRotation::kXYZ));

    MFnTransform hwrFn(skel.joints[kHindwingR]);
    hwrFn.setRotation(MEulerRotation(0, 0, deg2rad(-angles.thetaGamma),
        MEulerRotation::kXYZ));

    // Abdomen
    MFnTransform abdFn(skel.joints[kAbdomen]);
    abdFn.setRotation(MEulerRotation(
        deg2rad(angles.thetaPhi), 0.0, 0.0,
        MEulerRotation::kXYZ));
}
```

**Important:** The exact axis mapping (X/Y/Z) depends on how the FBX rig's
joints are oriented. After importing `Butterfly.fbx`, inspect the joint
orient values in the Attribute Editor and adjust the axis assignments above
accordingly. The design doc notes the FBX joints may need renaming to `BF_`
convention — that's a prerequisite.

### 2.3.3 — Keyframe writing

For the simulation bake, use `MFnAnimCurve::addKey` on each joint's rotation
attributes (`rotateX`, `rotateY`, `rotateZ`) at each frame. Wrap this in a
helper:

```cpp
static void writeRotationKey(const MDagPath& joint,
                             const MEulerRotation& rot,
                             MTime time);
```

This will be called from the main simulation loop in `doIt()` after
`applyAngles`.

---

## Subtask 2.4 — Visual Verification

### Test plan

1. **Import rig:** Load `Butterfly.fbx`, rename joints to `BF_` convention.
2. **Run 60-frame test:** Call `bfSimulate -rig BF_thorax` with a hardcoded
   60-frame loop, zero velocity (hovering). Verify:
   - Forewings flap symmetrically on the Z axis
   - Hindwings follow forewing flap in phase
   - Abdomen rotates counter-phase (-180 deg offset) to thorax pitch
   - Feather and sweep angles produce subtle secondary motion
3. **Velocity ramp test:** Set initial velocity to (1, 0, 0) m/s and observe
   frequency and amplitude increase via the sigmoid. At v=0 the wings should
   barely move; at v=u_max they should reach full range.
4. **Check parameter table:** Print computed f* and phiA* to the Maya script
   editor at each cycle boundary. Verify they match expected sigmoid curves.

### What "done" looks like

- Scrubbing the Maya timeline shows plausible Monarch butterfly wing motion
- All five angles animate with correct phase relationships (Figure 5 in paper)
- No popping, discontinuities, or joint flipping
- Frequency/amplitude respond smoothly to velocity changes

---

## Integration with `BFSimulateCmd::doIt()`

The simulation loop structure (to be added to `doIt()` after skeleton validation):

```cpp
BFWingModel wingModel;  // Monarch defaults
double fps = 24.0;      // from -fps flag or MAnimControl
double dt = 1.0 / fps;
int startFrame = 1;     // from -startFrame flag
int duration = 60;      // from -duration flag

for (int f = startFrame; f < startFrame + duration; ++f) {
    // 1. Evaluate maneuvering angles (Task 2)
    wingModel.update(m_state, dt);

    // 2. Apply joint rotations (Task 2)
    applyAngles(m_state.skeleton, m_state.angles);

    // 3. Write keyframes (Task 2)
    MTime time(f, MTime::uiUnit());
    writeRotationKeys(m_state.skeleton, m_state.angles, time);

    // --- Tasks 3–5 will add force computation and velocity integration here ---
}
```

---

## Files to modify/create

| File                      | Action                                                                             |
| ------------------------- | ---------------------------------------------------------------------------------- |
| `src/BFWingModel.h`     | Fill in:`BFAngleParams` struct, `BFWingModel` class declaration                |
| `src/BFWingModel.cpp`   | Fill in: sigmoid helpers,`update()`, Monarch default init                        |
| `src/BFSimulateCmd.cpp` | Extend `doIt()` with simulation loop, joint-rotation applicator, keyframe writer |
| `src/BFSimulateCmd.h`   | Add new flag constants if needed (`-fps`, `-startFrame`, `-duration`)        |
| `src/BFState.h`         | Possibly extend `BFState` with per-angle freq/amp arrays if needed               |

---

## Dependencies and blockers

- **FBX joint orientation:** The axis mapping in the rotation applicator (2.3.1)
  cannot be finalized until you inspect the imported `Butterfly.fbx` joint
  orientations in Maya. Plan to adjust after import.
- **No dependency on Tasks 3–5:** The maneuvering function works standalone with
  a fixed or zero velocity. Aerodynamic forces (Task 3), curl noise (Task 4),
  and the maneuvering controller (Task 5) will feed velocity back into
  `BFWingModel::update` later, but Task 2 can be completed and tested
  independently.
