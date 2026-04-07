# ButterFlight Implementation Log

## 2026-03-05 — Skeleton Reading & Plugin Bootstrap

### Files Implemented

#### `src/BFState.h`

- `BFJointNames` namespace: six string constants for the expected joint naming convention (`BF_thorax`, `BF_forewing_L`, `BF_forewing_R`, `BF_hindwing_L`, `BF_hindwing_R`, `BF_abdomen`)
- `BFJointId` enum: integer indices for indexing into joint arrays (kThorax through kAbdomen, plus kNumJoints sentinel)
- `BFSkeleton` struct: holds an `MDagPath` per joint and a `valid` flag
- `BFManeuverAngles` struct: the five maneuvering angles from Chen et al. 2022 Eq. 1 (thetaBeta, thetaGamma, thetaZeta, thetaPsi, thetaPhi)
- `BFState` struct: per-butterfly simulation state aggregating skeleton, world-space position/velocity, maneuvering angles, phase accumulator, flap cycle counter, smoothed frequency/amplitude, and sliding-window history vectors

#### `src/BFSimulateCmd.h`

- `BFSimulateCmd` class (subclass of `MPxCommand`)
- Declares `doIt`, `undoIt`, `redoIt`, `isUndoable`, `creator`, `newSyntax`
- Declares `static MStatus readSkeleton(const MString& rootJointName, BFSkeleton& outSkeleton)` — the skeleton reading function
- Stores a `BFState m_state` member for undo support

#### `src/BFSimulateCmd.cpp`

- Command name registered as `"bfSimulate"`
- `newSyntax()`: defines `-rig` (string) and `-mode` (long) flags; remaining flags marked TODO
- `readSkeleton()` implementation:
  1. Finds the root joint by name via `MSelectionList::add` + `getDagPath`
  2. Verifies the node is a `kJoint`
  3. Stores the root as the thorax joint
  4. Uses `MItDag` (depth-first, `MFn::kJoint` filter) to traverse all descendant joints
  5. Matches each descendant's name against the five expected child joints
  6. Early-exits when all joints are found
  7. Reports per-joint warnings for missing joints; returns `MS::kFailure` if skeleton is incomplete
- `doIt()`: parses `-rig` flag, calls `readSkeleton`, rest is TODO
- `undoIt()` / `redoIt()`: stubs for future keyframe cache restore

#### `src/PluginMain.cpp`

- `initializePlugin()`: registers `bfSimulate` command via `MFnPlugin::registerCommand`
- `uninitializePlugin()`: deregisters the command

---

## 2026-03-05 — Aerodynamics Force Model

### Approach

Simplified flat-plate model: each wing is a single panel whose orientation comes from the wing joint's world-space rotation. Uses the exact empirical coefficient polynomials from Chen et al. 2022.

### Files Modified

#### `src/BFState.h` (addition)

- `BFWingParams` struct: per-species wing geometry in SI units
  - `forewingArea` (m², default 6.5 cm² for Monarch)
  - `hindwingArea` (m², default 6.5 cm² for Monarch)
  - `meanChordRadius` (m, default 2.5 cm — mean distance from joint to center of pressure)

### Files Implemented

#### `src/BFAerodynamics.h`

- `BFAerodynamics` class with all-static methods
- `liftCoeff(alpha)` — C_l polynomial (Eq. 5)
- `dragCoeff(alpha)` — C_d polynomial (Eq. 6)
- `panelForce(normal, area, airVelocity)` — core per-panel lift+drag computation
- `getWingNormal(wingJoint)` — extracts wing panel normal from joint's world Y axis
- `getWingSpanDir(wingJoint)` — extracts span direction from joint's world X axis
- `computeWingForce(wingJoint, bodyVelocity, wind, flapOmega, wingArea, meanRadius)` — flat-plate force for one wing
- `computeTotalForce(skeleton, bodyVelocity, wind, flapOmega, wingParams)` — all four wings summed

#### `src/BFAerodynamics.cpp`

- `liftCoeff()`: C_l(α) = −0.0095953α² + 0.090635α − 0.34182
- `dragCoeff()`: C_d(α) = −0.0000079518α³ + 0.0011527α² + 0.0063148α + 0.51127
- `panelForce()`:
  1. Decomposes air velocity into normal (V_n) and tangential (V_t) components
  2. Computes angle of attack α = atan2(|V_n|, |V_t|) in radians
  3. Computes dynamic pressure q = 0.5 * ρ * |V|²
  4. Lift direction = component of panel normal perpendicular to airflow
  5. Drag direction = opposite to airflow
  6. Returns q * A * (C_l * liftDir + C_d * dragDir)
- `getWingNormal()` / `getWingSpanDir()`: extract world-space axes from joint's inclusive matrix
- `computeWingForce()`: computes air velocity at wing center (body + flap rotation + wind), calls panelForce()
- `computeTotalForce()`: sums forces from forewing L/R and hindwing L/R

### Files Not Yet Implemented (still empty stubs)

- `src/BFWingModel.h` / `src/BFWingModel.cpp` — maneuvering angle evaluation (Eqs. 1–3)
- `src/BFCurlNoise.h` / `src/BFCurlNoise.cpp` — curl-noise vortex force (Eq. 7)
- `src/BFManeuverController.h` / `src/BFManeuverController.cpp` — velocity integration and sliding-window smoother (Eqs. 8–12)

### Notes

- The MEL UI (`src/mel/butterFlight_ui.mel`) was already complete before this session
- The FBX model (`Butterfly.fbx`) joints may need renaming to match the `BF_` convention after import

---

## 2026-03-15 — Task 2 Prep: Skeleton Rig Fix-up & Bug Fixes (Yiding Tian)

### Planning

#### `doc/Others/skeleton_plan.md`

- Created implementation plan for Task 2 (Butterfly Skeleton Rig & Maneuvering Functions)
- Covers Subtasks 2.1–2.4: BFState review, maneuvering function (Eqs. 1–3), joint-rotation applicator, and visual verification
- Includes rotation axis mapping table, `BFWingModel` class API design, simulation loop integration outline, and verification test plan
- Notes that Subtask 2.1 (BFState struct and joint enum) was already completed by Cecilia

### Butterfly Model Rig Rework

#### Skeleton hierarchy cleanup (`model/Butterfly.ma`)

- **Problem:** Left wing joints (`BF_forewing_L`, `BF_hindwing_L`) were parented under intermediate transform nodes (`transform1`, `transform2`) with scaleZ = -1 for mirroring. This non-standard hierarchy would have caused issues with the simulation's joint traversal and rotation applicator.
- **Attempted fix:** `parent BF_forewing_L BF_thorax` — Maya auto-inserted a new intermediate `transform3` to compensate for the negative scale, making the problem worse.
- **Solution:** Deleted the left wing joints and their meshes entirely, then re-created them using Maya's `mirrorJoint` command from the right wing joints:
  ```mel
  select BF_forewing_R;
  mirrorJoint -mirrorYZ -mirrorBehavior -searchReplace "R" "L";
  select BF_hindwing_R;
  mirrorJoint -mirrorYZ -mirrorBehavior -searchReplace "R" "L";
  ```
- Left wing meshes were duplicated from right side, mirrored, and re-skinned.

#### Skin binding fix

- **Problem:** After re-creating left wing joints and meshes, binding one wing caused the other wing mesh to deform incorrectly — both meshes were being influenced by both joints.
- **Cause:** Default Bind Skin uses "Joint hierarchy" mode, which binds the mesh to all joints in the hierarchy, not just the selected one.
- **Fix:** Changed Bind Skin option **"Bind to"** from "Joint hierarchy" to **"Selected joints"**, then bound each wing mesh individually to its own joint.

#### Final skeleton hierarchy (clean)

```
Butterfly
└── BF_body
    └── BF_thorax
        ├── BF_head
        ├── BF_forewing_L    (direct child, mirrorJoint from R)
        ├── BF_forewing_R
        ├── BF_hindwing_L    (direct child, mirrorJoint from R)
        ├── BF_hindwing_R
        ├── BF_foreleg_L/R
        ├── BF_midleg_L/R
        ├── BF_hindleg_L/R
    └── BF_abdomen
```

### Bug Fixes

#### `src/BFSimulateCmd.cpp`

1. **Flag parsing crash — `newSyntax()` vs `syntax()`:**

   - Changed `MArgDatabase argData(newSyntax(), args, &status)` to use the inherited `syntax()` method, which returns the syntax registered with Maya rather than creating a detached temporary.
2. **Invalid flag name `-rig` (root cause of "Invalid flag '-rig'" error):**

   - Maya requires long flag names to be at least 4 characters (excluding the `-` prefix). The long flag `-rig` (3 chars) fell within the short flag range (1–3 chars) and was not recognized.
   - Renamed long flag from `-rig` to `-rigRoot`.
   - Command usage is now: `bfSimulate -r "BF_body"` or `bfSimulate -rigRoot "BF_body"`.

- All aerodynamics physics use SI units (m, kg, s); conversion from cm²/g happens upstream
- Bilateral symmetry: same flapOmega for all four wings (design doc assumption)

---

## 2026-03-19 — Subtask 2.2: Maneuvering Function (Yiding Tian)

### Overview

Implemented the parametric maneuvering function from Chen et al. 2022 (Eqs. 1–3, Table 3). Given the butterfly's current velocity and accumulated phase, the wing model now computes all five maneuvering angles per frame with sigmoid-modulated frequency and amplitude.

### Files Modified

#### `src/BFState.h`

- Added per-angle frequency and amplitude arrays (`perAngleFreq[5]`, `perAngleAmp[5]`) to `BFState`, initialized to Monarch hovering defaults
- Changed `frequency` to be the master frequency used for cycle-boundary detection (max of per-angle freqs)
- Changed sliding-window history vectors to 2D (`vector<vector<double>>`) to support per-angle smoothing in Task 5

### Files Implemented

#### `src/BFWingModel.h`

- `BFAngleId` enum: indices for the five maneuvering angles (kAngleBeta through kAnglePhi)
- `BFAngleParams` struct: per-angle parameter storage (freqRangeMin/Max, ampRangeMin/Max, phaseOffset, meanAngle)
- `BFWingModel` class:
  - Constructor initializes Monarch defaults from Table 3
  - `update(BFState&, dt)` — main entry point, advances phase, detects cycle boundaries, evaluates all angles
  - `maxSpeed` — configurable |u_max| (default 2.0 m/s)
  - `evalSigmoid()` — core of Eqs. 2-3: `range / (1 + exp(-16 * (|u|/|u_max| - 0.5)))`
  - `evalAngle()` — Eq. 1: `amp * cos(2*pi*f*t + phi_p) + phi_m`

#### `src/BFWingModel.cpp`

- **Constructor:** Table 3 parameter initialization for all five angles:
  - beta: f 0-3 Hz, amp 0-30°, phase -90°, mean 0°
  - gamma: f 0-11 Hz, amp 0-150°, phase 0°, mean 10°
  - zeta: f 0-11 Hz, amp 0-10°, phase -90°, mean 0°
  - psi: f 0-11 Hz, amp 0-20°, phase 0°, mean 0°
  - phi: f 0-11 Hz, amp 0-35°, phase -180°, mean -10°
- **`evalSigmoid()`:** Implements Eqs. 2-3 sigmoid with k=16 steepness
- **`evalAngle()`:** Implements Eq. 1 with phase offset in radians
- **`update()`:**
  1. Advances `state.phase` by dt
  2. Detects cycle boundary when phase >= 1/frequency; resets phase, increments flapCycle
  3. At cycle boundary: recomputes per-angle freq/amp from current velocity via sigmoid
  4. Master frequency = max across all per-angle frequencies (gamma dominates)
  5. Evaluates all five angles from Eq. 1 using per-angle freq, amp, and shared phase

#### `src/BFSimulateCmd.cpp`

- Added new flags: `-d`/`-duration` (long), `-f`/`-frameRate` (double), `-s`/`-startFrame` (long)
- Added `#include "BFWingModel.h"` and Maya animation headers
- `doIt()` now contains a simulation loop:
  1. Parses duration (default 60), fps (default 24), startFrame (default 1)
  2. Creates `BFWingModel` with Monarch defaults
  3. Loops `duration` frames, calling `wingModel.update(m_state, dt)` each frame
  4. Subtask 2.3 placeholders for `applyAngles()` and `writeRotationKeys()`
  5. Reports final cycle count and gamma angle on completion

### Design Decisions

- Each of the five angles has its own frequency and amplitude (per Table 3), not a single shared value
- Frequency/amplitude are held constant within one flapping cycle and recomputed at cycle boundaries (per paper Section 4.1)
- The master frequency for cycle-period detection uses the maximum across all angles (gamma in practice)
- Phase offset (`phi_p`) is converted to radians inside `evalAngle()`; all stored parameters and outputs are in degrees
- Sliding-window smoothing (Eq. 12) is deferred to Task 5 (BFManeuverController)

---

## 2026-03-19 — Subtask 2.3: Joint-Rotation Applicator & Keyframe Baking (Yiding Tian)

### Overview

Implemented the joint-rotation applicator that maps the five maneuvering angles to Maya joint rotations, and the keyframe writer that bakes the simulation into animation curves so the result is scrubbable on the timeline.

### Files Modified

#### `src/BFSimulateCmd.cpp`

- Added includes: `MFnAnimCurve.h`, `MPlug.h`, `MPlugArray.h`, `MFnDependencyNode.h`
- Added `deg2rad()` helper
- **`applyAngles()`** — static function that sets joint rotations from `BFManeuverAngles`:
  - Thorax: `thetaBeta` → rotateX (pitch)
  - Forewing L: `thetaZeta` → rotateX (feather), `thetaPsi` → rotateY (sweep), `thetaGamma` → rotateZ (flap)
  - Forewing R: same as L but flap and sweep negated for bilateral symmetry
  - Hindwing L: `thetaGamma` → rotateZ (flap, 1 DOF)
  - Hindwing R: flap negated
  - Abdomen: `thetaPhi` → rotateX (counter-phase to wings, encoded via phi_p = -180°)
- **`ensureAnimCurve()`** — finds an existing `kAnimCurveTA` (time→angular) on a rotation plug, or creates one if none exists
- **`writeRotationKey()`** — writes a single (X,Y,Z) rotation keyframe on a joint at a given `MTime`, using `MFnAnimCurve::addKey` with auto tangents
- **`writeAllKeys()`** — calls `writeRotationKey` for all 6 joints with the correct mirroring
- **Simulation loop** now calls `applyAngles()` and `writeAllKeys()` each frame
- After the loop, sets Maya's playback range (`MAnimControl::setMinTime/setMaxTime`) to the baked frame range
- Updated completion message to report frame range and flap cycle count

### Axis Mapping

| Angle | Joint(s) | Axis | Mirror |
|-------|----------|------|--------|
| thetaBeta (thorax pitch) | BF_thorax | rotateX | — |
| thetaGamma (flap) | BF_forewing_L/R, BF_hindwing_L/R | rotateZ | negate R |
| thetaZeta (feather) | BF_forewing_L/R | rotateX | same |
| thetaPsi (sweep) | BF_forewing_L/R | rotateY | negate R |
| thetaPhi (abdomen) | BF_abdomen | rotateX | — |

### Notes

- The axis mapping assumes Maya's default joint orientation. If the rig's joint orient values differ, the X/Y/Z assignments in `applyAngles()` and `writeAllKeys()` need to be adjusted after visual inspection.
- `writeRotationKey` values are in radians (Maya's `kAnimCurveTA` internal unit).
- Auto tangents are used for smooth interpolation between keys.

---

## 2026-03-22 — Full Pipeline Integration: Tasks 3-5 into Simulation Loop (Yiding Tian)

### Overview

Wired `BFManeuverController` (Tasks 3-5, implemented by Cecilia) into the main simulation loop in `BFSimulateCmd::doIt()`. Previously, the loop only evaluated maneuvering angles but left velocity at zero, causing the sigmoid to collapse all frequencies/amplitudes to near-zero after the first cycle boundary. Now the full pipeline runs: wing model → force computation → velocity integration → smoothing → keyframe baking.

### Files Modified

#### `src/BFSimulateCmd.cpp`

- Added `#include "BFManeuverController.h"` (pulls in BFAerodynamics and BFCurlNoise transitively)
- `doIt()` changes:
  1. Instantiates `BFManeuverController` with Monarch defaults (mass = 0.428g, maxSpeed synced with wingModel)
  2. Reads initial world-space position from thorax joint via `MFnTransform::getTranslation`
  3. Simulation loop now follows the pattern documented in `BFManeuverController.cpp`:
     - Records `prevCycle = state.flapCycle` before wing model update
     - Calls `wingModel.update(state, dt)` — evaluates Eqs. 1-3, detects cycle boundaries
     - Calls `controller.step(state, dt)` — computes aero + vortex + gravity forces, integrates velocity/position (Eqs. 8-11)
     - At cycle boundaries (`flapCycle != prevCycle`): calls `controller.smoothParameters(state)` for Eq. 12 sliding-window
     - Then applies joint rotations and writes keyframes as before

### Bug Fixed

- **"Only first 6 keyframes have real data":** Root cause was `state.velocity` staying at `(0,0,0)` throughout the simulation. The sigmoid (Eqs. 2-3) at zero speed returns `range / (1 + exp(8)) ≈ 0`, so after the first cycle boundary all frequencies and amplitudes collapsed to near-zero. Now `controller.step()` integrates velocity from aerodynamic lift/drag + curl-noise vortex force + gravity, feeding non-zero speed back into the sigmoid each cycle.

---

## 2026-03-27 — Maneuvering Control Re-integrated (Yiding Tian)

### Overview

Restored the full physics-based dynamics pipeline in `BFSimulateCmd::doIt()`. The alpha build had disabled this in favour of a pure kinematic loop (`updateAnglesOnly`). A detailed analysis of the disabled state was written to `doc/maneuvering.md` before changes were made.

### Root Causes of Previous Failure

- `state.velocity` was never updated (kinematic loop), so `velocity.length() == 0` at every cycle boundary. The sigmoid (Eqs. 2-3) at zero speed ≈ 0, collapsing all per-angle frequencies and amplitudes after the first cycle.
- `writeTranslationKey()` existed but was never called, so `BF_body` never moved in world space despite the controller computing a valid `state.position`.

### Changes — `src/BFSimulateCmd.cpp`

- **Replaced kinematic loop** (`updateAnglesOnly` + constant phase advance) with the full dynamics loop:
  1. `wingModel.update(state, dt)` — phase advance, cycle-boundary detection, per-angle freq/amp recomputation (Eqs. 2-3)
  2. `applyAngles()` — apply rotations to joints **before** `controller.step()` so that `BFAerodynamics` reads correct wing normals from the scene graph
  3. `controller.step(state, dt)` — integrates aerodynamic, vortex, and gravity forces → velocity → position (Eqs. 4-11)
  4. `controller.smoothParameters(state)` — called at each cycle boundary (Eq. 12)
  5. `writeAllKeys()` — rotation keyframes for all 6 joints
  6. `writeTranslationKey(skeleton.joints[kThorax], state.position, t)` — **new**: bakes root translation keyframe each frame so the butterfly moves through world space
- **Added controller initialisation**: `BFManeuverController controller; controller.maxSpeed = wingModel.maxSpeed;`
- **Added position seed**: reads BF_body's current world translation into `state.position` before the loop begins

---

## 2026-03-22 — Task 6: MEL UI Connected to C++ Simulation Command (Yiding Tian)

### Overview

Connected the ButterFlight MEL UI's Simulate button to the actual `bfSimulate` C++ plugin command. Previously the `bfSimulate()` MEL callback was a stub that only collected parameters and printed a "not yet connected" message.

### Files Modified

#### `src/mel/butterFlight_ui.mel`

- **Renamed `bfSimulate()` proc to `bfSimulateCallback()`** — the MEL proc name was shadowing the C++ `bfSimulate` command, causing Maya to call the MEL proc recursively instead of the plugin command. Renaming eliminates the collision.
- **Replaced the TODO stub** with a direct call to the C++ command: `` `bfSimulate -rigRoot $rig -duration $duration -frameRate $fps` `` wrapped in `catch` for error handling
- Shows a success message ("Simulation complete — N frames baked") or an error dialog on failure
- Updated validation error message to reference `BF_body` (the correct root joint) instead of the old `BF_thorax`
- Updated the Simulate button's `-command` to call `bfSimulateCallback`

### Bug Fixed

- **"Wrong number of arguments on call to bfSimulate":** The MEL callback proc was originally named `bfSimulate()`, identical to the C++ plugin command. When the callback used `eval("bfSimulate -rigRoot ...")`, MEL resolved `bfSimulate` to the MEL proc (which takes no arguments) instead of the C++ command. Fix: renamed the MEL proc to `bfSimulateCallback()`. Note: after renaming, a fresh Maya session is required to clear the cached proc definition.

### How to Use

1. **Load the plugin:** `loadPlugin "ButterFlight.mll";`
2. **Open the UI:** `source "butterFlight_ui.mel"; butterFlightUI;`
3. **Assign rig:** Click the "Select" button next to the Rig Root field, or type `BF_body` directly into the text field.
4. **Set simulation parameters:**
   - **Duration** — number of frames to simulate (default 120)
   - **FPS** — frame rate for the time step (default 24)
5. **Click "Simulate"** — this calls `bfSimulate -rigRoot "BF_body" -duration 120 -frameRate 24` and bakes keyframes onto all skeleton joints.
6. **Scrub the timeline** to preview the baked butterfly flight animation.

---

## 2026-04-05 — Path Following Mode (Yiding Tian)

### Overview

Implemented path following using NURBS EP curves. The butterfly flies along a user-drawn curve using a "carrot on a stick" design: a lead target slides along the curve ahead of the butterfly, pulling it forward via the existing `preferredAccel()` (Eq. 8) attraction mechanism. Flight remains physically simulated — the curve guides direction, not rigidity.

### Design

The plan was documented in `doc/Others/path_plan.md`. Key idea: each substep, the closest point on the curve to the butterfly's current position is found, then the target is advanced by `sensorRange * 0.5` (≈2.25 m) in arc length. This lead point becomes `controller.target`, and `preferredAccel()` steers the butterfly toward it. When the lead point reaches the curve end, `hasTarget` is set to `false` and the butterfly coasts in free flight.

### Files Modified

#### `src/BFSimulateCmd.h`

- Added `#include <maya/MFnNurbsCurve.h>` for curve API access

#### `src/BFSimulateCmd.cpp`

- **New flag:** `-p` / `-path` (string) — name of a NURBS curve in the scene
- **`newSyntax()`:** registered the new flag
- **`doIt()` — curve resolution (after controller init):**
  1. If `-path` is set, resolves the curve name to an `MDagPath`
  2. Extends transform to shape node if needed (`extendToShape()`)
  3. Validates it is `MFn::kNurbsCurve`, sets `MFnNurbsCurve` function set
  4. Sets `hasPath = true` and `controller.hasTarget = true`
  5. Displays warning and falls back to free flight if the node is not a NURBS curve
- **`doIt()` — position snap:** When a path is provided, snaps the butterfly's starting position to the curve's first CV (`getPointAtParam(uMin)`) and updates the root joint translation
- **`doIt()` — target advancement (inside substep loop, before `controller.step()`):**
  1. `closestPoint(pos, &uClosest)` — find nearest parameter on curve
  2. `findLengthFromParam(uClosest)` — convert to arc length
  3. Add `sensorRange * 0.5` to get lead arc length
  4. If lead arc >= total curve length → set `hasTarget = false` (coast)
  5. Otherwise → `findParamFromLength(leadArc)` → `getPointAtParam(uLead)` → set `controller.target`

#### `src/mel/butterFlight_ui.mel`

- **`bfSimulateCallback()`:** Added `$bf_pathField` to globals, reads the path text field, and appends `-path "curveName"` to the command string when non-empty
- Changed command invocation from backtick-literal to `eval($cmd)` with dynamic string building to support the optional path flag

### Maya API Methods Used

| Method | Purpose |
|--------|---------|
| `MFnNurbsCurve::closestPoint(pt, &u)` | Find parameter of nearest point |
| `MFnNurbsCurve::getPointAtParam(u, pt)` | Evaluate curve position at parameter |
| `MFnNurbsCurve::length()` | Total arc length |
| `MFnNurbsCurve::findLengthFromParam(u)` | Arc length from start to parameter |
| `MFnNurbsCurve::findParamFromLength(len)` | Parameter at a given arc length |
| `MFnNurbsCurve::getKnotDomain(uMin, uMax)` | Valid parameter range |

### How to Use

1. Draw an EP curve in Maya (Create → Curve Tools → EP Curve Tool)
2. Open the ButterFlight UI, set mode to **"Path Following"** (section 3)
3. Select the curve in the viewport, click **"Select"** in Path Settings (section 4)
4. Assign the rig root and click **Simulate**
5. The butterfly starts at the curve's first CV and flies along it, coasting in free flight after reaching the end

### No Changes Needed

- `BFManeuverController` — `target`, `hasTarget`, and `preferredAccel()` already work as designed for path following

---

## 2026-04-06 — Path Following: Speed Control & Cursor-Based Steering (Yiding Tian)

### Problem

Path speed was hardcoded to `controller.maxSpeed * 0.7` (1.4 m/s). On short curves the butterfly reached the end almost instantly and spent most of the animation in unguided free-fall. On long curves the speed was identical. The user had no way to control traversal rate.

Two earlier fix attempts failed:
1. **Auto-derive pathSpeed from `curveLen / realTime`** — the formula didn't account for `simRate` (physics clock compression), producing a speed ~5.5× too slow.
2. **Auto-derive from `curveLen / physicsTime`** — correct formula, but `std::clamp(..., maxSpeed)` capped it to 2.0 m/s for any curve longer than ~180 cm, making all curves appear the same speed. Additionally, the post-blend velocity clamp to `maxSpeed` prevented the butterfly from actually reaching the desired speed.

### Root Cause

The velocity-based approach is fundamentally limited: `maxSpeed` clamps both the desired velocity and the post-blend result, and the exponential blend (α ≈ 0.2%/substep) converges too slowly against physics forces at low target speeds. Speed control requires decoupling traversal rate from physics velocity.

### Solution: Time-Based Arc Cursor

Replaced `closestPoint()`-based progress tracking with a time-based cursor that advances at a fixed rate per substep:

```
arcRate = totalCurveLen / (duration × substeps) × pathSpeedScale
```

At scale 1.0 the cursor reaches the curve end exactly when the simulation ends, regardless of curve length, `simRate`, or physics forces. The butterfly chases the cursor via a spring; aero oscillation from `controller.step()` is preserved.

### Files Modified

#### `src/BFSimulateCmd.cpp`

- **New flag:** `-ps` / `-pathSpeed` (double, default 1.0) — path speed scale multiplier
- **`newSyntax()`:** registered the new flag
- **Added `#include <algorithm>`** for `std::clamp` (unused now but kept for future use)
- **Arc cursor computation** (before simulation loop):
  ```cpp
  double arcCursor = 0.0;
  double arcRate = totalCurveLen / ((double)duration * substeps) * pathSpeedScale;
  ```
- **Replaced the path-following steering block** — removed `closestPoint()`, monotonic arc progress (`lastArcProgress`), and velocity-based `pathSpeed`. New design:
  1. **Cursor advancement:** `arcCursor += arcRate` each substep (capped at `totalCurveLen`)
  2. **Target from cursor:** `curveFn.findParamFromLength(arcCursor)` → `getPointAtParam()` for position, `tangent()` for direction
  3. **Spring to cursor:** `desiredVel = toTarget * 15.0` — proportional spring toward the cursor point
  4. **Tangent bias:** `tangent * (arcRate * kCmToM / simDt * 0.3)` — smooths direction on curves
  5. **Blend rate:** increased from 12.0 to 18.0 for tighter tracking
  6. **No maxSpeed clamp** — neither on `desiredVel` nor on post-blend velocity. The cursor rate controls effective speed, not physics velocity limits
  7. **End-of-curve:** transitions to free flight when `arcCursor >= totalCurveLen` and butterfly is within 0.1 m of endpoint

#### `src/mel/butterFlight_ui.mel`

- **Repurposed "Path Follow Strength"** field → **"Path Speed Scale"** with tooltip annotation
- **New global:** `$bf_pathSpeedField` — stored control name for querying
- **`bfSimulateCallback()`:** reads `$bf_pathSpeedField` and appends `-pathSpeed <scale>` when a path is set
- **`bfReset()`:** resets path speed scale to 1.0

### Key Design Differences from Previous Approach

| Aspect | Old (velocity-based) | New (cursor-based) |
|--------|---------------------|--------------------|
| Progress tracking | `closestPoint()` → monotonic arc | Time-based cursor (`arcCursor += arcRate`) |
| Speed control | `desiredVel = tangent * pathSpeed` | Implicit from cursor rate |
| maxSpeed interaction | Clamped to `controller.maxSpeed` | No clamp — cursor rate is the speed limit |
| Traversal guarantee | Depends on physics convergence | Cursor always finishes on time |
| Dependencies | Needs correct `simRate` in formula | Only needs `duration × substeps` |

### Path Speed Scale Usage

| Scale | Behaviour |
|-------|-----------|
| 1.0 | Butterfly traverses full curve over full simulation duration |
| 2.0 | Twice as fast — finishes at the halfway point, then free flight |
| 0.5 | Half speed — only reaches the curve midpoint by simulation end |

---

## 2026-04-07 — Hover Mode (Yiding Tian)

### Overview

Added a Hover simulation mode (mode 4) where the butterfly stays at a fixed position with wings flapping naturally. The user can optionally specify a world-space position and full orientation (pitch, yaw, roll). If no position is specified, the butterfly hovers at its current rig location.

### Design Rationale

The existing physics pipeline (aero forces, vortex noise, gravity) is designed for flight. At zero velocity, aerodynamic lift vanishes (Eq. 4 scales with |V|²), vortex forces push the butterfly off-station, and gravity pulls it down. Rather than fighting these forces with a PD controller, hover mode bypasses `controller.step()` entirely and only runs the wing model for visual flapping. The `kMinFreq`/`kMinAmp` floors in `BFWingModel.cpp` guarantee the wings keep flapping at a baseline rate even at zero speed (gamma ≥ 5 Hz, 40° amplitude).

### Files Modified

#### `src/BFSimulateCmd.cpp`

- **New flag constants:**
  - `-hpx` / `-hoverPosX` — hover position X (cm)
  - `-hpy` / `-hoverPosY` — hover position Y (cm)
  - `-hpz` / `-hoverPosZ` — hover position Z (cm)
  - `-hrx` / `-hoverRotX` — pitch angle (degrees)
  - `-hry` / `-hoverRotY` — yaw/heading angle (degrees)
  - `-hrz` / `-hoverRotZ` — roll angle (degrees)
- **`newSyntax()`:** registered all 6 new flags as `MSyntax::kDouble`
- **`doIt()` — flag parsing:** detects hover mode when `-mode 4` is set. Reads optional position (any of X/Y/Z triggers custom position mode) and rotation values. Position is in Maya cm, rotation in degrees.
- **`doIt()` — hover setup (after path snap, before simulation loop):**
  1. If custom position flags are set, overrides `m_state.position` (converted to metres) and moves the rig joint to match
  2. Sets `m_state.heading` from yaw, stores pitch/roll as radians (`hoverPitchRad`, `hoverRollRad`)
  3. Zeros `m_state.velocity`
- **Simulation loop — hover branch:** wraps the existing flight code in an `if (hoverMode) { ... } else { ... }` block. In hover mode:
  1. `wingModel.update(m_state, simDt)` — runs the full wing model so freq/amp adapt toward zero-speed floor values via the continuous first-order filter (`kAdaptTime = 3s`), producing a natural "settling" animation
  2. `applyAngles()` — sets joint rotations from the maneuvering angles
  3. If user specified pitch or roll: overwrites the thorax rotation to `(thetaBeta + userPitch, heading, userRoll)`, preserving the wing model's body bob oscillation on top of the user's base pitch
  4. No `controller.step()` — no forces, no gravity, no velocity integration
  5. Position, velocity, and heading remain fixed throughout
- **Keyframe writing:** after `writeAllKeys()`, if hover mode has pitch/roll offsets, an additional `writeRotationKey` overwrites the thorax rotation key to include the offsets (since `writeAllKeys` writes `(thetaBeta, heading, 0)` by default)

#### `src/mel/butterFlight_ui.mel`

- **New globals:** `$bf_hoverFrame`, `$bf_hoverUseRigCheck`, `$bf_hoverPosXField`, `$bf_hoverPosYField`, `$bf_hoverPosZField`, `$bf_hoverRotXField`, `$bf_hoverRotYField`, `$bf_hoverRotZField`
- **Mode menu:** added "Hover" as the 4th menu item (index 4). Set as the default selection on window launch and after Reset.
- **"4b Hover Settings" frame** (visible only when mode == 4):
  - "Use Current Rig Position" checkbox (default on) — when checked, position fields are disabled and the butterfly uses its current rig location
  - Position X/Y/Z fields (cm) — enabled when the checkbox is unchecked
  - Separator + "Orientation:" label
  - Roll (deg), Yaw (deg), Pitch (deg) fields — always enabled
- **`bfHoverToggle(int $state)`:** new callback that enables/disables the position X/Y/Z fields when the checkbox is toggled
- **`bfUpdateMode()`:** updated to show/hide `$bf_hoverFrame` when mode == 4
- **`bfSimulateCallback()`:** when mode == 4, builds `-mode 4 -hoverRotX <rx> -hoverRotY <ry> -hoverRotZ <rz>` flags, and appends `-hoverPosX/Y/Z` only when "Use Current Rig Position" is unchecked
- **`bfReset()`:** resets hover checkbox to true, all position/rotation fields to 0.0, and calls `bfHoverToggle 0` to disable position fields. Mode menu resets to Hover (select 4).
- **Window launch (`butterFlightUI`):** after `showWindow`, sets mode menu to Hover (select 4) and calls `bfUpdateMode` to show the hover frame by default

### No Changes Needed

- **`BFWingModel`** — already works at zero speed; `kMinFreq`/`kMinAmp` floors keep flapping alive
- **`BFManeuverController`** — not called in hover mode
- **`BFState`** — no new fields; existing `position`, `heading`, `velocity`, and angle arrays suffice
- **`BFAerodynamics` / `BFCurlNoise`** — not invoked in hover mode

### Wing Behaviour in Hover

At zero velocity, the sigmoid (Eqs. 2-3) drives freq/amp toward near-zero, but floor values override:

| Angle | Floor Freq (Hz) | Floor Amp (deg) | Visual Effect |
|-------|-----------------|-----------------|---------------|
| Beta (thorax pitch) | 1.0 | 5.0 | Gentle body bob |
| Gamma (wing flap) | 5.0 | 40.0 | Steady hovering flap |
| Zeta (feather) | 5.0 | 2.0 | Subtle feathering |
| Psi (sweep) | 5.0 | 4.0 | Slight fore-aft sweep |
| Phi (abdomen) | 5.0 | 8.0 | Counter-phase abdomen sway |

The `kAdaptTime = 3.0s` filter ramps freq/amp from the initial mid-range defaults down to these floors over ~3 seconds, producing a natural settling animation at the start.

### Orientation Model

Thorax rotation in hover mode is `MEulerRotation(thetaBeta + userPitch, userYaw, userRoll, kXYZ)`:
- **Pitch (X):** user's base pitch is added on top of the wing model's `thetaBeta` oscillation, so the body bob is preserved
- **Yaw (Y):** sets the body heading (facing direction)
- **Roll (Z):** tilts the body left/right

### How to Use

1. Open the ButterFlight UI — defaults to Hover mode
2. Assign the rig root joint
3. Optionally uncheck "Use Current Rig Position" and enter X/Y/Z coordinates (cm)
4. Optionally set Roll, Yaw, Pitch values (degrees)
5. Click **Simulate** — the butterfly hovers in place with wings flapping for the specified duration
