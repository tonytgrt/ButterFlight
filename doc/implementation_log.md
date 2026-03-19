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
