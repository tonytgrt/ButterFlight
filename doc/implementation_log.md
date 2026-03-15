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
- All aerodynamics physics use SI units (m, kg, s); conversion from cm²/g happens upstream
- Bilateral symmetry: same flapOmega for all four wings (design doc assumption)
