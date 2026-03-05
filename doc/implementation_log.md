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

### Files Not Yet Implemented (still empty stubs)
- `src/BFWingModel.h` / `src/BFWingModel.cpp` — maneuvering angle evaluation (Eqs. 1–3)
- `src/BFAerodynamics.h` / `src/BFAerodynamics.cpp` — lift/drag force computation (Eqs. 4–6)
- `src/BFCurlNoise.h` / `src/BFCurlNoise.cpp` — curl-noise vortex force (Eq. 7)
- `src/BFManeuverController.h` / `src/BFManeuverController.cpp` — velocity integration and sliding-window smoother (Eqs. 8–12)

### Notes
- The MEL UI (`src/mel/butterFlight_ui.mel`) was already complete before this session
- The FBX model (`Butterfly.fbx`) joints may need renaming to match the `BF_` convention after import
