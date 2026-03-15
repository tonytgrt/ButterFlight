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
   - Command usage is now: `bfSimulate -r "BF_thorax"` or `bfSimulate -rigRoot "BF_thorax"`.
