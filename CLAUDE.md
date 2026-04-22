# ButterFlight: Butterfly Flight Authoring Tool

## Project Overview

**ButterFlight** is a Maya plugin that simulates and animates butterfly flight using biomechanics from Chen et al. 2022. Users can author butterfly animations via:
- **Five simulation modes:** free flight, hover, path following, swarm, and procedural (curl-noise vortex forces)
- **Cinematic cameras:** follow cam with viewport-derived offsets, and a stationary rotating camera with auto-zoom
- **Real-time parameter tuning** via Maya UI without source code changes

The plugin exports physics-based wing kinematics (five maneuvering angles) and full-body articulation as baked keyframe animation, scrubbable on the Maya timeline.

---

## Physics & Animation Model

### Core Model (Chen et al. 2022)

All dynamics are derived from a single reference: Chen, Combes, & Altshuler (2022) "Thoracic Kinematics Explain Acceleration and Deceleration in Monarch Butterflies."

**Key equations:**
- **Eq. 1** — Wing angle oscillation: `θ(t) = A(|u|) · cos(2πf(|u|)t + φ_p) + φ_m`
  - Five angles: beta (pitch), gamma (flap), zeta (feather), psi (sweep), phi (abdomen)
  - Each has frequency `f` and amplitude `A` that vary with speed `|u|` via sigmoid (Eqs. 2-3)
- **Eq. 4** — Aerodynamic lift/drag from flat-plate wing model
  - Per-wing normal from joint rotation, air velocity computed from body velocity + wing tip speed + wind field
  - Coefficients `C_l(α)` and `C_d(α)` from empirical polynomials (Eqs. 5-6)
- **Eq. 7** — Curl-noise vortex perturbation (implemented by Cecilia)
- **Eqs. 8-11** — Velocity integration: `a = (F_aero + F_vortex + F_gravity) / m`; Euler method; position update
- **Eq. 12** — Sliding-window parameter smoothing at flap-cycle boundaries

### Simulation Pipeline

```
doIt():
  1. Read skeleton, validate joints
  2. Create BFWingModel (frequency/amplitude tables)
  3. Seed initial state (position, velocity, phase)
  4. FOR each frame:
     a. wingModel.update(state, dt)  // Eq. 1: eval angles, detect cycles
     b. applyAngles()                 // Set joint rotations in Maya
     c. controller.step(state, dt)    // Eqs. 4-11: forces & integration
     d. [At cycle boundaries] controller.smoothParameters(state)  // Eq. 12
     e. writeAllKeys()                // Bake rotation keyframes
     f. writeTranslationKey()         // Bake root position keyframe
     g. [Path following] advanceArcCursor()  // Carrot-on-stick steering
     h. [Camera] recordPositions()    // For follow/stationary cam baking
  5. [Follow cam] bakeFollowCamera()
  6. [Stationary cam] bakeStationaryCamera()
```

### Simulation Parameters (Physics)

| Parameter | Type | Range | Default | Meaning |
|-----------|------|-------|---------|---------|
| `mass` | double | [0.1, 5.0] g | 0.428 | Butterfly mass (Monarch) |
| `maxSpeed` | double | [0.5, 5.0] m/s | 2.0 | Speed clamp for sigmoid |
| `wingArea` | struct | — | 6.5 cm² each | Forewing/hindwing areas |
| `meanChordRadius` | double | [0.5, 5.0] cm | 2.5 | Center-of-pressure distance from joint |
| `wind` | [x, y, z] | ℝ³ | [0, 0, 0] | Ambient wind velocity (m/s) |

---

## C++ Plugin Architecture

### File Organization

```
src/
├── BFState.h              // State struct, joint enum, wing/aero params
├── BFSimulateCmd.{h,cpp}  // Main command, simulation loop, keyframe baking
├── BFWingModel.{h,cpp}    // Eq. 1: angle evaluation & frequency/amplitude
├── BFAerodynamics.{h,cpp} // Eqs. 4-6: lift/drag computation
├── BFManeuverController.{h,cpp} // Eqs. 8-12: force integration & smoothing
├── BFCurlNoise.{h,cpp}    // Eq. 7: vortex noise (by Cecilia)
├── BFFlocking.{h,cpp}     // Flocking behavior (by Cecilia)
├── BFSwarmManager.{h,cpp} // Multi-butterfly coordination
├── PluginMain.cpp         // Plugin init/deinit
├── mel/butterFlight_ui.mel // User interface
└── test/                   // Unit tests (aerodynamics, integration)
```

### Key Classes

#### `BFState` (BFState.h)
Persistent per-butterfly state across all frames:
- Skeleton (joint paths)
- Position/velocity/heading (world space)
- Five maneuvering angles (in degrees)
- Per-angle frequency/amplitude (updated at cycle boundaries)
- Phase accumulator (0–1 per flap cycle)
- Flap cycle counter
- Sliding-window history vectors (for smoothing)

#### `BFSimulateCmd` (BFSimulateCmd.cpp)
MPxCommand subclass; the public API to Maya:
- **Flags:** `-rigRoot`, `-duration`, `-frameRate`, `-mode`, `-path`, `-pathSpeed`, `-pathFromStart`, `-velocity`, `-hoverPos*`, `-hoverRot*`, `-useCurrentCamera`, `-createStatCam`, `-statCamAutoZoom`, etc.
- **doIt():** parses flags, initializes state, runs the simulation loop, bakes keyframes
- **Undo support:** `undoIt()` / `redoIt()` (stubs; keyframes are undoable via Maya's native undo)

#### `BFWingModel` (BFWingModel.cpp)
Evaluates maneuvering angles (Eq. 1) per frame:
- Constructor stores Table 3 defaults (five angle params × 5 angles = 25 per-angle parameters)
- `update(state, dt)`: advances phase, detects cycle boundaries, recomputes per-angle freq/amp from speed via sigmoid
- `evalSigmoid()`: Eqs. 2-3 implementation with steepness k=16
- `evalAngle()`: Eq. 1, returns angle in degrees

#### `BFManeuverController` (BFManeuverController.cpp)
Integrates forces and updates state (Eqs. 8-12):
- `step(state, dt)`: computes aero + vortex + gravity forces, integrates velocity/position via Euler
- `smoothParameters(state)`: sliding-window filter at cycle boundaries
- `preferredAccel()`: target-seeking acceleration for path following & flocking

#### `BFAerodynamics` (BFAerodynamics.cpp)
Flat-plate wing model (Eqs. 4-6):
- `getWingNormal()` / `getWingSpanDir()`: extract wing panel orientation from joint's world matrix
- `computeWingForce()`: applies AoA computation, dynamic pressure, and empirical coefficients per wing
- `computeTotalForce()`: sums four wings with bilateral symmetry

---

## MEL UI (`src/mel/butterFlight_ui.mel`)

The UI is a dockable window with 10 collapsible panels grouped by feature:

| Panel | Name | Controls |
|-------|------|----------|
| 1 | Rig & Simulation | Rig root, duration, frame rate, mode menu |
| 2 | Free Flight | Velocity (m/s), heading (degrees) |
| 3 | Path Following | Path curve selector, speed scale toggle, scale value, start-mode toggle |
| 4 | (Hover) | Position/rotation checkboxes, XYZ position, roll/yaw/pitch |
| 5 | Simulation Mode | Wind (X/Y/Z), mass, max speed (disabled; physics params) |
| 6 | Flocking | Flock size, boid parameters (by Cecilia) |
| 7 | Swarm | — (placeholder) |
| 8 | Follow Camera | Create toggle, offset XYZ, viewport transform toggle |
| 9 | Stationary Camera | Create toggle, camera name, auto-zoom toggle |
| 10 | Reset | Reset all to defaults |

### Key Procedures

- `butterFlightUI()` — creates the window and all controls; called once on startup
- `bfUpdateMode(int $mode)` — shows/hides panels based on selected mode
- `bfSimulateCallback()` — builds command string from all UI fields and calls the C++ `bfSimulate` command
- `bfReset()` — resets all fields to defaults

### Global Variables

All UI control names are stored in globals (e.g., `$bf_pathField`, `$bf_modeMenu`) so callbacks can query and update them.

---

## Simulation Modes

### Mode 1: Free Flight
Butterfly flies under physics alone; no target or constraint.
- **UI inputs:** velocity (m/s), heading (degrees)
- **Command flags:** `-velocity`, `-heading`

### Mode 2: Path Following
Butterfly flies along a NURBS curve via a "carrot on a stick" (lead target).
- **Arc cursor:** `arcCursor += arcRate` per substep; `arcRate = (totalCurveLen - arcCursor) / (duration × substeps) × pathSpeedScale`
- **Start modes:**
  - **"Start from Path Start"** (off): snap to nearest point on curve (Cecilia's default; safest for editing)
  - **"Start from Path Start"** (on): snap to curve's first CV (legacy behavior; useful when the curve represents a planned path)
- **Speed scale:** 1.0 = traverse full remaining length over remaining duration; affects apparent speed without changing physics velocity limits
- **Command flags:** `-path`, `-pathSpeed`, `-pathFromStart`

### Mode 3: Hover
Butterfly stays at a fixed position; wings flap naturally via frequency/amplitude floors.
- **UI inputs:** custom position (cm, optional; default = current rig location), roll/yaw/pitch (degrees)
- **Physics:** `wingModel.update()` only; no `controller.step()`; no gravity or forces
- **Command flags:** `-hoverPosX/Y/Z`, `-hoverRotX/Y/Z`

### Mode 4: Swarm (Placeholder)
Reserved for multi-butterfly coordination (not yet implemented).

### Mode 5: Flocking
Multiple butterflies with cohesion/separation/alignment (implemented by Cecilia).
- Uses `BFFlocking` and `BFSwarmManager`

---

## Cinematic Cameras

Both camera modes bake keyframes using the same shared framing math:

```
offsetQ   = camWorldQ * lookAtQ_0.inverse()   // Viewport offset relative to butterfly
finalQ_t  = offsetQ * lookAtQ_t                // Per-frame rotation with preserved offset
```

### Follow Camera
- **Viewport offsets** (if "Use Current Camera Transformation" is on):
  - Capture the viewport camera's world rotation and the look-at rotation from butterfly → camera
  - Derive offset quaternion once at setup time
  - Per frame: compute look-at from butterfly's current position and apply offset on top
- **Manual offsets** (if "Use Current Camera Transformation" is off):
  - Use X/Y/Z offsets entered in the UI
- **Helper:** `bakeFollowCamera(positions, startFrame, offsetCm, offsetQ, ...)`

### Stationary Rotating Camera
- **Setup:**
  - Camera's world position is **locked** to the viewport camera's position at setup time
  - Viewport camera's rotation and FOV are captured; FOV is applied once via `setVerticalFieldOfView()`
- **Per-frame:**
  - Rotation animates to track the butterfly with the same screen-space offset
  - If **auto-zoom** is on: `focal_t = focal_0 × (d_t / d_0)`, clamped to [2.5, 500] mm
    - `d_0` = distance at setup, `d_t` = distance at frame t
    - Keeps butterfly's apparent size constant as it moves toward/away from camera
- **Helper:** `bakeStationaryCamera(positions, startFrame, camName, camPosCm, offsetQ, fovDeg, autoZoom, setupDistCm)`

### Implementation Details

- **Focal length is set on the camera shape**, not transform; `clearAnimCurves()` only wipes transform curves, so we added `clearAnimCurveOnPlug()` to remove stale animation from shape attributes
- **Camera creation uses MEL `camera -name`** for atomic naming (avoids C++ racy naming issues)
- **Auto-zoom keys use `kAnimCurveTU`** (time → unitless); rotation keys use `kAnimCurveTA` (time → angle)

---

## Units & Coordinate System

| Domain | Unit | Notes |
|--------|------|-------|
| Maya spatial | cm | Default Maya unit |
| Physics computation | m | SI; converted at simulation boundaries |
| Time | frames | Derived from frame rate; `dt = 1.0 / (fps * substeps)` |
| Rotation | degrees | UI and keyframes; converted to radians for computation |
| Angle of attack (AoA) | radians | Computed internally; output in degrees |

**Conversion constants in BFSimulateCmd:**
- `kMToCm = 100.0`
- `kCmToM = 0.01`

**Coordinate frame:** Maya's default (Y up, Z forward, X right; right-hand rule). Joint rotations use Maya's `kXYZ` Euler order.

---

## Joint Axis Mapping

The skeleton must have six joints parented under a root (`BF_body`):

| Joint | Rotation Axis | Angle | Mirror (R/L) |
|-------|---------------|-------|--------------|
| `BF_thorax` | rotateX | thetaBeta (pitch) | — |
| `BF_forewing_L` | rotateX, rotateY, rotateZ | thetaZeta, thetaPsi, thetaGamma | Z negated on R |
| `BF_forewing_R` | " | " | — |
| `BF_hindwing_L` | rotateZ | thetaGamma (flap only) | Z negated on R |
| `BF_hindwing_R` | " | " | — |
| `BF_abdomen` | rotateX | thetaPhi (counter-phase) | — |

**Important:** if your rig has different joint orientations, the X/Y/Z assignments in `applyAngles()` and `writeAllKeys()` must be adjusted.

---

## Building & Deployment

### Build System (CMake)

```bash
cd build
cmake ..
cmake --build . --config Release
```

Outputs: `ButterFlight.mll` (Windows), `ButterFlight.bundle` (macOS), `.so` (Linux)

### Loading in Maya

```mel
loadPlugin "ButterFlight.mll";
source "butterFlight_ui.mel";
butterFlightUI;
```

### Workspace Setup

`workspace.mel` configures Maya's Shelves and menus. Load once:
```mel
source "workspace.mel";
```

---

## Common Development Tasks

### Adding a New Simulation Flag

1. **Define the flag constant** in `BFSimulateCmd.cpp`:
   ```cpp
   static const char* kNewFlagShort = "-nf";
   static const char* kNewFlagLong  = "-newFlag";
   ```

2. **Register in `newSyntax()`:**
   ```cpp
   syntax.addFlag(kNewFlagShort, kNewFlagLong, MSyntax::kDouble);  // or kBoolean, kString
   ```

3. **Parse in `doIt()`:**
   ```cpp
   double newValue = 1.0;
   if (argData.isFlagSet(kNewFlagShort))
       argData.getFlagArgument(kNewFlagShort, 0, newValue);
   ```

4. **Add to MEL UI:**
   ```mel
   global string $bf_newField = `floatFieldGrp -label "New Param" -value1 1.0`;
   // In bfSimulateCallback:
   float $newVal = `floatFieldGrp -query -value1 $bf_newField`;
   $cmd += " -newFlag " + $newVal;
   ```

5. **Add to `bfReset()`:**
   ```mel
   floatFieldGrp -edit -value1 1.0 $bf_newField;
   ```

### Adding a New Maneuvering Angle

Would require modifying `BFAngleId` enum (in BFWingModel.h) and Table 3 initialization. Not currently done; all five angles are from Chen et al. 2022.

### Debugging Simulation Output

Check the Maya Script Editor for `MGlobal::displayInfo()` and `MGlobal::displayWarning()` messages, which print:
- Current simulation parameters (mass, wind, etc.)
- Arc rate and cursor progress (path following)
- Cycle boundary events
- Final statistics (total cycles, duration)

---

## Known Issues & Gotchas

### Physics

1. **Velocity blending is slow at low speeds.** The exponential blend (`α ≈ 0.2% per substep, blend rate = 18`) converges gradually. For path following, we use a time-based arc cursor instead of velocity-based speed control to guarantee traversal.

2. **maxSpeed clamp affects both desired and integrated velocity.** This decouples control from physics limits; path speed is defined by cursor rate, not velocity magnitude.

3. **Frequency/amplitude recompute at cycle boundaries only.** Within a flap cycle, freq/amp are constant, so rapid velocity changes may not be visible until the next cycle. This is by design (per Chen et al. 2022, Section 4.1) but can feel sluggish in path-following turns.

### Skeleton & Rigging

4. **Left-right symmetry must be exact.** If forewing R has different joint orientations than forewing L, the negate-on-R mirroring in `applyAngles()` will produce wrong rotations.

5. **Skin weight binding is fragile.** Always bind each wing mesh to its own joint using **"Bind to Selected joints"** mode, not "Joint hierarchy". Use Component Editor or Paint Skin Weights to diagnose split bindings.

6. **Character scaling after rig creation breaks the simulation.** If you scale the model after binding, re-bind the skin.

### Animation & Keyframing

7. **Stale animation keys accumulate.** Always call `bakeFollowCamera()` and `bakeStationaryCamera()` after clearing prior animation, or old keys will persist.

8. **Focal length requires special handling.** `clearAnimCurves()` only wipes transform plugs; use `clearAnimCurveOnPlug(camShapePath, "focalLength")` to wipe shape attributes.

9. **Undo of keyframe baking is slow.** Baking can create 1000s of keys. Use Maya's native undo (Ctrl+Z) but be patient.

### Curves & Path Following

10. **EP curves must be planar or nearly so.** Steeply out-of-plane sections can cause `closestPoint()` to overshoot or oscillate.

11. **Curve end detection uses position distance, not arc length.** If the curve end is very close to the next-to-last control point, the butterfly might coast before reaching it. Adjust the curve or lower the lead-detection threshold.

12. **Arc length computation is precomputed once.** If you edit the curve between runs, call `bfReset()` to clear cached values.

### UI

13. **Mode menu must be 1-indexed.** MEL's `optionMenuGrp -select` is 1-indexed, not 0-indexed.

14. **Global variables shadow each other.** All UI control names are stored in globals. If two panels create controls with the same name, one will overwrite the other. Use unique prefixes (e.g., `$bf_pathField`).

15. **Floating-point precision in UI fields.** `-precision 3` truncates to 3 decimal places; display artifacts may occur for very small or very large values.

---

## Performance Tips

1. **Reduce duration or substeps** to speed up iteration. `substeps = 5` (default) is a good balance; lower values risk instability.

2. **Disable camera baking** if you only care about butterfly animation; cameras slow down the loop by ~10%.

3. **Use free flight for quick tests.** Path following and flocking add overhead (curve operations, target seeking).

4. **Profile with the Profiler tool.** Check frame rate and find bottlenecks; usually it's `controller.step()` (aerodynamic force computation).

---

## Further Reading

- **Chen et al. 2022:** Source paper for all equations. In `doc/Others/` (if available) or request from the team.
- **Implementation log:** `doc/implementation_log.md` — chronological record of all features with technical details.
- **Design doc:** `doc/` — high-level architecture and task definitions.
- **Workspace:** `workspace.mel` — menu bar setup and shelf buttons.

---

## Team & Communication

- **Primary author (Physics/C++):** Yiding Tian (tonytgrt)
- **Physics & Aerodynamics:** Cecilia Chen (CeciliaChen98) — `BFAerodynamics`, `BFManeuverController`, `BFCurlNoise`, `BFFlocking`

Git history is the source of truth; refer to commit messages and pull requests for context on specific decisions.
