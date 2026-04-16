# Follow Camera — Design Plan

## Overview

Add a **keyable spring-arm camera** that follows the butterfly at a fixed relative position throughout the simulation. When the user toggles a checkbox in the MEL GUI, a Maya camera is created (or updated) and baked with one keyframe per simulation frame. The default framing sits the camera behind and slightly above the butterfly so the entire wingspan is visible; the user can freely adjust the local offset and rotation.

---

## Design

### Spring Arm Concept

A "spring arm" is a virtual rig that connects the camera to the follow target:

```
  butterfly ──── [spring arm] ──── camera
   (anchor)      (local offset)    (eye)
```

- The arm starts at the butterfly's thorax.
- The arm's local offset (X/Y/Z in the butterfly's local frame) places the camera relative to the body.
- The arm is *springy* — the camera lags behind the butterfly under fast motion, then settles into its rest position. This produces organic, less-robotic follow motion.
- The camera aims at the butterfly (look-at) plus an optional user rotation offset.

### Coordinate Convention

The butterfly's `heading = 0` means facing **-Z** (confirmed in `BFSimulateCmd.cpp` heading computation: `heading = atan2(-tx, -tz)`). So the butterfly's local frame is:

| Local axis | World direction when heading = 0 |
|-----------|---------------------------------|
| +X        | +X (butterfly's right)          |
| +Y        | +Y (up)                         |
| +Z        | +Z (behind the butterfly)       |

"Behind the butterfly" means +Z in its local frame. A default offset of `(0, +5, +30)` cm means 30 cm behind, 5 cm above.

### Default Framing

The camera should frame the entire butterfly (wingtip-to-wingtip, ~10 cm for Monarch) with modest margin.

Given camera vertical FOV α and desired frame fill ratio r (e.g. r = 0.7):

```
distance = (wingspan / 2) / (r * tan(α / 2))
```

For α = 35°, wingspan = 10 cm, r = 0.7: `distance ≈ 22.6 cm`. Round up to **30 cm** default behind distance with a small +5 cm vertical lift so the camera sees the butterfly slightly from above.

Default camera rotation: **look-at the butterfly thorax**, computed per frame from camera position → thorax position.

### Camera Motion Pipeline (per frame f)

1. **Read butterfly transform at frame f** — thorax world position `p_bf` and heading yaw `yaw_bf` (plus pitch/roll in hover mode).
2. **Compute desired camera position**:
   ```
   offset_world = rotateY(user_offset_local, yaw_bf)   // yaw-only for follow
   p_desired    = p_bf + offset_world
   ```
   Yaw-only rotation prevents the camera from rolling with the butterfly's body bob (which would induce motion sickness).
3. **Spring smoothing** (first-order lag):
   ```
   alpha    = 1 - exp(-stiffness * dt_frame)
   p_cam[f] = p_cam[f-1] + (p_desired - p_cam[f-1]) * alpha
   ```
   At `stiffness = 10` and `dt_frame = 1/24`, `alpha ≈ 0.34/frame` — noticeable lag but settles in under 0.5 s.
4. **Compute camera rotation**:
   ```
   look_dir = normalize(p_bf_smoothed - p_cam[f])   // look at smoothed butterfly pos
   base_rot = lookAt(look_dir, up = +Y)
   final_rot = base_rot * user_rot_offset            // user euler offset
   ```
5. **Write keyframes** — translation + rotation on the camera's transform node.

### Why Bake Keyframes (not Constraints)

Maya has native `parentConstraint` and `aimConstraint` that could approximate this. We prefer baked keyframes because:
- The butterfly is already baked to keyframes — the camera is the same "snapshot of a physics run" output product.
- Spring dynamics aren't available in Maya constraints without custom DG nodes.
- Baked keys play back deterministically; constraints re-evaluate on every viewport tick.
- The user can scrub, edit the camera curves by hand, and export as FBX/Alembic like the butterfly itself.

---

## User Parameters

| Parameter | Default | Description |
|-----------|---------|-------------|
| Create Follow Camera | off | Master toggle — when on, a camera is created/updated after simulation |
| Camera Name | `BF_followCam` | Name of the transform node (reused across runs) |
| Local Offset X (cm) | 0 | Lateral offset in butterfly's local frame |
| Local Offset Y (cm) | 5 | Vertical lift above butterfly |
| Local Offset Z (cm) | 30 | Distance behind butterfly (+Z = behind) |
| Rotation Offset X (deg) | 0 | Additional pitch on top of look-at (nose down = negative) |
| Rotation Offset Y (deg) | 0 | Additional yaw |
| Rotation Offset Z (deg) | 0 | Additional roll |
| Spring Stiffness | 10.0 | Follow responsiveness (higher = snappier, lower = more lag) |
| Camera FOV (deg) | 35 | Sets the camera's focal length; also used for default distance calc |
| Auto Frame Distance | on | When on, Z offset is auto-computed from FOV + wingspan |
| Frame Fill Ratio | 0.7 | Portion of vertical frame the butterfly should fill (only when auto-frame is on) |

---

## Implementation Steps

### Architecture Choice: Integrate into `bfSimulate` or separate `bfCamera` command?

**Recommendation: integrate into `bfSimulate`.** Reasons:
- Single simulate button for the user — camera stays in sync with butterfly automatically.
- We already have the butterfly's position/heading per frame in memory during the sim loop — no need to read animCurves back.
- A camera can be deleted easily if the user unchecks the toggle and re-runs.

A future extension could add a standalone `bfCamera` command for adjusting camera settings without re-running physics, but v1 keeps it simple.

### 1. C++ — `BFSimulateCmd.cpp`

#### a. Add flag constants

```cpp
static const char* kCamFlag             = "-cam";
static const char* kCamFlagLong         = "-createCamera";
static const char* kCamOffXFlag         = "-cox";
static const char* kCamOffXFlagLong     = "-camOffsetX";
static const char* kCamOffYFlag         = "-coy";
static const char* kCamOffYFlagLong     = "-camOffsetY";
static const char* kCamOffZFlag         = "-coz";
static const char* kCamOffZFlagLong     = "-camOffsetZ";
static const char* kCamRotXFlag         = "-crx";
static const char* kCamRotXFlagLong     = "-camRotX";
static const char* kCamRotYFlag         = "-cry";
static const char* kCamRotYFlagLong     = "-camRotY";
static const char* kCamRotZFlag         = "-crz";
static const char* kCamRotZFlagLong     = "-camRotZ";
static const char* kCamStiffFlag        = "-cst";
static const char* kCamStiffFlagLong    = "-camStiffness";
static const char* kCamFOVFlag          = "-cfv";
static const char* kCamFOVFlagLong      = "-camFOV";
static const char* kCamNameFlag         = "-cnm";
static const char* kCamNameFlagLong     = "-camName";
```

#### b. Register in `newSyntax()`

All kDouble except `-createCamera` (kBoolean) and `-camName` (kString).

#### c. Parse in `doIt()`

Parse all flags; validate defaults (stiffness > 0, FOV ∈ [10, 120], etc.).

#### d. Collect per-frame butterfly state during the sim loop

Currently the sim loop writes keys and discards the transform history. Add two vectors to accumulate:

```cpp
std::vector<MPoint>   bfPositions;   // world cm, per frame
std::vector<double>   bfHeadings;    // yaw radians, per frame
bfPositions.reserve(duration);
bfHeadings.reserve(duration);
```

Push at the end of each frame (after `writeAllKeys`):

```cpp
bfPositions.push_back(posCm);
bfHeadings.push_back(m_state.heading);
```

#### e. After sim loop, bake camera keys

New static helper `bakeFollowCamera()`:

```cpp
static void bakeFollowCamera(
    const std::vector<MPoint>&  positions,
    const std::vector<double>&  headings,
    int                          startFrame,
    const MString&               camName,
    MVector                      localOffsetCm,
    MEulerRotation               rotOffset,
    double                       stiffnessPerSec,
    double                       fovDeg,
    double                       fps);
```

Responsibilities:
1. **Find or create** the camera node (`MFnCamera::create()`), parented under world. Reuse if a transform with `camName` already exists.
2. **Set FOV** on the camera shape (`MFnCamera::setHorizontalFieldOfView()` or vertical equivalent).
3. **Clear existing anim curves** on the camera transform (same helper as the butterfly uses).
4. **Forward pass**:
   - `prevCamPos = p_desired[0]` (seed with no lag on frame 0)
   - For each frame f:
     - Compute `offset_world = rotateY(localOffset, headings[f])`
     - `p_desired = positions[f] + offset_world`
     - `alpha = 1 - exp(-stiffnessPerSec / fps)`
     - `p_cam = prevCamPos + (p_desired - prevCamPos) * alpha`
     - Look-at rotation: build from `(positions[f] - p_cam)` and world +Y; combine with `rotOffset`
     - Write translation + rotation keys at frame `startFrame + f`
     - `prevCamPos = p_cam`

Look-at matrix helper: construct orthonormal basis with forward = `normalize(target - eye)`, right = `forward × up`, up' = `right × forward`; convert to Euler XYZ.

### 2. MEL — `butterFlight_ui.mel`

#### a. Add a "9 Camera" frame after Output Settings

Collapsed by default. Contents:
- `checkBoxGrp` — "Create Follow Camera" (wires `bfCameraToggle` to enable/disable child fields)
- `textFieldGrp` — camera name
- `checkBoxGrp` — "Auto Frame Distance" (wires `bfCameraAutoFrameToggle` to enable/disable the Z offset field)
- `floatFieldGrp` × 3 — local offset X/Y/Z (cm)
- `floatFieldGrp` × 3 — rotation offset X/Y/Z (deg)
- `floatSliderGrp` — spring stiffness (range 0.5 – 30, default 10)
- `floatFieldGrp` — camera FOV (deg)
- `floatSliderGrp` — frame fill ratio (range 0.3 – 1.0, default 0.7)

#### b. Callbacks

- `bfCameraToggle(int $state)` — enables/disables all camera settings based on master toggle.
- `bfCameraAutoFrameToggle(int $state)` — when auto-frame is on, disable manual Z offset field and display the computed distance as a read-only label.

#### c. Update `bfSimulateCallback()`

Read all camera fields; if "Create Follow Camera" is checked, append the camera flags to `$cmd`:

```mel
if (`checkBoxGrp -query -value1 $bf_camEnableCheck`) {
    $camFlags = " -createCamera 1"
              + " -camName \"" + $camName + "\""
              + " -camOffsetX " + $cox + " -camOffsetY " + $coy + " -camOffsetZ " + $coz
              + " -camRotX " + $crx + " -camRotY " + $cry + " -camRotZ " + $crz
              + " -camStiffness " + $stiff
              + " -camFOV " + $fov;
    $cmd += $camFlags;
}
```

#### d. Update `bfReset()`

Restore camera defaults and call both toggles.

### 3. Auto-frame distance calculation

Either done in C++ (when a `-camAutoFrame` flag is set and `-camOffsetZ` is not explicitly provided) or in MEL just before passing the flag:

```
distance = (wingspan / 2) / (frameFillRatio * tan(fovDeg/2 * π/180))
```

Wingspan comes from the species preset (Monarch ≈ 10 cm tip-to-tip). Doing this in MEL is simplest because the species field is already on the UI.

---

## Edge Cases

1. **Frame 0 seed** — the spring has no history on frame 0. Seed with the desired position exactly (`p_cam[0] = p_desired[0]`) so the camera doesn't jerk into place.

2. **Stationary butterfly (hover mode)** — `headings` are constant, `positions` are constant. The spring converges to `p_desired` immediately; camera stays rock-steady. No special casing needed.

3. **Rapid heading changes** — the yaw-based offset rotation can snap by 180° when the butterfly flips direction. Mitigation: the spring smoothing on position already dampens this; additionally, unwrap the heading signal (keep a running `accumulatedYaw` that avoids ±π discontinuities) before computing the offset.

4. **Very low FPS output (e.g. 12 fps)** — `alpha` per frame gets large. Already handled by `1 - exp(-stiffness/fps)` formulation — the continuous-time stiffness is consistent regardless of fps.

5. **User re-runs simulation with camera off** — the old baked camera keys stay in the scene. Acceptable: user can delete the camera manually. Alternative: bfReset could delete `BF_followCam` if it exists.

6. **Conflict with an existing scene camera of the same name** — reuse the existing camera's transform (clear anim curves and re-bake) rather than creating a duplicate. Warn the user if the node is not a camera.

7. **Roll-free look-at when butterfly is directly above/below camera** — the `up = +Y` cross product degenerates when `look_dir ∥ Y`. Fall back to `up = +Z` in that case.

---

## Future Extensions (Out of Scope for v1)

- **Second-order spring** (stiffness + damping) for overshoot and settling behavior.
- **Collision-aware arm** — shorten the arm if a ray from the butterfly to the desired camera position hits geometry. Relevant only for scenes with complex environments.
- **Multiple camera presets** — "chase", "cockpit", "cinematic orbit" — each with its own offset and rotation.
- **Standalone `bfCamera` command** — recompute camera keys from existing butterfly animCurves without re-running the physics simulation. Useful for cinematographers iterating on shot framing.
- **Motion blur / focus pull** — drive Maya's depth-of-field from distance-to-butterfly.
- **Look-ahead** — aim the camera slightly ahead of the butterfly's velocity vector rather than straight at it, for a more active feel.
