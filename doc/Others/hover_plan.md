# Hover Mode — Design Plan

## Overview

Add a **Hover** simulation mode where the butterfly stays at a fixed position, wings flapping naturally. The user can optionally specify a world-space position and/or rotation; if omitted, the butterfly hovers at its current rig location and orientation.

---

## Paper Context

The Chen et al. 2022 model already supports near-zero-speed behavior:

- **Maneuvering functions (Eq. 1)**: Wing angles are driven by `θ*(t) = φ_a * cos(phase + φ_p) + φ_m`. At zero velocity the sigmoid (Eqs. 2-3) outputs near-zero frequency/amplitude, but our `kMinFreq`/`kMinAmp` floors in `BFWingModel.cpp` guarantee the wings keep flapping at a baseline rate (gamma >= 5 Hz, gamma amplitude >= 40 deg).

- **Aerodynamic forces (Eq. 4)**: Lift and drag scale with `|V|^2`. At zero airspeed, aero forces vanish — no net lift to fight gravity. Hover mode must counteract gravity explicitly.

- **Vortex force (Eq. 7)**: Curl-noise perturbation would push the butterfly off-station. Hover mode should suppress or heavily attenuate vortex forces.

- **`preferredAccel` (Eq. 8)**: The target-attraction mechanism (`a_pre`) already provides a force toward a point. Setting `controller.target = hoverPosition` and `hasTarget = true` would nominally hold the butterfly near the target, but the ramp function R(d) goes to zero as the butterfly approaches the target, meaning there is no restoring force at the target — the butterfly oscillates around it rather than staying put.

**Conclusion**: The existing physics pipeline is designed for flight. Hover mode needs to **bypass the force-based dynamics** and directly hold position, while still running the wing model for visual flapping.

---

## Design

### Mode Semantics

| Mode | Enum | Behavior |
|------|------|----------|
| Free Flight | 1 | Existing: full physics (aero + vortex + gravity + attraction) |
| Path Following | 2 | Existing: cursor-based steering along NURBS curve |
| Swarm | 3 | Planned (not yet implemented) |
| **Hover** | **4** | **New: fixed position, wing flapping only** |

### Simulation Loop Behavior

In hover mode, each substep:

1. **Wing model runs normally** — `wingModel.update(state, simDt)` advances phase, evaluates all 5 maneuvering angles via Eq. 1, and adapts freq/amp via the sigmoid. Since velocity is ~zero, the sigmoid will drive freq/amp toward the floor values (`kMinFreq`/`kMinAmp`), producing a gentle hovering flap.

2. **`controller.step()` is skipped entirely** — no aero forces, no vortex, no gravity, no velocity integration. Position and velocity remain at their initial values.

3. **Heading held constant** — use the user-provided rotation (if any) or the initial heading. No velocity-derived heading update.

4. **Keyframes written as usual** — rotation keys from maneuvering angles + fixed heading; translation keys from the fixed hover position.

### Pseudocode

```cpp
if (hoverMode) {
    // Position is fixed — set once before the loop
    // m_state.position = hoverPos (in metres)
    // m_state.heading  = hoverHeading (radians)

    for each frame:
        for each substep:
            wingModel.update(m_state, simDt);   // flapping continues
            applyAngles(skeleton, angles, heading);
            // NO controller.step() — position/velocity unchanged
        
        writeAllKeys(skeleton, angles, heading, frameTime);
        writeTranslationKey(thorax, hoverPosCm, frameTime);
}
```

### User Parameters

| Parameter | Flag | Type | Default | Description |
|-----------|------|------|---------|-------------|
| Hover Position X | `-hpx` / `-hoverPosX` | double | current rig X (cm) | World-space X position to hover at |
| Hover Position Y | `-hpy` / `-hoverPosY` | double | current rig Y (cm) | World-space Y position to hover at |
| Hover Position Z | `-hpz` / `-hoverPosZ` | double | current rig Z (cm) | World-space Z position to hover at |
| Hover Rotation Y | `-hry` / `-hoverRotY` | double | current heading (deg) | Body yaw/heading angle (degrees) |

**All position values are in Maya cm** (consistent with the scene). The command internally converts to metres for the physics state, then back to cm for keyframe output.

If none of the hover position flags are set, the butterfly uses its current rig root joint position — same behavior as seeding in free flight mode.

If hover rotation is not set, the heading defaults to 0 (facing -Z in Maya's coordinate system) or the current rig orientation.

---

## Implementation Steps

### 1. C++ — `BFSimulateCmd.cpp`

#### a. Add flag constants

```cpp
static const char* kHoverPosXFlag     = "-hpx";
static const char* kHoverPosXFlagLong = "-hoverPosX";
static const char* kHoverPosYFlag     = "-hpy";
static const char* kHoverPosYFlagLong = "-hoverPosY";
static const char* kHoverPosZFlag     = "-hpz";
static const char* kHoverPosZFlagLong = "-hoverPosZ";
static const char* kHoverRotYFlag     = "-hry";
static const char* kHoverRotYFlagLong = "-hoverRotY";
```

#### b. Register in `newSyntax()`

```cpp
syntax.addFlag(kHoverPosXFlag, kHoverPosXFlagLong, MSyntax::kDouble);
syntax.addFlag(kHoverPosYFlag, kHoverPosYFlagLong, MSyntax::kDouble);
syntax.addFlag(kHoverPosZFlag, kHoverPosZFlagLong, MSyntax::kDouble);
syntax.addFlag(kHoverRotYFlag, kHoverRotYFlagLong, MSyntax::kDouble);
```

#### c. Parse in `doIt()`

After parsing existing flags, detect hover mode from the `-mode` flag value (4):

```cpp
bool hoverMode = false;
MPoint hoverPos;      // metres
double hoverHeading = 0.0; // radians

if (argData.isFlagSet(kModeFlag)) {
    int mode;
    argData.getFlagArgument(kModeFlag, 0, mode);
    hoverMode = (mode == 4);
}

if (hoverMode) {
    // Default: use rig root position
    MFnTransform rootFn(m_state.skeleton.joints[kThorax]);
    MVector t = rootFn.getTranslation(MSpace::kWorld);
    hoverPos = MPoint(t.x * kCmToM, t.y * kCmToM, t.z * kCmToM);

    // Override with user-specified position if provided
    if (argData.isFlagSet(kHoverPosXFlag)) {
        double v; argData.getFlagArgument(kHoverPosXFlag, 0, v);
        hoverPos.x = v * kCmToM;
    }
    if (argData.isFlagSet(kHoverPosYFlag)) {
        double v; argData.getFlagArgument(kHoverPosYFlag, 0, v);
        hoverPos.y = v * kCmToM;
    }
    if (argData.isFlagSet(kHoverPosZFlag)) {
        double v; argData.getFlagArgument(kHoverPosZFlag, 0, v);
        hoverPos.z = v * kCmToM;
    }
    if (argData.isFlagSet(kHoverRotYFlag)) {
        double v; argData.getFlagArgument(kHoverRotYFlag, 0, v);
        hoverHeading = deg2rad(v);
    }

    m_state.position = hoverPos;
    m_state.heading  = hoverHeading;
    m_state.velocity = MVector::zero;
}
```

#### d. Simulation loop branch

Inside the frame/substep loop, add a hover branch before the existing free-flight/path code:

```cpp
if (hoverMode) {
    // Wing flapping only — no physics integration
    wingModel.update(m_state, simDt);
    applyAngles(m_state.skeleton, m_state.angles, m_state.heading);
    // Position and velocity stay fixed — no controller.step()
} else {
    // ... existing free flight / path following code ...
}
```

The keyframe writing code after the substep loop is unchanged — it already writes `m_state.position` and `m_state.angles`, which in hover mode are the fixed position and the flapping angles respectively.

### 2. MEL — `butterFlight_ui.mel`

#### a. Add "Hover" to the mode menu

In the mode `optionMenuGrp`, add a new menu item:

```mel
menuItem -label "Hover";
```

This makes Hover = index 4 in the option menu.

#### b. Add Hover Settings frame (conditional)

Create a new collapsible frame (visible only when mode == 4) with fields for hover position and rotation:

```mel
global string $bf_hoverFrame;
global string $bf_hoverPosXField;
global string $bf_hoverPosYField;
global string $bf_hoverPosZField;
global string $bf_hoverRotYField;
global string $bf_hoverUseRigCheck;

$bf_hoverFrame = `frameLayout
    -label "Hover Settings"
    -collapsable true
    -collapse false
    -visible false
    -marginWidth 6 -marginHeight 6`;

    columnLayout -adjustableColumn true -rowSpacing 4;

        checkBoxGrp
            -label "Use Current Rig Position"
            -value1 true
            -onCommand  "bfHoverToggle 0"
            -offCommand "bfHoverToggle 1";

        $bf_hoverPosXField = `floatFieldGrp
            -label "Position X (cm)" -numberOfFields 1
            -value1 0.0 -precision 3 -enable false
            -columnWidth2 120 160`;

        $bf_hoverPosYField = `floatFieldGrp
            -label "Position Y (cm)" -numberOfFields 1
            -value1 0.0 -precision 3 -enable false
            -columnWidth2 120 160`;

        $bf_hoverPosZField = `floatFieldGrp
            -label "Position Z (cm)" -numberOfFields 1
            -value1 0.0 -precision 3 -enable false
            -columnWidth2 120 160`;

        separator -height 6 -style "in";

        $bf_hoverRotYField = `floatFieldGrp
            -label "Heading (deg)" -numberOfFields 1
            -value1 0.0 -precision 3
            -annotation "Body yaw angle (0 = facing -Z)"
            -columnWidth2 120 160`;

    setParent ..;
setParent ..;
```

#### c. Update `bfUpdateMode()` to show/hide hover frame

```mel
global proc bfUpdateMode()
{
    global string $bf_modeMenu;
    global string $bf_pathFrame;
    global string $bf_swarmFrame;
    global string $bf_hoverFrame;

    int $idx = `optionMenuGrp -query -select $bf_modeMenu`;
    // 1=Free Flight, 2=Path Following, 3=Swarm, 4=Hover
    frameLayout -edit -visible ($idx == 2) $bf_pathFrame;
    frameLayout -edit -visible ($idx == 3) $bf_swarmFrame;
    frameLayout -edit -visible ($idx == 4) $bf_hoverFrame;
}
```

#### d. Update `bfSimulateCallback()` to pass hover flags

```mel
if ($mode == 4) {
    // Hover mode
    $cmd += " -mode 4";
    int $useRig = `checkBoxGrp -query -value1 $bf_hoverUseRigCheck`;
    if (!$useRig) {
        float $hx = `floatFieldGrp -query -value1 $bf_hoverPosXField`;
        float $hy = `floatFieldGrp -query -value1 $bf_hoverPosYField`;
        float $hz = `floatFieldGrp -query -value1 $bf_hoverPosZField`;
        $cmd += " -hoverPosX " + $hx + " -hoverPosY " + $hy + " -hoverPosZ " + $hz;
    }
    float $hry = `floatFieldGrp -query -value1 $bf_hoverRotYField`;
    $cmd += " -hoverRotY " + $hry;
}
```

#### e. Update `bfReset()` to reset hover fields

```mel
floatFieldGrp -edit -value1 0.0 $bf_hoverPosXField;
floatFieldGrp -edit -value1 0.0 $bf_hoverPosYField;
floatFieldGrp -edit -value1 0.0 $bf_hoverPosZField;
floatFieldGrp -edit -value1 0.0 $bf_hoverRotYField;
```

### 3. No changes needed

- **`BFWingModel`**: Already works at zero speed — `kMinFreq`/`kMinAmp` floors keep flapping alive.
- **`BFManeuverController`**: Not called in hover mode, so no changes needed.
- **`BFState`**: No new state fields required. Existing `position`, `heading`, `velocity`, and angle fields suffice.
- **`BFAerodynamics` / `BFCurlNoise`**: Not invoked in hover mode.

---

## Wing Behavior in Hover

At zero velocity, the sigmoid (Eq. 2-3) targets:
- `freq = range / (1 + exp(-16 * (0 - 0.5)))` = `range / (1 + exp(8))` ≈ `range * 0.000335`

This is near-zero, but the floor values override:

| Angle | Floor Freq (Hz) | Floor Amp (deg) | Visual Effect |
|-------|-----------------|-----------------|---------------|
| Beta (thorax pitch) | 1.0 | 5.0 | Gentle body bob |
| Gamma (wing flap) | 5.0 | 40.0 | Steady hovering flap |
| Zeta (feather) | 5.0 | 2.0 | Subtle feathering |
| Psi (sweep) | 5.0 | 4.0 | Slight fore-aft sweep |
| Phi (abdomen) | 5.0 | 8.0 | Counter-phase abdomen sway |

The `kAdaptTime = 3.0s` filter means freq/amp will smoothly ramp from the initial mid-range defaults down to these floor values over ~3 seconds. This produces a natural-looking "settling" animation at the start of the hover.

---

## Edge Cases

1. **Duration = 0**: No frames baked. No special handling needed (existing loop guard covers this).

2. **Hover position below ground (Y < 0)**: Allow it — the user may want to place the butterfly on a surface. No ground-plane collision in the current system.

3. **Very long durations**: Since position is fixed and angles are periodic, the animation will loop naturally. No accumulation errors. Memory is bounded by the sliding-window history (`kWindowSize + 1` entries), though in hover mode `smoothParameters` is still called at cycle boundaries since `wingModel.update()` runs. This is fine — it smooths the freq/amp values toward their floor targets.

4. **Transitioning from path to hover**: Not supported in v1. Each simulation run uses a single mode. A future extension could chain modes (e.g., path -> hover at endpoint), but this is out of scope for initial implementation.

---

## Future Extensions (Out of Scope for v1)

- **Subtle drift**: Add optional low-frequency Perlin noise displacement to make the hover feel more alive (real butterflies don't hover perfectly still). Could reuse `BFCurlNoise` at very low `eta`.
- **Banking on hover rotation change**: If heading changes over time, add a slight roll/bank.
- **Hover-to-flight transition**: Smoothly blend from hover into free flight or path following.
- **Altitude hold with physics**: Run full physics but with a PD controller counteracting gravity, producing more organic vertical oscillation. More complex but more realistic.
