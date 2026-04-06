# Path Following Debug — 2026-04-05

## Cecilia's Commit: `be2c9d5` "Fix unit conversion and orientation"

### Changes

#### 1. Unit conversion (cm / m)

Physics engine runs in SI metres; Maya scene is in centimetres. Previously the code assumed both used the same units, so the physics-integrated displacement (metres) was written directly as Maya translation values (interpreted as cm). The butterfly appeared to barely move because 1 m of physics displacement showed up as 1 cm in Maya.

Constants added in `BFSimulateCmd.cpp`:
```cpp
static constexpr double kCmToM = 0.01;
static constexpr double kMToCm = 100.0;
```

Conversions applied at every boundary between Maya and physics:

| Location | Conversion |
|----------|------------|
| Seed position from root joint | Maya cm -> physics m |
| Snap to curve start | curve cm -> physics m (position), stays cm for `setTranslation` |
| `closestPoint()` query | physics m -> Maya cm before query |
| Lead distance for arc advance | `sensorRange * 0.5` m -> cm via `* kMToCm` |
| `controller.target` from curve | curve cm -> physics m |
| `writeTranslationKey()` | physics m -> Maya cm |

#### 2. Orientation (heading)

Previously the thorax rotation was purely `(pitch, 0, 0)` — the butterfly always faced the same world direction regardless of velocity or path. Now:

- Added `BFState::heading` (double, radians) — body yaw.
- `applyAngles()` signature changed to include `heading`; applied as thorax Y rotation: `MEulerRotation(pitch, heading, 0)`.
- Heading computed from velocity each substep: `heading = atan2(-vx, -vz)` when horizontal speed > 0.01 m/s.
- When a path is provided, initial heading is seeded from the curve's start tangent: `atan2(-tx, -tz)`.
- `writeAllKeys()` and `writeRotationKey` for thorax now include heading.

#### 3. Other changes

- **`clearAnimCurves()`**: New function that strips all keys from every joint before re-simulation. Prevents stale keys from previous runs polluting the result.
- **Linear tangents**: Changed `kTangentAuto` -> `kTangentLinear` for both rotation and translation keys. Prevents Maya's auto-tangent interpolation from overshooting between per-frame keys.
- **Min frequency/amplitude floors** (`BFWingModel.cpp`): Added `kMinFreq` and `kMinAmp` arrays. Without these, the sigmoid drives freq to ~0 at near-zero speed, effectively freezing the wings after the first cycle. Floors: gamma >= 5 Hz, gamma amp >= 40 deg.
- **`updateAnglesOnly()`** (`BFWingModel.cpp`): New function that evaluates all 5 angles from accumulated phase/amp without advancing phase or running the sigmoid filter. Used in path-following mode.
- **Path-following uses fixed-rate phase**: In path mode, phase is advanced at the initial default frequencies (bypassing velocity-adaptive sigmoid). This keeps flap rate constant regardless of flight speed.

---

## Observed Behaviour After the Fix

### Free flight mode

Previously the butterfly barely moved (unit bug). Now it moves significantly — rapid downward flight while steering right. The "steering right" is likely the vortex curl noise producing a lateral bias (asymmetric perturbation). The rapid descent is gravity + limited lift at current default parameters.

### Path following mode

The following issues were observed:

1. **Short curves are completely ignored** — the butterfly immediately enters free flight.
2. **The butterfly "takes shortcuts"** — on curved paths, it cuts across bends instead of following the curve faithfully.
3. **The last segment of the path is skipped** — the butterfly stops following before reaching the curve's end.
4. **After the path ends, the butterfly enters free flight** for the remaining duration (this is by design, but feels abrupt).

---

## Root Cause Analysis

### The lead distance is a fixed 2.25 m (225 cm)

The lead target is placed at:
```
leadArc = arcAtClosest + sensorRange * 0.5 * kMToCm
        = arcAtClosest + 4.5 * 0.5 * 100
        = arcAtClosest + 225 cm
```

This is a constant 2.25 m ahead regardless of curve length. Consequences:

- **Short curve problem**: If the total curve length < 225 cm, then on the very first substep `leadArc >= curveLen` is already true, so `hasTarget` is immediately set to `false`. The butterfly never receives a target and enters free flight on frame 1.
- **End-of-curve skip**: The last 2.25 m of *any* curve is unreachable. Once the closest point is within 2.25 m of the end, the lead target overshoots the curve end, triggering the coast-off.

### `closestPoint()` is not monotonic — causes shortcutting

Each substep independently calls `closestPoint(pos)` with no memory of where the butterfly was on the curve before. On a curved path:

```
        B (butterfly drifted outward)
       /
   ---*---<====>--- curve
      ^closest     ^lead (2.25m ahead)
```

If the butterfly drifts outward at a bend, `closestPoint()` can snap to a point that is geometrically nearest but further along the curve than the butterfly has actually progressed. The lead target then jumps forward, and the butterfly aims across the bend — "taking a shortcut."

Worse: on S-curves or loops, `closestPoint()` can jump to a completely different segment of the curve, skipping large sections.

### No forward-only constraint on arc progress

There is no mechanism to ensure the butterfly's "progress along the curve" only moves forward. The algorithm has no state tracking where the butterfly was on the curve last substep. Each substep is fully memoryless with respect to curve progress.

### Interaction with unit conversion

The unit conversion itself is correct. But it exposed a scaling issue: before the fix, the butterfly barely moved, so it stayed close to the curve and the lead distance was in the right ballpark (relative to cm-scale curves). Now that displacement is 100x larger (m correctly converted), the butterfly moves much faster and drifts further from the curve, amplifying all of the above issues.

---

## Suggested Fixes

### 1. Track monotonic arc progress (critical)

Store a `lastArcProgress` variable (in cm, the curve's native unit). Each substep, compute `closestPoint()` as now, but clamp it forward-only:

```cpp
double arcAtClosest = curveFn.findLengthFromParam(uClosest);
arcAtClosest = std::max(arcAtClosest, lastArcProgress);  // never go backward
lastArcProgress = arcAtClosest;
```

This prevents the butterfly from jumping ahead on bends or regressing on S-curves.

### 2. Scale lead distance to remaining curve length

Instead of a fixed 2.25 m lead, use a fraction of the remaining path:

```cpp
double remaining = curveLen - arcAtClosest;
double leadDist = std::min(sensorRange * 0.5 * kMToCm, remaining * 0.5);
```

This ensures the lead target always stays within the remaining curve, and naturally shrinks to zero as the butterfly approaches the end — so the last segment is actually followed.

### 3. Use curve-end target instead of immediate coast-off

When `leadArc >= curveLen`, instead of setting `hasTarget = false` immediately, set the target to the curve's endpoint and only disable following when the butterfly is within some small radius of the end:

```cpp
if (leadArc >= curveLen) {
    // Target the endpoint directly
    MPoint endPt;
    curveFn.getPointAtParam(uMax, endPt, MSpace::kWorld);
    controller.target = MPoint(endPt.x * kCmToM, endPt.y * kCmToM, endPt.z * kCmToM);
    // Only coast off when actually close to the end
    double distToEnd = (MVector(endPt) * kCmToM - MVector(m_state.position)).length();
    if (distToEnd < 0.1)  // 10 cm
        controller.hasTarget = false;
}
```

---

## Update: Fixes 1-3 did NOT resolve shortcutting

The monotonic progress, scaled lead distance, and end-of-curve targeting were implemented but the butterfly still cut across curves. The root cause is more fundamental than the lead distance.

### Why "carrot on a stick" fails on curves

`preferredAccel()` (Eq. 8) computes a **straight-line** pull toward the target: `R(d) * (1/mass) * direction`. When the target is ahead *on the curve*, the straight-line direction from the butterfly to that target **cuts through the curve's interior**. No amount of adjusting the lead distance fixes this — even a small lead creates a chord that deviates on tight bends.

The paper (line 987) says `q_i` is "the closest attraction point." For path following, this means the closest point on the path — NOT a far-ahead lead target. The butterfly's own aerodynamic thrust provides forward motion; `a_pre` serves as lateral correction to keep it near the path.

### Implemented fix: direct velocity steering

Replaced the lead-target `a_pre` design with a two-component velocity steering system applied AFTER `controller.step()`:

1. **Tangent alignment**: desired velocity is set along the curve tangent at the butterfly's current arc position, at 70% of maxSpeed. This provides forward guidance along the curve.

2. **Lateral spring**: a correction vector pulls the butterfly back toward the closest point on the curve, proportional to distance (gain = 8.0, capped at `pathSpeed`). This prevents drift.

3. **Exponential blend**: the butterfly's actual velocity is blended toward the desired velocity each substep with `blendRate = 12.0`. This gives tight enough following (~40% correction per frame at 24fps) while preserving the organic aerodynamic oscillation from wing flapping.

4. **Heading from tangent**: in path mode, heading is derived from the curve tangent instead of velocity, giving smoother orientation that follows the curve.

5. **`controller.hasTarget = false`**: `preferredAccel()` returns zero during path following. The controller still runs `localAccel()` (aero + vortex + gravity), preserving flap dynamics.

### Tunable parameters

| Parameter | Value | Effect |
|-----------|-------|--------|
| `pathSpeed` | `maxSpeed * 0.7` (1.4 m/s) | Cruise speed along path |
| `lateralPull` gain | `8.0` | Stiffness of lateral correction (m/s per m offset) |
| `blendRate` | `12.0` | Velocity blend rate (1/s). Higher = tighter following, less drift |
| End-of-curve threshold | `0.1 m` | Distance to endpoint before coasting to free flight |

## Summary

The unit conversion and orientation fixes are correct. The initial "carrot on a stick" path-following design was fundamentally incompatible with curved paths because `preferredAccel()` pulls in straight lines. The fix bypasses `a_pre` in path mode and applies direct velocity steering (tangent + lateral spring), which is closer to how the paper uses `q_i` as the closest attraction point for path following.
