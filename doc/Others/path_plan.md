# Path Following — Implementation Plan

**ButterFlight | CIS 6600 | Yiding Tian & Cecilia Chen**

---

## Overview

The butterfly should be able to follow a user-drawn EP (edit-point) NURBS curve in Maya. The user selects or draws a curve in the viewport, assigns it in the UI, and the maneuvering controller steers the butterfly along it using the existing `preferredAccel()` (Eq. 8) attraction mechanism.

The key idea: **the curve defines a sequence of moving targets, not a rigid rail.** The butterfly is still physically simulated (aerodynamics, vortex, gravity), but a "carrot on a stick" target slides along the curve ahead of the butterfly, pulling it forward via `preferredAccel()`. This preserves the natural flight dynamics from the paper while adding directional guidance.

---

## What Already Exists

| Component | Status |
|-----------|--------|
| `BFManeuverController::target` (MPoint) | Declared, never set |
| `BFManeuverController::hasTarget` (bool) | Declared, defaults to `false` |
| `BFManeuverController::preferredAccel()` | Fully implemented (Eq. 8 with ramp) |
| MEL UI: Path Settings section (Section 4) | Built — shows when mode = "Path Following" |
| MEL UI: `bfSelectPath()` callback | Built — selects NURBS curve shape node |
| MEL UI: Path Follow Strength slider | Built — not wired |
| Command flag for path curve | **Missing** |
| Curve sampling logic in C++ | **Missing** |
| Target advancement logic | **Missing** |

---

## Architecture

### Data flow

```text
User draws EP curve → selects in UI → MEL passes curve name via flag
    → C++ resolves MFnNurbsCurve → samples closest point / lead point
    → sets controller.target each substep → preferredAccel() steers butterfly
```

### Target advancement strategy

Each substep:
1. Find the curve parameter `u_closest` nearest to `state.position` using `MFnNurbsCurve::closestPoint()`.
2. Compute a **lead parameter** `u_lead = u_closest + leadDistance / curveLength` (clamped to curve domain).
3. Evaluate the curve at `u_lead` to get the world-space target point.
4. Set `controller.target = leadPoint` and `controller.hasTarget = true`.

The `leadDistance` controls how far ahead the butterfly "looks" along the curve. A good default is `sensorRange * 0.5` (about 2.25 m). Too short and the butterfly oscillates; too long and it cuts corners.

When `u_closest` reaches the end of the curve (within some epsilon), set `hasTarget = false` to let the butterfly coast in free flight.

---

## Implementation Steps

### Step 1: Add `-path` flag to `BFSimulateCmd`

```cpp
// New flag constants
static const char* kPathFlag     = "-p";
static const char* kPathFlagLong = "-path";   // NURBS curve name

// In newSyntax():
syntax.addFlag(kPathFlag, kPathFlagLong, MSyntax::kString);
```

In `doIt()`, resolve the curve:

```cpp
MDagPath curvePath;
MFnNurbsCurve curveFn;
bool hasPath = false;

if (argData.isFlagSet(kPathFlag)) {
    MString curveName;
    argData.getFlagArgument(kPathFlag, 0, curveName);
    MSelectionList sel;
    sel.add(curveName);
    sel.getDagPath(0, curvePath);
    // If user selected transform, extend to shape
    if (curvePath.hasFn(MFn::kTransform))
        curvePath.extendToShape();
    curveFn.setObject(curvePath);
    hasPath = true;
    controller.hasTarget = true;
}
```

### Step 2: Add target advancement in the substep loop

Inside the substep loop, before `controller.step()`:

```cpp
if (hasPath) {
    // Find closest point on curve to current position
    MPoint pos(m_state.position);
    double uClosest;
    curveFn.closestPoint(pos, &uClosest, MSpace::kWorld);

    // Advance by leadDistance along the curve
    double curveLen = curveFn.length();
    double uMin, uMax;
    curveFn.getKnotDomain(uMin, uMax);
    double arcAtClosest = curveFn.findLengthFromParam(uClosest);
    double leadArc = arcAtClosest + controller.sensorRange * 0.5;

    if (leadArc >= curveLen) {
        // Reached the end — coast in free flight
        controller.hasTarget = false;
    } else {
        double uLead = curveFn.findParamFromLength(leadArc);
        MPoint leadPt;
        curveFn.getPointAtParam(uLead, leadPt, MSpace::kWorld);
        controller.target = leadPt;
        controller.hasTarget = true;
    }
}
```

### Step 3: Wire the MEL UI

In `bfSimulateCallback()`, read the path field and pass it as a flag:

```mel
string $path = `textFieldButtonGrp -query -text $bf_pathField`;
if ($path != "") {
    $cmd += " -path \"" + $path + "\"";
}
```

### Step 4: Initialize position on the curve

When a path is provided, optionally snap the butterfly's starting position to the curve start:

```cpp
if (hasPath) {
    double uMin, uMax;
    curveFn.getKnotDomain(uMin, uMax);
    MPoint startPt;
    curveFn.getPointAtParam(uMin, startPt, MSpace::kWorld);
    m_state.position = startPt;

    // Also set the root joint to the curve start
    MFnTransform rootFn(m_state.skeleton.joints[kThorax]);
    rootFn.setTranslation(MVector(startPt), MSpace::kWorld);
}
```

---

## Maya API Reference

| Method | Purpose |
|--------|---------|
| `MFnNurbsCurve::closestPoint(pt, &u)` | Find parameter `u` of nearest point |
| `MFnNurbsCurve::getPointAtParam(u, pt)` | Evaluate curve position at parameter |
| `MFnNurbsCurve::length()` | Total arc length |
| `MFnNurbsCurve::findLengthFromParam(u)` | Arc length from start to parameter `u` |
| `MFnNurbsCurve::findParamFromLength(len)` | Parameter at a given arc length |
| `MFnNurbsCurve::getKnotDomain(uMin, uMax)` | Valid parameter range |

Required header: `#include <maya/MFnNurbsCurve.h>`

---

## Tuning Parameters

| Parameter | Default | Effect |
|-----------|---------|--------|
| `leadDistance` | `sensorRange * 0.5` = 2.25 m | How far ahead the target slides. Larger = smoother turns, less responsive. Smaller = tighter following, possible oscillation. |
| `sensorRange` | 4.5 m | Controls ramp falloff in `preferredAccel()`. When butterfly is far from target, acceleration is capped by `ramp(d)`. |
| Path Follow Strength (UI) | 1.0 | Could scale `preferredAccel()` output: `aPre *= strength`. At 0 the butterfly ignores the path; at 1 it follows fully. |

---

## Files to Modify

| File | Change |
|------|--------|
| `src/BFSimulateCmd.cpp` | Add `-path` flag, resolve curve, target advancement in substep loop, optional position snap |
| `src/BFSimulateCmd.h` | Add `#include <maya/MFnNurbsCurve.h>` |
| `src/mel/butterFlight_ui.mel` | Wire `$bf_pathField` value into the `bfSimulate` command string in `bfSimulateCallback()` |

No changes needed to `BFManeuverController` — `target`, `hasTarget`, and `preferredAccel()` already work as-is.

---

## Acceptance Criteria

- [ ] User draws an EP curve, selects it in the Path Settings panel, clicks Simulate
- [ ] Butterfly starts at the curve's first CV and flies along the curve
- [ ] Flight is physically simulated (wings flap, body oscillates) — not rigidly locked to the curve
- [ ] Butterfly coasts in free flight after reaching the curve end
- [ ] Path Follow Strength slider scales the attraction force
- [ ] Works at any output fps (30, 60, etc.)
