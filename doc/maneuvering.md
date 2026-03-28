# Maneuvering Control — Re-integration Plan

**ButterFlight | CIS 6600 | Yiding Tian & Cecilia Chen**
Last updated: 2026-03-27

---

## 1. Current State (Post-Alpha)

The alpha build disabled the full dynamics pipeline and replaced it with a **pure kinematic loop** in `BFSimulateCmd::doIt()`:

```cpp
// Alpha: kinematic only — constant freq/amp, no physics
m_state.phase += dt;
wingModel.updateAnglesOnly(m_state);   // angles only, no cycle detection
applyAngles(...);
writeAllKeys(...);
```

What is **built but not called**:
| Class | Method | Purpose |
|---|---|---|
| `BFWingModel` | `update()` | Phase advance + cycle-boundary freq/amp recomputation |
| `BFManeuverController` | `step()` | Force integration → velocity + position update |
| `BFManeuverController` | `smoothParameters()` | Sliding-window smoother (Eq. 12) |
| `BFAerodynamics` | `computeTotalForce()` | Lift + drag from all four wings |
| `BFCurlNoise` | `computeForce()` | Vortex perturbation force |

Root joint translation (`BF_body`) is **never keyframed** — the butterfly stays at the world origin regardless of what `state.position` computes.

---

## 2. Why It Was Disabled

The previous integration attempt produced two visible artifacts:

1. **Flat keyframes after frame ~6** — `state.velocity` was never updated, so `state.velocity.length() == 0` every cycle. The sigmoid (Eqs. 2-3) at zero speed returns `range / (1 + exp(8)) ≈ 0`, collapsing all frequencies and amplitudes to near-zero after the first cycle boundary.

2. **Incorrect cycle-boundary detection** — `wingModel.update()` uses `state.frequency` for period detection. When `state.frequency` diverged from the actual per-angle frequencies being used, cycle boundaries fired at wrong times, causing phase discontinuities (wing "snapping").

The dynamics code was correct; the wiring between modules was broken.

---

## 3. What Needs to Change

### 3.1 Restore the full simulation loop in `BFSimulateCmd::doIt()`

Replace the kinematic loop with the dynamics loop:

```cpp
BFWingModel wingModel;
BFManeuverController controller;
controller.maxSpeed = wingModel.maxSpeed;

// Initialize position from the BF_body joint's current world translation
{
    MFnTransform rootFn(m_state.skeleton.joints[kThorax]);
    MVector t = rootFn.getTranslation(MSpace::kWorld);
    m_state.position = MPoint(t.x, t.y, t.z);
}

for (int f = startFrame; f < startFrame + duration; ++f) {
    int prevCycle = m_state.flapCycle;

    // 1. Advance wing kinematics + cycle detection + freq/amp recompute
    wingModel.update(m_state, dt);

    // 2. Integrate forces → velocity → position
    controller.step(m_state, dt);

    // 3. Apply Eq. 12 smoother at every cycle boundary
    if (m_state.flapCycle != prevCycle)
        controller.smoothParameters(m_state);

    // 4. Apply angles to joints and bake rotation keyframes
    applyAngles(m_state.skeleton, m_state.angles);
    MTime frameTime((double)f, MTime::uiUnit());
    writeAllKeys(m_state.skeleton, m_state.angles, frameTime);

    // 5. NEW: also bake root translation keyframe
    writeTranslationKey(m_state.skeleton.joints[kThorax],
                        m_state.position, frameTime);
}
```

---

### 3.2 Add `writeTranslationKey()` to `BFSimulateCmd.cpp`

The controller integrates `state.position` each frame but the result is never applied to the skeleton. Need a new helper function alongside `writeAllKeys()`:

```cpp
// Writes one translation keyframe on BF_body (X, Y, Z).
// Uses MFnAnimCurve::kAnimCurveTL (translate) curves on
// translateX / translateY / translateZ plugs.
static MStatus writeTranslationKey(const MDagPath& rootJoint,
                                   const MPoint&   pos,
                                   const MTime&    t);
```

Implementation mirrors `writeRotationKey()` but targets the `translate[XYZ]` plugs instead of `rotate[XYZ]`, and uses `kAnimCurveTL` (translate-linear) curve type.

---

### 3.3 Fix the `BFAerodynamics` wing-normal input timing issue

`BFAerodynamics::computeWingForce()` reads joint world-space matrices via `getWingNormal()` and `getWingSpanDir()`. During baking, joints have **stale transforms** from the previous frame because `applyAngles()` sets DG nodes but the Maya evaluator may not have refreshed.

Fix: call `applyAngles()` and then `MGlobal::executeCommand("dgdirty -a")` (or `MDGContext`) **before** `controller.step()` evaluates aerodynamics, so the joint matrices reflect the current frame's rotations.

Alternatively, pass the maneuvering angles directly into an analytic wing-normal computation rather than reading from the scene graph — this is more robust but requires a small refactor of `BFAerodynamics`.

---

### 3.4 Validate sigmoid behavior at typical speeds

The sigmoid controls freq/amp adaptation:

```
f(speed) = range / (1 + exp(-16 * (|u| / |u_max| - 0.5)))
```

At `speed = 0`, `f ≈ 0.00034 * range` (near-zero but nonzero).
At `speed = maxSpeed`, `f ≈ 0.9997 * range` (near-full).
At `speed = 0.5 * maxSpeed`, `f = 0.5 * range` (hover midpoint).

**Risk**: if `maxSpeed` is set too high (e.g. 3 m/s) and the butterfly barely moves, the sigmoid always evaluates near zero and collapses amplitudes. Suggested default: `maxSpeed = 1.0 m/s` so that even slow flight produces visible wing motion.

Check this by printing `state.velocity.length()` and `state.perAngleAmp[kAngleGamma]` at each cycle boundary during a test run.

---

### 3.5 Expose physics parameters as flags (optional but recommended)

`BFSimulateCmd::syntax()` has a TODO at line 60 for additional flags. Adding these will let the UI expose physics tuning:

| Flag | Default | Description |
|---|---|---|
| `-mass` | `0.000428` | Body mass (kg) |
| `-maxSpeed` | `1.0` | Velocity clamp (m/s) |
| `-windX/Y/Z` | `0 0 0` | Uniform wind vector (m/s) |
| `-target` | none | World-space target point (enables `hasTarget`) |
| `-gainX/Y/Z` | `22 5.5 22` | Curl-noise spatial gains |
| `-eta` | `3.66` | Curl-noise force magnitude |

---

## 4. Step-by-Step Implementation Order

1. **Add `writeTranslationKey()`** — implement alongside existing `writeRotationKey()` in `BFSimulateCmd.cpp`.
2. **Restore the dynamics loop** — replace `updateAnglesOnly` call with the full `update` + `step` + `smoothParameters` loop (Section 3.1).
3. **Add root translation baking** — call `writeTranslationKey()` at each frame inside the loop.
4. **Test with a 30-frame hovering clip** — set `hasTarget = false`, check that wing angles vary across frames and that `BF_body` translates slightly (gravity + lift should roughly cancel for a Monarch at hover speed).
5. **Fix DG timing if aero forces look wrong** — add `dgdirty` or switch to analytic normal computation (Section 3.3).
6. **Expose flags** — add `-mass`, `-maxSpeed`, `-windX/Y/Z` to `syntax()` and wire into the UI fields that already exist in `butterFlight_ui.mel`.

---

## 5. Files to Modify

| File | Change |
|---|---|
| `src/BFSimulateCmd.cpp` | Restore dynamics loop; add `writeTranslationKey()` |
| `src/BFState.h` | No change needed |
| `src/BFWingModel.h/cpp` | No change needed (`update()` already correct) |
| `src/BFManeuverController.h/cpp` | No change needed (`step()` already correct) |
| `src/BFAerodynamics.h/cpp` | Possibly add analytic normal path (Section 3.3) |
| `src/mel/butterFlight_ui.mel` | Wire Force Parameters panel fields to new flags |

---

## 6. Acceptance Criteria

- [ ] 120-frame simulation produces **different** rotation values across all frames (no flat segments)
- [ ] `BF_body` has non-zero translation keyframes — butterfly visibly moves through space
- [ ] Wing amplitude and frequency change between flapping cycles (sigmoid is active)
- [ ] Butterfly does not fly to infinity — velocity is clamped by `maxSpeed`
- [ ] No phase discontinuities at cycle boundaries (wing angles are continuous)

---

## 7. Bug Investigation: Wing Angle Discontinuity at First Cycle Boundary

**Status:** Active bug — reproduced after dynamics loop was restored with
decoupled FPS.

### 7.1 Observed symptom

After re-enabling maneuvering control, the four wing joints snap to
extreme rotations at **exactly** the first flap-cycle boundary.  The
boundary always falls at frame `fps × flapPeriod`:

| fps | flapPeriod | Boundary frame | framesPerCycle |
|-----|-----------|----------------|----------------|
| 60  | 2.0 sec   | 120            | 120            |
| 60  | 1.0 sec   | 60             | 60             |
| 30  | 2.0 sec   | 60             | 60             |

`BF_forewing_L` rotation data around the boundary (`MControlDebug.csv`,
fps=60, flapPeriod=2s):

| Frame | rotateX (zeta) | rotateY (psi) | rotateZ (gamma) |
|-------|---------------|---------------|-----------------|
| 118   | −0.278        | 9.985         | 59.923          |
| 119   | −0.098        | 9.998         | 59.990          |
| **120** | **0.327**   | **19.983**    | **159.869**     |
| 121   | 1.045         | 19.884        | 159.129         |

Gamma jumps from ~60° to ~160° in **one frame** (+100°).  Psi doubles
from ~10° to ~20°.  After the boundary, motion continues smoothly (no
further snaps) because the smoother begins to function.

### 7.2 Root cause: two simultaneous discontinuities

The bug is a **two-hit** problem — both phase and amplitude change
abruptly at the same instant.

#### Hit 1: Gravity-induced velocity → sigmoid overreaction

The butterfly starts from rest.  During the first cycle, `controller.step()`
integrates gravity every frame:

```
sim_time per cycle = 1 / f_gamma = 1/5.5 = 0.1818 sec

Gravity-only speed after one cycle:
  v = g × t = 9.81 × 0.1818 = 1.78 m/s

Fraction of maxSpeed (2.0 m/s):
  s = 1.78 / 2.0 = 0.89
```

The sigmoid (Eq. 2) at s=0.89:

```
sigmoid(1.78, R) = R / (1 + exp(-16 × (0.89 − 0.5)))
                 = R / (1 + exp(-6.24))
                 = R / (1 + 0.00195)
                 ≈ 0.998 × R
```

This returns **near-maximum values** for every parameter:

| Angle | Param | Default (cycle 0) | Sigmoid output (cycle 1) | Jump |
|-------|-------|--------------------|--------------------------|------|
| gamma | freq  | 5.5 Hz             | 10.98 Hz                 | +5.5 |
| gamma | amp   | 50°                | 149.7°                   | +100° |
| psi   | amp   | 10°                | 19.96°                   | +10° |
| zeta  | amp   | 5°                 | 9.98°                    | +5° |
| phi   | amp   | 17°                | 34.93°                   | +18° |

The sigmoid interprets the **downward falling speed** as fast forward
flight.  A 0.428 g insect reaches 89% of maxSpeed from gravity alone
in just 0.18 seconds, with or without any meaningful lift being
generated.

#### Hit 2: Smoother bypassed on the first boundary

`smoothParameters()` (Eq. 12) is supposed to dampen parameter changes
between cycles.  But at the **first** boundary:

```cpp
state.freqHistory.push_back(freqSnap);   // push raw sigmoid values
int n = state.freqHistory.size();         // n = 1
if (n < 2) return;                        // ← EXIT: no smoothing
```

The history is empty before the first cycle.  The raw, unsmoothed
sigmoid output is used directly — the smoother does nothing.

#### Combined effect at frame 120

1. **Phase wraps** to near 0:  `phase -= 1/5.5 → ≈ 0.001`
2. **Amplitude jumps** from 50° to ~150° (raw sigmoid)
3. **evalAngle** computes:  `gamma = 149.7 × cos(2π × 10.98 × 0.001 + 0) + 10`
   `≈ 149.7 × 0.998 + 10 ≈ 159.5°`
4. Previous frame had:  `gamma = 50 × cos(near-end-of-cycle) + 10 ≈ 60°`
5. **Net jump: +100° in one frame**

After the first boundary, the smoother has history and begins
functioning — subsequent transitions are dampened, which is why the
motion appears smooth from frame 121 onward.

### 7.3 Why FPS decoupling didn't fix this

The FPS decoupling correctly separates keyframe rate from flap speed.
But this bug is not caused by the FPS-flap coupling — it's caused by:

1. The sigmoid mapping `|velocity|` (including gravity freefall) to
   wing parameters
2. The smoother having no history to dampen the first cycle transition

These two factors are independent of how `dt` is computed.  The
decoupling was necessary (and still correct), but it was a separate
concern.

### 7.4 Proposed fixes

#### Fix A: Seed smoother history with initial defaults

Pre-populate `freqHistory` and `ampHistory` with k copies of the
initial default freq/amp before the simulation starts.  This gives the
smoother a full window to blend against at the first boundary.

```cpp
// Before the simulation loop:
std::vector<double> initFreq(BFState::kNumAngles);
std::vector<double> initAmp(BFState::kNumAngles);
for (int a = 0; a < BFState::kNumAngles; ++a) {
    initFreq[a] = m_state.perAngleFreq[a];
    initAmp[a]  = m_state.perAngleAmp[a];
}
for (int i = 0; i < BFManeuverController::kWindowSize; ++i) {
    m_state.freqHistory.push_back(initFreq);
    m_state.ampHistory.push_back(initAmp);
}
```

**Effect:** First boundary would compute
`smoothed_amp = 0.5 × 50 + 0.5 × 149.7 = 99.85°` — still a jump,
but cut in half.  Over subsequent cycles, the smoother continues
dampening.

**Limitation:** 0.5 × history + 0.5 × current still allows a 50% jump
in one cycle.  May need multiple cycles to settle.

#### Fix B: Clamp maximum parameter change per cycle

After the sigmoid computes raw values, clamp the change relative to
the previous cycle:

```cpp
double maxFreqDelta = 2.0;    // Hz per cycle
double maxAmpDelta  = 20.0;   // degrees per cycle

for (int i = 0; i < BFState::kNumAngles; ++i) {
    double rawFreq = params[i].freqRangeMin + evalSigmoid(speed, freqRange);
    double rawAmp  = params[i].ampRangeMin  + evalSigmoid(speed, ampRange);

    state.perAngleFreq[i] = std::clamp(rawFreq,
        state.perAngleFreq[i] - maxFreqDelta,
        state.perAngleFreq[i] + maxFreqDelta);
    state.perAngleAmp[i]  = std::clamp(rawAmp,
        state.perAngleAmp[i] - maxAmpDelta,
        state.perAngleAmp[i] + maxAmpDelta);
}
```

**Effect:** Gamma amplitude can grow by at most 20°/cycle: 50→70→90→...
Smooth ramp-up over several cycles.

**Limitation:** Adds tuning parameters not in the paper; may delay
response to legitimate speed changes.

#### Fix C (Recommended): Combine A + B

Seed the history (Fix A) AND apply per-cycle clamping (Fix B).
History seeding handles the cold-start; clamping prevents any single
cycle from producing a visible discontinuity regardless of velocity.

This matches the paper's intent: Eq. 12 is designed to prevent abrupt
transitions, but the paper assumes the simulation starts in a
quasi-steady state (Unity initializes butterflies with nonzero
velocity), so it never encounters the empty-history edge case.

#### Fix D (Alternative): Exclude vertical velocity from sigmoid

Use only horizontal speed for the sigmoid evaluation:

```cpp
double horizSpeed = std::sqrt(state.velocity.x * state.velocity.x
                            + state.velocity.z * state.velocity.z);
```

This prevents gravity-induced freefall from being interpreted as fast
forward flight.  However, this deviates from the paper's formulation
which uses `|u|` (total velocity magnitude), and may cause issues for
butterflies that intentionally dive.

### 7.5 Recommended implementation order

1. Apply **Fix A** (seed history) — minimal code change, immediately
   prevents the smoother bypass.
2. Apply **Fix B** (per-cycle clamping) — prevents any future
   discontinuity regardless of velocity conditions.
3. Test with fps=60, flapPeriod=2s — verify no snap at frame 120.
4. Tune `maxAmpDelta` / `maxFreqDelta` if the ramp-up is too slow
   or too fast for the desired visual result.
