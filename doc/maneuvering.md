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

### 7.3 Debug CSV analysis (doc/debug/1.csv)

**Command:**
`bfSimulate -rigRoot BF_body -duration 960 -flapPeriod 2 -dbg "...1.csv"`

This run includes Fix A (history seeding) and Fix B (±20°/cycle clamp).
The CSV shows ±10 frames around every cycle boundary.

#### 7.3.1 All cycle boundaries

| Frame | GammaFreq | GammaAmp | ThetaGamma | Prev ThetaGamma | Jump |
|-------|-----------|----------|------------|-----------------|------|
| 119   | 5.5       | 50       | 59.93      | —               | —    |
| **120** | **6.5** | **60** | **80.0**   | 59.93           | **+20.1°** |
| 221   | 6.5       | 60       | 69.97      | —               | —    |
| **222** | **7.1** | **66** | **89.9**   | 69.97           | **+19.9°** |
| 314   | 7.1       | 66       | ~76        | —               | —    |
| **315** | **7.55**| **70.5**| **95.9**   | ~76             | **~+20°** |
| 401   | 7.55      | 70.5     | ~80        | —               | —    |
| **402** | **7.96**| **74.6**| **100.5**  | ~80             | **~+20°** |

The clamp IS working — amplitude grows by exactly `kMaxAmpDelta=20°`
per cycle instead of 100°.  But the ~20° discontinuity repeats at
**every** cycle boundary.  It does not diminish over time.

Note: speed is clamped at 2.0 m/s (maxSpeed) throughout.  The sigmoid
evaluates at s=1.0 and always requests maximum parameter values, so the
clamp fires at every single boundary, producing a permanent +20° stair-
step pattern.

#### 7.3.2 The structural problem: amplitude changes at the cosine peak

The cycle boundary fires when `phase >= 1/frequency`, then wraps phase
to ~0.  At `phase ≈ 0`, `cos(0 + phi_p) = cos(0) = 1.0` for gamma
(whose `phi_p = 0°`).  And at the end of the old cycle,
`cos(2π × f × phase_end)` is also ≈ 1.0 because `phase_end ≈ 1/f`
(one full period).

This means both the **last frame of the old cycle** and the
**first frame of the new cycle** evaluate the cosine near its peak.
The angle difference between them is:

```
theta_new - theta_old = (amp_new × cos_new + mean) - (amp_old × cos_old + mean)
                      ≈ (amp_new × 1.0) - (amp_old × 1.0)
                      = amp_new - amp_old
                      = delta_amp
```

**Any nonzero amplitude change at a cycle boundary creates a
discontinuity exactly equal to that amplitude change.**  This is a
structural consequence of the boundary always falling at `cos ≈ 1`.
It cannot be fixed by tuning `kMaxAmpDelta` — even `kMaxAmpDelta = 5`
would produce a visible 5° snap every single cycle.

For the same reason, frequency changes also cause a smaller
discontinuity: the new frequency shifts the cosine argument for the
wrapped phase, but since `phase ≈ 0`, this effect is minimal compared
to the amplitude term.

#### 7.3.3 Smoother timing: one frame too late

The simulation loop order is:

```
1. wingModel.update()       ← evaluates angles using NEW freq/amp
2. applyAngles()
3. controller.step()
4. smoothParameters()       ← dampens freq/amp AFTER angles are baked
5. writeAllKeys()           ← writes angles from step 1
6. debugLog records freq/amp from step 4 (post-smooth)
```

At a cycle boundary, `wingModel.update()` detects the boundary,
computes new clamped freq/amp, and immediately evaluates `evalAngle()`
with those new values.  Only **after** the angles are already computed
does `smoothParameters()` run and dampen the freq/amp for the **next**
frame.

This means:
- The boundary frame always uses **raw clamped** parameters (not
  smoothed).
- The CSV shows **post-smooth** values (because debug recording is
  after step 4), which differ from what was actually used to compute
  the baked angles.
- The smoother never has a chance to affect the boundary frame itself.

**Evidence from CSV:**

Frame 120 shows `GammaAmp = 60.0` (post-smooth = 0.5×50 + 0.5×70),
but `ThetaGamma = 80.0`, which corresponds to `amp = 70` (pre-smooth
clamped value): `70 × cos(0) + 10 = 80`.  If the smoothed value of 60
had been used: `60 × cos(0) + 10 = 70`, which would be a 10° jump
instead of 20°.

### 7.4 Why FPS decoupling didn't fix this

The FPS decoupling correctly separates keyframe rate from flap speed.
But this bug is not caused by the FPS-flap coupling — it's caused by:

1. The sigmoid mapping `|velocity|` (including gravity freefall) to
   wing parameters at near-maximum values
2. Amplitude changes at cycle boundaries produce discontinuities
   structurally equal to the amplitude delta (cos peak alignment)
3. The smoother runs after angle evaluation, so it never affects the
   boundary frame

These factors are independent of how `dt` is computed.  The decoupling
was necessary (and still correct), but it was a separate concern.

### 7.5 Why Fix A + Fix B (history seeding + clamping) are insufficient

Fix A (history seeding) ensures the smoother doesn't exit early at the
first boundary.  Fix B (per-cycle clamping) limits amplitude growth to
`kMaxAmpDelta` per cycle.  Together they reduced the first-boundary
jump from 100° to 20°.

However, the 20° jump **repeats at every boundary** indefinitely:
- Speed is maxed at 2.0 m/s from gravity alone.
- Sigmoid always requests maximum parameter values.
- Clamp fires every cycle, always adding exactly `kMaxAmpDelta`.
- The jump equals the amplitude delta because cos ≈ 1 at both sides
  of the boundary.
- Reducing `kMaxAmpDelta` makes the jump smaller but also makes the
  butterfly take more cycles to reach cruising parameters — and the
  jump still exists.

The smoother (Fix A) does dampen the stored values, but because it
runs after angle evaluation, it only helps frames **after** the
boundary — not the boundary frame itself.

### 7.6 Proposed fix strategies

#### Strategy 1: Evaluate angles AFTER smoothing at boundaries

Move `evalAngle()` for boundary frames to after `smoothParameters()`.
Currently, `wingModel.update()` does both cycle detection and angle
evaluation in one call.  Splitting these into separate steps would
allow:

```
1. wingModel.advancePhaseAndDetectBoundary(state, dt)
   → advances phase, detects boundary, computes raw new freq/amp
   → does NOT evaluate angles yet

2. smoothParameters(state)   [if boundary]
   → dampens freq/amp using history

3. wingModel.evaluateAngles(state)
   → evaluates angles using the SMOOTHED freq/amp
```

**Effect:** At frame 120, smoothed amp = 0.5×50 + 0.5×70 = 60°.
`theta = 60 × cos(0) + 10 = 70°`.  Jump = 70 - 60 = 10° (from 60
to 70).  Still nonzero, but halved.  Over subsequent boundaries,
the history blends further and the jumps shrink.

**Limitation:** The smoother's 0.5 × history + 0.5 × current blend
still allows jumps equal to half the amplitude delta.  At the start
when the clamp fires every cycle, this is `0.5 × kMaxAmpDelta = 10°`
— visible but much less severe.

#### Strategy 2: Interpolate parameters within the cycle (no boundary discontinuity)

Instead of changing parameters abruptly at the cycle boundary,
interpolate between old and new parameters over the first N frames of
the new cycle:

```cpp
// At boundary: store old and new params
double ampOld = state.perAngleAmp[i];   // before sigmoid
double ampNew = clampedSigmoidResult;   // after sigmoid + clamp
state.perAngleAmpTarget[i] = ampNew;    // target to ramp toward

// Each frame within the cycle:
double t = state.phase / cyclePeriod;   // 0..1 within cycle
double blendFraction = smoothstep(0, 0.25, t);  // ramp over first 25%
double ampEffective = lerp(ampOld, ampTarget, blendFraction);
theta = ampEffective * cos(2*pi*f*phase + phi_p) + phi_m;
```

**Effect:** Parameter changes ramp in gradually over the first quarter
of each cycle.  The boundary frame uses the old parameters, so there
is **zero discontinuity** at the boundary.  The new parameters are
fully active by 25% into the cycle, where the cosine has moved away
from its peak and the amplitude change is less visible.

**Limitation:** Adds per-angle target/old state arrays and a blend
computation.  Deviates from the paper's instantaneous-at-boundary
design, but arguably more physically plausible (a real butterfly
cannot instantaneously change its wing stroke amplitude).

#### Strategy 3: Blend old and new angles (post-hoc smoothing)

Keep the current boundary logic, but blend the post-boundary angle
with the pre-boundary angle over several frames:

```cpp
// At boundary: save the angle values from the previous frame
double prevAngles[kNumAngles] = { state.angles.thetaBeta, ... };

// After evalAngle:
if (framesAfterBoundary < blendWindow) {
    double t = framesAfterBoundary / (double)blendWindow;
    state.angles.thetaGamma = lerp(prevAngles[gamma], state.angles.thetaGamma, t);
    // ... same for all angles
}
```

**Effect:** The visible angle transitions smoothly from pre-boundary
to post-boundary over `blendWindow` frames.  Zero discontinuity.

**Limitation:** The blended angles don't correspond to any physical
equation — they're purely a visual smoothing post-process.  Could
produce non-physical intermediate poses.  Also, the aero model would
see the smoothed angles rather than the true equations, which could
affect force computation.

#### Strategy 4 (Recommended): Strategy 1 + Strategy 2

Split `wingModel.update()` so that:
1. Phase advance + boundary detection + clamped sigmoid → produces
   new raw parameters
2. `smoothParameters()` at boundary → dampens parameters
3. Within-cycle interpolation ramps from old to new smoothed
   parameters over first ~25% of cycle
4. `evalAngle()` uses the interpolated parameters

This eliminates both the smoother-timing problem (Strategy 1) and
the structural cos-peak alignment problem (Strategy 2).  The result
is zero discontinuity at boundaries, physically motivated parameter
ramps, and full compatibility with the paper's Eq. 12 smoother.

### 7.7 Implementation plan for Strategy 4

#### Step 1: Add per-angle "old" and "target" arrays to BFState

```cpp
// In BFState:
double perAngleFreqOld[kNumAngles];     // values at start of current cycle
double perAngleFreqTarget[kNumAngles];  // smoothed values to ramp toward
double perAngleAmpOld[kNumAngles];
double perAngleAmpTarget[kNumAngles];
```

#### Step 2: Split BFWingModel::update() into three methods

```cpp
// Advance phase, detect boundary, compute raw clamped params.
// Returns true if a boundary was crossed.
bool advancePhase(BFState& state, double dt);

// Evaluate Eq. 1 using current effective params.
// Blends old→target over first 25% of cycle.
void evaluateAngles(BFState& state) const;

// The existing evalAngle() and evalSigmoid() remain as-is.
```

#### Step 3: Update simulation loop in BFSimulateCmd::doIt()

```cpp
for (int f = startFrame; f < startFrame + duration; ++f) {
    // 1. Phase advance + boundary detection
    bool boundary = wingModel.advancePhase(m_state, dt_sim);

    // 2. Smooth at boundaries (now BEFORE angle evaluation)
    if (boundary)
        controller.smoothParameters(m_state);

    // 3. Evaluate angles (uses smoothed + interpolated params)
    wingModel.evaluateAngles(m_state);

    // 4-5. Apply and bake (unchanged)
    applyAngles(m_state.skeleton, m_state.angles);
    controller.step(m_state, dt_sim);
    MTime frameTime((double)f, MTime::uiUnit());
    writeAllKeys(m_state.skeleton, m_state.angles, frameTime);
    writeTranslationKey(m_state.skeleton.joints[kThorax],
                        m_state.position, frameTime);
}
```

#### Step 4: Implement within-cycle interpolation in evaluateAngles()

```cpp
void BFWingModel::evaluateAngles(BFState& state) const
{
    double cyclePeriod = 1.0 / state.frequency;
    double cycleT = state.phase / cyclePeriod;       // 0..1 in cycle

    // Ramp fraction: 0 at boundary, 1 at 25% into cycle
    double ramp = std::min(1.0, cycleT / 0.25);
    // Smooth the ramp (cubic ease)
    ramp = ramp * ramp * (3.0 - 2.0 * ramp);

    for (int i = 0; i < BFState::kNumAngles; ++i) {
        double freq = state.perAngleFreqOld[i]
                    + ramp * (state.perAngleFreqTarget[i] - state.perAngleFreqOld[i]);
        double amp  = state.perAngleAmpOld[i]
                    + ramp * (state.perAngleAmpTarget[i] - state.perAngleAmpOld[i]);
        // evalAngle uses the interpolated freq/amp
        // ...
    }
}
```

#### Step 5: Files to modify

| File | Change |
|------|--------|
| `src/BFState.h` | Add `perAngleFreqOld/Target`, `perAngleAmpOld/Target` arrays |
| `src/BFWingModel.h` | Declare `advancePhase()` and `evaluateAngles()` |
| `src/BFWingModel.cpp` | Split `update()` into three methods; implement within-cycle interpolation |
| `src/BFSimulateCmd.cpp` | Reorder loop to: advancePhase → smoothParameters → evaluateAngles → applyAngles → step → writeKeys |

#### Step 6: Expected results

At frame 120 (first boundary, fps=60, flapPeriod=2s):
- `advancePhase()` detects boundary, clamps amp to 70° (old=50 + 20).
- `smoothParameters()` blends: target = 0.5×50 + 0.5×70 = 60°.
- Old amp = 50°, target amp = 60°.  `cycleT = 0/period = 0`.
  `ramp = 0`.  Effective amp = 50° (old value).
- `theta = 50 × cos(0) + 10 = 60°` — **identical to frame 119**.
  **Zero discontinuity.**

At frame 121:
- `cycleT ≈ 1/120 ≈ 0.008`.  `ramp ≈ 0.033`.
  Effective amp = 50 + 0.033 × (60 - 50) = 50.33°.
- Smooth ramp begins.

At frame 150 (25% into cycle = 30 frames):
- `ramp = 1.0`.  Effective amp = 60° (full target).
- Cycle continues at the new parameters.
