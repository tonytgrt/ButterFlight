# FPS Parameter and Flap Cycle Mechanism

This document traces how the `-frameRate` (FPS) flag flows through the
simulation, how the flap-cycle counter works, and where the two interact.
It also identifies the suspected root cause of the **wing Z-flip bug**
observed when the maneuvering controller is enabled.

---

## 1. FPS to dt Conversion

**File:** `src/BFSimulateCmd.cpp`, lines 432-433

```cpp
if (fps <= 0.0) fps = 24.0;
double dt = 1.0 / fps;
```

The user-supplied `-frameRate` flag (default 24) is inverted to produce
`dt`, the **fixed timestep** used for every frame of the simulation.  At
24 FPS this gives `dt = 1/24 ~= 0.04167 seconds`.

**Important:** `dt` serves two roles simultaneously:

| Role | Description |
|------|-------------|
| **Physics timestep** | The force integrator in `BFManeuverController::step()` uses `dt` as the Euler integration step for velocity and position. |
| **Keyframe spacing** | One keyframe is written per frame, so `dt` also determines how densely the resulting animation curve samples the underlying cosine waveform. |

These two concerns have different requirements: accurate force integration
benefits from small `dt`, while the keyframe spacing only needs to satisfy
the Nyquist criterion for the highest oscillation frequency.

---

## 2. Phase Accumulator

**File:** `src/BFState.h`, lines 67-83

```cpp
double phase     = 0.0;   // time (seconds) within the current flapping cycle
int    flapCycle = 0;      // number of completed flapping cycles
double frequency = 5.5;    // Hz — master frequency for cycle detection
```

The phase accumulator tracks **elapsed time in seconds** within the
current flapping cycle.  It is shared by all five maneuvering angles
(beta, gamma, zeta, psi, phi), even though each angle has its own
per-angle frequency and amplitude.

Each frame, phase advances by `dt`:

```
phase_new = phase_old + dt
```

### Numerical example at 24 FPS, 5.5 Hz hovering default

| Frame | phase (sec) | 2*pi*5.5*phase (rad) | cos(...) |
|-------|-------------|----------------------|----------|
| 1     | 0.0417      | 1.4399               | 0.1288   |
| 2     | 0.0833      | 2.8798               | -0.9602  |
| 3     | 0.1250      | 4.3197               | -0.3664  |
| 4     | 0.1667      | 5.7596               | 0.8660   |
| 5     | 0.2083      | >= cyclePeriod        | BOUNDARY |

At 24 FPS, a 5.5 Hz flap cycle completes in ~4.36 frames.  The boundary
is detected on frame 5, when `phase` first exceeds `1/5.5 = 0.1818 sec`.

---

## 3. Cycle-Boundary Detection

**File:** `src/BFWingModel.cpp`, lines 94-124

```cpp
double cyclePeriod = (state.frequency > 0.0)
                     ? 1.0 / state.frequency
                     : 1.0;

if (state.phase >= cyclePeriod) {
    state.phase -= cyclePeriod;   // carry-over remainder
    state.flapCycle++;

    // Recompute per-angle frequency and amplitude via sigmoid
    ...
    state.frequency = maxFreq;    // update master frequency
}
```

When the accumulated phase exceeds one full period of the **master
frequency**, three things happen in rapid succession:

1. **Phase wraps** — the overshoot is preserved: `phase -= 1/freq_old`.
2. **Cycle counter increments** — `flapCycle` goes from 0 to 1.
3. **Parameters recompute** — per-angle frequency and amplitude are
   recalculated from the butterfly's current speed via the sigmoid
   (Eqs. 2-3).  The master frequency is then set to the maximum across
   all per-angle frequencies.

### Carry-over remainder

The remainder after wrapping is `phase_new = phase_old - 1/freq_old`.
In the example above: `0.2083 - 0.1818 = 0.0265 sec`.  This leftover
time is "credited" to the next cycle so that the average frequency stays
correct over many cycles.

### Why the master frequency matters

The cycle period `1/state.frequency` is determined by the **master
frequency**, which is the max across all five per-angle frequencies.  In
practice this is the gamma (flap) frequency, which has the largest range
(0-11 Hz).  A change in master frequency after the sigmoid update
directly changes how long the next cycle lasts.

---

## 4. Sigmoid-Modulated Frequency and Amplitude (Eqs. 2-3)

**File:** `src/BFWingModel.cpp`, lines 54-59

```cpp
double evalSigmoid(double speed, double range) const {
    if (range <= 0.0) return 0.0;
    double s = (maxSpeed > 0.0) ? speed / maxSpeed : 0.0;
    return range / (1.0 + std::exp(-16.0 * (s - 0.5)));
}
```

The sigmoid maps the butterfly's flight speed to a value in `[0, range]`:

| speed / maxSpeed | sigmoid output |
|------------------|----------------|
| 0.0              | range / 2981 ~= 0 |
| 0.25             | range / 55.6 ~= 0.018 * range |
| 0.5              | range / 2.0 = 0.5 * range |
| 0.75             | range * 0.982 |
| 1.0              | range * 0.9997 |

Key property: at **low speed** (near zero), the output is essentially
**zero**.  This is by design (hovering butterflies have low flap
frequency), but creates a cliff edge when the simulation starts from
rest.

At each cycle boundary, for each of the five angles:

```cpp
state.perAngleFreq[i] = params[i].freqRangeMin + evalSigmoid(speed, freqRange);
state.perAngleAmp[i]  = params[i].ampRangeMin  + evalSigmoid(speed, ampRange);
```

Since all `freqRangeMin` and `ampRangeMin` are 0 in Table 3, the
per-angle freq and amp are essentially `sigmoid(speed) * rangeMax`.

---

## 5. Angle Evaluation (Eq. 1)

**File:** `src/BFWingModel.cpp`, lines 69-75

```cpp
double evalAngle(int id, double phase, double freq, double amp) const {
    const BFAngleParams& p = params[id];
    double phiP = deg2rad(p.phaseOffset);
    return amp * cos(2 * pi * freq * phase + phiP) + p.meanAngle;
}
```

Each maneuvering angle is evaluated as:

```
theta(t) = A * cos(2*pi*f*t + phi_p) + phi_m
```

All five angles use the **same `phase`** (shared accumulator) but
different per-angle `freq` and `amp`.  The gamma angle (flap) maps to
`rotateZ` on the forewing and hindwing joints.

### Interaction between shared phase and per-angle frequency

Because the phase accumulator wraps based on the **master frequency**
(max of all per-angle frequencies), angles with lower frequencies will
NOT have completed a full cosine period when the wrap occurs.  For
example, if master frequency is 5.5 Hz but beta frequency is 1.5 Hz:

- The cycle period for the master is `1/5.5 = 0.1818 sec`.
- In that time, beta completes `1.5 * 0.1818 = 0.2727` of a period.
- When phase wraps, beta is at `cos(2*pi*0.2727 + phi_p)`, not at the
  start of its own cycle.

This is by design for the **constant-frequency** case: `cos` is
naturally periodic, so the waveform continues seamlessly across wraps.
But when the **frequency changes at a boundary**, continuity breaks.

---

## 6. The Maneuvering Controller

### `step()` — force integration

**File:** `src/BFManeuverController.cpp`, lines 173-218

```cpp
void BFManeuverController::step(BFState& state, double dt) {
    // 1. Estimate flapOmega from current gamma freq/amp
    double flapOmega = 2*pi * fGamma * aGamma_rad;

    // 2. Compute local accel (aero + vortex + gravity)
    MVector aLoc = localAccel(state, flapOmega);

    // 3. Compute preferred accel (target attraction)
    MVector aPre = preferredAccel(state);

    // 4. Euler integration of velocity (Eq. 11)
    state.velocity += (aLoc + aPre) * dt;
    // clamp to maxSpeed

    // 5. Euler integration of position
    state.position += state.velocity * dt;
}
```

The aerodynamic force depends on `flapOmega`, which is estimated as
the **peak** angular velocity of the flap motion:
`omega = 2*pi * f_gamma * A_gamma`.  This is a constant approximation —
the true angular velocity varies sinusoidally within the cycle.

### `smoothParameters()` — sliding-window smoother (Eq. 12)

**File:** `src/BFManeuverController.cpp`, lines 97-161

Called once per cycle boundary (externally, from the simulation loop).
It pushes the current raw freq/amp into a history buffer, computes the
average of the previous k=10 cycles, and blends:

```
smoothed = 0.5 * avg(previous k cycles) + 0.5 * current_raw
```

After smoothing, the master frequency is **re-synchronized** with the
smoothed per-angle frequencies (line 149):

```cpp
state.frequency = maxSmoothed;
```

This prevents the cycle-period detection from drifting relative to the
actual angle frequencies used in `evalAngle()`.

---

## 7. The Two Simulation Loops

### Current loop: kinematics only (controller disabled)

**File:** `src/BFSimulateCmd.cpp`, lines 438-456

```cpp
for (int f = startFrame; f < startFrame + duration; ++f) {
    m_state.phase += dt;
    wingModel.updateAnglesOnly(m_state);   // no cycle detection
    applyAngles(m_state.skeleton, m_state.angles);
    writeAllKeys(m_state.skeleton, m_state.angles, frameTime);
}
```

This loop:
- Manually advances `phase` by `dt` each frame.
- Calls `updateAnglesOnly()`, which evaluates the five angles from the
  current phase and the **constant default** freq/amp arrays.
- **Never** calls `wingModel.update()`, so no cycle boundary is ever
  detected, `flapCycle` stays at 0, and frequencies/amplitudes never
  change.
- The result is a perfectly periodic flapping animation because `cos` is
  naturally periodic — the initial defaults (5.5 Hz, 50 deg amplitude)
  repeat every ~4.36 frames indefinitely.

### Full dynamics loop: controller enabled (disabled due to bug)

As documented in `BFManeuverController.cpp` lines 211-217 and the
implementation log:

```cpp
for (int f = startFrame; f < startFrame + duration; ++f) {
    int prevCycle = m_state.flapCycle;
    wingModel.update(m_state, dt);         // phase + cycle detection + sigmoid
    controller.step(m_state, dt);          // force integration
    if (m_state.flapCycle != prevCycle)
        controller.smoothParameters(m_state);
    applyAngles(m_state.skeleton, m_state.angles);
    writeAllKeys(m_state.skeleton, m_state.angles, frameTime);
}
```

This loop:
- Calls `wingModel.update()`, which advances phase, detects cycle
  boundaries, and recomputes freq/amp via the sigmoid.
- Calls `controller.step()`, which integrates forces into velocity and
  position.
- At cycle boundaries, calls `smoothParameters()` to blend the new
  freq/amp with history.

---

## 8. The Wing Z-Flip Bug: Analysis

When the maneuvering controller is enabled, the wings flip in the Z
rotation axis after the first flap cycle boundary.  This section traces
the chain of events that leads to the flip.

### 8.1 Frequency collapse at the first cycle boundary

The simulation starts from rest (`velocity = (0,0,0)`).  The initial
per-angle freq/amp are mid-range hovering defaults (e.g. gamma: 5.5 Hz,
50 deg).  For the first ~4-5 frames, the wings flap normally using these
defaults.

At frame 5 (at 24 FPS), the first cycle boundary is detected.  The
sigmoid evaluates the current flight speed:

```
speed = |state.velocity|
```

Even though the controller has been integrating forces for 4-5 frames,
the butterfly starts from rest and the aerodynamic forces on a 0.428g
insect at near-zero airspeed are small.  If the accumulated speed is
`s = 0.1 m/s` (maxSpeed = 2.0):

```
sigmoid(0.1, 11.0) = 11.0 / (1 + exp(-16*(0.05 - 0.5)))
                    = 11.0 / (1 + exp(7.2))
                    = 11.0 / 1340.4
                    ~= 0.008 Hz

sigmoid(0.1, 150.0) = 150.0 / 1340.4
                     ~= 0.112 deg
```

**Result:** gamma frequency drops from 5.5 Hz to ~0.008 Hz, and
amplitude drops from 50 deg to ~0.11 deg.  The wing angle snaps from a
wide oscillation to approximately `meanAngle = 10 deg` (the phi_m for
gamma).

### 8.2 Discontinuity in the cosine argument

Just before the boundary (frame 4), the gamma angle was:

```
theta_gamma = 50 * cos(2*pi*5.5*0.1667 + 0) + 10
            = 50 * cos(5.76 rad) + 10
            = 50 * 0.866 + 10
            = 53.3 deg
```

Just after the boundary (frame 5), with collapsed parameters:

```
theta_gamma = 0.112 * cos(2*pi*0.008*0.0265 + 0) + 10
            ~= 0.112 * 1.0 + 10
            = 10.1 deg
```

The gamma angle (which drives `rotateZ` on the wing joints) jumps from
~53 deg to ~10 deg in a single frame.  This manifests as the wings
suddenly snapping flat — and depending on the cosine phase at the
moment of the boundary, the jump can cross zero, causing the wings to
appear to "flip" to the opposite side.

### 8.3 The role of FPS in determining where the boundary lands

Because the phase is advanced in discrete `dt = 1/fps` increments, the
**exact phase value at the boundary** depends on FPS:

| FPS | dt (sec) | Frames per cycle (5.5 Hz) | Phase overshoot at boundary |
|-----|----------|---------------------------|----------------------------|
| 24  | 0.0417   | 4.36                      | 0.0265 sec (14.6% of cycle) |
| 30  | 0.0333   | 5.45                      | 0.0152 sec (8.3%) |
| 48  | 0.0208   | 8.73                      | 0.0152 sec (8.3%) |
| 60  | 0.0167   | 10.91                     | 0.0152 sec (8.3%) |
| 120 | 0.0083   | 21.82                     | 0.0068 sec (3.7%) |

The phase overshoot determines **where in the cosine cycle** the
post-boundary evaluation begins, and therefore the magnitude and
direction of the discontinuity.  Different FPS values produce different
overshoot amounts, changing whether the jump is small or crosses zero
(the "flip").

### 8.4 Coarse angular sampling at typical FPS

At 24 FPS, a 5.5 Hz wave is sampled only ~4.4 times per period.  The
highest frequency in Table 3 is 11 Hz (gamma, zeta, psi, phi at max
speed), which at 24 FPS gives only ~2.2 samples per period — **below
the Nyquist limit of 12 Hz**.  At higher flight speeds, the simulation
could attempt frequencies that are critically undersampled, producing
aliased (reversed-appearing) motion even without the frequency-collapse
issue.

### 8.5 Summary of contributing factors

| Factor | Effect |
|--------|--------|
| **Velocity near zero at first boundary** | Sigmoid collapses freq/amp to near-zero, causing a large discontinuity in angle values |
| **Phase carries over from old frequency** | The overshoot `phase_new = phase_old - 1/freq_old` is evaluated with `freq_new`, producing a cosine value inconsistent with the previous frame |
| **FPS determines overshoot magnitude** | Different FPS values place the post-boundary evaluation at different cosine phases, changing the severity and direction of the jump |
| **FPS = physics dt = keyframe rate** | No sub-stepping; the force integrator runs at the same rate as keyframe output, making both force accuracy and waveform sampling dependent on a single parameter |
| **Peak flapOmega used as constant** | The aero model uses peak angular velocity instead of the instantaneous value, overestimating forces at some phases and underestimating at others |

---

## 9. Data Flow Diagram

```
User sets FPS (e.g. 24)
        |
        v
   dt = 1/FPS = 0.04167 sec
        |
        +----------------------------+
        |                            |
        v                            v
  Phase accumulator            Force integrator
  phase += dt                  velocity += (aLoc + aPre) * dt
        |                      position += velocity * dt
        v                            |
  Cycle boundary?                    v
  phase >= 1/freq_master?      speed = |velocity|
        |                            |
      [YES]                          |
        |                            |
        v                            v
  phase -= 1/freq_old          sigmoid(speed, range)
  flapCycle++                        |
        |                            v
        +<------ new freq/amp -------+
        |
        v
  evalAngle(phase_new, freq_new, amp_new)
        |
        v
  theta_gamma -> rotateZ on wing joints
        |
        v
  writeRotationKey at frame time
```

---

## 10. Current vs. Full-Dynamics Behavior at a Glance

| Aspect | Current (kinematics only) | Full dynamics (controller enabled) |
|--------|--------------------------|-----------------------------------|
| Phase advancement | Manual: `phase += dt` | Inside `wingModel.update()`: `phase += dt` |
| Cycle detection | **None** — `flapCycle` stays 0 | Active — triggers at `phase >= 1/freq` |
| Freq/amp source | Constant defaults (5.5 Hz, 50 deg) | Sigmoid from velocity, recomputed per cycle |
| Velocity | Always zero (no forces) | Integrated from aero + vortex + gravity |
| Position | Static (no translation keys) | Integrated from velocity |
| Smoothing | None | Eq. 12 sliding-window at boundaries |
| Wing motion | Perfectly periodic, correct | Flips at first boundary due to freq collapse |

---

## 11. Plan: Decouple FPS from Flap Speed

### 11.1 The problem

The single `-frameRate` (FPS) parameter currently controls **three
independent concerns** through `dt = 1/fps`:

| Concern | What it should control | What it actually controls |
|---------|-----------------------|--------------------------|
| **Keyframe density** | How many keys per playback second | Yes (1 key per frame) |
| **Visible flap speed** | How many frames one flap cycle spans | Yes — and it shouldn't |
| **Physics timestep** | Euler integration accuracy (future) | Yes — and it shouldn't |

Because `phase += 1/fps` each frame and `evalAngle` uses
`cos(2*pi*f*phase)`, the number of frames per flap cycle is:

```
framesPerCycle = (1/f) / (1/fps) = fps / f
```

At the default gamma frequency of 5.5 Hz:

| FPS | Frames per flap cycle | Playback at 24 fps |
|-----|-----------------------|---------------------|
| 24  | 4.36                  | 0.18 sec — too fast to see detail |
| 30  | 5.45                  | 0.23 sec — still very fast |
| 480 | 87.3                  | 3.64 sec — usable, but FPS is a lie |
| 960 | 174.5                 | 7.27 sec — slow motion |

This is why `fps = 480` or `960` was required in the alpha demo:
inflating FPS was the only way to slow down the visible flaps.  But this
produces 480 or 960 keyframes per second of timeline, far more than Maya
needs, and the "Frame Rate" label becomes meaningless to the artist.

### 11.2 Desired behavior

The user should be able to say:

> "I want exactly **1 flap cycle per second** of playback, sampled at
> **24 keyframes per second**."

Changing FPS from 24 to 60 should produce **more keyframes per cycle**
(smoother curves) but the same visible flap speed.  Changing the flap
period from 1.0 to 0.5 should make the wings flap **twice as fast** at
any FPS.

### 11.3 Design rationale: align with the paper

The paper (Chen et al. 2022) uses Eq. 1 in the form:

```
θ*(t) = φ_a*(u) · cos(2π · f*(u) · t + φ_p*) + φ_m*
```

where `t` is continuous time in **seconds**, `f` is in **Hz**, and all
internal parameters stay in SI units throughout.  The paper's
implementation runs in Unity, where simulation time (`Time.deltaTime`)
and display frame rate are **naturally separate** — Unity never ties
physics Δt to the render rate.

Our bug is that we set `dt = 1/fps`, forcing the simulation clock to
advance at the display rate.  The fix should mirror the paper's
architecture: keep simulation time in SI seconds, keep frequencies in
Hz, and only touch the **time-scale layer** between simulation time and
playback time.  No equation from the paper needs to change.

### 11.4 New parameters

Introduce one new internal parameter — **`simRate`** — and one
artist-facing convenience parameter — **`flapPeriod`**.

#### `simRate` (simulation seconds per playback second)

This is the ratio between simulation time and playback time.  It is
analogous to Unity's `Time.timeScale`.

```
dt_sim = simRate / fps          // simulation seconds per frame
```

No internal frequency appears in this formula.  `simRate` is a pure
time-scale factor.

| simRate | Meaning |
|---------|---------|
| 1.0     | Real-time: 5.5 Hz gamma = 5.5 flaps/sec of playback |
| 0.1818  | Slow-motion: 5.5 Hz gamma takes 1.0 sec of playback per flap |
| 0.0909  | Slower: 5.5 Hz gamma takes 2.0 sec of playback per flap |

#### `flapPeriod` (artist-facing, playback seconds per flap cycle)

This is what the artist actually thinks in: "I want one flap per
second."  It converts to `simRate` using the **initial** gamma
frequency, which is a known constant for each species preset:

```
simRate = 1 / (f_gamma_default * flapPeriod)
```

For Monarch hovering defaults (f_gamma = 5.5 Hz):

```
flapPeriod = 1.0  →  simRate = 1/(5.5 * 1.0) = 0.1818
flapPeriod = 0.5  →  simRate = 1/(5.5 * 0.5) = 0.3636
flapPeriod = 2.0  →  simRate = 1/(5.5 * 2.0) = 0.0909
```

This conversion happens **once** at the start of `doIt()`.  After that,
only `simRate` is used.  The internal frequencies in Hz never appear in
any per-frame computation.

#### Why two parameters instead of one

`simRate` is what the engine uses — a dimensionless time-scale with no
dependency on any internal frequency.  `flapPeriod` is what the artist
uses — an intuitive "seconds per flap" control.  The conversion between
them uses `f_gamma_default`, which is a **species constant** (5.5 Hz
for Monarch, from Table 3), not a simulation variable.  Changing FPS,
enabling the controller, or letting the sigmoid adjust frequencies at
runtime never affects this conversion.

#### Command flags

| Parameter | Flag | Type | Default | Meaning |
|-----------|------|------|---------|---------|
| `flapPeriod` | `-fp` / `-flapPeriod` | double | 1.0 | Playback seconds per flap cycle |
| `frameRate` | `-f` / `-frameRate` | double | 24.0 | Keyframes per playback second (unchanged) |
| `duration` | `-d` / `-duration` | int | 120 | Total frames to bake (unchanged) |
| `startFrame` | `-s` / `-startFrame` | int | 1 | First frame number (unchanged) |

Derived quantities the artist can reason about:

```
framesPerCycle = flapPeriod * fps
totalPlaybackTime = duration / fps
totalFlapCycles = totalPlaybackTime / flapPeriod
```

**Examples:**

| flapPeriod | fps | frames/cycle | 120 frames = | flap cycles |
|------------|-----|--------------|--------------|-------------|
| 1.0 sec    | 24  | 24           | 5.0 sec      | 5.0         |
| 1.0 sec    | 60  | 60           | 2.0 sec      | 2.0         |
| 0.5 sec    | 24  | 12           | 5.0 sec      | 10.0        |
| 2.0 sec    | 24  | 48           | 5.0 sec      | 2.5         |
| 0.1 sec    | 24  | 2.4          | 5.0 sec      | 50 (undersampled!) |

### 11.5 Internal mechanism

The key change is replacing `dt = 1/fps` with:

```cpp
double simRate = 1.0 / (f_gamma_default * flapPeriod);
double dt_sim  = simRate / fps;
```

Then the simulation loop uses `dt_sim` for phase advancement (and
later for force integration), while FPS only affects keyframe placement.

**Verification** at defaults (f_gamma=5.5 Hz, flapPeriod=1.0, fps=24):

```
simRate = 1/(5.5 * 1.0) = 0.1818
dt_sim  = 0.1818 / 24   = 0.007576 sec of simulation time per frame

Gamma cycle period in sim-time = 1/5.5 = 0.1818 sec
Frames per cycle = 0.1818 / 0.007576 = 24 frames
Playback time per cycle = 24 / 24 = 1.0 sec  ✓
```

**Verification** at flapPeriod=1.0, fps=60:

```
simRate = 0.1818 (unchanged — fps does not affect simRate)
dt_sim  = 0.1818 / 60 = 0.003030 sec per frame

Frames per cycle = 0.1818 / 0.003030 = 60 frames
Playback time per cycle = 60 / 60 = 1.0 sec  ✓  (same speed, denser keys)
```

The existing `evalAngle` formula is unchanged — it stays exactly as
the paper's Eq. 1:

```
θ*(t) = φ_a* · cos(2π · f* · t + φ_p*) + φ_m*
```

with `f` in Hz and `t` (`state.phase`) in simulation seconds.  The
per-angle frequency ratios (e.g. beta at 1.5 Hz vs gamma at 5.5 Hz)
are automatically preserved because they share the same simulation
clock.

### 11.6 Cycle-boundary detection: use gamma, not max

The paper (Section 4.1) defines a flapping cycle as one full period of
wing flapping — from highest position to lowest and back.  This is the
gamma angle's period.  The paper does not define a "master frequency."

Our code currently uses `state.frequency = max(perAngleFreq[0..4])` for
cycle detection.  In practice this is always gamma's frequency, because
gamma/zeta/psi/phi all share the 0-11 Hz range and the sigmoid at any
given speed returns the same value for the same range.  But using `max`
is indirect and could break if parameter ranges are later customized
per-species.

**Change:** replace the master frequency with an explicit reference to
gamma:

```cpp
// In BFWingModel::update(), replace:
//   state.frequency = (maxFreq > 0.0) ? maxFreq : 0.01;
// With:
state.frequency = (state.perAngleFreq[kAngleGamma] > 0.0)
                  ? state.perAngleFreq[kAngleGamma]
                  : 0.01;
```

This aligns with the paper and makes the code's intent explicit: the
flapping cycle is defined by the gamma (wing flap) frequency.

### 11.7 Changes per file

#### `src/BFSimulateCmd.cpp`

**`newSyntax()`** — add new flag:

```cpp
static const char* kFlapPeriodFlag     = "-fp";
static const char* kFlapPeriodFlagLong = "-flapPeriod";
// ...
syntax.addFlag(kFlapPeriodFlag, kFlapPeriodFlagLong, MSyntax::kDouble);
```

**`doIt()`** — parse and compute simRate:

```cpp
double flapPeriod = 1.0;
if (argData.isFlagSet(kFlapPeriodFlag))
    argData.getFlagArgument(kFlapPeriodFlag, 0, flapPeriod);
if (flapPeriod <= 0.0) flapPeriod = 1.0;

// Convert artist-facing flapPeriod to engine simRate.
// f_gamma_default is the initial gamma frequency from BFState
// (species constant: 5.5 Hz for Monarch).
double f_gamma_default = m_state.perAngleFreq[kAngleGamma];  // 5.5 Hz
double simRate = 1.0 / (f_gamma_default * flapPeriod);
double dt_sim  = simRate / fps;
```

**Kinematic loop** — replace `phase += dt` with `phase += dt_sim`:

```cpp
for (int f = startFrame; f < startFrame + duration; ++f) {
    m_state.phase += dt_sim;
    wingModel.updateAnglesOnly(m_state);
    applyAngles(m_state.skeleton, m_state.angles);
    MTime frameTime((double)f, MTime::uiUnit());
    writeAllKeys(m_state.skeleton, m_state.angles, frameTime);
}
```

FPS now only affects keyframe placement (via `frameTime`) — not the
wing motion.

**Future dynamics loop** — when the controller is re-enabled, use
`dt_sim` for both phase advancement and force integration.  `simRate`
is computed once and held constant; it does NOT change when the sigmoid
updates frequencies at cycle boundaries.  This means:

- The simulation's internal time scale is fixed.
- When the sigmoid changes gamma's frequency from e.g. 5.5 to 7.0 Hz,
  the gamma cycle becomes shorter **in simulation time** (1/7.0 vs
  1/5.5 sec), which maps to a proportionally shorter **playback time**.
  Faster flight = faster visible flaps.  This matches the paper's
  behavior: the sigmoid naturally produces speed-responsive flapping
  (Section 4.1, Eqs. 2-3).

```cpp
double simRate = 1.0 / (f_gamma_default * flapPeriod);
double dt_sim  = simRate / fps;

for (int f = startFrame; f < startFrame + duration; ++f) {
    int prevCycle = m_state.flapCycle;
    wingModel.update(m_state, dt_sim);
    controller.step(m_state, dt_sim);
    if (m_state.flapCycle != prevCycle)
        controller.smoothParameters(m_state);

    applyAngles(m_state.skeleton, m_state.angles);
    MTime frameTime((double)f, MTime::uiUnit());
    writeAllKeys(m_state.skeleton, m_state.angles, frameTime);
}
```

The `flapPeriod` parameter defines the baseline flap speed at hovering.
As the butterfly accelerates and the sigmoid pushes gamma's frequency
higher, flaps get visually faster — just as they would for a real
butterfly.  The sliding-window smoother (Eq. 12) prevents abrupt
transitions.

#### `src/BFSimulateCmd.h`

No structural changes needed — `simRate` and `flapPeriod` are local
variables in `doIt()`, not stored in `BFState`.

#### `src/BFState.h`

No changes needed.  `state.frequency` continues to store the gamma
frequency in Hz (see Section 11.6 for the cycle-detection change).

#### `src/BFWingModel.h` / `src/BFWingModel.cpp`

One change in `update()`: set `state.frequency` from gamma specifically
instead of `max(all)` (Section 11.6).  No other changes — `update()`
and `updateAnglesOnly()` already take `dt` as a parameter and `evalAngle`
stays exactly as Eq. 1.

#### `src/BFManeuverController.h` / `src/BFManeuverController.cpp`

No changes needed.  `step()` already takes `dt` as a parameter.  The
`smoothParameters()` re-sync of `state.frequency` (line 149) should
also be updated to use gamma specifically instead of `max(all)`.

#### `src/mel/butterFlight_ui.mel`

Add a "Flap Period" field to the **Output Settings** section (section 8)
and rename the existing FPS label for clarity:

```mel
global string $bf_flapPeriodField;
$bf_flapPeriodField = `floatFieldGrp
    -label "Flap Period (sec)"
    -numberOfFields 1
    -value1 1.0
    -precision 3
    -annotation "Playback seconds per flap cycle (hovering baseline)"
    -columnWidth2 140 140`;
```

Update `bfSimulateCallback` to read and pass the new flag:

```mel
float $flapPeriod = `floatFieldGrp -query -value1 $bf_flapPeriodField`;
// ...
bfSimulate -rigRoot $rig -duration $duration -frameRate $fps -flapPeriod $flapPeriod
```

Update `bfReset` to restore the default:

```mel
floatFieldGrp -edit -value1 1.0 $bf_flapPeriodField;
```

### 11.8 Future consideration: physics sub-stepping

Once the maneuvering controller is re-enabled, force integration
accuracy becomes a concern.  The Euler integrator in Eq. 11 needs a
small enough Δt for stability, but the keyframe rate may be much
coarser.

A robust solution is to sub-step the physics while writing one keyframe
per frame.  This adds a third independent rate — matching the paper's
implicit architecture where Unity's physics and rendering run at
independent rates:

```cpp
double simRate    = 1.0 / (f_gamma_default * flapPeriod);
double dt_sim     = simRate / fps;                         // sim-time per frame
double dt_physics = 1.0 / 240.0;                          // fixed physics step (e.g. 240 Hz)
int substeps      = std::max(1, (int)std::ceil(dt_sim / dt_physics));
dt_physics        = dt_sim / substeps;                     // evenly divide

for (int f = startFrame; f < startFrame + duration; ++f) {
    for (int s = 0; s < substeps; ++s) {
        wingModel.update(m_state, dt_physics);
        controller.step(m_state, dt_physics);
    }
    // One keyframe per frame, regardless of substep count
    applyAngles(m_state.skeleton, m_state.angles);
    MTime frameTime((double)f, MTime::uiUnit());
    writeAllKeys(m_state.skeleton, m_state.angles, frameTime);
}
```

This fully decouples all three rates:

| Rate | Controlled by | Paper equivalent |
|------|---------------|------------------|
| **Keyframe rate** | `fps` | Unity render frame rate |
| **Visible flap speed** | `flapPeriod` → `simRate` | Unity `Time.timeScale` |
| **Physics accuracy** | `substeps` / `dt_physics` | Unity `Time.fixedDeltaTime` |

### 11.9 Updated data flow (after decoupling)

```
User sets:  FPS = 24,  flapPeriod = 1.0 sec

    simRate = 1 / (f_gamma_default * flapPeriod)    [computed once]
            = 1 / (5.5 * 1.0)
            = 0.1818

    dt_sim = simRate / fps                           [no Hz in formula]
           = 0.1818 / 24
           = 0.007576 sec
                              |
          +-------------------+-------------------+
          |                                       |
          v                                       v
    Phase accumulator                       Force integrator
    phase += dt_sim                         velocity += F/m * dt_sim
          |                                 position += velocity * dt_sim
          v                                       |
    Cycle boundary?                               v
    phase >= 1/f_gamma?                     speed = |velocity|
          |                                       |
        [YES]                                     |
          |                                       v
          v                                 sigmoid(speed, range)
    phase -= 1/f_gamma                            |
    flapCycle++                                   v
          +<-------- new f, φ_a ---------+  (Eqs. 2-3, per-cycle)
          |                              |
          v                              |
    smoothParameters (Eq. 12)            |
    state.frequency = f_gamma_smoothed --+
          |
          v
    evalAngle(phase, f, amp)             (Eq. 1, unchanged)
          |
          v
    θ_gamma → rotateZ on wing joints
          |
          v
    writeRotationKey at MTime(frame, uiUnit)
                              ^
                              |
                    Frame time from FPS
                    (only affects keyframe
                     placement, not wing speed)
```
