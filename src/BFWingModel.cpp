// ============================================================
// BFWingModel.cpp
// ButterFlight — Maneuvering function implementation
//                (Chen et al. 2022, Eqs. 1-3, Table 3)
// ============================================================

#include "BFWingModel.h"
#include <cmath>
#include <algorithm>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

static inline double deg2rad(double d) { return d * M_PI / 180.0; }

// ============================================================
// Constructor — initialise per-angle params from Table 3
// ============================================================
//
//  Table 3 (Monarch butterfly defaults):
//  ---------------------------------------------------------------
//  Angle       R_f (Hz)   R_thetaA (deg)   phi_p (deg)  phi_m (deg)
//  theta_beta    0-3         0-30             -90           0
//  theta_gamma   0-11        0-150              0          10
//  theta_zeta    0-11        0-10             -90           0
//  theta_psi     0-11        0-20               0           0
//  theta_phi     0-11        0-35            -180         -10
//  ---------------------------------------------------------------
//
//  NOTE: The paper states the flapping frequency for gamma is
//  "0~11 Hz" but only lists the R_f column once for all wing
//  angles.  We interpret Table 3 as giving each angle its own
//  frequency range; for gamma/zeta/psi/phi they share 0-11 Hz,
//  and for beta (thorax pitch) it is 0-3 Hz.

BFWingModel::BFWingModel()
{
    // { freqRangeMin, freqRangeMax, ampRangeMin, ampRangeMax, phaseOffset, meanAngle }
    params[kAngleBeta]  = {  0.0,  3.0,    0.0,  30.0,   -90.0,   0.0 };
    params[kAngleGamma] = {  0.0, 11.0,    0.0, 150.0,     0.0,  10.0 };
    params[kAngleZeta]  = {  0.0, 11.0,    0.0,  10.0,   -90.0,   0.0 };
    params[kAnglePsi]   = {  0.0, 11.0,    0.0,  20.0,     0.0,   0.0 };
    params[kAnglePhi]   = {  0.0, 11.0,    0.0,  35.0,  -180.0, -10.0 };
}

// ============================================================
// evalSigmoid — core of Eqs. 2 and 3
//
//   result = range / (1 + exp(-16 * (|u| / |u_max| - 0.5)))
//
// "range" is R_f* (for frequency) or R_thetaA* (for amplitude).
// ============================================================
double BFWingModel::evalSigmoid(double speed, double range) const
{
    if (range <= 0.0) return 0.0;
    double s = (maxSpeed > 0.0) ? speed / maxSpeed : 0.0;
    return range / (1.0 + std::exp(-16.0 * (s - 0.5)));
}

// ============================================================
// evalAngle — Eq. 1
//
//   theta*(t) = phiA * cos(2*pi*f*t + phi_p) + phi_m
//
// All angles are in degrees.  `phase` is in seconds (accumulated
// time within the current flapping cycle).
// ============================================================
double BFWingModel::evalAngle(int id, double phase,
                              double freq, double amp) const
{
    const BFAngleParams& p = params[id];
    double phiP = deg2rad(p.phaseOffset);
    return amp * std::cos(2.0 * M_PI * freq * phase + phiP) + p.meanAngle;
}

// ============================================================
// update — advance the wing model by dt seconds
//
// 1.  Advance phase accumulator.
// 2.  Detect flapping-cycle boundary (when accumulated phase
//     exceeds 1/frequency).  At the boundary, recompute
//     per-angle frequency and amplitude from current velocity
//     via the sigmoid (Eqs. 2-3).
// 3.  Evaluate all five maneuvering angles (Eq. 1).
// ============================================================
void BFWingModel::update(BFState& state, double dt)
{
    double speed = state.velocity.length();

    // ---- 1. Advance phase (time within cycle) ------------------
    state.phase += dt;

    // ---- 2. Cycle-boundary detection ---------------------------
    //  A full cycle lasts 1/f seconds.  When phase exceeds this
    //  period, a new cycle starts and we recompute freq/amp.
    double cyclePeriod = (state.frequency > 0.0)
                         ? 1.0 / state.frequency
                         : 1.0;

    // ---- 3. Cycle-boundary: advance cycle counter only ----------
    //  Freq/amp are NO LONGER updated abruptly here.  Abrupt updates
    //  cause visible amplitude discontinuities at the seam when speed
    //  has changed significantly during the cycle.  Instead, a
    //  continuous first-order filter (step 4) drives freq/amp smoothly
    //  toward the sigmoid target every frame.
    if (state.phase >= cyclePeriod) {
        state.phase -= cyclePeriod;
        state.flapCycle++;
    }

    // ---- 4. Evaluate maneuvering angles (Eq. 1) ----------------
    state.angles.thetaBeta  = evalAngle(kAngleBeta,  state.phase,
                                        state.perAngleFreq[kAngleBeta],
                                        state.perAngleAmp[kAngleBeta]);

    state.angles.thetaGamma = evalAngle(kAngleGamma, state.phase,
                                        state.perAngleFreq[kAngleGamma],
                                        state.perAngleAmp[kAngleGamma]);

    state.angles.thetaZeta  = evalAngle(kAngleZeta,  state.phase,
                                        state.perAngleFreq[kAngleZeta],
                                        state.perAngleAmp[kAngleZeta]);

    state.angles.thetaPsi   = evalAngle(kAnglePsi,   state.phase,
                                        state.perAngleFreq[kAnglePsi],
                                        state.perAngleAmp[kAnglePsi]);

    state.angles.thetaPhi   = evalAngle(kAnglePhi,   state.phase,
                                        state.perAngleFreq[kAnglePhi],
                                        state.perAngleAmp[kAnglePhi]);

    // ---- 5. Continuous first-order filter toward sigmoid target -
    //  alpha = dt / kAdaptTime gives a time constant of kAdaptTime
    //  seconds. At 960 fps (dt=1/960) with kAdaptTime=3s:
    //    alpha ≈ 3.5e-4 → max change per frame ≈ 0.05° for gamma
    //  Over one 5.5 Hz cycle (174 frames): ≈ 9° max drift — smooth.
    //  No discrete jump at any cycle boundary regardless of speed.
    static constexpr double kAdaptTime = 3.0;  // seconds
    double alpha = dt / kAdaptTime;
    if (alpha > 1.0) alpha = 1.0;

    double maxFreq = 0.0;
    for (int i = 0; i < BFState::kNumAngles; ++i) {
        double freqRange  = params[i].freqRangeMax - params[i].freqRangeMin;
        double ampRange   = params[i].ampRangeMax  - params[i].ampRangeMin;
        double targetFreq = params[i].freqRangeMin + evalSigmoid(speed, freqRange);
        double targetAmp  = params[i].ampRangeMin  + evalSigmoid(speed, ampRange);

        state.perAngleFreq[i] += alpha * (targetFreq - state.perAngleFreq[i]);
        state.perAngleAmp[i]  += alpha * (targetAmp  - state.perAngleAmp[i]);

        if (state.perAngleFreq[i] > maxFreq)
            maxFreq = state.perAngleFreq[i];
    }
    state.frequency = (maxFreq > 0.0) ? maxFreq : 0.01;
}

// ============================================================
// updateAnglesOnly — re-evaluate angles without advancing phase
// ============================================================
void BFWingModel::updateAnglesOnly(BFState& state) const
{
    state.angles.thetaBeta  = evalAngle(kAngleBeta,  state.phase,
                                        state.perAngleFreq[kAngleBeta],
                                        state.perAngleAmp[kAngleBeta]);
    state.angles.thetaGamma = evalAngle(kAngleGamma, state.phase,
                                        state.perAngleFreq[kAngleGamma],
                                        state.perAngleAmp[kAngleGamma]);
    state.angles.thetaZeta  = evalAngle(kAngleZeta,  state.phase,
                                        state.perAngleFreq[kAngleZeta],
                                        state.perAngleAmp[kAngleZeta]);
    state.angles.thetaPsi   = evalAngle(kAnglePsi,   state.phase,
                                        state.perAngleFreq[kAnglePsi],
                                        state.perAngleAmp[kAnglePsi]);
    state.angles.thetaPhi   = evalAngle(kAnglePhi,   state.phase,
                                        state.perAngleFreq[kAnglePhi],
                                        state.perAngleAmp[kAnglePhi]);
}
