#pragma once
// ============================================================
// BFWingModel.h
// ButterFlight — Maneuvering function (Chen et al. 2022, Eqs. 1-3)
// ============================================================

#include "BFState.h"
#include <cmath>

// ---- Maneuvering-angle index (matches chi set in paper) ----
enum BFAngleId : int {
    kAngleBeta  = 0,   // thorax pitch
    kAngleGamma = 1,   // fore-wing flap
    kAngleZeta  = 2,   // fore-wing feather
    kAnglePsi   = 3,   // fore-wing sweep
    kAnglePhi   = 4,   // abdomen rotation
    kNumAngles  = 5
};

// ---- Per-angle parameters from Table 3 of the paper --------
struct BFAngleParams {
    double freqRangeMin;   // R_f* lower bound (Hz)
    double freqRangeMax;   // R_f* upper bound (Hz)
    double ampRangeMin;    // R_thetaA* lower bound (degrees)
    double ampRangeMax;    // R_thetaA* upper bound (degrees)
    double phaseOffset;    // phi_p* (degrees)
    double meanAngle;      // phi_m* (degrees)
};

// ---- Wing model: evaluates all five maneuvering angles -----
class BFWingModel {
public:
    BFWingModel();

    // Compute maneuvering angles for this frame.
    // Updates state.angles, state.phase, state.flapCycle,
    // state.frequency, state.amplitude, and per-angle arrays.
    void update(BFState& state, double dt);

    // Re-evaluate angles from current phase and per-angle freq/amp
    // without advancing phase or detecting cycle boundaries.
    void updateAnglesOnly(BFState& state) const;

    // Maximum flight speed |u_max| (m/s). Monarch default ~2.0
    double maxSpeed = 2.0;

private:
    BFAngleParams params[kNumAngles];

    // Sigmoid helper: Eq. 2/3 core —  range / (1 + exp(-16*(|u|/|u_max| - 0.5)))
    double evalSigmoid(double speed, double range) const;

    // Evaluate Eq. 1 for a single angle (accumPhase is the per-angle accumulated phase in radians)
    double evalAngle(int angleId, double accumPhase, double amp) const;
};
