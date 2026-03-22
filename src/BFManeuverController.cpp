// ============================================================
// BFManeuverController.cpp
// ButterFlight — Velocity integration, attraction, and
//                sliding-window maneuvering update
// Based on Chen et al. 2022, Section 6 (Eqs. 8–12)
// ============================================================

#include "BFManeuverController.h"
#include "BFWingModel.h"

#include <cmath>
#include <algorithm>
#include <numeric>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

// ============================================================
// ramp — R(d), Eq. 9
//
// Quintic smoothstep that is 0 at d=0, 1 at d=1, with zero
// first derivatives at both endpoints.  Smoothly cools down
// the preferred acceleration as the butterfly nears the target.
//
//   R(d) = 6d^5 - 15d^4 + 10d^3
// ============================================================
double BFManeuverController::ramp(double d)
{
    if (d <= 0.0) return 0.0;
    if (d >= 1.0) return 1.0;
    return d * d * d * (d * (d * 6.0 - 15.0) + 10.0);
}

// ============================================================
// preferredAccel — a_pre (Eq. 8)
//
//   a_pre = R(d) * (1/m) * (p - q_i) / |p - q_i|
//
// where d = min(1, |p - q_i| / L).
// Returns zero if no target is set or distance is negligible.
// ============================================================
MVector BFManeuverController::preferredAccel(const BFState& state) const
{
    if (!hasTarget) return MVector::zero;

    MVector toTarget = MVector(target.x - state.position.x,
                               target.y - state.position.y,
                               target.z - state.position.z);
    double dist = toTarget.length();
    if (dist < 1.0e-8) return MVector::zero;

    MVector dir = toTarget / dist;

    double d = std::min(1.0, dist / sensorRange);
    double R = ramp(d);

    // a_pre = R(d) * (1/m) * direction
    return dir * (R / mass);
}

// ============================================================
// localAccel — a_loc (Eq. 10)
//
//   a_loc = (Σ F_j + F_vor + g) / m
//
// where Σ F_j is the total aerodynamic force from all four
// wings, F_vor is the curl-noise vortex force, and g is gravity.
// ============================================================
MVector BFManeuverController::localAccel(const BFState& state,
                                          double flapOmega) const
{
    // Aerodynamic force from all four wings
    MVector aeroForce = BFAerodynamics::computeTotalForce(
        state.skeleton, state.velocity, wind, flapOmega, wingParams);

    // Curl-noise vortex force at thorax position
    MVector vortexForce = BFCurlNoise::computeForce(state.position);

    // Gravity
    MVector gravity(0.0, kGravY * mass, 0.0);  // weight = m*g

    // Newton's second law: a = F_total / m
    return (aeroForce + vortexForce + gravity) / mass;
}

// ============================================================
// smoothParameters — Eq. 12
//
//   c*_{n+1} = 0.5 * (Σ_{i=max(n-k,1)}^{n-1} w_i * c*_i) + 0.5 * c*_n
//
// where w_i = 1 (uniform weights, normalized by the window
// count), k = 10, and * ∈ {f, φ_a} for each of the 5 angles.
//
// This is called once per flapping-cycle boundary.
// ============================================================
void BFManeuverController::smoothParameters(BFState& state) const
{
    const int numAngles = BFState::kNumAngles;

    // --- 1. Push current raw values into history -----------------
    std::vector<double> freqSnap(numAngles);
    std::vector<double> ampSnap(numAngles);
    for (int a = 0; a < numAngles; ++a) {
        freqSnap[a] = state.perAngleFreq[a];
        ampSnap[a]  = state.perAngleAmp[a];
    }
    state.freqHistory.push_back(freqSnap);
    state.ampHistory.push_back(ampSnap);

    // --- 2. Apply sliding-window average -------------------------
    int n = (int)state.freqHistory.size();  // current cycle index (1-based count)

    // Need at least 2 entries (previous + current) to smooth
    if (n < 2) return;

    // Window range: indices [max(n-k, 0) .. n-2]  (i.e. the previous cycles)
    int windowStart = std::max(0, n - 1 - kWindowSize);
    int windowEnd   = n - 2;  // last previous cycle
    int windowCount = windowEnd - windowStart + 1;

    for (int a = 0; a < numAngles; ++a) {
        // Average of previous cycles in the window
        double freqSum = 0.0;
        double ampSum  = 0.0;
        for (int i = windowStart; i <= windowEnd; ++i) {
            freqSum += state.freqHistory[i][a];
            ampSum  += state.ampHistory[i][a];
        }
        double freqAvg = freqSum / windowCount;
        double ampAvg  = ampSum / windowCount;

        // c*_{n+1} = 0.5 * avg(previous) + 0.5 * c*_n (current)
        state.perAngleFreq[a] = 0.5 * freqAvg + 0.5 * state.perAngleFreq[a];
        state.perAngleAmp[a]  = 0.5 * ampAvg  + 0.5 * state.perAngleAmp[a];
    }

    // --- 3. Sync master frequency with smoothed per-angle freqs --
    //  The master frequency drives cycle-period detection.  If it
    //  stays at the raw (unsmoothed) value while per-angle freqs
    //  are smoothed, the cosine in evalAngle won't complete a full
    //  period before the cycle resets, causing cumulative phase
    //  drift that inverts the wings after several cycles.
    double maxSmoothed = 0.0;
    for (int a = 0; a < numAngles; ++a) {
        if (state.perAngleFreq[a] > maxSmoothed)
            maxSmoothed = state.perAngleFreq[a];
    }
    state.frequency = (maxSmoothed > 0.0) ? maxSmoothed : 0.01;

    // --- 4. Trim history to avoid unbounded growth ---------------
    //  Keep at most kWindowSize + 1 entries.
    int maxHistory = kWindowSize + 1;
    if (n > maxHistory) {
        int excess = n - maxHistory;
        state.freqHistory.erase(state.freqHistory.begin(),
                                state.freqHistory.begin() + excess);
        state.ampHistory.erase(state.ampHistory.begin(),
                               state.ampHistory.begin() + excess);
    }
}

// ============================================================
// step — main simulation step
//
// 1. Compute flapping angular velocity from current wing state.
// 2. Compute local acceleration (aero + vortex + gravity).
// 3. Compute preferred acceleration (target attraction).
// 4. Integrate velocity and position (Eq. 11).
// 5. Detect flapping-cycle boundaries; at boundaries, apply
//    the sliding-window smoother (Eq. 12).
// ============================================================
void BFManeuverController::step(BFState& state, double dt)
{
    // ---- 1. Estimate flapping angular velocity ------------------
    //  ω ≈ 2π * f_gamma * A_gamma (peak angular velocity of flap)
    //  This is a simplified estimate used for the aerodynamic model.
    double fGamma = state.perAngleFreq[kAngleGamma];
    double aGamma = state.perAngleAmp[kAngleGamma] * M_PI / 180.0;  // to radians
    double flapOmega = 2.0 * M_PI * fGamma * aGamma;

    // ---- 2. Compute accelerations (Eqs. 8, 10) -----------------
    MVector aLoc = localAccel(state, flapOmega);
    MVector aPre = preferredAccel(state);

    // ---- 3. Integrate velocity (Eq. 11) -------------------------
    //   u_t = u_{t-1} + (a_loc + a_pre) * Δt
    state.velocity += (aLoc + aPre) * dt;

    // Clamp to max speed to prevent divergence
    double speed = state.velocity.length();
    if (speed > maxSpeed) {
        state.velocity *= maxSpeed / speed;
    }

    // ---- 4. Integrate position ----------------------------------
    state.position.x += state.velocity.x * dt;
    state.position.y += state.velocity.y * dt;
    state.position.z += state.velocity.z * dt;

    // ---- 5. Cycle-boundary smoothing ----------------------------
    //  The WingModel::update() (called separately) handles cycle
    //  detection and raw parameter recomputation.  We detect
    //  boundaries by comparing the flapCycle counter before and
    //  after the wing model update.  Since step() is called AFTER
    //  wingModel.update() in the simulation loop, we record the
    //  cycle count and apply smoothing when it changes.
    //
    //  NOTE: smoothing is triggered externally by the simulation
    //  loop after wingModel.update() increments flapCycle.
    //  See BFSimulateCmd::doIt() for the integration pattern:
    //
    //      int prevCycle = state.flapCycle;
    //      wingModel.update(state, dt);
    //      controller.step(state, dt);
    //      if (state.flapCycle != prevCycle)
    //          controller.smoothParameters(state);
}
