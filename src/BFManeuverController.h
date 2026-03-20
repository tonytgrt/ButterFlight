#pragma once
// ============================================================
// BFManeuverController.h
// ButterFlight — Velocity integration, attraction, and
//                sliding-window maneuvering update
// Based on Chen et al. 2022, Section 6 (Eqs. 8–12)
// ============================================================

#include "BFState.h"
#include "BFAerodynamics.h"
#include "BFCurlNoise.h"

#include <maya/MVector.h>
#include <maya/MPoint.h>

class BFManeuverController
{
public:
    // ---- Physical parameters -----------------------------------
    double mass        = 0.000428;  // kg  (0.428 g, Monarch — Table 2)
    double maxSpeed    = 2.0;       // |u_max| (m/s)
    double sensorRange = 4.5;       // L  — max sensory length (m)

    // ---- Wing geometry (forwarded to aerodynamics) --------------
    BFWingParams wingParams;

    // ---- External inputs ----------------------------------------
    MVector wind;                   // world-space wind velocity (m/s)
    MPoint  target;                 // attraction target (world-space)
    bool    hasTarget = false;      // set true to enable attraction

    // ---- Gravity ------------------------------------------------
    static constexpr double kGravY = -9.81;  // m/s²

    // ---- Sliding-window parameters (Eq. 12) ---------------------
    static constexpr int kWindowSize = 10;   // k = 10

    // ---- Main interface -----------------------------------------

    /// Advance the simulation by one time-step.
    /// Computes forces, integrates velocity/position, and (at cycle
    /// boundaries) applies the sliding-window smoother.
    /// @param state  Per-butterfly simulation state (modified in-place).
    /// @param dt     Time-step (seconds).
    void step(BFState& state, double dt);

    // ---- Force sub-components (exposed for testing) -------------

    /// Preferred acceleration toward the attraction target (Eq. 8).
    MVector preferredAccel(const BFState& state) const;

    /// Local acceleration from aerodynamic + vortex + gravity (Eq. 10).
    MVector localAccel(const BFState& state, double flapOmega) const;

    /// Sliding-window smoother (Eq. 12).
    /// Called internally at each cycle boundary.
    void smoothParameters(BFState& state) const;

private:
    /// Ramp function R(d) (Eq. 9) — quintic smoothstep.
    static double ramp(double d);
};
