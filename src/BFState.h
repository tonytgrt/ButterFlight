#pragma once
// ============================================================
// BFState.h
// ButterFlight — Per-butterfly skeleton and simulation state
// ============================================================

#include <maya/MDagPath.h>
#include <maya/MVector.h>
#include <maya/MPoint.h>
#include <maya/MString.h>
#include <vector>

// ---- Expected joint names (BF_ naming convention) ----------
namespace BFJointNames {
    static const char* kThorax    = "BF_thorax";
    static const char* kForewingL = "BF_forewing_L";
    static const char* kForewingR = "BF_forewing_R";
    static const char* kHindwingL = "BF_hindwing_L";
    static const char* kHindwingR = "BF_hindwing_R";
    static const char* kAbdomen   = "BF_abdomen";
}

// ---- Joint index enum --------------------------------------
enum BFJointId : int {
    kThorax = 0,
    kForewingL,
    kForewingR,
    kHindwingL,
    kHindwingR,
    kAbdomen,
    kNumJoints
};

// ---- Resolved skeleton from the Maya scene -----------------
struct BFSkeleton {
    MDagPath joints[kNumJoints];   // DAG path for each expected joint
    bool     valid = false;        // true when all joints were found
};

// ---- Five maneuvering angles (Chen et al. 2022, Eq. 1) ----
struct BFManeuverAngles {
    double thetaBeta  = 0.0;  // thorax pitch
    double thetaGamma = 0.0;  // fore-wing flap
    double thetaZeta  = 0.0;  // fore-wing feather
    double thetaPsi   = 0.0;  // fore-wing sweep
    double thetaPhi   = 0.0;  // abdomen rotation
};

// ---- Per-species wing geometry (SI units) ------------------
struct BFWingParams {
    double forewingArea    = 0.00065;  // m²  (6.5 cm² per forewing, Monarch)
    double hindwingArea    = 0.00065;  // m²  (6.5 cm² per hindwing, Monarch)
    double meanChordRadius = 0.025;    // m   (mean distance from joint to center of pressure)
};

// ---- Per-butterfly simulation state ------------------------
struct BFState {
    BFSkeleton       skeleton;

    // World-space position and velocity
    MPoint           position;
    MVector          velocity;

    // Current maneuvering angles
    BFManeuverAngles angles;

    // Phase accumulator (time within current flapping cycle, wraps at cycle boundary)
    double           phase     = 0.0;
    int              flapCycle = 0;

    // Per-angle current frequency (Hz) and amplitude (degrees),
    // held constant within one flapping cycle, recomputed at cycle boundaries.
    // Indexed by BFAngleId (0..4).
    // Defaults are mid-range hovering values (approximately sigmoid at |u|/|u_max| ≈ 0.5).
    // The sigmoid in BFWingModel::update() will adjust these at the first cycle
    // boundary based on the actual velocity.
    //                                        beta  gamma  zeta   psi    phi
    static constexpr int kNumAngles = 5;
    double           perAngleFreq[kNumAngles]  = { 1.5,  5.5,   5.5,   5.5,   5.5  };
    double           perAngleAmp [kNumAngles]  = { 15.0, 50.0,  5.0,   10.0,  17.0 };

    // Per-angle accumulated phase (radians, never wraps).
    // Incremented each frame by 2π * freq * dt to avoid phase jumps
    // when frequency changes over time.
    double           perAnglePhase[kNumAngles] = { 0.0, 0.0, 0.0, 0.0, 0.0 };

    // Master frequency used for cycle-boundary detection (max of per-angle freqs)
    double           frequency = 5.5;  // Hz  (hovering default, ramps up with speed)

    // Sliding-window history for the smoother (Eq. 12, k = 10)
    // Each inner vector has kNumAngles entries per cycle snapshot.
    std::vector<std::vector<double>> freqHistory;
    std::vector<std::vector<double>> ampHistory;
};
