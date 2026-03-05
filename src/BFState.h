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

// ---- Per-butterfly simulation state ------------------------
struct BFState {
    BFSkeleton       skeleton;

    // World-space position and velocity
    MPoint           position;
    MVector          velocity;

    // Current maneuvering angles
    BFManeuverAngles angles;

    // Phase accumulator (time within current flapping cycle)
    double           phase     = 0.0;
    int              flapCycle = 0;

    // Current frequency and amplitude (smoothed across cycles)
    double           frequency = 9.0;  // Hz  (Monarch default ~9-11 Hz)
    double           amplitude = 1.0;  // normalised

    // Sliding-window history for the smoother (Eq. 12, k = 10)
    std::vector<double> freqHistory;
    std::vector<double> ampHistory;
};
