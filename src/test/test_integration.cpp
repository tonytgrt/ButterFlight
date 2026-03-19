// ============================================================
// test_integration.cpp
// ButterFlight — Integration test: WingModel + Aerodynamics
//
// Prerequisites:
//   Import Butterfly.fbx (or open Butterfly.ma) so the BF_
//   skeleton hierarchy is present in the scene.
//
// Usage in Maya Script Editor (MEL):
//   bfTestIntegration;
// ============================================================

#include "BFTestIntegrationCmd.h"
#include "../BFWingModel.h"
#include "../BFAerodynamics.h"
#include "../BFSimulateCmd.h"

#include <maya/MGlobal.h>
#include <maya/MVector.h>
#include <maya/MDagPath.h>
#include <maya/MFnTransform.h>
#include <maya/MEulerRotation.h>
#include <maya/MString.h>
#include <cmath>
#include <cstdio>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

static inline double deg2rad(double d) { return d * M_PI / 180.0; }

const char* BFTestIntegrationCmd::kCommandName = "bfTestIntegration";

// ---- Test helper -------------------------------------------
struct TestCtx {
    int passed = 0;
    int failed = 0;
    void check(bool cond, const char* name) {
        if (cond) { MGlobal::displayInfo(MString("  PASS: ") + name); ++passed; }
        else      { MGlobal::displayWarning(MString("  FAIL: ") + name); ++failed; }
    }
};

// ---- Apply maneuvering angles to the skeleton --------------
// (mirrors the applyAngles in BFSimulateCmd.cpp)
static void applyAngles(const BFSkeleton& skel, const BFManeuverAngles& ang)
{
    MStatus st;

    MFnTransform thoraxFn(skel.joints[kThorax], &st);
    if (st == MS::kSuccess)
        thoraxFn.setRotation(MEulerRotation(deg2rad(ang.thetaBeta), 0, 0));

    MFnTransform fwlFn(skel.joints[kForewingL], &st);
    if (st == MS::kSuccess)
        fwlFn.setRotation(MEulerRotation(
            deg2rad(ang.thetaZeta), deg2rad(ang.thetaPsi), deg2rad(ang.thetaGamma)));

    MFnTransform fwrFn(skel.joints[kForewingR], &st);
    if (st == MS::kSuccess)
        fwrFn.setRotation(MEulerRotation(
            deg2rad(ang.thetaZeta), deg2rad(ang.thetaPsi), deg2rad(ang.thetaGamma)));

    MFnTransform hwlFn(skel.joints[kHindwingL], &st);
    if (st == MS::kSuccess)
        hwlFn.setRotation(MEulerRotation(0, 0, deg2rad(ang.thetaGamma)));

    MFnTransform hwrFn(skel.joints[kHindwingR], &st);
    if (st == MS::kSuccess)
        hwrFn.setRotation(MEulerRotation(0, 0, deg2rad(ang.thetaGamma)));

    MFnTransform abdFn(skel.joints[kAbdomen], &st);
    if (st == MS::kSuccess)
        abdFn.setRotation(MEulerRotation(deg2rad(ang.thetaPhi), 0, 0));
}

// ============================================================
// doIt — run the integration tests
// ============================================================
MStatus BFTestIntegrationCmd::doIt(const MArgList& /*args*/)
{
    TestCtx t;
    MGlobal::displayInfo("=== BFIntegration Tests ===");
    MGlobal::displayInfo("  (Requires butterfly rig in scene)");

    // --------------------------------------------------------
    // Setup: resolve skeleton from the scene
    // --------------------------------------------------------
    MGlobal::displayInfo("[Setup]");

    BFSkeleton skeleton;
    MStatus status = BFSimulateCmd::readSkeleton("BF_thorax", skeleton);
    t.check(status == MS::kSuccess, "readSkeleton finds BF_thorax in scene");
    t.check(skeleton.valid, "Skeleton is valid (all 6 joints found)");

    if (!skeleton.valid) {
        MGlobal::displayError(
            "Cannot continue — make sure the butterfly model "
            "(Butterfly.fbx or Butterfly.ma) is loaded in the scene.");
        char summary[128];
        snprintf(summary, sizeof(summary),
                 "=== Results: %d passed, %d failed ===", t.passed, t.failed);
        MGlobal::displayWarning(summary);
        return MS::kFailure;
    }

    // --------------------------------------------------------
    // 1. WingModel: verify angles change over time
    // --------------------------------------------------------
    MGlobal::displayInfo("[WingModel]");

    BFWingModel wingModel;
    BFState state;
    state.skeleton = skeleton;
    state.velocity = MVector(1.0, 0.0, 0.0);  // 1 m/s forward

    double gamma0 = state.angles.thetaGamma;
    double dt = 1.0 / 24.0;

    wingModel.update(state, dt);

    t.check(state.angles.thetaGamma != gamma0,
            "WingModel: thetaGamma changes after update");
    t.check(state.phase > 0.0,
            "WingModel: phase accumulator advances");

    // Run enough updates to complete one full flap cycle
    int initialCycle = state.flapCycle;
    for (int i = 0; i < 500; ++i) {
        wingModel.update(state, dt);
        if (state.flapCycle > initialCycle) break;
    }
    t.check(state.flapCycle > initialCycle,
            "WingModel: flapCycle increments after full period");

    // ---- Diagnostic: dump one flap cycle of angles ----------
    //  Print 8 evenly spaced snapshots so you can compare
    //  against Chen et al. 2022, Figure 5 phase relationships:
    //    - gamma (flap):    phi_p =   0°  → starts at max
    //    - beta  (pitch):   phi_p = -90°  → 1/4 cycle behind gamma
    //    - zeta  (feather): phi_p = -90°  → 1/4 cycle behind gamma
    //    - psi   (sweep):   phi_p =   0°  → in phase with gamma
    //    - phi   (abdomen): phi_p = -180° → counter-phase to gamma
    MGlobal::displayInfo("[Angle Diagnostic — one flap cycle]");
    MGlobal::displayInfo("  t/T    | beta(pitch) | gamma(flap) | zeta(feath) | psi(sweep)  | phi(abdomen)");
    MGlobal::displayInfo("  -------+-----------+-----------+-----------+-----------+------------");

    BFState diagState;
    diagState.skeleton = skeleton;
    diagState.velocity = MVector(1.0, 0.0, 0.0);
    BFWingModel diagModel;

    // First update to initialize per-angle freq/amp
    diagModel.update(diagState, dt);

    double cyclePeriod = 1.0 / diagState.frequency;
    int stepsPerCycle = (int)(cyclePeriod / dt);
    int printInterval = stepsPerCycle / 8;
    if (printInterval < 1) printInterval = 1;

    int diagCycle = diagState.flapCycle;
    for (int i = 0; i < stepsPerCycle + 1; ++i) {
        if (i % printInterval == 0) {
            double tRatio = (double)i / stepsPerCycle;
            char line[256];
            snprintf(line, sizeof(line),
                     "  %5.2f  | %+8.2f    | %+8.2f    | %+8.2f    | %+8.2f    | %+8.2f",
                     tRatio,
                     diagState.angles.thetaBeta,
                     diagState.angles.thetaGamma,
                     diagState.angles.thetaZeta,
                     diagState.angles.thetaPsi,
                     diagState.angles.thetaPhi);
            MGlobal::displayInfo(line);
        }
        diagModel.update(diagState, dt);
    }

    // Verify phase relationships
    // At t=0: gamma should be near max, phi should be near min (counter-phase)
    BFState phaseState;
    phaseState.skeleton = skeleton;
    phaseState.velocity = MVector(1.0, 0.0, 0.0);
    BFWingModel phaseModel;
    phaseModel.update(phaseState, dt);

    bool counterPhase = (phaseState.angles.thetaGamma > 0.0 && phaseState.angles.thetaPhi < 0.0)
                     || (phaseState.angles.thetaGamma < 0.0 && phaseState.angles.thetaPhi > 0.0);
    t.check(counterPhase,
            "WingModel: gamma and phi have opposite signs (counter-phase)");

    // --------------------------------------------------------
    // 2. Angles → joint orientation → wing normal changes
    // --------------------------------------------------------
    MGlobal::displayInfo("[Joint Orientation]");

    MVector normalBefore = BFAerodynamics::getWingNormal(skeleton.joints[kForewingL]);

    applyAngles(skeleton, state.angles);

    MVector normalAfter = BFAerodynamics::getWingNormal(skeleton.joints[kForewingL]);
    double normalDelta = (normalAfter - normalBefore).length();

    t.check(normalDelta > 1e-6,
            "Forewing L normal changes after applyAngles");

    MVector spanDir = BFAerodynamics::getWingSpanDir(skeleton.joints[kForewingL]);
    t.check(spanDir.length() > 0.99 && spanDir.length() < 1.01,
            "Span direction is unit length");

    // --------------------------------------------------------
    // 3. Aerodynamics: force is non-zero with flapping wings
    // --------------------------------------------------------
    MGlobal::displayInfo("[Aerodynamics Force]");

    BFWingParams wp;
    MVector bodyVel(1.0, 0.0, 0.0);
    MVector wind(0.0, 0.0, 0.0);

    double flapFreq = state.perAngleFreq[kAngleGamma];
    double flapAmp  = state.perAngleAmp[kAngleGamma];
    double flapOmega = deg2rad(flapAmp) * 2.0 * M_PI * flapFreq;

    MVector totalForce = BFAerodynamics::computeTotalForce(
        skeleton, bodyVel, wind, flapOmega, wp);

    t.check(totalForce.length() > 1e-10,
            "Total aero force is non-zero during flapping");

    char buf[256];
    snprintf(buf, sizeof(buf),
             "  INFO: totalForce = (%.6f, %.6f, %.6f) N, |F| = %.6f N",
             totalForce.x, totalForce.y, totalForce.z, totalForce.length());
    MGlobal::displayInfo(buf);

    // --------------------------------------------------------
    // 4. Force with wind vs without wind
    // --------------------------------------------------------
    MGlobal::displayInfo("[Wind Effect]");

    MVector windForce = BFAerodynamics::computeTotalForce(
        skeleton, bodyVel, MVector(3.0, 0.0, 0.0), flapOmega, wp);

    double forceDiff = (windForce - totalForce).length();
    t.check(forceDiff > 1e-10,
            "Wind changes the aerodynamic force");

    snprintf(buf, sizeof(buf),
             "  INFO: wind force = (%.6f, %.6f, %.6f) N, delta = %.6f N",
             windForce.x, windForce.y, windForce.z, forceDiff);
    MGlobal::displayInfo(buf);

    // --------------------------------------------------------
    // 5. Force scales with wing area
    // --------------------------------------------------------
    MGlobal::displayInfo("[Area Scaling]");

    BFWingParams wpBig;
    wpBig.forewingArea = wp.forewingArea * 2.0;
    wpBig.hindwingArea = wp.hindwingArea * 2.0;

    MVector bigForce = BFAerodynamics::computeTotalForce(
        skeleton, bodyVel, wind, flapOmega, wpBig);

    t.check(bigForce.length() > totalForce.length(),
            "Larger wing area produces larger force");

    // --------------------------------------------------------
    // 6. Multi-frame pipeline: simulate 24 frames end-to-end
    // --------------------------------------------------------
    MGlobal::displayInfo("[Multi-frame Pipeline]");

    BFState simState;
    simState.skeleton = skeleton;
    simState.velocity = MVector(1.5, 0.0, 0.0);
    BFWingModel simModel;

    bool allForcesNonZero = true;
    double maxForce = 0.0;
    double minForce = 1e20;

    for (int frame = 0; frame < 24; ++frame) {
        simModel.update(simState, dt);
        applyAngles(skeleton, simState.angles);

        double omega = deg2rad(simState.perAngleAmp[kAngleGamma])
                     * 2.0 * M_PI * simState.perAngleFreq[kAngleGamma];
        MVector force = BFAerodynamics::computeTotalForce(
            skeleton, simState.velocity, wind, omega, wp);

        double mag = force.length();
        if (mag < 1e-15) allForcesNonZero = false;
        if (mag > maxForce) maxForce = mag;
        if (mag < minForce) minForce = mag;
    }

    t.check(allForcesNonZero,
            "24-frame sim: all frames produce non-zero force");
    t.check(maxForce > minForce,
            "24-frame sim: force magnitude varies across frames");

    snprintf(buf, sizeof(buf),
             "  INFO: force range over 24 frames: min=%.6f N, max=%.6f N",
             minForce, maxForce);
    MGlobal::displayInfo(buf);

    snprintf(buf, sizeof(buf),
             "  INFO: flap cycles completed: %d", simState.flapCycle);
    MGlobal::displayInfo(buf);

    // --------------------------------------------------------
    // Cleanup: reset all joints to rest pose (zero rotation)
    // --------------------------------------------------------
    BFManeuverAngles zeroAngles;  // all zeros by default
    applyAngles(skeleton, zeroAngles);

    // --------------------------------------------------------
    // Summary
    // --------------------------------------------------------
    char summary[128];
    snprintf(summary, sizeof(summary),
             "=== Results: %d passed, %d failed ===", t.passed, t.failed);
    if (t.failed == 0)
        MGlobal::displayInfo(summary);
    else
        MGlobal::displayWarning(summary);

    return (t.failed == 0) ? MS::kSuccess : MS::kFailure;
}
