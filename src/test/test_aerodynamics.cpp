// ============================================================
// test_aerodynamics.cpp
// ButterFlight — Lightweight test command for BFAerodynamics
//
// Usage in Maya Script Editor (MEL):
//   bfTestAero;
//
// Prints PASS/FAIL results to Maya's output window.
// ============================================================

#include "BFTestAeroCmd.h"
#include "../BFAerodynamics.h"

#include <maya/MGlobal.h>
#include <maya/MVector.h>
#include <cmath>
#include <cstdio>

const char* BFTestAeroCmd::kCommandName = "bfTestAero";

// ---- Helper: approximate equality --------------------------
static bool approx(double a, double b, double tol = 1e-4)
{
    return std::fabs(a - b) < tol;
}

// ---- Helper: format and display a test result ---------------
struct TestContext {
    int passed = 0;
    int failed = 0;

    void check(bool cond, const char* name) {
        if (cond) {
            MGlobal::displayInfo(MString("  PASS: ") + name);
            ++passed;
        } else {
            MGlobal::displayWarning(MString("  FAIL: ") + name);
            ++failed;
        }
    }
};

// ============================================================
// doIt — run all aerodynamics tests
// ============================================================
MStatus BFTestAeroCmd::doIt(const MArgList& /*args*/)
{
    TestContext t;
    MGlobal::displayInfo("=== BFAerodynamics Tests ===");

    // --------------------------------------------------------
    // 1. Coefficient polynomials at known alpha values
    // --------------------------------------------------------
    MGlobal::displayInfo("[Coefficients]");

    t.check(approx(BFAerodynamics::liftCoeff(0.0), -0.34182),
            "C_l(0) = -0.34182");

    t.check(approx(BFAerodynamics::dragCoeff(0.0), 0.51127),
            "C_d(0) =  0.51127");

    t.check(approx(BFAerodynamics::liftCoeff(0.7854), -0.2771, 1e-3),
            "C_l(pi/4) ~ -0.277");

    t.check(approx(BFAerodynamics::dragCoeff(0.7854), 0.5170, 1e-3),
            "C_d(pi/4) ~  0.517");

    t.check(approx(BFAerodynamics::liftCoeff(1.5708), -0.2231, 1e-3),
            "C_l(pi/2) ~ -0.223");

    t.check(approx(BFAerodynamics::dragCoeff(1.5708), 0.5241, 1e-3),
            "C_d(pi/2) ~  0.524");

    // Drag must be positive across the entire valid range
    bool dragPositive = true;
    for (int i = 0; i <= 100; ++i) {
        double a = (M_PI / 2.0) * i / 100.0;
        if (BFAerodynamics::dragCoeff(a) < 0.0) {
            dragPositive = false;
            break;
        }
    }
    t.check(dragPositive, "C_d > 0 for all alpha in [0, pi/2]");

    // --------------------------------------------------------
    // 2. panelForce — zero velocity
    // --------------------------------------------------------
    MGlobal::displayInfo("[panelForce]");

    MVector n(0.0, 1.0, 0.0);
    MVector f = BFAerodynamics::panelForce(n, 0.001, MVector::zero);
    t.check(f.length() < 1e-12,
            "Zero velocity -> zero force");

    // --------------------------------------------------------
    // 3. panelForce — parallel flow (alpha = 0)
    // --------------------------------------------------------
    MVector airPar(2.0, 0.0, 0.0);   // along X, normal is Y
    f = BFAerodynamics::panelForce(n, 0.001, airPar);

    double q = 0.5 * BFAerodynamics::kRho * 4.0;  // 2.45
    t.check(approx(f.x, q * 0.001 * (-0.51127), 1e-6),
            "Parallel flow: drag component in -X");
    t.check(approx(f.y, q * 0.001 * (-0.34182), 1e-6),
            "Parallel flow: C_l(0) lift component in Y");
    t.check(approx(f.z, 0.0, 1e-12),
            "Parallel flow: no Z component");

    // --------------------------------------------------------
    // 4. panelForce — force scales with V^2
    // --------------------------------------------------------
    MVector air1(1.0, 0.5, 0.0);
    MVector air2(2.0, 1.0, 0.0);     // same direction, 2x speed
    MVector f1 = BFAerodynamics::panelForce(n, 0.001, air1);
    MVector f2 = BFAerodynamics::panelForce(n, 0.001, air2);
    double ratio = f2.length() / f1.length();
    t.check(approx(ratio, 4.0, 0.01),
            "Double speed -> 4x force magnitude");

    // --------------------------------------------------------
    // 5. panelForce — force scales with area
    // --------------------------------------------------------
    MVector airT(1.0, 1.0, 0.0);
    MVector fS = BFAerodynamics::panelForce(n, 0.001, airT);
    MVector fB = BFAerodynamics::panelForce(n, 0.002, airT);
    double aRatio = fB.length() / fS.length();
    t.check(approx(aRatio, 2.0, 0.01),
            "Double area -> 2x force magnitude");

    // --------------------------------------------------------
    // 6. panelForce — pure normal flow (alpha = pi/2)
    // --------------------------------------------------------
    MVector airNorm(0.0, 2.0, 0.0);  // air straight into the panel
    f = BFAerodynamics::panelForce(n, 0.001, airNorm);
    // At alpha=pi/2 there is no tangential component, so
    // liftDir is degenerate and only drag survives.
    // drag dir = -Vhat = (0,-1,0), C_d(pi/2) ~ 0.524
    double qn = 0.5 * BFAerodynamics::kRho * 4.0;
    t.check(approx(f.y, qn * 0.001 * (-0.5241), 1e-3),
            "Normal flow: drag only in -Y");
    t.check(approx(f.x, 0.0, 1e-12),
            "Normal flow: no X component");

    // --------------------------------------------------------
    // 7. panelForce — angled flow produces both lift and drag
    // --------------------------------------------------------
    MVector airAng(2.0, 1.0, 0.0);
    f = BFAerodynamics::panelForce(n, 0.001, airAng);
    t.check(f.length() > 1e-8,
            "Angled flow: non-zero force");
    // Both X and Y should be non-zero
    t.check(std::fabs(f.x) > 1e-10 && std::fabs(f.y) > 1e-10,
            "Angled flow: has both X (drag) and Y (lift) components");

    // Print the actual force vector for inspection
    char buf[128];
    snprintf(buf, sizeof(buf),
             "  INFO: angled-flow force = (%.6f, %.6f, %.6f) N",
             f.x, f.y, f.z);
    MGlobal::displayInfo(buf);

    // --------------------------------------------------------
    // Summary
    // --------------------------------------------------------
    char summary[128];
    snprintf(summary, sizeof(summary),
             "=== Results: %d passed, %d failed ===",
             t.passed, t.failed);
    if (t.failed == 0)
        MGlobal::displayInfo(summary);
    else
        MGlobal::displayWarning(summary);

    return (t.failed == 0) ? MS::kSuccess : MS::kFailure;
}
